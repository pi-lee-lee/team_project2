#!/usr/bin/env python3
"""Analyze drive records (logs/records/**/record.jsonl.gz, written by bridge_node for each goal).

  python3 tools/drive_replay.py                     summarize the latest record (events, 1 s table, slow/stopped segments explained)
                                                    + auto-save plots of key moments (blocked start, just before replan/back-off,
                                                    closest approach, middle of slow segments; plot_<t>_<reason>.png in that goal folder, --no-plots to disable)
  python3 tools/drive_replay.py --list              list sessions (one per bridge start) / drives (goals) with results
  python3 tools/drive_replay.py DIR|FILE            summarize that record (goal folder, session folder (latest inside), record.jsonl.gz)
  python3 tools/drive_replay.py --at 23.4           detail the Frenet cycle closest to 23.4 s (why that command was issued)
  python3 tools/drive_replay.py --at 23.4 --plot    + plot (plot_23.4.png in that goal folder)
  python3 tools/drive_replay.py --set frenet.safety_margin=0.05 [--set ...]
                                                    recompute with the same inputs (scan/pose/tracks) and compare to the original commands
                                                    (open loop: not what would happen if the robot had actually moved differently)
For a closed-loop re-drive use tools/frenet_bench.py.

Location: logs/records/<bridge start time>_frenet/<time>_<goal>/record.jsonl.gz
One line = JSON. type: start (goal, params) / plan (A* path, map) / step (every 0.1 s) / event (log) / end.
step: pose (map), odom, vw (actual odom velocity), raw (follower's v, w, state), cmd (command actually sent), mode (frenet/recovery/backoff),
      front (LiDAR min distance in front +-25 deg), tracks ([id, x, y, vx, vy, r, hits, misses] odom),
      frenet on cycles where Frenet was recomputed (all inputs + result + debug (branch) + eval (per-candidate verdict/cost)).
"""
import argparse
import glob
import gzip
import json
import math
import os
import sys

import numpy as np

HOME = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(HOME, 'ros2_ws', 'src', 'warehouse_robot'))
from warehouse_robot.frenet import (REASONS, FrenetConfig, FrenetFollower, frenet_to_cartesian,  # noqa: E402
                                    lateral_shape, transform_points)


def load(path):
    """Read a record. Files still being written or cut off (truncated end) are read up to the break."""
    rows = []
    with gzip.open(path, 'rt', encoding='utf-8') as fh:
        try:
            for line in fh:
                try:
                    rows.append(json.loads(line))
                except json.JSONDecodeError:
                    break
        except (EOFError, OSError):
            pass
    return rows


def all_records():
    """All record files under logs/records (by modification time)."""
    files = glob.glob(os.path.join(HOME, 'logs', 'records', '**', '*.jsonl.gz'), recursive=True)
    return sorted(files, key=os.path.getmtime)


def latest():
    files = all_records()
    if not files:
        sys.exit('기록이 없습니다 (logs/records/). 브리지 record.enabled 를 확인하세요.')
    return files[-1]


def resolve(arg):
    """Goal folder / session folder (latest inside) / file -> record file."""
    if os.path.isdir(arg):
        files = sorted(glob.glob(os.path.join(arg, '**', '*.jsonl.gz'), recursive=True), key=os.path.getmtime)
        if not files:
            sys.exit(f'{arg}: 기록이 없습니다')
        return files[-1]
    return arg


def list_records():
    root = os.path.join(HOME, 'logs', 'records')
    last = None
    for f in all_records():
        rows = load(f)
        start = rows[0] if rows and rows[0].get('type') == 'start' else {}
        end = next((r for r in reversed(rows) if r.get('type') == 'end'), None)
        rel = os.path.relpath(f, root)
        session = rel.split(os.sep)[0] if os.sep in rel else '(폴더 없음)'
        if session != last:
            print(f'\n[{session}]')
            last = session
        g = start.get('goal', {})
        dur = rows[-1].get('t', 0.0) if rows else 0.0
        print(f"  {os.path.dirname(rel) or rel:<55} 목표 ({g.get('x', 0):.2f}, {g.get('y', 0):.2f})  {dur:5.1f}s  "
              f"{end['status'] + ' ' + end.get('message', '') if end else '끝 기록 없음'}")


def make_frenet_cfg(params, overrides=None):
    kw = {}
    for k, v in {**params, **(overrides or {})}.items():
        if k.startswith('frenet.') and k[7:] in FrenetConfig.__dataclass_fields__:
            kw[k[7:]] = tuple(v) if isinstance(v, list) else v
    return FrenetConfig(**kw)


def kind(s):
    """'frenet' if Frenet was recomputed in this cycle, else None."""
    return 'frenet' if 'frenet' in s else None


def arr(x, shape=(-1, 2)):
    return np.asarray(x if x is not None else [], dtype=float).reshape(shape)


def body_clearance(cfg, pose, pts):
    """Distance from the body outline (rectangle + attachments) at this pose to the nearest point (m, 0 if inside)."""
    pts = arr(pts)
    if not len(pts) or pose is None:
        return math.inf
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    lx = c * (pts[:, 0] - x) + s * (pts[:, 1] - y)
    ly = -s * (pts[:, 0] - x) + c * (pts[:, 1] - y)
    x0, x1, hw = cfg.footprint
    ex = cfg.footprint_extra
    best = math.inf
    for a, b, ylo, yhi in [(x0, x1, -hw, hw)] + [tuple(ex[i:i + 4]) for i in range(0, len(ex) - 3, 4)]:
        dx = np.maximum(np.maximum(a - lx, lx - b), 0.0)
        dy = np.maximum(np.maximum(ylo - ly, ly - yhi), 0.0)
        best = min(best, float(np.hypot(dx, dy).min()))
    return best


class Record:
    def __init__(self, path):
        self.path = path
        self.rows = load(path)
        if not self.rows or self.rows[0].get('type') != 'start':
            sys.exit(f'{path}: 기록 시작 줄이 없습니다')
        self.start = self.rows[0]
        self.params = self.start['params']
        self.cfg = make_frenet_cfg(self.params)
        self.steps = [r for r in self.rows if r['type'] == 'step']
        self.events = [r for r in self.rows if r['type'] in ('event', 'plan', 'end')]
        self.end = next((r for r in self.rows if r['type'] == 'end'), None)

    def plan_at(self, t):
        """A* path being followed at time t (map)."""
        path = None
        for r in self.rows:
            if r['type'] == 'plan' and r['t'] <= t:
                path = r['path']
        return arr(path)

    def step_at(self, t, planned_only=True):
        cand = [s for s in self.steps if not planned_only or kind(s)]
        if not cand:
            sys.exit('Frenet 으로 계산한 주기가 없습니다 (예전 PP/DWA 기록이거나 움직이기 전에 끝남)')
        return min(cand, key=lambda s: abs(s['t'] - t))

    def scan_of(self, s):
        src = s[kind(s)] if kind(s) else s
        return arr(src.get('scan')), arr(src.get('unmapped'))

    def odom_pose(self, s):
        return s[kind(s)]['pose'] if kind(s) else s.get('odom')


# ------------------------------------------------------------------ recompute
def replay(rec, s, overrides=None):
    """Redo the same computation from the recorded Frenet inputs (with parameters changed by overrides). (v, w, state, follower)."""
    d = s['frenet']
    p = {**rec.params, **(overrides or {})}
    f = FrenetFollower(rec.plan_at(s['t']), rec.start['goal']['yaw'], make_frenet_cfg(rec.params, overrides))
    pre = d['pre']
    f.progress, f.offset, f.turning, f.arrived = pre['progress'], pre['offset'], pre['turning'], pre['arrived']
    f.prev_shape = pre.get('prev_shape')   # missing in older records -> recompute with initial d'' = 0
    v, w, state = f.compute(tuple(d['pose']), d['vw'][0], d['vw'][1], tuple(d['map_to_odom']), arr(d['scan']),
                            arr(d['dyn'], (-1, 5)), moving_speed=p.get('tracking.moving_speed', d['moving_speed']))
    return v, w, state, f


# ------------------------------------------------------------------ explain
def explain(rec, s, out=print):
    d = s['frenet']
    ev, dbg = d.get('eval') or {}, d.get('debug') or {}
    cfg = rec.cfg
    pose = d['pose']
    scan = arr(d['scan'])
    out(f"t={s['t']:.2f}s  모드 frenet")
    out(f"  실제 속도 v={d['vw'][0]:.3f} w={d['vw'][1]:.2f}   계획 결과 v={d['result'][0]:.3f} w={d['result'][1]:.2f} "
        f"({d['result'][2]})   실제로 낸 명령 {fmt_vw(s.get('cmd'))}" + ('   [비상 정지로 전진 막힘]' if s.get('estop') else ''))
    out(f"  차체~가장 가까운 점 {body_clearance(cfg, pose, scan):.3f} m, LiDAR 앞 ±25° {s.get('front', math.inf):.3f} m")
    for t in s.get('tracks') or []:
        sp = math.hypot(t[3], t[4])
        out(f"  track {t[0]}: 차체에서 {body_clearance(cfg, pose, [t[1:3]]) - t[5]:.2f} m, 반경 {t[5]:.2f}, 속도 {sp:.2f} m/s"
            f" ({'움직이는 물체로 예측' if sp > d['moving_speed'] else '정지 물체'})")
    br = dbg.get('branch')
    if br != 'frenet':
        out("  후보 계산 전에 정해짐: " + dict(arrived='도착 후 방향 맞춤', turn_first='제자리 돌아서기',
                                                make_room='돌 자리 만들기 (짧게 앞/뒤)',
                                                approach='경로 끝 너머 목표로 접근').get(br, str(br)))
        return
    if not ev:
        return
    reason = np.asarray(ev['reason'])
    off, D, v = (np.asarray(ev[k]) for k in ('off', 'D', 'v'))
    H = np.asarray(ev['hold']) if 'hold' in ev else None      # only in older records (with hold/return)
    ms, md, total = np.asarray(ev['min_static']), np.asarray(ev['min_dynamic']), np.asarray(ev['total'])
    terms = {k: np.asarray(x) for k, x in ev['terms'].items()}
    cnt = {REASONS[r]: int((reason == r).sum()) for r in REASONS}
    out(f"  경로에서 d={ev['d0']:+.3f} m, 기울기 d'={ev['dp0']:+.2f}, 속도 상한 {ev['v_top']:.3f} (경로 끝 감속 포함), "
        f"지금 여유 {ev['now_static']:.3f} → 필요 여유 정적 {ev['need_static']:.3f} / 움직이는 물체 {ev['need_dynamic']:.3f}")
    out(f"  후보 {len(reason)}개: " + ', '.join(f'{k} {c}' for k, c in cnt.items()))
    hold = lambda h: '유지' if h < 0 else f'{h:.1f}m 뒤 복귀'  # noqa: E731

    def row(name, i):
        parts = ' '.join(f"{k}={terms[k][i]:.2f}" for k in terms)
        out(f"   {name:<10} 오프셋 {off[i]:+.3f} 거리 {D[i]:.1f} {'' if H is None else hold(H[i]):<9} 속도 {v[i]:.2f}  정적여유 {ms[i]:.3f} "
            f"동적여유 {md[i]:.3f}  {parts}  합 {total[i]:.2f}  [{REASONS[int(reason[i])]}]")

    best = ev.get('best')
    if best is None:
        out('  통과한 후보 없음 → 정지 (blocked). 여유가 가장 큰 후보들:')
        for i in np.argsort(-np.minimum(ms, md))[:3]:
            row('', int(i))
        return
    row('선택', best)
    for name, m in [('경로 위(0)', np.isclose(off, 0)), ('최고 속도', v >= v.max() - 1e-6)]:
        idx = np.flatnonzero(m)
        idx = idx[idx != best]
        if len(idx):
            row(name, int(idx[np.argmin(np.where(reason[idx] == 0, total[idx], total[idx] + 100))]))
    if abs(off[best]) > 1e-6:
        on = np.flatnonzero(np.isclose(off, 0))
        why = {REASONS[int(r)]: int((reason[on] == r).sum()) for r in np.unique(reason[on])}
        out('  경로 위(오프셋 0) 후보: ' + ', '.join(f'{k} {c}' for k, c in why.items()))


def fmt_vw(c):
    return '-' if not c else f'v={c[0]:.3f} w={c[1]:.2f}'


# ------------------------------------------------------------------ summary
def summary(rec):
    st, cfg = rec.start, rec.cfg
    out = print
    out(f"기록 {os.path.basename(rec.path)}  ({st['wall']}, 맵 {st['map']})")
    g = st['goal']
    out(f"목표 {g['id']}: ({g['x']:.2f}, {g['y']:.2f}, {math.degrees(g['yaw']):.0f}°)   결과: "
        + (f"{rec.end['status']} {rec.end.get('message', '')}" if rec.end else '끝 기록 없음 (주행 중이거나 브리지가 꺼짐)'))
    out('Frenet: ' + ', '.join(f'{k}={rec.params.get("frenet." + k)}' for k in
                               ('target_speed', 'safety_margin', 'clear_pref', 'max_offset', 'offset_step', 'lateral_dists',
                                'w_offset', 'w_change', 'w_turn', 'w_speed', 'w_clear', 'rotate_speed')))
    out('\n[사건]')
    for e in rec.events:
        if e['type'] == 'plan':
            out(f"  {e['t']:7.2f}s  A* 경로 {len(e['path'])}점 (재시도 {e['retries']})")
        elif e['type'] == 'event':
            out(f"  {e['t']:7.2f}s  {e['text']}")
        else:
            out(f"  {e['t']:7.2f}s  끝: {e['status']} {e.get('message', '')}")
    out('\n[1초 간격]  t  모드  명령 v/w  실제 v  차체~장애물  앞(LiDAR)  track(가장 가까운)  Frenet')
    nxt = 0.0
    for s in rec.steps:
        if s['t'] < nxt:
            continue
        nxt = s['t'] + 1.0
        pose = rec.odom_pose(s)
        scan, _ = rec.scan_of(s)
        tr = ''
        if s.get('tracks') and pose:
            near = min(s['tracks'], key=lambda t: body_clearance(cfg, pose, [t[1:3]]) - t[5])
            tr = f"{body_clearance(cfg, pose, [near[1:3]]) - near[5]:.2f}m {math.hypot(near[3], near[4]):.2f}m/s"
        dw = ''
        if 'frenet' in s:
            ev = s['frenet'].get('eval') or {}
            br = (s['frenet'].get('debug') or {}).get('branch')
            if br != 'frenet' or not ev:
                dw = br or ''
            elif ev.get('best') is None:
                dw = f"막힘 (통과 0/{len(ev['reason'])})"
            else:
                b = ev['best']
                dw = (f"오프셋 {ev['off'][b]:+.3f} 속도 {ev['v'][b]:.2f} 통과 {int(np.sum(np.asarray(ev['reason']) == 0))}"
                      f"/{len(ev['reason'])}")
        out(f"  {s['t']:6.1f}  {s['mode'] or '-':8} {fmt_vw(s.get('cmd')):18} {s['vw'][0]:+.3f}  "
            f"{body_clearance(cfg, pose, scan):6.3f}  {s.get('front', math.inf):6.3f}  {tr:16} {dw}")
    # slow/stopped segments: driving with Frenet below 0.03 m/s for more than 1 s
    out('\n[느리게(<0.03 m/s) 1초 넘게 간 구간 (제자리 회전 제외)]')
    seg = []
    found = False
    for s in rec.steps + [None]:
        slow = (s is not None and kind(s) is not None and abs(s[kind(s)]['result'][0]) < 0.03
                and s[kind(s)]['result'][2] != 'rotating')
        if slow:
            seg.append(s)
            continue
        if seg and seg[-1]['t'] - seg[0]['t'] > 1.0:
            found = True
            mid = seg[len(seg) // 2]
            out(f"\n  {seg[0]['t']:.1f}s ~ {seg[-1]['t']:.1f}s  (가운데 {mid['t']:.1f}s 설명)")
            explain(rec, mid, out=lambda x: print('  ' + x))
        seg = []
    if not found:
        out('  없음')


# ------------------------------------------------------------------ plots
def plot(rec, s, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    d = s['frenet']
    ev = d.get('eval') or {}
    x, y, yaw = d['pose']
    scan = arr(d['scan'])
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.scatter(scan[:, 0], scan[:, 1], s=4, c='0.5', label='scan')
    m_o = d['map_to_odom']
    route = transform_points(rec.plan_at(s['t']), *m_o)
    ax.plot(route[:, 0], route[:, 1], 'b--', lw=1, label='A* path')
    for t in s.get('tracks') or []:
        ax.add_patch(plt.Circle((t[1], t[2]), t[5], fill=False, color='m'))
        ax.arrow(t[1], t[2], t[3], t[4], head_width=0.02, color='m')
    if ev.get('off') is not None:
        # candidate shapes (independent of speed, over the static check distance) on the recomputed reference line
        _, _, _, f = replay(rec, s)
        ref = f.ref_map.transformed(*m_o)
        fc = f.cfg
        off, D, reason = (np.asarray(ev[k]) for k in ('off', 'D', 'reason'))
        end = np.asarray(ev['off_end']) if 'off_end' in ev else off          # newer records: off = middle, off_end = end
        sm = np.asarray(ev['s_mid']) if ev.get('s_mid') is not None else None
        seen = set()
        for i in range(len(off)):
            key = (off[i], end[i], D[i])
            if key in seen:
                continue
            seen.add(key)
            q = np.arange(0.0, D[i] + 1e-9, fc.check_step)              # up to the candidate length (redrawn with the current shape)
            dd, dp, ddp = lateral_shape(q, ev['d0'], ev['dp0'], end[i], D[i], ev.get('dpp0', 0.0),
                                        None if sm is None else off[i], None if sm is None else sm[i])
            sq = ev['s0'] + q
            g = frenet_to_cartesian(*ref.at(sq), np.ones_like(q), np.zeros_like(q), dd, dp, ddp)
            col = {0: 'tab:green', 1: 'tab:purple', 2: 'tab:red', 3: 'tab:orange'}[int(reason[i])]
            ax.plot(g[sq <= ref.s_end + 0.02, 0], g[sq <= ref.s_end + 0.02, 1], color=col, lw=0.5, alpha=0.4)
    if d.get('traj') is not None and len(d['traj']):
        tr = arr(d['traj'])
        ax.plot(tr[:, 0], tr[:, 1], color='k', lw=2.5, label='chosen')
    x0, x1, hw = rec.cfg.footprint
    c, sn = math.cos(yaw), math.sin(yaw)
    box = [(x0, -hw), (x1, -hw), (x1, hw), (x0, hw), (x0, -hw)]
    ax.plot([x + c * a - sn * b for a, b in box], [y + sn * a + c * b for a, b in box], 'b-', lw=2)
    ax.set_xlim(x - 1.2, x + 1.2)
    ax.set_ylim(y - 1.2, y + 1.2)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper right', fontsize=8)
    ax.set_title(f"t={s['t']:.1f}s v={d['result'][0]:.3f} w={d['result'][1]:.2f}  "
                 "green: ok / red: static / orange: moving / purple: kinematic")
    fig.savefig(path, dpi=110, bbox_inches='tight')
    print(f'그림: {path}')


def plot_path(rec, s, tag=''):
    """Plot file: plot_<t>[_<reason>].png in the goal folder for goal-folder records, otherwise <name>_<t>.png next to the file."""
    name = f"{s['t']:.1f}" + (f'_{tag}' if tag else '')
    if os.path.basename(rec.path) == 'record.jsonl.gz':
        return os.path.join(os.path.dirname(rec.path), f'plot_{name}.png')
    return rec.path.replace('.jsonl.gz', f'_{name}.png')


def key_moments(rec, limit=10):
    """Key moments to plot [(step, reason)]: blocked start, just before replan/back-off/bump/e-stop, closest approach to
    obstacles, middle of slow segments. Within 1 s of each other only the earlier (higher-priority) one is kept. Only cycles
    where Frenet was computed (so the plot shows candidates)."""
    planned = [s for s in rec.steps if kind(s)]
    if not planned:
        return []
    picks = []

    def before(t):
        cand = [s for s in planned if s['t'] <= t + 1e-6]
        return cand[-1] if cand else None
    prev = None
    for s in planned:                                       # blocked start
        st = s[kind(s)]['result'][2]
        if st == 'blocked' and prev != 'blocked':
            picks.append((s, 'blocked'))
        prev = st
    for e in rec.events:                                    # just before replan/back-off/bump/e-stop
        text = e.get('text', '')
        if e['type'] == 'event' and any(k in text for k in ('재계획', '다시 계획', '물러', '부딪힘', '비상 정지', '나온 뒤')):
            s = before(e['t'])
            if s is not None:
                picks.append((s, 'event'))
    driving = [s for s in planned if s[kind(s)]['result'][2] == 'driving']
    if driving:                                             # closest moment (while driving)
        s = min(driving, key=lambda s: body_clearance(rec.cfg, rec.odom_pose(s), rec.scan_of(s)[0]))
        picks.append((s, 'closest'))
    seg = []
    for s in planned + [None]:                              # middle of slow segments (excluding in-place turns)
        if s is not None and abs(s[kind(s)]['result'][0]) < 0.03 and s[kind(s)]['result'][2] not in ('rotating', 'blocked'):
            seg.append(s)
            continue
        if seg and seg[-1]['t'] - seg[0]['t'] > 1.0:
            picks.append((seg[len(seg) // 2], 'slow'))
        seg = []
    # per-reason quota (so plots are not all 'blocked'), priority: event > closest > blocked > slow
    quota = {'event': 4, 'closest': 1, 'blocked': 5, 'slow': 2}
    out = []
    for why in quota:
        n = 0
        for s, w in picks:
            if w == why and n < quota[why] and all(abs(s['t'] - o['t']) > 1.0 for o, _ in out):
                out.append((s, w))
                n += 1
    return sorted(out[:limit], key=lambda p: p[0]['t'])


def auto_plots(rec):
    import matplotlib
    matplotlib.use('Agg')
    moments = key_moments(rec)
    print('\n[그림 자동 저장]' + ('' if moments else '  없음 (Frenet 을 계산한 주기가 없음)'))
    for s, why in moments:
        try:
            plot(rec, s, plot_path(rec, s, why))
        except Exception as e:                              # one failed plot must not stop the whole analysis
            print(f"  {s['t']:.1f}s 그림 실패: {e}")


def parse_set(items):
    out = {}
    for it in items or []:
        k, _, v = it.partition('=')
        try:
            out[k] = json.loads(v)
        except json.JSONDecodeError:
            out[k] = v
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('file', nargs='?', help='기록 (목표 폴더, 세션 폴더, 파일. 기본: 가장 최근)')
    ap.add_argument('--list', action='store_true', help='세션/주행 목록과 결과')
    ap.add_argument('--no-plots', action='store_true', help='요약할 때 중요한 순간 그림을 저장하지 않음')
    ap.add_argument('--at', type=float, help='이 시각(s, 기록 시작부터)의 Frenet 주기를 자세히')
    ap.add_argument('--plot', action='store_true', help='--at 시각의 그림 저장')
    ap.add_argument('--set', action='append', metavar='이름=값',
                    help='다시 계산할 때 바꿀 파라미터 (frenet.*, tracking.moving_speed)')
    a = ap.parse_args()
    if a.list:
        list_records()
        return
    rec = Record(resolve(a.file) if a.file else latest())
    ov = parse_set(a.set)
    if a.at is not None:
        s = rec.step_at(a.at)
        explain(rec, s)
        v, w, state, _ = replay(rec, s)
        r = s[kind(s)]['result']
        same = abs(v - r[0]) < 1e-3 and abs(w - r[1]) < 5e-3   # records store inputs to 4 decimals -> tiny differences count as equal
        print(f"  (기록 입력으로 다시 계산: v={v:.3f} w={w:.2f} {'= 기록과 같음' if same else '≠ 기록과 다름!'})")
        if ov:
            v2, w2, st2, f2 = replay(rec, s, ov)
            print(f"\n바꾼 파라미터 {ov} 로 다시 계산: v={v2:.3f} w={w2:.2f} ({st2})")
        if a.plot:
            plot(rec, s, plot_path(rec, s))
        return
    if ov:
        print(f'바꾼 파라미터 {ov}: 같은 입력으로 다시 계산 (열린 루프)\n   t     기록 v / w        다시 v / w')
        nxt = 0.0
        for s in rec.steps:
            if not kind(s) or s['t'] < nxt:
                continue
            nxt = s['t'] + 0.5
            v2, w2, st2, _ = replay(rec, s, ov)
            r = s[kind(s)]['result']
            print(f"  {s['t']:6.1f}  {r[0]:+.3f} / {r[1]:+.2f}   {v2:+.3f} / {w2:+.2f}  {st2}")
        return
    summary(rec)
    if not a.no_plots:
        auto_plots(rec)


if __name__ == '__main__':
    main()
