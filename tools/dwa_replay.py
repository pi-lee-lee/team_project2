#!/usr/bin/env python3
"""주행 기록(logs/records/*.jsonl.gz, bridge_node 가 목표마다 남김) 분석.

  python3 tools/dwa_replay.py                     가장 최근 기록 요약 (사건, 1초 간격 표, 느림/정지 구간 설명)
                                                  + 중요한 순간 그림 자동 저장 (막힘 시작, 재계획/물러나기 직전, 가장 가까움,
                                                  느린 구간 가운데. 그 목표 폴더의 plot_<t>_<이유>.png, --no-plots 로 끔)
  python3 tools/dwa_replay.py --list              세션(브리지 켠 단위) / 주행(목표) 목록과 결과
  python3 tools/dwa_replay.py 폴더|파일            그 기록 요약 (목표 폴더, record.jsonl.gz, 예전 *.jsonl.gz)
  python3 tools/dwa_replay.py --at 23.4           23.4초에 가장 가까운 로컬 플래너(Frenet/DWA) 주기를 자세히 (왜 그 명령이 나왔나)
  python3 tools/dwa_replay.py --at 23.4 --plot    + 그림 (그 목표 폴더의 plot_23.4.png)

기록 위치: logs/records/<브리지 켠 시각>_<frenet|hybrid>/<시각>_<목표>/record.jsonl.gz
  python3 tools/dwa_replay.py --set frenet.safety_margin=0.05 [--set ...]   (DWA 기록이면 dwa.*)
                                                  같은 입력(스캔/위치/트래킹)으로 다시 계산해서 원래 명령과 비교
                                                  (열린 루프: 로봇이 실제로 다르게 움직였을 때의 결과는 아님)

기록 한 줄 = JSON. type: start(목표, 파라미터) / plan(A* 경로, map) / step(0.1초마다) / event(로그) / end.
step: pose(map), odom, vw(odom 실제 속도), raw(추종기가 낸 v, w, state), cmd(실제로 낸 명령),
      mode(frenet/pp/dwa/recovery/backoff), why(DWA 로 바뀐 이유), hybrid, front(LiDAR 앞 ±25° 최소 거리), box,
      tracks([id, x, y, vx, vy, r, hits, misses] odom),
      로컬 플래너를 새로 계산한 주기면 frenet 또는 dwa (입력 전부 + 결과 + debug(갈래) + eval(후보별 판정/비용)).
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
from warehouse_robot.dwa import DWAConfig, DWAController, DWAFollower, RobotState, transform_points  # noqa: E402
from warehouse_robot.frenet import (REASONS, FrenetConfig, FrenetFollower, frenet_to_cartesian,  # noqa: E402
                                    lateral_shape)

TERMS = ('heading', 'path', 'goal', 'obstacle', 'speed', 'collision')


def load(path):
    """기록을 읽는다. 주행 중이거나 끊긴 파일(끝이 잘림)도 읽은 데까지."""
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
    """logs/records 아래 모든 기록 파일 (수정 시각 순)."""
    files = glob.glob(os.path.join(HOME, 'logs', 'records', '**', '*.jsonl.gz'), recursive=True)
    return sorted(files, key=os.path.getmtime)


def latest():
    files = all_records()
    if not files:
        sys.exit('기록이 없습니다 (logs/records/). 브리지 record.enabled 를 확인하세요.')
    return files[-1]


def resolve(arg):
    """목표 폴더 / 세션 폴더(그 안 최근) / 파일 → 기록 파일."""
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


def make_cfg(params, overrides=None):
    kw = {}
    for k, v in params.items():
        if k.startswith('dwa.') and k[4:] in DWAConfig.__dataclass_fields__:
            kw[k[4:]] = tuple(v) if isinstance(v, list) else v
    for k, v in (overrides or {}).items():
        if k.startswith('dwa.'):
            kw[k[4:]] = v
    return DWAConfig(**kw)


def make_frenet_cfg(params, overrides=None):
    kw = {}
    for k, v in {**params, **(overrides or {})}.items():
        if k.startswith('frenet.') and k[7:] in FrenetConfig.__dataclass_fields__:
            kw[k[7:]] = tuple(v) if isinstance(v, list) else v
    return FrenetConfig(**kw)


def kind(s):
    """이 주기에 새로 계산한 로컬 플래너 ('frenet' | 'dwa' | None)."""
    return 'frenet' if 'frenet' in s else 'dwa' if 'dwa' in s else None


def arr(x, shape=(-1, 2)):
    return np.asarray(x if x is not None else [], dtype=float).reshape(shape)


def body_clearance(cfg, pose, pts):
    """지금 자세에서 차체 직사각형 외곽 ~ 가장 가까운 점 거리 (m)."""
    pts = arr(pts)
    if not len(pts) or pose is None:
        return math.inf
    d = DWAController(cfg)
    return float(d._static_clearance(np.array([[pose]], dtype=float), pts)[0, 0])


class Record:
    def __init__(self, path):
        self.path = path
        self.rows = load(path)
        if not self.rows or self.rows[0].get('type') != 'start':
            sys.exit(f'{path}: 기록 시작 줄이 없습니다')
        self.start = self.rows[0]
        self.params = self.start['params']
        self.cfg = make_cfg(self.params)
        self.steps = [r for r in self.rows if r['type'] == 'step']
        self.events = [r for r in self.rows if r['type'] in ('event', 'plan', 'end')]
        self.end = next((r for r in self.rows if r['type'] == 'end'), None)

    def plan_at(self, t):
        """t 시점에 따르던 A* 경로 (map)."""
        path = None
        for r in self.rows:
            if r['type'] == 'plan' and r['t'] <= t:
                path = r['path']
        return arr(path)

    def step_at(self, t, dwa_only=True):
        cand = [s for s in self.steps if not dwa_only or kind(s)]
        if not cand:
            sys.exit('로컬 플래너로 계산한 주기가 없습니다 (이 주행은 pure pursuit 만 썼음)')
        return min(cand, key=lambda s: abs(s['t'] - t))

    def scan_of(self, s):
        src = s[kind(s)] if kind(s) else s
        return arr(src.get('scan')), arr(src.get('unmapped'))

    def odom_pose(self, s):
        return s[kind(s)]['pose'] if kind(s) else s.get('odom')


# ------------------------------------------------------------------ 다시 계산
def replay(rec, s, overrides=None):
    """기록된 로컬 플래너 입력으로 같은 계산을 다시 (overrides 로 파라미터를 바꿔서). (v, w, state, follower)."""
    if kind(s) == 'frenet':
        return replay_frenet(rec, s, overrides)
    d = s['dwa']
    p = {**rec.params, **(overrides or {})}
    cfg = make_cfg(rec.params, overrides)
    cfg.control_period = 0.1
    f = DWAFollower(rec.plan_at(s['t']), rec.start['goal']['yaw'], cfg,
                    goal_tolerance=p['dwa_follow.goal_tolerance'], lookahead=p['dwa_follow.lookahead'])
    f.target_clear, f.target_max = p['dwa_follow.target_clear'], p['dwa_follow.target_max']
    pre = d['pre']
    f.progress, f.turning, f.arrived = pre['progress'], pre['turning'], pre['arrived']
    f.sidestep = None if pre['sidestep'] is None else np.asarray(pre['sidestep'], dtype=float)
    v, w, state = f.compute(tuple(d['pose']), d['vw'][0], d['vw'][1], tuple(d['map_to_odom']), arr(d['scan']),
                            arr(d['dyn'], (-1, 5)), unmapped_points=arr(d['unmapped']),
                            obstacle_near=d['obstacle_near'], moving_speed=p.get('dwa_follow.moving_speed', d['moving_speed']))
    return v, w, state, f


def replay_frenet(rec, s, overrides=None):
    d = s['frenet']
    p = {**rec.params, **(overrides or {})}
    f = FrenetFollower(rec.plan_at(s['t']), rec.start['goal']['yaw'], make_frenet_cfg(rec.params, overrides))
    pre = d['pre']
    f.progress, f.offset, f.turning, f.arrived = pre['progress'], pre['offset'], pre['turning'], pre['arrived']
    v, w, state = f.compute(tuple(d['pose']), d['vw'][0], d['vw'][1], tuple(d['map_to_odom']), arr(d['scan']),
                            arr(d['dyn'], (-1, 5)), moving_speed=p.get('dwa_follow.moving_speed', d['moving_speed']))
    return v, w, state, f


# ------------------------------------------------------------------ 설명
def explain_frenet(rec, s, out=print):
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
        out(f"  후보 계산 전에 정해짐: {dict(arrived='도착 후 방향 맞춤', turn_first='출발 전 제자리 돌아서기').get(br, br)}")
        return
    if not ev:
        return
    reason = np.asarray(ev['reason'])
    off, D, H, v = (np.asarray(ev[k]) for k in ('off', 'D', 'hold', 'v'))
    ms, md, total = np.asarray(ev['min_static']), np.asarray(ev['min_dynamic']), np.asarray(ev['total'])
    terms = {k: np.asarray(x) for k, x in ev['terms'].items()}
    cnt = {REASONS[r]: int((reason == r).sum()) for r in REASONS}
    out(f"  경로에서 d={ev['d0']:+.3f} m, 기울기 d'={ev['dp0']:+.2f}, 속도 상한 {ev['v_top']:.3f} (경로 끝 감속 포함), "
        f"지금 여유 {ev['now_static']:.3f} → 필요 여유 정적 {ev['need_static']:.3f} / 움직이는 물체 {ev['need_dynamic']:.3f}")
    out(f"  후보 {len(reason)}개: " + ', '.join(f'{k} {c}' for k, c in cnt.items()))
    hold = lambda h: '유지' if h < 0 else f'{h:.1f}m 뒤 복귀'  # noqa: E731

    def row(name, i):
        parts = ' '.join(f"{k}={terms[k][i]:.2f}" for k in terms)
        out(f"   {name:<10} 오프셋 {off[i]:+.3f} 거리 {D[i]:.1f} {hold(H[i]):<9} 속도 {v[i]:.2f}  정적여유 {ms[i]:.3f} "
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


def stage_of(ev, i):
    """후보 i 가 어디서 빠졌는지 / 남았는지."""
    if not ev['valid'][i]:
        return '충돌 판정 탈락 (멈출 수 없음)'
    if ev.get('how') != 'cost':
        return '남음'
    if ev['safe_only'] and not ev['safe'][i]:
        return '"끝까지 안전" 규칙 탈락 (여유 침범)'
    if ev['fwd_only'] and ev['v'][i] < 0:
        return '후진 제외 규칙 탈락'
    return '남음'


def explain(rec, s, out=print):
    if kind(s) == 'frenet':
        return explain_frenet(rec, s, out)
    d = s['dwa']
    ev, dbg = d.get('eval'), d.get('debug') or {}
    cfg = rec.cfg
    pose = d['pose']
    scan, unmapped = arr(d['scan']), arr(d['unmapped'])
    out(f"t={s['t']:.2f}s  모드 {s['mode']}  전환 이유: {s.get('why')}")
    out(f"  실제 속도 v={d['vw'][0]:.3f} w={d['vw'][1]:.2f}   DWA 결과 v={d['result'][0]:.3f} w={d['result'][1]:.2f} "
        f"({d['result'][2]})   실제로 낸 명령 {fmt_vw(s.get('cmd'))}" + ('   [비상 정지로 전진 막힘]' if s.get('estop') else ''))
    out(f"  차체~가장 가까운 점 {body_clearance(cfg, pose, scan):.3f} m (지도에 없는 점만 {body_clearance(cfg, pose, unmapped):.3f} m),"
        f" LiDAR 앞 ±25° {s.get('front', math.inf):.3f} m, 경로이탈 비용 {'꺼짐' if d['obstacle_near'] else '켜짐'}")
    for t in s.get('tracks') or []:
        sp = math.hypot(t[3], t[4])
        dist = body_clearance(cfg, pose, [t[1:3]]) - t[5]
        out(f"  track {t[0]}: 차체에서 {dist:.2f} m, 반경 {t[5]:.2f}, 속도 {sp:.2f} m/s"
            f" ({'움직이는 물체로 예측' if sp > d['moving_speed'] else '정지 물체'}), hits {t[6]}")
    br = dbg.get('branch')
    if br != 'dwa':
        out(f"  DWA 비용 계산 전에 정해짐: {dict(arrived='도착 후 방향 맞춤', turn_first='출발 전 제자리 돌아서기', turn_escape='돌 공간이 없어 앞/뒤로 빠져나오기').get(br, br)}")
        return
    g0, g = dbg.get('goal_raw'), dbg.get('goal')
    moved = g0 is not None and g is not None and math.dist(g0, g) > 1e-3
    out(f"  목표점 {np.round(g, 2).tolist()}" + (f" (장애물 너머로 옮김, 원래 {np.round(g0, 2).tolist()})" if moved else '')
        + ('  [옆 차선 비키는 중]' if dbg.get('sidestep') is not None else ''))
    if not ev:
        return
    v, w = np.asarray(ev['v']), np.asarray(ev['w'])
    valid = np.asarray(ev['valid'], dtype=bool)
    n = len(v)
    if ev['how'] != 'cost':
        out(f"  후보 {n}개 중 충돌 판정 통과 {valid.sum()}개 → " +
            ('통과 없음: 여유가 가장 커지는 쪽으로 빠져나옴' if ev['how'] == 'escape' else '통과 없음: 정지 (blocked)'))
        return
    safe = np.asarray(ev['safe'], dtype=bool)
    out(f"  후보 {n}개 (v {v.min():.3f}~{v.max():.3f}, w {w.min():.2f}~{w.max():.2f}): 충돌 판정 통과 {valid.sum()}, "
        f"끝까지 안전 {safe.sum()}" + (' → 안전한 것 중에서만 고름' if ev['safe_only'] else ' → 안전한 게 없어 비용으로 고름')
        + (', 후진 제외' if ev['fwd_only'] else ''))
    terms = {k: np.asarray(ev['terms'][k]) for k in TERMS}
    total = np.asarray(ev['total'])
    best = ev['best']

    def row(name, i):
        parts = ' '.join(f"{k}={terms[k][i]:.3f}" for k in TERMS)
        out(f"   {name:<14} v={v[i]:+.3f} w={w[i]:+.2f}  최소여유 {ev['clear'][i]:.3f}  {parts}  합 {total[i]:.3f}  [{stage_of(ev, i)}]")

    row('선택', best)
    groups = [('가장 빠른 전진', v >= v.max() - 1e-6), ('제자리 회전', (v == 0) & (w != 0)), ('가만히', (v == 0) & (w == 0)),
              ('후진', v < 0)]
    for name, m in groups:
        idx = np.flatnonzero(m)
        idx = idx[idx != best]
        if len(idx):
            row(name, int(idx[np.argmin(total[idx])]))
    # 선택보다 빠른 후보들이 왜 졌나
    faster = np.flatnonzero(v > v[best] + 0.02)
    if len(faster):
        why = {}
        for i in faster:
            st = stage_of(ev, i)
            if st == '남음':
                k = max(TERMS, key=lambda k: terms[k][i] - terms[k][best])
                st = f'비용에서 짐 (가장 큰 차이: {k})'
            why[st] = why.get(st, 0) + 1
        out('  더 빠른 후보 %d개가 진 이유: %s' % (len(faster), ', '.join(f'{k} {c}개' for k, c in why.items())))


def fmt_vw(c):
    return '-' if not c else f'v={c[0]:.3f} w={c[1]:.2f}'


# ------------------------------------------------------------------ 요약
def summary(rec):
    st, cfg = rec.start, rec.cfg
    out = print
    out(f"기록 {os.path.basename(rec.path)}  ({st['wall']}, 맵 {st['map']})")
    g = st['goal']
    out(f"목표 {g['id']}: ({g['x']:.2f}, {g['y']:.2f}, {math.degrees(g['yaw']):.0f}°)   결과: "
        + (f"{rec.end['status']} {rec.end.get('message', '')}" if rec.end else '끝 기록 없음 (주행 중이거나 브리지가 꺼짐)'))
    if rec.params.get('drive.controller', 'hybrid') == 'frenet' or any('frenet' in s for s in rec.steps[:50]):
        out('Frenet: ' + ', '.join(f'{k}={rec.params.get("frenet." + k)}' for k in
                                   ('target_speed', 'safety_margin', 'clear_pref', 'max_offset', 'offset_step', 'check_dist',
                                    'w_offset', 'w_change', 'w_turn', 'w_speed', 'w_clear')))
    else:
        out('DWA: ' + ', '.join(f'{k}={rec.params.get("dwa." + k)}' for k in
                                ('max_v', 'predict_time', 'safety_margin', 'obstacle_influence', 'w_obstacle', 'w_goal',
                                 'w_path', 'w_speed', 'w_heading', 'max_acc_w')))
    out('\n[사건]')
    for e in rec.events:
        if e['type'] == 'plan':
            out(f"  {e['t']:7.2f}s  A* 경로 {len(e['path'])}점 (재시도 {e['retries']})")
        elif e['type'] == 'event':
            out(f"  {e['t']:7.2f}s  {e['text']}")
        else:
            out(f"  {e['t']:7.2f}s  끝: {e['status']} {e.get('message', '')}")
    out('\n[1초 간격]  t  모드  명령 v/w  실제 v  차체~장애물  앞(LiDAR)  track(가장 가까운)  로컬 플래너')
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
        elif 'dwa' in s:
            ev = s['dwa'].get('eval') or {}
            br = (s['dwa'].get('debug') or {}).get('branch')
            dw = br if br != 'dwa' else (f"{ev.get('how')} 안전 {int(np.sum(ev['safe']))}/{len(ev['v'])}"
                                         if ev.get('how') == 'cost' else ev.get('how', ''))
        out(f"  {s['t']:6.1f}  {s['mode'] or '-':8} {fmt_vw(s.get('cmd')):18} {s['vw'][0]:+.3f}  "
            f"{body_clearance(cfg, pose, scan):6.3f}  {s.get('front', math.inf):6.3f}  {tr:16} {dw}")
    # 느림/정지 구간: DWA 로 가는데 0.03 m/s 미만이 1초 넘게
    out('\n[로컬 플래너로 느리게(<0.03 m/s) 1초 넘게 간 구간 (제자리 회전 제외)]')
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


# ------------------------------------------------------------------ 그림
def plot_frenet(rec, s, path, plt):
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
        # 후보 모양 (속도와 무관, 정적 검사 거리만큼): 다시 계산한 기준선 위에
        _, _, _, f = replay_frenet(rec, s)
        ref = f.ref_map.transformed(*m_o)
        fc = f.cfg
        q = np.arange(0.0, fc.check_dist + 1e-9, fc.check_step)
        off, D, H, reason = (np.asarray(ev[k]) for k in ('off', 'D', 'hold', 'reason'))
        seen = set()
        for i in range(len(off)):
            key = (off[i], D[i], H[i])
            if key in seen:
                continue
            seen.add(key)
            dd, dp, ddp = lateral_shape(q, ev['d0'], ev['dp0'], off[i], D[i], H[i])
            sq = ev['s0'] + q
            g = frenet_to_cartesian(*ref.at(sq), np.ones_like(q), np.zeros_like(q), dd, dp, ddp)
            col = {0: 'tab:green', 1: 'tab:purple', 2: 'tab:red', 3: 'tab:orange'}[int(reason[i])]
            ax.plot(g[sq <= ref.s_end + 0.02, 0], g[sq <= ref.s_end + 0.02, 1], color=col, lw=0.5, alpha=0.4)
    if d.get('traj') is not None and len(d['traj']):
        tr = arr(d['traj'])
        ax.plot(tr[:, 0], tr[:, 1], color='k', lw=2.5, label='chosen')
    cfg = make_cfg(rec.params)
    x0, x1, hw = cfg.footprint
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
    """그림 파일: 목표 폴더 기록이면 그 폴더의 plot_<t>[_<이유>].png, 예전 파일이면 옆에 <이름>_<t>.png."""
    name = f"{s['t']:.1f}" + (f'_{tag}' if tag else '')
    if os.path.basename(rec.path) == 'record.jsonl.gz':
        return os.path.join(os.path.dirname(rec.path), f'plot_{name}.png')
    return rec.path.replace('.jsonl.gz', f'_{name}.png')


def key_moments(rec, limit=10):
    """그림으로 남길 중요한 순간 [(step, 이유)]: 막힘 시작, 재계획/물러나기/부딪힘/비상 정지 직전, 장애물과 가장 가까운 순간,
    느린 구간 가운데. 1초 안에 겹치면 앞의 것(우선순위 높은 것)만. 로컬 플래너를 계산한 주기만 (그림에 후보가 나오게)."""
    planned = [s for s in rec.steps if kind(s)]
    if not planned:
        return []
    picks = []

    def before(t):
        cand = [s for s in planned if s['t'] <= t + 1e-6]
        return cand[-1] if cand else None
    prev = None
    for s in planned:                                       # 막힘 시작
        st = s[kind(s)]['result'][2]
        if st == 'blocked' and prev != 'blocked':
            picks.append((s, 'blocked'))
        prev = st
    for e in rec.events:                                    # 재계획/물러나기/부딪힘/비상 정지 직전
        text = e.get('text', '')
        if e['type'] == 'event' and any(k in text for k in ('재계획', '다시 계획', '물러', '부딪힘', '비상 정지', '나온 뒤')):
            s = before(e['t'])
            if s is not None:
                picks.append((s, 'event'))
    driving = [s for s in planned if s[kind(s)]['result'][2] == 'driving']
    if driving:                                             # 가장 가까운 순간 (주행 중)
        s = min(driving, key=lambda s: body_clearance(rec.cfg, rec.odom_pose(s), rec.scan_of(s)[0]))
        picks.append((s, 'closest'))
    seg = []
    for s in planned + [None]:                              # 느린 구간 (제자리 회전 제외) 가운데
        if s is not None and abs(s[kind(s)]['result'][0]) < 0.03 and s[kind(s)]['result'][2] not in ('rotating', 'blocked'):
            seg.append(s)
            continue
        if seg and seg[-1]['t'] - seg[0]['t'] > 1.0:
            picks.append((seg[len(seg) // 2], 'slow'))
        seg = []
    # 이유별 개수 제한 (막힘만 10장이 되지 않게), 우선순위: 사건 > 가장 가까움 > 막힘 > 느림
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
    print('\n[그림 자동 저장]' + ('' if moments else '  없음 (로컬 플래너를 계산한 주기가 없음)'))
    for s, why in moments:
        try:
            plot(rec, s, plot_path(rec, s, why))
        except Exception as e:                              # 그림 하나 실패로 분석 전체가 멈추지 않게
            print(f"  {s['t']:.1f}s 그림 실패: {e}")


def plot(rec, s, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    if kind(s) == 'frenet':
        return plot_frenet(rec, s, path, plt)
    d = s['dwa']
    ev = d.get('eval') or {}
    cfg = make_cfg(rec.params)
    x, y, yaw = d['pose']
    scan, unmapped = arr(d['scan']), arr(d['unmapped'])
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.scatter(scan[:, 0], scan[:, 1], s=4, c='0.5', label='scan')
    if len(unmapped):
        ax.scatter(unmapped[:, 0], unmapped[:, 1], s=8, c='tab:orange', label='not in map')
    route = transform_points(rec.plan_at(s['t']), *d['map_to_odom'])
    ax.plot(route[:, 0], route[:, 1], 'b--', lw=1, label='A* path')
    for t in s.get('tracks') or []:
        ax.add_patch(plt.Circle((t[1], t[2]), t[5], fill=False, color='m'))
        ax.arrow(t[1], t[2], t[3], t[4], head_width=0.02, color='m')
    if ev.get('v') is not None:
        ctl = DWAController(cfg)
        v, w = np.asarray(ev['v']), np.asarray(ev['w'])
        trajs = ctl._rollout(RobotState(x, y, yaw, *d['vw']), v, w)
        valid = np.asarray(ev['valid'], dtype=bool)
        safe = np.asarray(ev.get('safe', valid), dtype=bool)
        for i, tr in enumerate(trajs):
            col = 'tab:red' if not valid[i] else ('tab:orange' if ev.get('safe_only') and not safe[i] else 'tab:green')
            ax.plot(tr[:, 0], tr[:, 1], color=col, lw=0.5, alpha=0.5)
        if ev.get('best') is not None:
            tr = trajs[ev['best']]
            ax.plot(tr[:, 0], tr[:, 1], color='k', lw=2.5, label='chosen')
    x0, x1, hw = cfg.footprint
    c, sn = math.cos(yaw), math.sin(yaw)
    box = [(x0, -hw), (x1, -hw), (x1, hw), (x0, hw), (x0, -hw)]
    ax.plot([x + c * a - sn * b for a, b in box], [y + sn * a + c * b for a, b in box], 'b-', lw=2)
    dbg = d.get('debug') or {}
    if dbg.get('goal_raw') is not None:
        ax.plot(*dbg['goal_raw'], 'c^', ms=10, label='goal (lookahead)')
    if dbg.get('goal') is not None:
        ax.plot(*dbg['goal'], 'g*', ms=14, label='goal (used)')
    ax.set_xlim(x - 1.2, x + 1.2)
    ax.set_ylim(y - 1.2, y + 1.2)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper right', fontsize=8)
    ax.set_title(f"t={s['t']:.1f}s v={d['result'][0]:.3f} w={d['result'][1]:.2f}  "
                 "red: collision / orange: not safe / green: eligible")
    fig.savefig(path, dpi=110, bbox_inches='tight')
    print(f'그림: {path}')


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
    ap.add_argument('--at', type=float, help='이 시각(s, 기록 시작부터)의 로컬 플래너 주기를 자세히')
    ap.add_argument('--plot', action='store_true', help='--at 시각의 그림 저장')
    ap.add_argument('--set', action='append', metavar='이름=값',
                    help='다시 계산할 때 바꿀 파라미터 (frenet.*, dwa.*, dwa_follow.*)')
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
        same = abs(v - r[0]) < 1e-3 and abs(w - r[1]) < 5e-3   # 기록은 입력을 소수 4자리로 저장 → 아주 작은 차이는 같음
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
