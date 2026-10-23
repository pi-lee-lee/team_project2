#!/usr/bin/env python3
"""실물 주행 기록으로 Frenet 추종기 폐루프 비교 (로봇 없이).

기록(logs/records/**/record.jsonl.gz) 하나마다: 그 주행 동안 본 스캔 점 중 여러 주기(min_hits)에 반복해서 보인 2 cm 칸만
정적 환경으로 삼고(지나간 사람은 빠짐), 첫 A* 경로와 출발 자세에서 지금 코드의 FrenetFollower 로 다시 달린다
(명령을 가속 한계 안에서 따르는 로봇, 움직이는 물체 없음. 정적으로 1초 막히면 그 환경으로 A* 재계획(목표당 4번, 브리지와 같음),
5초 넘게 막히면 '막힘' 으로 끝 — 브리지는 여기서 물러나기).
서 있던 사람/물건은 남으므로 절대값보다 '코드 바꾸기 전/후 비교' 용.

  python3 tools/frenet_bench.py                 오늘(가장 최근 날짜) 기록 전부
  python3 tools/frenet_bench.py 폴더 [폴더 ...]   그 기록들
  python3 tools/frenet_bench.py --set frenet.w_change=1.0   파라미터 바꿔서
  python3 tools/frenet_bench.py --recorded                  기록 당시 파라미터로 (기본은 지금 params.yaml 의 frenet.*)

지표: 도착(done)/시간, 막힘(blocked) 시간, 주행 중 제자리 회전(출발 전 제외) 횟수, 각속도 흔들림(명령 w 의 주기당 변화 합/초),
오프셋 바뀜 횟수, 최소 여유(차체+부착물, m).
"""
import argparse
import math
import os
import sys

import numpy as np

HOME = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(HOME, 'tools'))
sys.path.insert(0, os.path.join(HOME, 'ros2_ws', 'src', 'warehouse_robot'))
import drive_replay as R  # noqa: E402
from warehouse_robot.frenet import FrenetFollower, FrenetPlanner, StaticGrid, body_points  # noqa: E402


def environment(rec, min_hits=10):
    """주행 동안 본 스캔 점 (odom) 중 min_hits 주기 넘게 보인 2 cm 칸 (칸마다 점 하나).
    그 주기 로봇 차체(+부착물) 둘레 3 cm 안의 점은 뺀다 (서 있는 동안 LiDAR 가 본 자기 부착물이 벽처럼 쌓이지 않게)."""
    cfg = R.make_frenet_cfg(rec.params)
    x0, x1, hw = cfg.footprint
    ex = cfg.footprint_extra
    rects = [(x0, x1, -hw, hw)] + [tuple(ex[i:i + 4]) for i in range(0, len(ex) - 3, 4)]
    cells = {}
    for s in rec.steps:
        src = s.get('frenet') or s.get('dwa') or s   # 예전 DWA 기록도 스캔은 'dwa' 안에
        p = R.arr(src.get('scan'))
        pose = src.get('pose') if 'pose' in src and src is not s else s.get('odom')
        if not len(p):
            continue
        if pose is not None:
            c, sn = math.cos(pose[2]), math.sin(pose[2])
            lx = c * (p[:, 0] - pose[0]) + sn * (p[:, 1] - pose[1])
            ly = -sn * (p[:, 0] - pose[0]) + c * (p[:, 1] - pose[1])
            own = np.zeros(len(p), dtype=bool)
            for a, b, ylo, yhi in rects:
                own |= (lx > a - 0.03) & (lx < b + 0.03) & (ly > ylo - 0.03) & (ly < yhi + 0.03)
            p = p[~own]
        for cell in set(map(tuple, np.floor(p / 0.02).astype(np.int64))):
            cells[cell] = cells.get(cell, 0) + 1
    keep = np.array([c for c, n in cells.items() if n >= min_hits], dtype=float).reshape(-1, 2)
    return (keep + 0.5) * 0.02


def astar_for(env, pts_extra=(), res=0.05, pad=1.0):
    """환경 점으로 만든 점유 격자의 A* (본 적 없는 칸은 빈 칸으로). 브리지/플래너 설정과 같은 반경."""
    from warehouse_robot.astar import GridPlanner
    pts = np.vstack([env] + [np.asarray(pts_extra, dtype=float).reshape(-1, 2)])
    ox, oy = pts.min(axis=0) - pad
    w, h = (np.ceil((pts.max(axis=0) + pad - [ox, oy]) / res)).astype(int)
    grid = np.zeros((h, w), dtype=np.int16)
    j = np.floor((env[:, 0] - ox) / res).astype(int)
    i = np.floor((env[:, 1] - oy) / res).astype(int)
    grid[i, j] = 100
    return GridPlanner(grid.ravel(), w, h, res, ox, oy, robot_radius=0.24, inflation_radius=0.55)


def run(rec, overrides=None, max_time=120.0):
    """odom 프레임에서 폐루프. 정적으로 1초 넘게 막히면 A* 재계획 (목표당 4번, 브리지와 같음), 5초 넘게 막히면 끝."""
    first = next((s for s in rec.steps if 'frenet' in s), None)
    if first is None:
        return None
    d = first['frenet']
    m_o = tuple(d['map_to_odom'])
    cfg = R.make_frenet_cfg(rec.params, overrides)
    path = rec.plan_at(first['t'])
    if not len(path):
        path = R.arr(next((r['path'] for r in rec.rows if r['type'] == 'plan'), None))
    if len(path) < 2:
        return None
    path = R.transform_points(path, *m_o)                      # odom 으로 (재계획도 odom 에서)
    goal = path[-1]
    goal_yaw = rec.start['goal']['yaw'] + m_o[2]
    f = FrenetFollower(path, goal_yaw, cfg)
    env = environment(rec)
    planner = None
    x, y, yaw = d['pose']
    v = 0.0
    body = body_points(cfg.footprint, cfg.footprint_extra)
    out = {'state': None, 'time': 0.0, 'blocked': 0.0, 'mid_turns': 0, 'w_jerk': 0.0, 'offset_changes': 0,
           'clear': math.inf, 'replans': 0}
    w_prev, off_prev, moved, turning_prev, blocked_run = 0.0, None, False, False, 0.0
    for k in range(int(max_time / 0.1)):
        near = env[np.hypot(env[:, 0] - x, env[:, 1] - y) < 2.5]
        vc, wc, st = f.compute((x, y, yaw), v, 0.0, (0.0, 0.0, 0.0), near, np.zeros((0, 5)))
        br = f.last_debug.get('branch')
        if st == 'done':
            out['state'] = 'done'
            break
        out['blocked'] += 0.1 * (st == 'blocked')
        blocked_run = blocked_run + 0.1 if st == 'blocked' else 0.0
        if st == 'blocked' and f.static_blocked and blocked_run > 1.0 and out['replans'] < 4:
            planner = planner or astar_for(env, [(x, y), goal])
            try:
                newp = planner.plan((x, y), tuple(goal))
            except Exception:
                newp = None
            out['replans'] += 1
            if newp and len(newp) >= 2:
                f = FrenetFollower(newp, goal_yaw, cfg)
                off_prev, blocked_run = None, 0.0
        if blocked_run > 5.0:          # 실제 브리지는 여기서 물러나기
            out['state'] = 'blocked'
            break
        turning = br in ('turn_first', 'make_room')
        if turning and not turning_prev and moved:
            out['mid_turns'] += 1
        turning_prev = turning
        out['w_jerk'] += abs(wc - w_prev)
        w_prev = wc
        if br == 'frenet' and f.offset is not None:
            if off_prev is not None and abs(f.offset - off_prev) > 1e-6:
                out['offset_changes'] += 1
            off_prev = f.offset
        v += float(np.clip(vc - v, -0.06, 0.03))
        moved = moved or v > 0.05
        yaw += wc * 0.1
        x += v * math.cos(yaw) * 0.1
        y += v * math.sin(yaw) * 0.1
        g = StaticGrid.around(near, x, y, 0.6, 0.01)
        out['clear'] = min(out['clear'], float(FrenetPlanner._static_clear(
            g, np.array([x]), np.array([y]), np.array([yaw]), body)[0]))
        out['state'] = st
        out['time'] = (k + 1) * 0.1
    out['time'] = max(out['time'], 0.1)
    out['w_jerk'] /= out['time']
    return out


def current_params():
    """지금 params.yaml 의 브리지 frenet.* (다음 실험에 쓸 값)."""
    import yaml
    path = os.path.join(HOME, 'ros2_ws', 'src', 'warehouse_robot', 'config', 'params.yaml')
    p = yaml.safe_load(open(path, encoding='utf-8'))['warehouse_bridge']['ros__parameters']
    out = {f'frenet.{k}': v for k, v in p.get('frenet', {}).items()}
    if 'footprint' in p:
        out['frenet.footprint'] = p['footprint']
    if 'footprint_extra' in p:
        out['frenet.footprint_extra'] = p['footprint_extra']
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('records', nargs='*', help='기록 (목표 폴더/파일). 기본: 가장 최근 날짜의 Frenet 기록 전부')
    ap.add_argument('--set', action='append', metavar='이름=값', help='frenet.* 파라미터 바꿔서')
    ap.add_argument('--recorded', action='store_true', help='기록 당시 파라미터로 (기본: 지금 params.yaml)')
    a = ap.parse_args()
    files = [R.resolve(p) for p in a.records] if a.records else None
    if not files:
        allf = [f for f in R.all_records() if '_frenet' in f]
        day = os.path.basename(os.path.dirname(os.path.dirname(allf[-1])))[:8] if allf else ''
        files = [f for f in allf if os.path.basename(os.path.dirname(os.path.dirname(f))).startswith(day)]
    ov = {} if a.recorded else current_params()
    ov.update(R.parse_set(a.set))
    print(f"{'주행':<34} {'결과':<8} {'시간':>6} {'막힘':>6} {'재계획':>5} {'주행중회전':>8} {'w흔들림':>8} {'오프셋바뀜':>8} {'최소여유':>8}")
    tot = {'done': 0, 'n': 0, 'time': 0.0, 'blocked': 0.0, 'mid_turns': 0, 'w_jerk': 0.0, 'offset_changes': 0}
    for f in files:
        rec = R.Record(f)
        r = run(rec, ov)
        if r is None:
            continue
        name = os.path.relpath(os.path.dirname(f), os.path.join(HOME, 'logs', 'records'))[-34:]
        print(f"{name:<34} {r['state']:<8} {r['time']:6.1f} {r['blocked']:6.1f} {r['replans']:5d} {r['mid_turns']:8d} {r['w_jerk']:8.2f} "
              f"{r['offset_changes']:8d} {r['clear']:8.3f}")
        tot['n'] += 1
        tot['done'] += r['state'] == 'done'
        for k in ('time', 'blocked', 'mid_turns', 'w_jerk', 'offset_changes'):
            tot[k] += r[k]
    if tot['n']:
        print(f"{'합계/평균':<34} {tot['done']}/{tot['n']:<6} {tot['time']:6.1f} {tot['blocked']:6.1f} {'':5} {tot['mid_turns']:8d} "
              f"{tot['w_jerk'] / tot['n']:8.2f} {tot['offset_changes']:8d}")


if __name__ == '__main__':
    main()
