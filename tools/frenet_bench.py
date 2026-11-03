#!/usr/bin/env python3
"""Closed-loop comparison of the Frenet follower on real drive records (no robot needed).

For each record (logs/records/**/record.jsonl.gz): of the scan points seen during that drive, only the 2 cm cells seen
repeatedly over several cycles (min_hits) are taken as the static environment (people passing by drop out), and the drive is
re-run with the current code's FrenetFollower from the first A* path and start pose
(a robot that follows commands within acceleration limits, no moving objects. Statically blocked for 1 s -> A* replan on that
environment (up to 4 per goal, same as the bridge); blocked for more than 5 s -> ends as 'blocked' (the bridge would back off here)).
People/objects that stood still remain, so it is for before/after comparison of code changes rather than absolute values.

  python3 tools/frenet_bench.py                 all records from today (the latest date)
  python3 tools/frenet_bench.py DIR [DIR ...]   those records
  python3 tools/frenet_bench.py --set frenet.w_change=1.0   with changed parameters
  python3 tools/frenet_bench.py --recorded                  with the parameters at recording time (default: frenet.* from the current params.yaml)
  python3 tools/frenet_bench.py --map ~/maps/map2_clean.yaml DIR   replan A* like the real robot: map + points not in the map (remembered obstacles)
      (default: a grid from accumulated scan points only -> walls build up thick and 0.4 m gaps yield no path. Only when the map frame matches the record)

Metrics: arrival (done)/time, blocked time, in-place turns while driving (excluding before start), angular-velocity jitter
(sum of per-cycle change of commanded w / s), offset change count, minimum clearance (body + attachments, m).
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
    """2 cm cells (one point each) seen in more than min_hits cycles among the scan points (odom) seen during the drive.
    Points within 3 cm of the robot body (+attachments) in that cycle are removed (so the robot's own attachments seen by
    the LiDAR while standing don't build up like a wall)."""
    cfg = R.make_frenet_cfg(rec.params)
    x0, x1, hw = cfg.footprint
    ex = cfg.footprint_extra
    rects = [(x0, x1, -hw, hw)] + [tuple(ex[i:i + 4]) for i in range(0, len(ex) - 3, 4)]
    cells = {}
    for s in rec.steps:
        src = s.get('frenet') or s.get('dwa') or s   # old DWA records keep the scan under 'dwa'
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


def astar_for(env, pts_extra=(), res=0.05, pad=1.0, robot_radius=0.24):
    """A* on an occupancy grid built from environment points (unseen cells are free). Same radii as the bridge/planner settings."""
    from warehouse_robot.astar import GridPlanner
    pts = np.vstack([env] + [np.asarray(pts_extra, dtype=float).reshape(-1, 2)])
    ox, oy = pts.min(axis=0) - pad
    w, h = (np.ceil((pts.max(axis=0) + pad - [ox, oy]) / res)).astype(int)
    grid = np.zeros((h, w), dtype=np.int16)
    j = np.floor((env[:, 0] - ox) / res).astype(int)
    i = np.floor((env[:, 1] - oy) / res).astype(int)
    grid[i, j] = 100
    return GridPlanner(grid.ravel(), w, h, res, ox, oy, robot_radius=robot_radius, inflation_radius=0.55)


MAP = None   # --map: (occupancy grid int16 (bottom row first), resolution, origin x, y) - map frame


def load_map(path):
    """pgm -> -1/0/100 like map_server (negate 0, occupied/free_thresh)"""
    import yaml
    sys.path.insert(0, os.path.join(HOME, 'tools'))
    from clean_map import read_pgm
    conf = yaml.safe_load(open(path, encoding='utf-8'))
    img = read_pgm(os.path.join(os.path.dirname(path), conf['image'].strip('\'"')))[::-1]
    p = (255.0 - img) / 255.0
    grid = np.where(p > conf['occupied_thresh'], 100, np.where(p < conf['free_thresh'], 0, -1)).astype(np.int16)
    return grid, float(conf['resolution']), float(conf['origin'][0]), float(conf['origin'][1])


def replan(env, start, goal, m_o=None):
    """Like the real A* node: robot_radius 0.24, falling back to fallback_radius 0.20 if no path (gaps down to 0.4 m).
    With MAP: plans in the map frame on the map + environment points within 2.5 m of the robot that are more than 0.15 m
    from map walls (= remembered obstacles), and returns the path in odom."""
    from warehouse_robot.astar import GridPlanner
    if MAP is not None and m_o is not None:
        grid0, res, ox, oy = MAP
        tx, ty, th = m_o
        c, sn = math.cos(th), math.sin(th)
        to_map = lambda P: np.c_[c * (P[:, 0] - tx) + sn * (P[:, 1] - ty), -sn * (P[:, 0] - tx) + c * (P[:, 1] - ty)]  # noqa: E731
        env = np.asarray(env, dtype=float)
        env = env[np.hypot(env[:, 0] - start[0], env[:, 1] - start[1]) < 2.5]   # the real memory also keeps only recently seen points (20 s)
        pm = to_map(env)
        occ = np.argwhere(grid0 >= 65)
        from scipy.spatial import cKDTree
        far = cKDTree((occ[:, ::-1] + 0.5) * res + [ox, oy]).query(pm)[0] > 0.15   # A* node scan_wall_margin
        grid = grid0.copy()
        # like the real memory, only 5 cm cells seen persistently: at least 2 environment points per cell (stray single points dropped so the remembered cell count resembles the real node)
        cell, cnt = np.unique(np.floor(pm[far] / 0.05).astype(int), axis=0, return_counts=True)
        mem = (cell[cnt >= 2] + 0.5) * 0.05
        j, i = np.floor((mem[:, 0] - ox) / res).astype(int), np.floor((mem[:, 1] - oy) / res).astype(int)
        ok = (i >= 0) & (j >= 0) & (i < grid.shape[0]) & (j < grid.shape[1])
        grid[i[ok], j[ok]] = 100
        sm, gm = to_map(np.array([start, goal], dtype=float))
        for r in (0.24, 0.20):
            try:
                p = GridPlanner(grid.ravel(), grid.shape[1], grid.shape[0], res, ox, oy, robot_radius=r,
                                inflation_radius=0.55).plan(tuple(sm), tuple(gm))
            except Exception:
                p = None
            if p and len(p) >= 2:
                return R.transform_points(np.asarray(p, dtype=float), *m_o)
        return None
    for r in (0.24, 0.20):
        try:
            p = astar_for(env, [start, goal], robot_radius=r).plan(tuple(start), tuple(goal))
        except Exception:
            p = None
        if p and len(p) >= 2:
            return p
    return None


def run(rec, overrides=None, max_time=120.0):
    """Closed loop in the odom frame. Statically blocked for more than 1 s -> A* replan (up to 4 per goal, same as the bridge); blocked for more than 5 s -> end."""
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
    path = R.transform_points(path, *m_o)                      # to odom (replans are in odom too)
    goal = path[-1]
    goal_yaw = rec.start['goal']['yaw'] + m_o[2]
    f = FrenetFollower(path, goal_yaw, cfg)
    env = environment(rec)
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
            newp = replan(env, (x, y), goal, m_o)
            out['replans'] += 1
            if newp is not None and len(newp) >= 2:
                f = FrenetFollower(newp, goal_yaw, cfg)
                off_prev, blocked_run = None, 0.0
        if blocked_run > 5.0:          # the real bridge backs off here
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
    """The bridge's frenet.* from the current params.yaml (values for the next run)."""
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
    ap.add_argument('--map', help='재계획 A* 에 쓸 지도 yaml (실물 A* 처럼 지도 + 기억 장애물)')
    ap.add_argument('--recorded', action='store_true', help='기록 당시 파라미터로 (기본: 지금 params.yaml)')
    a = ap.parse_args()
    global MAP
    if a.map:
        MAP = load_map(os.path.expanduser(a.map))
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
