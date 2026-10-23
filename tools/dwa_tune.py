"""DWA 비용 계수 찾기: 시나리오 묶음을 폐루프로 돌려 점수를 매기고, 계수를 무작위로 바꿔 가며 더 좋은 조합을 찾는다.

    python3 tools/dwa_tune.py [--trials 120] [--seed 1] [--workers 10]
    python3 tools/dwa_tune.py --eval        # 지금 params.yaml 값만 평가 (시나리오별 결과 출력)

ROS 없이 돈다 (warehouse_robot.dwa 만 사용). 장애물 정보는 정답(위치/속도)을 쓴다 — 추적 오차는 sim 에서 따로 확인.
속도/가속/외곽/정적 여유 같은 '물리' 값은 고정하고, 비용 계수(w_*), obstacle_influence,
움직이는 장애물 여유(dynamic_margin/growth), 예측 시간(predict_time)을 찾는다.

시나리오 (로봇은 직사각형 외곽, 장애물과의 최소 여유를 직사각형 기준으로 잰다):
  cross_*   : 경로를 가로지르는 원기둥 (출발 시각/속도 여러 가지)
  headon    : 경로를 따라 마주 오는 원기둥 (넓은 공간에서 비켜야 함)
  detour    : A* 가 돌아가는 경로를 준 정적 상자
  corridor  : 폭 0.62m 통로 (벽 점)
  parked    : 뒤 벽에서 3.8cm 에서 출발
점수: 충돌 -100, 도착 +20, 시간 -0.3/초, 최소 여유(0.15m 까지) +40/m
"""
import argparse
import json
import math
import os
import random
import sys
import time
from concurrent.futures import ProcessPoolExecutor

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'ros2_ws', 'src', 'warehouse_robot'))
from warehouse_robot.dwa import DWAConfig, DWAFollower  # noqa: E402

FOOT = (-0.197, 0.069, 0.153)
WEIGHTS = ['w_heading', 'w_path', 'w_goal', 'w_obstacle', 'w_speed', 'w_collision', 'obstacle_influence',
           'dynamic_margin', 'dynamic_growth', 'predict_time']
RANGES = {'w_heading': (0.0, 1.5), 'w_path': (0.2, 3.0), 'w_goal': (0.2, 3.0), 'w_obstacle': (0.0, 3.0),
          'w_speed': (0.0, 1.5), 'w_collision': (0.5, 10.0), 'obstacle_influence': (0.2, 0.8),
          # 움직이는 장애물 여유: 크면 마주 올 때 옆으로 비킬 수 없음, 작으면 가깝게 스침
          'dynamic_margin': (0.05, 0.3), 'dynamic_growth': (0.0, 0.1), 'predict_time': (2.0, 3.5)}


def load_physical():
    """params.yaml 의 warehouse_bridge.dwa 값 (물리 값 + 지금 계수)."""
    import yaml
    path = os.path.join(HERE, '..', 'ros2_ws', 'src', 'warehouse_robot', 'config', 'params.yaml')
    p = yaml.safe_load(open(path))['warehouse_bridge']['ros__parameters']['dwa']
    p = {k: tuple(v) if isinstance(v, list) else v for k, v in p.items()}
    p['control_period'] = 0.1
    return p


def rect_gap(x, y, yaw, cx, cy, r):
    c, s = math.cos(-yaw), math.sin(-yaw)
    lx, ly = c * (cx - x) - s * (cy - y), s * (cx - x) + c * (cy - y)
    qx, qy = min(max(lx, FOOT[0]), FOOT[1]), min(max(ly, -FOOT[2]), FOOT[2])
    return math.hypot(lx - qx, ly - qy) - r


def scenarios():
    out = []
    straight = [[x, 0.0] for x in np.linspace(0.0, 2.0, 41)]
    for delay in (0.5, 1.5, 2.5, 3.5):
        for speed in (0.1, 0.2):
            # x=1.0 에서 위(y=0.6)에서 아래로 가로지름
            out.append(dict(name=f'cross_{delay}s_{speed}', path=straight, goal=(2.0, 0.0), walls=[],
                            mover=dict(x0=1.0, y0=0.6, vx=0.0, vy=-speed, start=delay, r=0.12), limit=40))
    # 돌아오는 길(서쪽으로)에 위에서 내려오는 원기둥: sim 에서 비켜서기 판정이 잘못되어 부딪혔던 상황
    back_path = [[x, 0.0] for x in np.linspace(2.0, 0.0, 41)]
    for delay in (0.5, 2.0, 3.5, 5.0):
        out.append(dict(name=f'cross_back_{delay}s', path=back_path, goal=(0.0, 0.0), walls=[], start=(2.0, 0.0, math.pi),
                        mover=dict(x0=1.0, y0=0.6, vx=0.0, vy=-0.1, start=delay, r=0.12), limit=50))
    out.append(dict(name='headon', path=straight, goal=(2.0, 0.0), walls=[],
                    mover=dict(x0=2.2, y0=0.0, vx=-0.1, vy=0.0, start=0.0, r=0.12), limit=60))
    box = [[x, y] for x in np.linspace(0.9, 1.1, 5) for y in np.linspace(-0.1, 0.1, 5)]
    corners = np.array([[0, 0], [0.6, 0.45], [1.4, 0.45], [2.0, 0]])
    detour = np.vstack([np.linspace(a, b, 15, endpoint=False) for a, b in zip(corners, corners[1:])]
                       + [corners[-1:]]).tolist()
    out.append(dict(name='detour', path=detour, goal=(2.0, 0.0), walls=box, mover=None, limit=40))
    walls = [[x, yy] for x in np.arange(-0.5, 2.5, 0.03) for yy in (-0.31, 0.31)]
    out.append(dict(name='corridor', path=straight, goal=(2.0, 0.0), walls=walls, mover=None, limit=40))
    back = [[-0.235, yy] for yy in np.arange(-1.0, 1.0, 0.02)]
    out.append(dict(name='parked', path=[[x, 0.0] for x in np.linspace(0.0, 1.5, 31)], goal=(1.5, 0.0),
                    walls=back, mover=None, limit=30))
    return out


def run_scenario(cfg, sc, dt=0.1):
    f = DWAFollower(sc['path'], 0.0, DWAConfig(**cfg))
    x, y, yaw = sc.get('start', (0.0, 0.0, 0.0))
    v = w = 0.0
    walls = np.array(sc['walls'], dtype=float).reshape(-1, 2)
    m = sc['mover']
    min_gap = math.inf
    t = 0.0
    while t < sc['limit']:
        dyn = []
        if m:
            k = max(0.0, t - m['start'])
            mx, my = m['x0'] + m['vx'] * k, m['y0'] + m['vy'] * k
            moving = t >= m['start']
            dyn = [[mx, my, m['vx'] * moving, m['vy'] * moving, m['r']]]
        v, w, state = f.compute((x, y, yaw), v, w, (0, 0, 0), walls, dyn)
        if state == 'done':
            break
        yaw += w * dt
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        t += dt
        if m:
            min_gap = min(min_gap, rect_gap(x, y, yaw, *dyn[0][:2], m['r']))
        if len(walls):
            min_gap = min(min_gap, min(rect_gap(x, y, yaw, px, py, 0.0) for px, py in walls[::3]))
    reached = math.hypot(x - sc['goal'][0], y - sc['goal'][1]) < 0.15
    return dict(name=sc['name'], reached=reached, time=round(t, 1), gap=round(min_gap, 3))


def score(results):
    s = 0.0
    for r in results:
        if r['gap'] < 0:
            s -= 100
        if r['reached']:
            s += 20
        s -= 0.3 * r['time']
        s += 40 * min(max(r['gap'], 0.0), 0.15) if math.isfinite(r['gap']) else 40 * 0.15
    return round(s, 1)


def evaluate(cfg):
    res = [run_scenario(cfg, sc) for sc in scenarios()]
    return score(res), res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--trials', type=int, default=120)
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--workers', type=int, default=max(1, (os.cpu_count() or 2) - 2))
    ap.add_argument('--eval', action='store_true')
    args = ap.parse_args()
    base = load_physical()
    if args.eval:
        s, res = evaluate(base)
        for r in res:
            print(r)
        print('점수', s)
        return
    rng = random.Random(args.seed)
    cands = [dict(base)]
    for _ in range(args.trials):
        c = dict(base)
        for k in WEIGHTS:
            lo, hi = RANGES[k]
            c[k] = round(rng.uniform(lo, hi), 2)
        cands.append(c)
    t0 = time.time()
    with ProcessPoolExecutor(args.workers) as ex:
        out = list(ex.map(evaluate, cands))
    ranked = sorted(zip(out, cands), key=lambda z: -z[0][0])
    print(f'[dwa_tune] {len(cands)}개 조합, {time.time() - t0:.0f}초')
    bs, br = out[0]
    print(f'지금 값 점수 {bs}: 충돌 {sum(r["gap"] < 0 for r in br)}, 도착 {sum(r["reached"] for r in br)}/{len(br)}')
    for (s, res), c in ranked[:5]:
        print(f'점수 {s}: 충돌 {sum(r["gap"] < 0 for r in res)}, 도착 {sum(r["reached"] for r in res)}/{len(res)}, '
              f'시간 합 {sum(r["time"] for r in res):.0f}초 | ' + ', '.join(f'{k}={c[k]}' for k in WEIGHTS))
    with open(os.path.join(HERE, '..', 'logs', 'dwa_tune.json'), 'w') as f:
        json.dump([{'score': s, 'cfg': {k: c[k] for k in WEIGHTS}, 'results': res}
                   for (s, res), c in ranked[:20]], f, ensure_ascii=False, indent=1)


if __name__ == '__main__':
    main()
