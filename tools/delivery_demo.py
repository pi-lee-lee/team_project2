"""(sim) LED 배달 전체 시험: 요청 → LED 위치로 이동 → 싣기(카메라 흉내) → 배달 도중 움직이는 장애물 → 꺼냄 → 홈.

    source scripts/env.sh
    python3 tools/delivery_demo.py --led B --to X Y [--obstacle] [--seconds 400]

- 웹과 같은 요청(start_delivery)을 보낸다. 받는 자리 (X, Y) = 사람이 '여기로 이동' 으로 찍은 자리
- 적재 대기가 되면 tools/cargo_sim.py 로 'B' 가 실린 것처럼, 수령 대기가 되면 '' (꺼냄)
- --obstacle: 배달(수령 위치로 가는) 도중 로봇 앞 경로 위 약 1.2m 지점을 원기둥이 한 번 가로지른다 (tools/sim_mover.py)
끝에 단계별 시간, 원기둥과 최소 여유/충돌, 최종 상태를 출력한다.
"""
import argparse
import json
import math
import os
import subprocess
import sys
import time

import websocket

HERE = os.path.dirname(os.path.abspath(__file__))


def cargo(label):
    subprocess.Popen([sys.executable, os.path.join(HERE, 'cargo_sim.py'), label, '--seconds', '2.5'],
                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def crossing(path, pose, ahead=1.2, half=1.0):
    """경로 위 로봇에서 ahead 만큼 앞 점을 지나 경로에 수직으로 가로지르는 선분 (from, to)."""
    i = min(range(len(path)), key=lambda k: math.hypot(path[k][0] - pose['x'], path[k][1] - pose['y']))
    s = 0.0
    j = i
    while j + 1 < len(path) and s < ahead:
        s += math.hypot(path[j + 1][0] - path[j][0], path[j + 1][1] - path[j][1])
        j += 1
    k = min(j + 3, len(path) - 1)
    dx, dy = path[k][0] - path[max(j - 3, 0)][0], path[k][1] - path[max(j - 3, 0)][1]
    n = math.hypot(dx, dy) or 1.0
    nx, ny = -dy / n, dx / n
    cx, cy = path[j]
    return (cx + nx * half, cy + ny * half), (cx - nx * half, cy - ny * half)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--led', default='B')
    ap.add_argument('--to', type=float, nargs=2, required=True)
    ap.add_argument('--obstacle', action='store_true')
    ap.add_argument('--speed', type=float, default=0.15)
    ap.add_argument('--ahead', type=float, default=1.2, help='원기둥이 가로지를 지점: 로봇 앞 경로 위 거리 (m)')
    ap.add_argument('--seconds', type=float, default=400)
    args = ap.parse_args()
    ws = websocket.create_connection('ws://localhost:8000/ws/ui/', timeout=0.5)
    snap = json.loads(ws.recv())
    loc = next((l for l in snap['locations'] if l['name'] == args.led or l['cargo'] == args.led), None)
    if loc is None:
        raise SystemExit(f'위치 {args.led} 없음: {[l["name"] for l in snap["locations"]]}')
    ws.send(json.dumps({'action': 'start_delivery', 'location_id': loc['id'], 'x': args.to[0], 'y': args.to[1], 'yaw': 0.0}))
    t0 = time.time()
    path, pose, last_state, mover = [], None, None, None
    events = []
    done = set()
    while time.time() - t0 < args.seconds:
        try:
            m = json.loads(ws.recv())
        except websocket.WebSocketTimeoutException:
            continue
        t = time.time() - t0
        if m['type'] == 'error':
            events.append((t, f'오류: {m["message"]}'))
            break
        if m['type'] == 'path':
            pts = m.get('points') or []
            # 배달 경로(끝점이 받는 자리)를 받은 뒤에만 쓴다 (이전 경로로 계산하면 원기둥이 엉뚱한 곳을 지남)
            if pts and math.hypot(pts[-1][0] - args.to[0], pts[-1][1] - args.to[1]) < 0.4:
                path = pts
        elif m['type'] == 'pose' and 'x' in m:
            pose = m
        elif m['type'] == 'log':
            events.append((t, m['log']['message']))
        elif m['type'] == 'state':
            st = m['robot']['state']
            step = next((s for s in (m.get('task') or {}).get('steps', []) if s['status'] not in ('done', 'skipped')), None)
            if st != last_state:
                last_state = st
                events.append((t, f'[{st}] {m["robot"]["message"]}'))
            if st == 'WAITING_LOAD' and 'load' not in done:
                done.add('load')
                time.sleep(2)
                cargo(args.led)
            if st == 'WAITING_UNLOAD' and 'unload' not in done:
                done.add('unload')
                time.sleep(2)
                cargo('')
            if args.obstacle and st == 'MOVING' and step and step['action'] == 'unload' and 'obs' not in done:
                done.add('obs')
                path = []   # 배달 경로를 새로 받을 때까지 기다림
            task = m.get('task')
            if task and task['status'] != 'running' and 'load' in done:
                events.append((t, f'작업 {task["status"]}'))
                break
        if args.obstacle and 'obs' in done and mover is None and path and pose \
                and math.hypot(pose['x'] - path[0][0], pose['y'] - path[0][1]) > 0.5:   # 출발해서 조금 간 뒤
            a, b = crossing(path, pose, ahead=args.ahead)
            events.append((t, f'원기둥 출발: ({a[0]:.2f}, {a[1]:.2f}) → ({b[0]:.2f}, {b[1]:.2f}), {args.speed} m/s'))
            mover = subprocess.Popen([sys.executable, os.path.join(HERE, 'sim_mover.py'), '--from', str(a[0]), str(a[1]),
                                      '--to', str(b[0]), str(b[1]), '--speed', str(args.speed), '--once',
                                      '--seconds', '60'], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
    for t, e in events:
        print(f'[{t:6.1f}s] {e}', flush=True)
    if mover:
        mover.send_signal(2)
        out = mover.communicate(timeout=20)[0]
        print([l for l in out.splitlines() if '최소 여유' in l][-1:] or '원기둥 결과 없음', flush=True)


if __name__ == '__main__':
    main()
