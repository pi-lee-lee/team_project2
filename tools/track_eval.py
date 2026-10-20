"""(sim) 추적 정확도: 브리지가 웹으로 보내는 추적 결과(obstacles)를 Gazebo 정답(움직이는 원기둥 위치)과 비교.

    source scripts/env.sh
    python3 tools/track_eval.py [--name mover] [--seconds 120]

tools/sim_mover.py 로 원기둥을 움직이는 동안 실행. 끝(Ctrl-C 또는 시간)에 요약:
- 검출률: 추적 메시지 중 원기둥 0.3m 안에 track 이 있던 비율
- 위치 오차, 속도 오차 (평균 / 95%), 추정 반경
- track 번호 바뀜 (같은 물체를 놓쳤다가 새로 잡은 횟수)
"""
import argparse
import json
import math
import signal
import time

import numpy as np
import rclpy
import websocket
from gazebo_msgs.srv import GetEntityState
from rclpy.node import Node


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--name', default='mover')
    ap.add_argument('--seconds', type=float, default=120)
    ap.add_argument('--server', default='ws://localhost:8000/ws/ui/')
    args = ap.parse_args()
    rclpy.init()
    stop = []
    signal.signal(signal.SIGINT, lambda *_: stop.append(1))
    signal.signal(signal.SIGTERM, lambda *_: stop.append(1))
    node = Node('track_eval')
    cli = node.create_client(GetEntityState, '/get_entity_state')
    if not cli.wait_for_service(timeout_sec=5):
        raise SystemExit('/get_entity_state 없음 (월드에 gazebo_ros_state 플러그인 필요)')
    ws = websocket.create_connection(args.server, timeout=0.2)
    ws.recv()

    truth = []          # (시각, x, y)
    pos_err, vel_err, radii = [], [], []
    msgs = hits = 0
    ids, switches, last_id = set(), 0, None
    fut = None
    t0 = time.time()
    while not stop and time.time() - t0 < args.seconds:
        rclpy.spin_once(node, timeout_sec=0.01)
        if fut is None:
            fut = cli.call_async(GetEntityState.Request(name=args.name))
        elif fut.done():
            r = fut.result()
            fut = None
            if r and r.success:
                truth.append((time.time(), r.state.pose.position.x, r.state.pose.position.y))
                truth = truth[-50:]
        try:
            m = json.loads(ws.recv())
        except (websocket.WebSocketTimeoutException, ValueError):
            continue
        if m.get('type') != 'obstacles' or len(truth) < 5:
            continue
        msgs += 1
        _, tx, ty = truth[-1]
        # 정답 속도: 0.5초 전 위치와 비교
        old = next((p for p in truth if truth[-1][0] - p[0] <= 0.5), truth[0])
        dt = max(truth[-1][0] - old[0], 1e-3)
        tvx, tvy = (tx - old[1]) / dt, (ty - old[2]) / dt
        near = [o for o in m['obstacles'] if math.hypot(o['x'] - tx, o['y'] - ty) < 0.3]
        if not near:
            continue
        o = min(near, key=lambda o: math.hypot(o['x'] - tx, o['y'] - ty))
        hits += 1
        pos_err.append(math.hypot(o['x'] - tx, o['y'] - ty))
        vel_err.append(math.hypot(o['vx'] - tvx, o['vy'] - tvy))
        radii.append(o['r'])
        if last_id is not None and o['id'] != last_id:
            switches += 1
        last_id = o['id']
        ids.add(o['id'])
    if not msgs:
        print('[track_eval] 추적 메시지가 없습니다 (브리지 tracking.enabled, 원기둥 이름 확인)')
        return
    pe, ve = np.array(pos_err), np.array(vel_err)
    print(f'[track_eval] 검출률 {hits}/{msgs} ({100 * hits / msgs:.0f}%), track 번호 바뀜 {switches}회 (번호 {len(ids)}개)', flush=True)
    if hits:
        print(f'[track_eval] 위치 오차 평균 {pe.mean() * 100:.1f}cm / 95% {np.percentile(pe, 95) * 100:.1f}cm, '
              f'속도 오차 평균 {ve.mean():.3f} / 95% {np.percentile(ve, 95):.3f} m/s, 추정 반경 평균 {np.mean(radii):.3f} m',
              flush=True)


if __name__ == '__main__':
    main()
