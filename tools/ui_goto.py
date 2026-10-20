"""웹 UI와 같은 방식으로 '여기로 이동'을 보내고 진행 상황을 출력한다.

    python3 tools/ui_goto.py X Y [YAW_RAD] [--timeout 60] [--server ws://localhost:8000/ws/ui/]
    python3 tools/ui_goto.py --home        # "홈 복귀" 버튼과 같음

시간 안에 끝나지 않으면 비상정지를 보낸다.
"""
import argparse
import json
import time

import websocket


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('x', type=float, nargs='?')
    ap.add_argument('y', type=float, nargs='?')
    ap.add_argument('yaw', type=float, nargs='?', default=0.0)
    ap.add_argument('--timeout', type=float, default=60.0)
    ap.add_argument('--server', default='ws://localhost:8000/ws/ui/')
    ap.add_argument('--home', action='store_true', help='홈 복귀')
    args = ap.parse_args()
    if not args.home and (args.x is None or args.y is None):
        ap.error('X Y 또는 --home 을 지정하세요')

    ws = websocket.create_connection(args.server, timeout=0.5)
    snapshot = json.loads(ws.recv())
    if args.home:
        home = next((l for l in snapshot['locations'] if l['kind'] == 'home'), None)
        if home is None:
            raise SystemExit('홈 위치가 등록되어 있지 않습니다.')
        args.x, args.y = home['x'], home['y']
        ws.send(json.dumps({'action': 'go_home'}))
    else:
        ws.send(json.dumps({'action': 'goto_point', 'x': args.x, 'y': args.y, 'yaw': args.yaw}))
    t0 = time.time()
    pose = state = None
    last_print = -1
    while time.time() - t0 < args.timeout:
        try:
            m = json.loads(ws.recv())
        except websocket.WebSocketTimeoutException:
            continue
        elapsed = time.time() - t0
        if m['type'] == 'pose':
            pose = m
        elif m['type'] == 'log':
            print(f'[{elapsed:5.1f}s] {m["log"]["message"]}', flush=True)
        elif m['type'] == 'error':
            print(f'[{elapsed:5.1f}s] 오류: {m["message"]}')
            break
        elif m['type'] == 'state':
            state = m['robot']['state']
            if m['robot']['message']:
                print(f'[{elapsed:5.1f}s] 상태 {state}: {m["robot"]["message"]}', flush=True)
        if pose and 'x' in pose and int(elapsed) != last_print:
            last_print = int(elapsed)
            print(f'[{elapsed:5.1f}s] 위치 ({pose["x"]:.2f}, {pose["y"]:.2f})', flush=True)
        if state in ('IDLE', 'ERROR') and elapsed > 1:
            break
    else:
        ws.send(json.dumps({'action': 'estop'}))
        print('시간 초과 → 비상정지')
    if pose:
        print(f'최종 상태 {state}, 위치 ({pose["x"]:.2f}, {pose["y"]:.2f}), '
              f'목표까지 {((pose["x"] - args.x) ** 2 + (pose["y"] - args.y) ** 2) ** 0.5:.2f} m')


if __name__ == '__main__':
    main()
