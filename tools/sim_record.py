"""(sim) 주행 기록: 로봇 정답 위치, 움직이는 원기둥 위치, cmd_vel, 추적 결과(웹으로 가는 obstacles)를 10Hz 로 남긴다.

    source scripts/env.sh
    python3 tools/sim_record.py logs/record.csv [--seconds 120] [--name mover]

부딪힘/넘어짐 원인을 볼 때: 충돌 직전 몇 초 동안 로봇이 무엇을 했는지 (명령, 추적이 본 위치/속도).
"""
import argparse
import csv
import json
import math
import signal
import threading
import time

import rclpy
import websocket
from gazebo_msgs.srv import GetEntityState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def yaw_q(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('out')
    ap.add_argument('--seconds', type=float, default=120)
    ap.add_argument('--name', default='mover')
    args = ap.parse_args()
    rclpy.init()
    stop = []
    signal.signal(signal.SIGINT, lambda *_: stop.append(1))
    signal.signal(signal.SIGTERM, lambda *_: stop.append(1))
    n = Node('sim_record')
    d = {'tracks': [], 'state': '', 'msg': ''}
    n.create_subscription(Odometry, 'odom', lambda m: d.__setitem__('odom', m), qos_profile_sensor_data)
    n.create_subscription(Twist, 'cmd_vel', lambda m: d.__setitem__('cmd', m), 10)
    cli = n.create_client(GetEntityState, '/get_entity_state')
    cli.wait_for_service(timeout_sec=5)

    def ui():
        ws = websocket.create_connection('ws://localhost:8000/ws/ui/', timeout=1)
        ws.recv()
        while not stop:
            try:
                m = json.loads(ws.recv())
            except Exception:
                continue
            if m['type'] == 'obstacles':
                d['tracks'] = m['obstacles']
            elif m['type'] == 'state':
                d['state'], d['msg'] = m['robot']['state'], m['robot']['message']
    threading.Thread(target=ui, daemon=True).start()

    t0 = time.time()
    last = 0.0
    fut = None
    mover = (math.nan, math.nan)
    with open(args.out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t', 'x', 'y', 'yaw_deg', 'v_odom', 'cmd_v', 'cmd_w', 'mover_x', 'mover_y',
                    'track_x', 'track_y', 'track_vx', 'track_vy', 'state', 'message'])
        while not stop and time.time() - t0 < args.seconds:
            rclpy.spin_once(n, timeout_sec=0.01)
            if fut is None:
                fut = cli.call_async(GetEntityState.Request(name=args.name))
            elif fut.done():
                r = fut.result()
                fut = None
                if r and r.success:
                    mover = (r.state.pose.position.x, r.state.pose.position.y)
            now = time.time()
            if now - last < 0.1 or 'odom' not in d:
                continue
            last = now
            o = d['odom']
            c = d.get('cmd')
            tr = min(d['tracks'], key=lambda t: math.hypot(t['x'] - mover[0], t['y'] - mover[1]), default=None) \
                if d['tracks'] and not math.isnan(mover[0]) else None
            w.writerow([round(now - t0, 2), round(o.pose.pose.position.x, 3), round(o.pose.pose.position.y, 3),
                        round(math.degrees(yaw_q(o.pose.pose.orientation))), round(o.twist.twist.linear.x, 3),
                        round(c.linear.x, 3) if c else '', round(c.angular.z, 3) if c else '',
                        round(mover[0], 3), round(mover[1], 3),
                        tr and tr['x'], tr and tr['y'], tr and tr['vx'], tr and tr['vy'], d['state'], d['msg']])
    print(f'[sim_record] {args.out}', flush=True)


if __name__ == '__main__':
    main()
