"""SLAM 위치 보정(map->odom) 기록. SLAM 지도가 틀어지는지 수치로 확인한다.

    source scripts/env.sh
    python3 tools/slam_drift.py [--period 1.0] [--seconds 0]

- 시뮬레이션: Gazebo 오도메트리는 실제 위치 그대로(diff drive 기본값 odometry_source=world)이므로
  map->odom 은 0 이어야 정상이고, 0 에서 벗어난 만큼이 곧 SLAM 오차다 (월드에 플러그인이 없어도 됨).
- 실물: 오도메트리 오차를 바로잡는 정상 보정도 섞이지만, 한 번에 크게 튀면(--jump) 잘못된 스캔 맞춤/루프 클로저.
끝날 때(Ctrl-C 또는 --seconds) 최대 이동/회전과 큰 튐 횟수를 요약한다.
"""
import argparse
import math
import os
import signal
import time

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--period', type=float, default=1.0)
    ap.add_argument('--seconds', type=float, default=0.0, help='0 이면 Ctrl-C 까지')
    ap.add_argument('--jump', type=float, default=0.1, help='한 주기에 이보다 크게(m) 바뀌면 튐으로 셈')
    args = ap.parse_args()
    rclpy.init()
    node = Node('slam_drift', parameter_overrides=[
        rclpy.parameter.Parameter('use_sim_time', value=os.environ.get('WAREHOUSE_SIM_TIME') == 'true')])
    buf = Buffer()
    TransformListener(buf, node)
    stop = []
    signal.signal(signal.SIGINT, lambda *_: stop.append(1))
    signal.signal(signal.SIGTERM, lambda *_: stop.append(1))
    start = time.time()
    worst_d = worst_a = 0.0
    jumps = 0
    last = None
    while not stop and (args.seconds <= 0 or time.time() - start < args.seconds):
        end = time.time() + args.period
        while time.time() < end and not stop:
            rclpy.spin_once(node, timeout_sec=0.05)
        try:
            t = buf.lookup_transform('map', 'odom', rclpy.time.Time()).transform
        except Exception:
            continue
        q = t.rotation
        yaw = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        d = math.hypot(t.translation.x, t.translation.y)
        worst_d, worst_a = max(worst_d, d), max(worst_a, abs(yaw))
        if last is not None and math.hypot(t.translation.x - last[0], t.translation.y - last[1]) > args.jump:
            jumps += 1
            print(f'[slam_drift] 튐: {math.hypot(t.translation.x - last[0], t.translation.y - last[1]):.2f}m', flush=True)
        last = (t.translation.x, t.translation.y)
        print(f'[slam_drift] {time.time() - start:6.0f}s map->odom ({t.translation.x:+.3f}, {t.translation.y:+.3f}) '
              f'{yaw:+.2f}°', flush=True)
    print(f'[slam_drift] 끝: 최대 이동 {worst_d:.3f}m, 최대 회전 {worst_a:.2f}°, 큰 튐 {jumps}번', flush=True)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
