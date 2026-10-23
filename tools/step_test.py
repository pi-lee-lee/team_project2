"""로봇이 실제로 낼 수 있는 가속/감속을 잰다: cmd_vel 로 계단 명령을 주고 odom 속도 변화를 기록.

    source scripts/env.sh
    python3 tools/step_test.py [--v 0.15] [--w 1.0]

주의: 로봇이 직접 움직인다 (앞으로 약 0.5m, 제자리 회전). 주변이 비어 있을 때, 작업이 없을 때만 실행.
결과: 직진 가속/감속 (m/s², 10%→90% 구간), 정지 거리, 회전 가속/감속 (rad/s²), 명령 대비 실제 최고 속도
→ 로컬 플래너의 가/감속 한계(frenet.max_accel 등)를 정할 때 사용.
"""
import argparse
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class StepTest(Node):
    def __init__(self):
        super().__init__('step_test')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self.log = []   # (시각, v, w, x, y)
        self.create_subscription(Odometry, 'odom', self.on_odom, qos_profile_sensor_data)

    def on_odom(self, m):
        self.log.append((time.time(), m.twist.twist.linear.x, m.twist.twist.angular.z,
                         m.pose.pose.position.x, m.pose.pose.position.y))

    def hold(self, v, w, seconds):
        end = time.time() + seconds
        while time.time() < end:
            t = Twist()
            t.linear.x, t.angular.z = float(v), float(w)
            self.pub.publish(t)
            rclpy.spin_once(self, timeout_sec=0.02)


def rise(log, t0, idx, target):
    """t0 이후 속도가 target 의 10%→90% (또는 90%→10%) 로 가는 데 걸린 시간 → 가속도."""
    seg = [(t, abs(r[idx])) for r in log if (t := r[0]) >= t0]
    tgt = abs(target)
    if tgt < 1e-6:   # 감속: 시작 속도 기준
        start = seg[0][1] if seg else 0.0
        hi, lo = 0.9 * start, 0.1 * start
        t_hi = next((t for t, s in seg if s <= hi), None)
        t_lo = next((t for t, s in seg if s <= lo), None)
        span = 0.8 * start
    else:
        t_hi = next((t for t, s in seg if s >= 0.1 * tgt), None)
        t_lo = next((t for t, s in seg if s >= 0.9 * tgt), None)
        span = 0.8 * tgt
    if t_hi is None or t_lo is None or t_lo <= t_hi:
        return None
    return span / (t_lo - t_hi)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--v', type=float, default=0.15)
    ap.add_argument('--w', type=float, default=1.0)
    args = ap.parse_args()
    rclpy.init()
    n = StepTest()
    n.hold(0, 0, 1.0)
    # 직진: 출발 → 유지 → 정지
    t_go = time.time()
    n.hold(args.v, 0, 3.0)
    vmax = max((r[1] for r in n.log if r[0] >= t_go), default=0.0)
    t_stop = time.time()
    p_stop = n.log[-1][3:5]
    n.hold(0, 0, 2.0)
    p_end = n.log[-1][3:5]
    acc_v = rise(n.log, t_go, 1, args.v)
    dec_v = rise(n.log, t_stop, 1, 0.0)
    stop_d = float(np.hypot(p_end[0] - p_stop[0], p_end[1] - p_stop[1]))
    # 회전
    t_turn = time.time()
    n.hold(0, args.w, 3.0)
    wmax = max((abs(r[2]) for r in n.log if r[0] >= t_turn), default=0.0)
    t_tstop = time.time()
    n.hold(0, 0, 2.0)
    acc_w = rise(n.log, t_turn, 2, args.w)
    dec_w = rise(n.log, t_tstop, 2, 0.0)
    fmt = lambda a: '-' if a is None else f'{a:.2f}'  # noqa: E731
    print(f'[step_test] 직진 명령 {args.v} m/s → 실제 최고 {vmax:.3f} m/s, 가속 {fmt(acc_v)} m/s², '
          f'감속 {fmt(dec_v)} m/s², 정지 거리 {stop_d * 100:.1f} cm', flush=True)
    print(f'[step_test] 회전 명령 {args.w} rad/s → 실제 최고 {wmax:.2f} rad/s, 가속 {fmt(acc_w)} rad/s², '
          f'감속 {fmt(dec_w)} rad/s²', flush=True)
    n.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
