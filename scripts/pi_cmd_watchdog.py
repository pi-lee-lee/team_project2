#!/usr/bin/env python3
"""Pi 위에서 도는 명령 감시: 통신이 끊기면 로봇을 세운다. 실물 run.sh 가 Pi 의 ~/warehouse_cmd_watchdog.py 로 복사해서 띄움.

PC 의 브리지는 이동 명령을 cmd_vel_pc 로 보내고, 이 노드가 받아서 그대로 cmd_vel(turtlebot3 bringup)로 넘긴다.
마지막으로 받은 명령이 '움직임' 인데 timeout(기본 0.5초) 넘게 새 명령이 안 오면 정지(0)를 보낸다.
(브리지는 움직이는 동안 20Hz 로 계속 보내고, 멈춰 있을 때는 0 을 한 번 보내고 조용함 → 조용한 정지 상태는 정상)

왜: turtlebot3 는 마지막 cmd_vel 을 계속 실행한다. PC→Pi 명령이 Wi-Fi 에서 9초 동안 안 들어가 도착 후에도
0.12~0.18 m/s 로 가서 벽에 부딪혔음 (실물 2026-10-26, 기록 160957). PC 쪽 안전장치(앞 막힘 정지, TF 지연 정지)는
명령으로 세우는 것이라 통신이 끊기면 소용이 없어서, 로봇 쪽에서 세운다.
끊길 때/다시 받을 때 로그를 남긴다 (Pi ~/warehouse_cmd_watchdog.log).
"""
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


class CmdWatchdog(Node):
    def __init__(self):
        super().__init__('cmd_watchdog')
        self.timeout = float(self.declare_parameter('timeout', 0.5).value)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        # 늦게 도착한 옛 명령을 다시 받지 않게 best effort, 마지막 하나만
        self.create_subscription(Twist, 'cmd_vel_pc', self.on_cmd,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.last = None          # 마지막 명령을 받은 시각 (Pi 시계)
        self.moving = False       # 마지막 명령이 움직임인가
        self.stops_sent = 0       # 끊긴 뒤 보낸 정지 수 (몇 번만 보내고 조용히)
        self.trips = 0
        self.create_timer(0.05, self.check)
        self.get_logger().info(f'명령 감시 시작: cmd_vel_pc → cmd_vel, {self.timeout:.2f}초 끊기면 정지')

    def on_cmd(self, msg):
        now = time.monotonic()
        if self.stops_sent and self.last is not None:
            self.get_logger().warn(f'명령 다시 받음 ({now - self.last:.2f}초 끊겼음)')
        self.last = now
        self.stops_sent = 0
        self.moving = abs(msg.linear.x) > 1e-3 or abs(msg.angular.z) > 1e-3
        self.pub.publish(msg)

    def check(self):
        if self.last is None or not self.moving:
            return
        gap = time.monotonic() - self.last
        if gap <= self.timeout:
            return
        self.pub.publish(Twist())
        self.stops_sent += 1
        if self.stops_sent == 1:
            self.trips += 1
            self.get_logger().warn(f'통신 끊김 정지 #{self.trips}: {gap:.2f}초 동안 명령 없음')
        if self.stops_sent >= 5:      # 0.25초 동안 5번 보내면 충분 (그다음엔 다른 도구의 cmd_vel 을 막지 않게 조용히)
            self.moving = False


def main():
    rclpy.init()
    node = CmdWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
