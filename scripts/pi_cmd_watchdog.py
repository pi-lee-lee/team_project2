#!/usr/bin/env python3
"""Command watchdog running on the Pi: stops the robot when communication drops. The real-robot run.sh copies it to ~/warehouse_cmd_watchdog.py on the Pi and starts it.

The PC bridge sends motion commands to cmd_vel_pc; this node forwards them unchanged to cmd_vel (turtlebot3 bringup).
If the last command received was a motion and no new command arrives for more than timeout (default 0.5 s), it sends a stop (0).
(The bridge publishes continuously at 20 Hz while moving and sends a single 0 when stopped, so a silent stopped state is normal.)

Why: the turtlebot3 keeps executing its last cmd_vel. If PC->Pi commands stall over Wi-Fi, the robot keeps driving
and can hit a wall. The PC-side safety stops (front blocked, stale TF) work by sending commands, so they are useless
when the link drops; the robot has to stop itself.
Logs when commands stop and resume (Pi ~/warehouse_cmd_watchdog.log).
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
        # Best effort, keep only the latest, so stale commands arriving late are not replayed
        self.create_subscription(Twist, 'cmd_vel_pc', self.on_cmd,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.last = None          # Time the last command was received (Pi clock)
        self.moving = False       # Whether the last command was a motion
        self.stops_sent = 0       # Stops sent since the drop (send a few, then stay quiet)
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
        if self.stops_sent >= 5:      # 5 stops over 0.25 s is enough (then stay quiet so other tools' cmd_vel isn't overridden)
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
