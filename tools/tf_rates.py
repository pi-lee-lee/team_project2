"""Measure the publish rate and lag (now - stamp) of each transform arriving on /tf.

    source scripts/env.sh
    python3 tools/tf_rates.py [seconds]
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from tf2_msgs.msg import TFMessage


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0
    rclpy.init()
    node = Node('tf_rates')
    stats = {}

    def on_tf(msg):
        now = node.get_clock().now().nanoseconds / 1e9
        for t in msg.transforms:
            key = f'{t.header.frame_id} -> {t.child_frame_id}'
            stamp = t.header.stamp.sec + t.header.stamp.nanosec / 1e9
            count, _ = stats.get(key, (0, 0.0))
            stats[key] = (count + 1, now - stamp)

    node.create_subscription(TFMessage, 'tf', on_tf, QoSProfile(depth=100))
    end = time.time() + duration
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    for key, (count, lag) in sorted(stats.items()):
        print(f'{key}: {count / duration:.1f} Hz, 지연 {lag:+.3f} s')
    if not stats:
        print('/tf 메시지를 받지 못했습니다.')
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
