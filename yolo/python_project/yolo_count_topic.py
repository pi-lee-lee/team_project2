#!/usr/bin/env python3
"""Publish the running YOLO detector's count text as /yolo/counts."""
import os
from pathlib import Path
import subprocess
import signal
import sys


def main():
    # Bootstrap ROS with system Python; no model or inference environment needed.
    if os.environ.get('_YOLO_COUNT_ROS_READY') != '1':
        result = subprocess.run(
            ['/bin/bash', '-c', 'source /opt/ros/humble/setup.bash && env -0'],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=True)
        env = dict(os.fsdecode(item).split('=', 1)
                   for item in result.stdout.split(b'\0') if b'=' in item)
        env['ROS_DOMAIN_ID'] = os.environ.get('YOLO_DOMAIN_ID', '44')
        env['_YOLO_COUNT_ROS_READY'] = '1'
        env.pop('PYTHONHOME', None)
        os.execve('/usr/bin/python3',
                  ['/usr/bin/python3', str(Path(__file__).resolve()), *sys.argv[1:]], env)

    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from rcl_interfaces.msg import Log
    from std_msgs.msg import String

    class CountTextPublisher(Node):
        def __init__(self):
            super().__init__('yolo_count_topic')
            self.publisher = self.create_publisher(String, '/yolo/counts', 10)
            self.subscription = self.create_subscription(
                Log, '/rosout', self.receive,
                QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT))
            self.get_logger().info(
                'YOLO 개수 대기 → /yolo/counts (std_msgs/msg/String). '
                'yolo_ros.py 또는 yolo_preview.py를 함께 실행하세요.')

        def receive(self, message):
            if message.name not in ('component_detector', 'component_detector_preview'):
                return
            prefix = '현재 개수 | '
            if not message.msg.startswith(prefix):
                return
            output = String()
            output.data = message.msg[len(prefix):]
            self.publisher.publish(output)

    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = CountTextPublisher()
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
