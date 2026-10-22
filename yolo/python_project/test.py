"""Capture CSI images from /csi/image_raw on domain 44 by default."""
import argparse
from datetime import datetime
import math
import os
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class ImageCapture(Node):
    def __init__(self, args):
        super().__init__('image_capture_zoom' if args.topic == '/camera/zoom' else 'image_capture')
        self.output_dir = Path(args.output).expanduser()
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.interval = args.interval
        self.limit = args.count
        self.saved = 0
        self.last_saved = None
        self.last_received = time.monotonic()
        self.subscription = self.create_subscription(
            Image, args.topic, self.callback, qos_profile_sensor_data)
        self.timer = self.create_timer(5.0, self.check_input)
        self.get_logger().info(
            f'도메인 {args.domain} | 토픽 {args.topic} | 저장 위치 {self.output_dir}')
        self.get_logger().info(f'저장 간격 {args.interval}초 | 종료: Ctrl+C')

    def check_input(self):
        if time.monotonic() - self.last_received > 5:
            self.get_logger().warning('영상 수신 대기 중: 카메라 실행 및 토픽/도메인을 확인하세요.')

    def callback(self, msg):
        now = time.monotonic()
        self.last_received = now
        if self.limit and self.saved >= self.limit:
            return
        if self.last_saved is not None and now - self.last_saved < self.interval:
            return
        try:
            if msg.encoding not in ('rgb8', 'bgr8', 'mono8'):
                raise ValueError(f'지원하지 않는 인코딩: {msg.encoding}')
            channels = 1 if msg.encoding == 'mono8' else 3
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
            frame = frame[:, :msg.width * channels]
            frame = frame.reshape(msg.height, msg.width, channels)
            if msg.encoding == 'rgb8':
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            filename = datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '.jpg'
            path = self.output_dir / filename
            if not cv2.imwrite(str(path), frame, [cv2.IMWRITE_JPEG_QUALITY, 95]):
                raise RuntimeError(f'파일 저장 실패: {path}')
            self.saved += 1
            self.last_saved = now
            self.get_logger().info(f'저장 #{self.saved}: {path}')
        except Exception as error:
            self.get_logger().error(str(error))


def main(default_topic='/csi/image_raw', default_output=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain', type=int, default=44)
    parser.add_argument('--topic', default=default_topic)
    parser.add_argument('--output', default=str(default_output or Path.home() / 'camera_dataset/images'))
    parser.add_argument('--interval', type=float, default=1.0)
    parser.add_argument('--count', type=int, default=0, help='저장 장수, 0이면 계속 저장')
    args = parser.parse_args()
    if not math.isfinite(args.interval) or args.interval < 0 or args.count < 0:
        parser.error('interval/count는 0 이상이어야 합니다.')
    if not 0 <= args.domain <= 232:
        parser.error('domain은 0~232 사이여야 합니다.')
    # Explicitly override inherited ROS_DOMAIN_ID=12 for this camera workflow.
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    rclpy.init(args=[])
    node = None
    try:
        node = ImageCapture(args)
        while rclpy.ok() and (not args.count or node.saved < args.count):
            rclpy.spin_once(node, timeout_sec=0.5)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
