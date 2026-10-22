#!/usr/bin/env python3
"""Show a centered digital zoom without changing the source image topic."""
import math

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Image


class ZoomCamera(Node):
    def __init__(self):
        super().__init__('zoom_camera')
        self.declare_parameter('zoom', 1.5)
        self.declare_parameter('input_topic', '/csi/image_raw')
        if not self.valid_zoom(self.get_parameter('zoom').value):
            raise ValueError('zoom must be between 1.0 and 8.0')
        self.add_on_set_parameters_callback(self.check_parameters)
        self.publisher = self.create_publisher(Image, '/camera/zoom', qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            Image, self.get_parameter('input_topic').value,
            self.callback, qos_profile_sensor_data)
        self.get_logger().info('확대 영상: /camera/zoom')

    @staticmethod
    def valid_zoom(value):
        return isinstance(value, (int, float)) and math.isfinite(value) and 1 <= value <= 8

    def check_parameters(self, parameters):
        for parameter in parameters:
            if parameter.name == 'zoom' and not self.valid_zoom(parameter.value):
                return SetParametersResult(successful=False, reason='zoom: 1.0~8.0')
        return SetParametersResult(successful=True)

    def callback(self, message):
        if message.encoding not in ('rgb8', 'bgr8'):
            self.get_logger().error(f'지원하지 않는 인코딩: {message.encoding}')
            return
        image = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)
        image = image[:, :message.width * 3].reshape(message.height, message.width, 3)
        factor = float(self.get_parameter('zoom').value)
        width = max(1, round(message.width / factor))
        height = max(1, round(message.height / factor))
        x, y = (message.width - width) // 2, (message.height - height) // 2
        enlarged = cv2.resize(image[y:y + height, x:x + width],
                              (message.width, message.height), interpolation=cv2.INTER_LINEAR)
        output = Image()
        output.header = message.header
        output.width, output.height = message.width, message.height
        output.encoding = message.encoding
        output.step = message.width * 3
        output.is_bigendian = 0
        output.data = enlarged.tobytes()
        if rclpy.ok():
            self.publisher.publish(output)


def main():
    rclpy.init()
    node = None
    try:
        node = ZoomCamera()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
