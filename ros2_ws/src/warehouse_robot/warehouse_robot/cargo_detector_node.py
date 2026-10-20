"""카메라 영상에서 LED(물건)를 색으로 찾아 cargo/detected 로 발행하는 임시 인식 노드.

구독: image_topic (sensor_msgs/CompressedImage 또는 Image, compressed 파라미터)
발행: cargo/detected (std_msgs/String) - 보이는 라벨, 없으면 ''
      cargo/debug (sensor_msgs/CompressedImage) - publish_debug 일 때 인식 결과를 그린 영상

학습 모델(YOLO 등)로 바꿀 때는 이 노드 대신 같은 토픽에 라벨을 내는 노드를 띄우면 된다.
브리지가 1초 동안 같은 값일 때만 서버에 알린다 (cargo_stable_sec).
"""
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String

from .cargo_detect import detect, parse_ranges


class CargoDetectorNode(Node):
    def __init__(self):
        super().__init__('cargo_detector')
        self.declare_parameter('image_topic', 'camera/image_raw/compressed')
        self.declare_parameter('compressed', True)
        self.declare_parameter('rate', 5.0)              # 초당 처리 횟수 (Pi 부하를 줄이려고 프레임을 건너뜀)
        self.declare_parameter('labels', ['A', 'B', 'C'])
        # 라벨별 HSV 범위 [h_min, s_min, v_min, h_max, s_max, v_max]. 기본: A 빨강, B 초록, C 파랑 (켜진 LED)
        self.declare_parameter('hsv_A', [170, 120, 180, 10, 255, 255])
        self.declare_parameter('hsv_B', [45, 120, 180, 85, 255, 255])
        self.declare_parameter('hsv_C', [100, 120, 180, 130, 255, 255])
        self.declare_parameter('min_area_ratio', 0.002)  # 영상 면적 대비 이보다 작은 덩어리는 무시
        self.declare_parameter('roi', [0.0, 0.0, 1.0, 1.0])  # 적재 칸만 볼 때 (x0, y0, x1, y1 비율)
        self.declare_parameter('publish_debug', False)

        p = lambda n: self.get_parameter(n).value  # noqa: E731
        labels = list(p('labels'))
        for label in labels:
            if not self.has_parameter(f'hsv_{label}'):
                self.declare_parameter(f'hsv_{label}', [0, 0, 0, 0, 0, 0])
        self.ranges = parse_ranges(labels, [list(p(f'hsv_{l}')) for l in labels])
        self.period = 1.0 / p('rate')
        self.last = 0.0
        self.pub = self.create_publisher(String, 'cargo/detected', 10)
        self.debug_pub = self.create_publisher(CompressedImage, 'cargo/debug', 1) if p('publish_debug') else None
        msg_type = CompressedImage if p('compressed') else Image
        self.create_subscription(msg_type, p('image_topic'), self.on_image, qos_profile_sensor_data)
        self.get_logger().info(f'LED 인식 시작: {p("image_topic")}, 라벨 {labels}')

    def decode(self, msg):
        if isinstance(msg, CompressedImage):
            return cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, -1)
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if msg.encoding == 'rgb8' else img

    def on_image(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last < self.period:
            return
        self.last = now
        img = self.decode(msg)
        if img is None:
            return
        label, areas = detect(img, self.ranges, self.get_parameter('min_area_ratio').value,
                              tuple(self.get_parameter('roi').value))
        self.pub.publish(String(data=label))
        if self.debug_pub is not None:
            text = f'{label or "-"}  ' + ' '.join(f'{k}:{v * 100:.1f}%' for k, v in areas.items())
            cv2.putText(img, text, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            out = CompressedImage(header=msg.header, format='jpeg')
            out.data = cv2.imencode('.jpg', img)[1].tobytes()
            self.debug_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CargoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
