"""Independent detector with runtime enabled switch and direct count publishing."""
import os
import signal
from collections import Counter
from datetime import datetime
from time import monotonic
from pathlib import Path

ROOT=Path(__file__).resolve().parent
os.environ.setdefault('YOLO_CONFIG_DIR',str(ROOT/'.config'))
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter
from ultralytics import YOLO
import torch


class Detector(Node):
    def __init__(self):
        super().__init__('component_detector_switch')
        self.declare_parameter('weights',str(ROOT/'best_components7_finetuned.pt'))
        # 상대 이름: 네임스페이스 없이 띄우면 /csi/image_raw, -r __ns:=/tb3_1 이면 /tb3_1/csi/image_raw (로봇 여러 대)
        self.declare_parameter('input_topic','csi/image_raw')
        self.declare_parameter('confidence',0.35)
        self.declare_parameter('imgsz',320)
        self.declare_parameter('person_only',False)
        self.declare_parameter('enabled',False)  # 웹 'YOLO 시작' 버튼이 true, 'YOLO 정지' 가 false
        self.declare_parameter('capture_once',False)
        self.capture_dir=Path.home()/'camera_dataset/captures'
        self.add_on_set_parameters_callback(self.validate_switch)
        torch.set_num_threads(3)
        self.model=YOLO(self.get_parameter('weights').value)
        self.active_classes=[class_id for class_id,name in self.model.names.items()
                             if name.lower()!='rgb']
        self.latest=None
        self.last_count_log=None
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT,
                       durability=DurabilityPolicy.VOLATILE)
        # 카메라 영상은 실행 중(enabled)일 때만 구독한다. 정지하면 구독을 끊어 원본 영상을 받지도 않음 (Wi-Fi, CPU 절약)
        self.qos=qos
        self.sub=None
        self.sync_input()
        self.pub=self.create_publisher(Image,'yolo/image_raw',qos)
        self.pub_counts=self.create_publisher(String,'yolo/counts',10)
        self.timer=self.create_timer(0.05,self.detect)
        self.last_received=self.get_clock().now()
        self.input_warning=self.create_timer(5.0,self.check_input)
        self.get_logger().info(f"모델: {self.get_parameter('weights').value}")
        self.get_logger().info(f"입력: {self.get_parameter('input_topic').value}")
        self.get_logger().info('검출 결과 영상: /yolo/image_raw (CPU)')

    def validate_switch(self, parameters):
        for parameter in parameters:
            if parameter.name in ('enabled','capture_once') and parameter.type_!=Parameter.Type.BOOL:
                return SetParametersResult(successful=False, reason=f'{parameter.name} must be bool')
        if any(p.name=='enabled' for p in parameters):
            # Discard cached frames on either transition; resume only with new input.
            self.latest=None
            self.last_count_log=None
        return SetParametersResult(successful=True)

    def receive(self,msg):
        if not self.get_parameter('enabled').value:
            self.latest=None
            return
        self.latest=msg
        self.last_received=self.get_clock().now()

    def check_input(self):
        if not self.get_parameter('enabled').value:return
        if (self.get_clock().now()-self.last_received).nanoseconds>5_000_000_000:
            self.get_logger().warning('카메라 영상 대기 중: 로봇 카메라와 토픽/도메인을 확인하세요.')

    def log_class_counts(self, result):
        now=monotonic()
        if self.last_count_log is not None and now-self.last_count_log<1.0:
            return
        counts=Counter(int(class_id) for class_id in result.boxes.cls.tolist())
        names={class_id:name for class_id,name in result.names.items()
               if class_id in self.active_classes}
        if self.get_parameter('person_only').value:
            names={0:names[0]}
        summary=' / '.join(f'{name}: {counts[class_id]}'
                           for class_id,name in names.items())
        self.get_logger().info(f'현재 개수 | {summary}')
        output=String()
        output.data=summary
        self.pub_counts.publish(output)
        self.last_count_log=now

    def capture_if_requested(self, annotated):
        if not self.get_parameter('capture_once').value:
            return
        try:
            self.capture_dir.mkdir(parents=True, exist_ok=True)
            path=self.capture_dir/('yolo_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'.png')
            if not cv2.imwrite(str(path), annotated):
                raise RuntimeError(f'이미지 저장 실패: {path}')
            self.get_logger().info(f'검출 장면 캡처 저장: {path}')
        except (OSError, RuntimeError, cv2.error) as error:
            self.get_logger().error(f'캡처 실패: {error}')
        finally:
            # One request consumes one frame, including failed saves.
            self.set_parameters([Parameter('capture_once', value=False)])

    def sync_input(self):
        enabled=self.get_parameter('enabled').value
        if enabled and self.sub is None:
            self.sub=self.create_subscription(Image,self.get_parameter('input_topic').value,
                                              self.receive,self.qos)
            self.last_received=self.get_clock().now()
        elif not enabled and self.sub is not None:
            self.destroy_subscription(self.sub)
            self.sub=None

    def detect(self):
        self.sync_input()
        if not self.get_parameter('enabled').value:
            self.latest=None
            return
        msg=self.latest
        if msg is None:return
        self.latest=None
        if msg.encoding not in ('rgb8','bgr8'):
            self.get_logger().error(f'지원 인코딩: rgb8/bgr8, 수신: {msg.encoding}')
            return
        # Respect row padding (step), rather than assuming packed rows.
        frame=np.frombuffer(msg.data,dtype=np.uint8).reshape(msg.height,msg.step)
        frame=frame[:,:msg.width*3].reshape(msg.height,msg.width,3)
        if msg.encoding=='rgb8':frame=cv2.cvtColor(frame,cv2.COLOR_RGB2BGR)
        result=self.model.predict(frame,imgsz=self.get_parameter('imgsz').value,
                                  device='cpu',verbose=False,
                                  conf=self.get_parameter('confidence').value,
                                  classes=[0] if self.get_parameter('person_only').value else self.active_classes)[0]
        self.log_class_counts(result)
        annotated=np.ascontiguousarray(result.plot())
        self.capture_if_requested(annotated)
        output=Image()
        output.header=msg.header
        output.height,output.width=annotated.shape[:2]
        output.encoding='bgr8';output.is_bigendian=0
        output.step=output.width*3;output.data=annotated.tobytes()
        if rclpy.ok():
            self.pub.publish(output)


def main():
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=None
    try:
        node=Detector()
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
    except (KeyboardInterrupt,ExternalShutdownException):
        pass
    finally:
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
