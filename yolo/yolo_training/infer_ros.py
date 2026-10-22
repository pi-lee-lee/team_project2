"""Detect components in the CSI image topic using the fine-tuned model."""
import os
from collections import Counter
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
from ultralytics import YOLO
import torch


class Detector(Node):
    def __init__(self):
        super().__init__('component_detector')
        self.declare_parameter('weights',str(ROOT/'best_components7_finetuned.pt'))
        self.declare_parameter('input_topic','/csi/image_raw')
        self.declare_parameter('confidence',0.35)
        self.declare_parameter('imgsz',320)
        self.declare_parameter('person_only',False)
        torch.set_num_threads(3)
        self.model=YOLO(self.get_parameter('weights').value)
        self.active_classes=[class_id for class_id,name in self.model.names.items()
                             if name.lower()!='rgb']
        self.latest=None
        self.last_count_log=None
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT,
                       durability=DurabilityPolicy.VOLATILE)
        self.sub=self.create_subscription(Image,self.get_parameter('input_topic').value,
                                          self.receive,qos)
        self.pub=self.create_publisher(Image,'/yolo/image_raw',qos)
        self.timer=self.create_timer(0.05,self.detect)
        self.last_received=self.get_clock().now()
        self.input_warning=self.create_timer(5.0,self.check_input)
        self.get_logger().info(f"모델: {self.get_parameter('weights').value}")
        self.get_logger().info(f"입력: {self.get_parameter('input_topic').value}")
        self.get_logger().info('검출 결과 영상: /yolo/image_raw (CPU)')

    def receive(self,msg):
        self.latest=msg
        self.last_received=self.get_clock().now()

    def check_input(self):
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
        self.last_count_log=now

    def detect(self):
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
        output=Image()
        output.header=msg.header
        output.height,output.width=annotated.shape[:2]
        output.encoding='bgr8';output.is_bigendian=0
        output.step=output.width*3;output.data=annotated.tobytes()
        if rclpy.ok():
            self.pub.publish(output)


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=None
    try:
        node=Detector()
        rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):
        pass
    finally:
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
