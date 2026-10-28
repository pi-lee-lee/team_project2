#!/usr/bin/env python3
"""시뮬레이션 로봇의 출력을 실물 Pi(turtlebot3 bringup)와 같은 형식으로 바꾼다. scripts/start_sim.sh 가 로봇과 같이 띄움.
시뮬레이션 모델(worlds/models/turtlebot3_waffle_pi_real/model.sdf)은 LiDAR 를 scan_gz 로 내고 Gazebo 오도메트리를 끈다.

1) LiDAR: scan_gz → /scan. 실물 LDS-02 드라이버처럼 안 맞은 빔(Gazebo 는 inf, 최소 거리 안쪽은 -inf)을 NaN 으로,
   range_min 0, range_max 100, scan_time / time_increment 를 회전 주기 기준으로 (logs/real_scan_capture 와 같은 형식).
   그래서 브리지의 scan_level / scan_slam 처리(빔 재배열, inf 만 빈 공간으로)가 실물과 똑같이 동작한다.
2) 오도메트리: 실물 turtlebot3_node 처럼 이동은 바퀴 엔코더(joint_states), 회전은 IMU 방향 변화로 누적해
   /odom 과 TF odom → base_footprint 를 20 Hz 로 낸다 (원점 = 출발 자리, 바퀴 미끄러짐만큼 오차가 쌓임).
   Gazebo 실제 위치를 그대로 쓰던 예전 방식은 오차가 0 이라 실물과 SLAM/AMCL 동작이 달랐다.
"""
import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState, LaserScan
from tf2_ros import TransformBroadcaster

WHEEL_RADIUS = 0.033      # Waffle Pi (turtlebot3_node waffle_pi.yaml 과 같음)
WHEELS = ('wheel_left_joint', 'wheel_right_joint')
MAX_WHEEL_STEP = 2.0      # rad. 한 번에 이보다 크게 바뀌면(Gazebo 월드 초기화로 순간 이동) 누적하지 않고 다시 맞춤


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class SimRealIO(Node):
    def __init__(self):
        super().__init__('sim_real_io', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.scan_pub = self.create_publisher(LaserScan, 'scan', qos_profile_sensor_data)
        self.create_subscription(LaserScan, 'scan_gz', self.on_scan, qos_profile_sensor_data)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(JointState, 'joint_states', self.on_joints, 10)
        self.create_subscription(Imu, 'imu', self.on_imu, qos_profile_sensor_data)
        self.x = self.y = self.th = 0.0
        self.last_wheels = None   # (왼쪽, 오른쪽) 바퀴 각도 rad
        self.last_imu_yaw = None
        self.imu_yaw = None
        self.imu_wz = 0.0
        self.wheel_v = 0.0
        self.stamp = None
        self.create_timer(0.05, self.publish_odom)
        self.get_logger().info('시뮬레이션 출력을 실물 형식으로: scan_gz → scan (NaN, range_max 100), '
                               'odom = 바퀴 엔코더 + IMU')

    def on_scan(self, msg):
        out = LaserScan()
        out.header = msg.header
        out.angle_min, out.angle_max, out.angle_increment = msg.angle_min, msg.angle_max, msg.angle_increment
        out.scan_time = 1.0 / 9.3
        out.time_increment = out.scan_time / max(1, len(msg.ranges))
        out.range_min, out.range_max = 0.0, 100.0
        out.ranges = [r if math.isfinite(r) else math.nan for r in msg.ranges]
        out.intensities = msg.intensities
        self.scan_pub.publish(out)

    def on_imu(self, msg):
        self.imu_yaw = yaw_of(msg.orientation)
        self.imu_wz = msg.angular_velocity.z

    def on_joints(self, msg):
        try:
            idx = [msg.name.index(n) for n in WHEELS]
        except ValueError:
            return
        wheels = tuple(msg.position[i] for i in idx)
        if len(msg.velocity) == len(msg.name):
            self.wheel_v = WHEEL_RADIUS * (msg.velocity[idx[0]] + msg.velocity[idx[1]]) / 2.0
        self.stamp = msg.header.stamp
        if self.imu_yaw is None:
            return
        if self.last_wheels is None:
            self.last_wheels, self.last_imu_yaw = wheels, self.imu_yaw
            return
        dl, dr = wheels[0] - self.last_wheels[0], wheels[1] - self.last_wheels[1]
        dth = math.atan2(math.sin(self.imu_yaw - self.last_imu_yaw), math.cos(self.imu_yaw - self.last_imu_yaw))
        self.last_wheels, self.last_imu_yaw = wheels, self.imu_yaw
        if abs(dl) > MAX_WHEEL_STEP or abs(dr) > MAX_WHEEL_STEP:
            self.get_logger().warn(f'바퀴 각도가 한 번에 크게 바뀜 ({dl:.2f}, {dr:.2f} rad): 월드 초기화로 보고 누적하지 않음')
            return
        ds = WHEEL_RADIUS * (dl + dr) / 2.0
        self.x += ds * math.cos(self.th + dth / 2.0)
        self.y += ds * math.sin(self.th + dth / 2.0)
        self.th = math.atan2(math.sin(self.th + dth), math.cos(self.th + dth))

    def publish_odom(self):
        if self.stamp is None or self.last_wheels is None:
            return
        qz, qw = math.sin(self.th / 2.0), math.cos(self.th / 2.0)
        odom = Odometry()
        odom.header.stamp = self.stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x, odom.pose.pose.position.y = self.x, self.y
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = qz, qw
        odom.twist.twist.linear.x = self.wheel_v
        odom.twist.twist.angular.z = self.imu_wz
        self.odom_pub.publish(odom)
        t = TransformStamped()
        t.header = odom.header
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x, t.transform.translation.y = self.x, self.y
        t.transform.rotation.z, t.transform.rotation.w = qz, qw
        self.tf.sendTransform(t)


def main():
    rclpy.init()
    node = SimRealIO()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
