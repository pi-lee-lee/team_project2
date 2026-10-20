"""시뮬레이션 진단: 정답 위치(Gazebo odom = 월드 좌표)와 비교해 좌표 오차, 충돌, 기울기를 기록한다.

    source scripts/env.sh   (sim 모드)
    python3 tools/sim_watch.py [--world worlds/maze_room.world] [--seconds 300] [--every 5]

- 좌표 오차: SLAM/AMCL 이 추정한 map->base_footprint 와 정답 위치의 거리/각도 차이
  (sim 에서는 slam_toolbox 맵 좌표 = odom 좌표 = 월드 좌표라서 그대로 비교 가능)
- 벽 여유: 월드 파일의 상자(벽) 와 로봇 외곽(직사각형) 사이 최소 거리. 0 이하면 충돌
- 기울기: IMU 로 계산
끝나면 최대 오차, 충돌 횟수, 최소 여유를 요약한다.
"""
import argparse
import math
import re
import time
import xml.etree.ElementTree as ET

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from tf2_ros import Buffer, TransformListener

# Waffle Pi 외곽 (base_footprint 기준, m): Gazebo 충돌 상자 0.265x0.265 가 x=-0.064 에 있고 바퀴가 좌우로 조금 더 나옴
FOOT = (-0.197, 0.069, -0.153, 0.153)


def yaw_q(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def load_boxes(path):
    """월드 파일에서 (cx, cy, sx, sy, yaw) 상자 목록 (월드 좌표). <model><pose> + <collision><pose> 단순 합성."""
    boxes = []
    root = ET.parse(path).getroot()
    for model in root.iter('model'):
        mp = [float(v) for v in (model.findtext('pose') or '0 0 0 0 0 0').split()]
        for col in model.iter('collision'):
            size = col.find('.//box/size')
            if size is None:
                continue
            sx, sy, _ = [float(v) for v in size.text.split()]
            cp = [float(v) for v in (col.findtext('pose') or '0 0 0 0 0 0').split()]
            c, s = math.cos(mp[5]), math.sin(mp[5])
            boxes.append((mp[0] + c * cp[0] - s * cp[1], mp[1] + s * cp[0] + c * cp[1], sx, sy, mp[5] + cp[5]))
    return boxes


def rect_gap(pose, box):
    """로봇 외곽 직사각형과 상자 사이 대략적인 최소 거리 (외곽 점들을 상자 좌표계로 옮겨 계산)."""
    x, y, yaw = pose
    bx, by, sx, sy, byaw = box
    pts = []
    for fx in (FOOT[0], FOOT[1]):
        for fy in (FOOT[2], FOOT[3]):
            pts.append((fx, fy))
    for i in range(1, 4):  # 변 위의 점도 몇 개
        pts += [(FOOT[0] + (FOOT[1] - FOOT[0]) * i / 4, FOOT[2]), (FOOT[0] + (FOOT[1] - FOOT[0]) * i / 4, FOOT[3]),
                (FOOT[0], FOOT[2] + (FOOT[3] - FOOT[2]) * i / 4), (FOOT[1], FOOT[2] + (FOOT[3] - FOOT[2]) * i / 4)]
    best = math.inf
    c, s = math.cos(yaw), math.sin(yaw)
    cb, sb = math.cos(-byaw), math.sin(-byaw)
    for px, py in pts:
        wx, wy = x + c * px - s * py - bx, y + s * px + c * py - by
        lx, ly = cb * wx - sb * wy, sb * wx + cb * wy
        dx, dy = abs(lx) - sx / 2, abs(ly) - sy / 2
        d = math.hypot(max(dx, 0), max(dy, 0)) if (dx > 0 or dy > 0) else max(dx, dy)
        best = min(best, d)
    return best


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--world', action='append', default=[],
                    help='충돌 계산용 월드/모델 SDF 파일 (여러 번 줄 수 있음, 없으면 충돌 계산 생략)')
    ap.add_argument('--seconds', type=float, default=300)
    ap.add_argument('--every', type=float, default=5)
    args = ap.parse_args()
    boxes = [b for w in args.world for b in load_boxes(w)]

    rclpy.init()
    node = Node('sim_watch', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
    buf = Buffer()
    TransformListener(buf, node)
    d = {}
    node.create_subscription(Odometry, 'odom', lambda m: d.__setitem__('odom', m), qos_profile_sensor_data)
    node.create_subscription(Imu, 'imu', lambda m: d.__setitem__('imu', m), qos_profile_sensor_data)

    t0 = last = time.time()
    worst_err = worst_yaw = 0.0
    min_gap = math.inf
    min_at = None
    contacts = 0
    in_contact = False
    max_tilt = 0.0
    while time.time() - t0 < args.seconds:
        rclpy.spin_once(node, timeout_sec=0.02)
        if 'odom' not in d:
            continue
        o = d['odom'].pose.pose
        truth = (o.position.x, o.position.y, yaw_q(o.orientation))
        if boxes:
            gaps = [rect_gap(truth, b) for b in boxes]
            gap = min(gaps)
            if gap < min_gap:
                min_gap = gap
                min_at = (truth, boxes[gaps.index(gap)])
            if gap <= 0.005 and not in_contact:
                contacts += 1
                print(f'[watch] {time.time() - t0:5.0f}s 충돌! 위치 ({truth[0]:.2f}, {truth[1]:.2f})', flush=True)
            in_contact = gap <= 0.005
        if 'imu' in d:
            q = d['imu'].orientation
            max_tilt = max(max_tilt, math.degrees(math.acos(max(-1, min(1, 1 - 2 * (q.x * q.x + q.y * q.y))))))
        err = yerr = None
        if buf.can_transform('map', 'base_footprint', rclpy.time.Time()):
            t = buf.lookup_transform('map', 'base_footprint', rclpy.time.Time()).transform
            est = (t.translation.x, t.translation.y, yaw_q(t.rotation))
            err = math.hypot(est[0] - truth[0], est[1] - truth[1])
            yerr = abs(math.degrees(math.atan2(math.sin(est[2] - truth[2]), math.cos(est[2] - truth[2]))))
            worst_err, worst_yaw = max(worst_err, err), max(worst_yaw, yerr)
        if time.time() - last >= args.every:
            last = time.time()
            print(f'[watch] {last - t0:5.0f}s 정답 ({truth[0]:.2f}, {truth[1]:.2f}) '
                  f'추정 오차 {err if err is None else round(err, 3)} m / {yerr if yerr is None else round(yerr, 1)}° '
                  f'벽 여유 {"-" if not boxes else f"{gap:.3f} m"}', flush=True)
    print(f'[watch] 요약: 최대 위치 오차 {worst_err:.3f} m, 최대 방향 오차 {worst_yaw:.1f}°, '
          f'충돌 {contacts}회, 최소 벽 여유 {min_gap:.3f} m, 최대 기울기 {max_tilt:.1f}°', flush=True)
    if min_at:
        (x, y, yaw), b = min_at
        print(f'[watch] 가장 가까웠던 때: 로봇 ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f}°), '
              f'상자 중심 ({b[0]:.2f}, {b[1]:.2f}) 크기 {b[2]:.2f}x{b[3]:.2f}', flush=True)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
