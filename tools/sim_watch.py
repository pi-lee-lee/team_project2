"""Simulation diagnostics: compares against the ground-truth pose (Gazebo odom = world coordinates) and logs pose error, collisions and tilt.

    source scripts/env.sh   (sim mode)
    python3 tools/sim_watch.py [--world worlds/maze_room.world] [--seconds 300] [--every 5]

- Pose error: distance/angle difference between the SLAM/AMCL estimate of map->base_footprint and the ground truth
  (in sim, slam_toolbox map coordinates = odom coordinates = world coordinates, so they compare directly)
- Wall clearance: minimum distance between the world file's boxes (walls) and the robot outline (rectangle). <= 0 means collision
- Tilt: computed from the IMU
At the end, summarizes the max error, number of collisions and minimum clearance.
"""
import argparse
import math
import signal
import time
import xml.etree.ElementTree as ET

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from tf2_ros import Buffer, TransformListener

# Waffle Pi outline (relative to base_footprint, m): the Gazebo collision box 0.265x0.265 sits at x=-0.064 and the wheels stick out a little further to the sides
FOOT = (-0.197, 0.069, -0.153, 0.153)


def yaw_q(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def load_boxes(path):
    """List of (cx, cy, sx, sy, yaw) boxes from a world file (world coordinates). Simple composition of <model><pose> + <collision><pose>."""
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
    """Approximate minimum distance between the robot outline rectangle and a box (outline points transformed into the box frame)."""
    x, y, yaw = pose
    bx, by, sx, sy, byaw = box
    pts = []
    for fx in (FOOT[0], FOOT[1]):
        for fy in (FOOT[2], FOOT[3]):
            pts.append((fx, fy))
    for i in range(1, 4):  # plus a few points along the edges
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
    # override rclpy's signal handling so the summary is printed even on Ctrl-C/kill
    stop = []
    signal.signal(signal.SIGINT, lambda *_: stop.append(1))
    signal.signal(signal.SIGTERM, lambda *_: stop.append(1))
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
    max_tilt_at = None
    try:
        loop_until = t0 + args.seconds
        while time.time() < loop_until and not stop:
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
                tilt = math.degrees(math.acos(max(-1, min(1, 1 - 2 * (q.x * q.x + q.y * q.y)))))
                if tilt > max_tilt:
                    max_tilt, max_tilt_at = tilt, (time.time() - t0, truth)
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
    except KeyboardInterrupt:
        pass  # still print the summary on Ctrl-C
    print(f'[watch] 요약: 최대 위치 오차 {worst_err:.3f} m, 최대 방향 오차 {worst_yaw:.1f}°, '
          f'충돌 {contacts}회, 최소 벽 여유 {min_gap:.3f} m, 최대 기울기 {max_tilt:.1f}°', flush=True)
    if max_tilt_at and max_tilt > 5:
        s_, (x, y, yaw) = max_tilt_at
        print(f'[watch] 가장 기울었던 때: {s_:.0f}s, 로봇 ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f}°) {max_tilt:.0f}°', flush=True)
    if min_at:
        (x, y, yaw), b = min_at
        print(f'[watch] 가장 가까웠던 때: 로봇 ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f}°), '
              f'상자 중심 ({b[0]:.2f}, {b[1]:.2f}) 크기 {b[2]:.2f}x{b[3]:.2f}', flush=True)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
