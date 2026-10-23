"""ros2 bag 에 녹화한 odom / scan / tf 를 CSV 로 풀어 쓴다 (로봇·ROS 노드 없이 파일만 읽음).

    source /opt/ros/humble/setup.bash
    python3 tools/bag_to_csv.py ~/bags/run1 [--out 폴더] [--fixed map] [--base base_footprint]

출력 (기본 폴더 <bag>_csv/):
    odom.csv   Odometry 토픽마다 하나 (/tb3/odom → tb3_odom.csv): 위치, yaw, 속도
    scan.csv   LaserScan 토픽마다 하나: 한 줄 = 스캔 한 번, ranges 는 공백으로 이은 거리 목록
    tf.csv     /tf + /tf_static 의 변환 하나하나 (static 열로 구분)
    pose.csv   tf 를 이어 붙인 fixed → base 위치 (지도 위 궤적). 사슬 중 하나라도 바뀔 때마다 한 줄
토픽은 이름이 아니라 메시지 타입으로 고른다. t 는 녹화 시작부터의 초(녹화 시각), stamp 는 메시지 헤더 시각.
"""
import argparse
import csv
import math
import os


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def stamp_of(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def compose(a, b):
    """평면 변환 a∘b. 각 변환은 (x, y, yaw)."""
    ax, ay, ath = a
    bx, by, bth = b
    c, s = math.cos(ath), math.sin(ath)
    return (ax + c * bx - s * by, ay + s * bx + c * by,
            math.atan2(math.sin(ath + bth), math.cos(ath + bth)))


class TfChain:
    """child → (parent, 변환) 을 최신값으로 들고 있다가 fixed → base 를 이어 붙인다."""

    def __init__(self, fixed, base):
        self.fixed = fixed.lstrip('/')
        self.base = base.lstrip('/')
        self.parent = {}

    def update(self, parent, child, xyyaw):
        self.parent[child.lstrip('/')] = (parent.lstrip('/'), xyyaw)

    def lookup(self):
        frame, out, seen = self.base, (0.0, 0.0, 0.0), set()
        while frame != self.fixed:
            if frame in seen or frame not in self.parent:
                return None  # 사슬이 아직 안 이어짐 (예: SLAM/AMCL 이 map → odom 을 아직 안 냄)
            seen.add(frame)
            frame, tf = self.parent[frame]
            out = compose(tf, out)
        return out


def file_name(topic):
    return topic.strip('/').replace('/', '_') + '.csv'


class Writers:
    """파일 이름별 csv.writer 를 처음 쓸 때 연다."""

    def __init__(self, out_dir):
        self.out_dir = out_dir
        self.files = {}
        self.counts = {}

    def row(self, name, header, values):
        if name not in self.files:
            f = open(os.path.join(self.out_dir, name), 'w', newline='', encoding='utf-8')
            w = csv.writer(f)
            w.writerow(header)
            self.files[name] = (f, w)
            self.counts[name] = 0
        self.files[name][1].writerow(values)
        self.counts[name] += 1

    def close(self):
        for f, _ in self.files.values():
            f.close()


ODOM_HEADER = ['t', 'stamp', 'frame', 'child', 'x', 'y', 'yaw_deg', 'vx', 'wz']
SCAN_HEADER = ['t', 'stamp', 'frame', 'angle_min', 'angle_increment',
               'range_min', 'range_max', 'n', 'ranges']
TF_HEADER = ['t', 'stamp', 'static', 'parent', 'child', 'x', 'y', 'z', 'yaw_deg']
POSE_HEADER = ['t', 'stamp', 'x', 'y', 'yaw_deg']


def convert(messages, out, chain):
    """messages: (topic, 타입 이름, 메시지, 녹화 시각 ns) 를 시간순으로. ROS 없이도 돌도록 분리해 둠."""
    t0 = None
    last_pose = None
    for topic, kind, m, t_ns in messages:
        if t0 is None:
            t0 = t_ns
        t = f'{(t_ns - t0) * 1e-9:.3f}'
        if kind == 'nav_msgs/msg/Odometry':
            p, tw = m.pose.pose, m.twist.twist
            out.row(file_name(topic), ODOM_HEADER, [
                t, f'{stamp_of(m.header):.3f}', m.header.frame_id, m.child_frame_id,
                f'{p.position.x:.4f}', f'{p.position.y:.4f}',
                f'{math.degrees(yaw_of(p.orientation)):.2f}',
                f'{tw.linear.x:.4f}', f'{tw.angular.z:.4f}'])
        elif kind == 'sensor_msgs/msg/LaserScan':
            out.row(file_name(topic), SCAN_HEADER, [
                t, f'{stamp_of(m.header):.3f}', m.header.frame_id,
                f'{m.angle_min:.6f}', f'{m.angle_increment:.6f}',
                f'{m.range_min:.3f}', f'{m.range_max:.3f}', len(m.ranges),
                ' '.join(f'{r:.3f}' for r in m.ranges)])
        elif kind == 'tf2_msgs/msg/TFMessage':
            static = int('static' in topic)
            stamp = None
            for tr in m.transforms:
                v, q = tr.transform.translation, tr.transform.rotation
                yaw = yaw_of(q)
                stamp = stamp_of(tr.header)
                out.row('tf.csv', TF_HEADER, [
                    t, f'{stamp:.3f}', static, tr.header.frame_id, tr.child_frame_id,
                    f'{v.x:.4f}', f'{v.y:.4f}', f'{v.z:.4f}', f'{math.degrees(yaw):.2f}'])
                chain.update(tr.header.frame_id, tr.child_frame_id, (v.x, v.y, yaw))
            pose = chain.lookup()
            if pose is not None and pose != last_pose:
                last_pose = pose
                out.row('pose.csv', POSE_HEADER, [
                    t, f'{stamp:.3f}', f'{pose[0]:.4f}', f'{pose[1]:.4f}',
                    f'{math.degrees(pose[2]):.2f}'])


def read_bag(path, storage):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=storage),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    wanted = ('nav_msgs/msg/Odometry', 'sensor_msgs/msg/LaserScan', 'tf2_msgs/msg/TFMessage')
    types = {t.name: t.type for t in reader.get_all_topics_and_types() if t.type in wanted}
    classes = {name: get_message(kind) for name, kind in types.items()}
    print('변환할 토픽:', ', '.join(f'{n} ({k})' for n, k in sorted(types.items())) or '없음')
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic in classes:
            yield topic, types[topic], deserialize_message(data, classes[topic]), t_ns


def main():
    ap = argparse.ArgumentParser(description='ros2 bag 의 odom / scan / tf 를 CSV 로')
    ap.add_argument('bag', help='bag 폴더 (metadata.yaml 이 있는 곳)')
    ap.add_argument('--out', help='출력 폴더 (기본 <bag>_csv)')
    ap.add_argument('--fixed', default='map', help='pose.csv 의 기준 프레임 (SLAM/AMCL 없이 녹화했으면 odom)')
    ap.add_argument('--base', default='base_footprint', help='pose.csv 의 로봇 프레임')
    ap.add_argument('--storage', default='sqlite3', help='저장 형식 (sqlite3 / mcap)')
    args = ap.parse_args()

    bag = os.path.expanduser(args.bag).rstrip('/')
    out_dir = os.path.expanduser(args.out) if args.out else bag + '_csv'
    os.makedirs(out_dir, exist_ok=True)
    out = Writers(out_dir)
    try:
        convert(read_bag(bag, args.storage), out, TfChain(args.fixed, args.base))
    finally:
        out.close()
    for name, n in sorted(out.counts.items()):
        print(f'  {os.path.join(out_dir, name)}: {n} 줄')
    if 'pose.csv' not in out.counts and 'tf.csv' in out.counts:
        print(f'  pose.csv 없음: tf 에서 {args.fixed} → {args.base} 사슬이 이어지지 않았습니다 (--fixed odom 시도)')


if __name__ == '__main__':
    main()
