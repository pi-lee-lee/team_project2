"""(sim) 움직이는 장애물: 원기둥(사람 다리 정도)을 두 점 사이로 왕복시킨다. 추적/Frenet 시험용.

    source scripts/env.sh
    python3 tools/sim_mover.py --from 1.0 -1.6 --to 2.1 -1.6 --speed 0.2 [--radius 0.12] [--seconds 120]

월드에 gazebo_ros_state 플러그인(/set_entity_state)이 있어야 한다 (worlds/maze_room.world 에는 있음).
끝나면(Ctrl-C 또는 --seconds) 원기둥을 지우고, 로봇(정답 odom)과 원기둥 사이 최소 여유와 충돌 횟수를 출력한다.
원기둥은 위치를 강제로 옮기므로(순간 이동) 로봇이 길목에 서 있으면 실제 사람보다 세게 밀어낸다.
"""
import argparse
import math
import os
import signal
import sys
import time

import rclpy
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import DeleteEntity, SetEntityState, SpawnEntity
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sim_watch import rect_gap, yaw_q  # noqa: E402

SDF = '''<?xml version="1.0"?><sdf version="1.6"><model name="{name}"><static>true</static><link name="l">
<collision name="c"><pose>0 0 0.25 0 0 0</pose><geometry><cylinder><radius>{r}</radius><length>0.5</length></cylinder></geometry></collision>
<visual name="v"><pose>0 0 0.25 0 0 0</pose><geometry><cylinder><radius>{r}</radius><length>0.5</length></cylinder></geometry>
<material><ambient>0.9 0.4 0.1 1</ambient><diffuse>0.9 0.4 0.1 1</diffuse></material></visual></link></model></sdf>'''


def call(node, client, req, timeout=5.0):
    if not client.wait_for_service(timeout_sec=timeout):
        raise SystemExit(f'서비스 없음: {client.srv_name} (월드에 gazebo_ros_state 플러그인이 있는지 확인)')
    fut = client.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    return fut.result()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--from', dest='a', type=float, nargs=2, required=True)
    ap.add_argument('--to', dest='b', type=float, nargs=2, required=True)
    ap.add_argument('--speed', type=float, default=0.2)
    ap.add_argument('--radius', type=float, default=0.12)
    ap.add_argument('--seconds', type=float, default=0, help='0 = Ctrl-C 까지')
    ap.add_argument('--name', default='mover')
    ap.add_argument('--once', action='store_true', help='왕복하지 않고 한 번만 지나간 뒤 끝점에 멈춤 (사람이 지나가듯)')
    ap.add_argument('--delay', type=float, default=0.0, help='출발 전 대기 (초)')
    args = ap.parse_args()
    rclpy.init()
    stop = []   # Ctrl-C 로 끝내도 원기둥을 지우고 요약을 출력하도록
    signal.signal(signal.SIGINT, lambda *_: stop.append(1))
    signal.signal(signal.SIGTERM, lambda *_: stop.append(1))
    node = Node('sim_mover')
    odom = {}
    node.create_subscription(Odometry, 'odom', lambda m: odom.__setitem__('o', m), qos_profile_sensor_data)
    spawn = node.create_client(SpawnEntity, '/spawn_entity')
    setter = node.create_client(SetEntityState, '/set_entity_state')
    delete = node.create_client(DeleteEntity, '/delete_entity')
    call(node, delete, DeleteEntity.Request(name=args.name), timeout=2.0)
    req = SpawnEntity.Request(name=args.name, xml=SDF.format(name=args.name, r=args.radius))
    req.initial_pose.position.x, req.initial_pose.position.y = args.a
    call(node, spawn, req)
    length = math.hypot(args.b[0] - args.a[0], args.b[1] - args.a[1])
    period = 2 * length / args.speed
    print(f'[sim_mover] {args.a} ↔ {args.b}, {args.speed} m/s (왕복 {period:.1f}초)', flush=True)
    t0 = time.time()
    best, contacts, touching = (math.inf, None, None), 0, False
    while not stop and (args.seconds <= 0 or time.time() - t0 < args.seconds):
        el = time.time() - t0 - args.delay
        if el < 0:
            s = 0.0
        elif args.once:
            s = min(1.0, el / (period / 2))
        else:
            phase = (el % period) / period
            s = 2 * phase if phase < 0.5 else 2 - 2 * phase   # 0→1→0
        mx = args.a[0] + (args.b[0] - args.a[0]) * s
        my = args.a[1] + (args.b[1] - args.a[1]) * s
        st = EntityState(name=args.name)
        st.pose.position.x, st.pose.position.y = mx, my
        st.pose.orientation.w = 1.0
        setter.call_async(SetEntityState.Request(state=st))
        rclpy.spin_once(node, timeout_sec=0.05)
        if 'o' in odom:
            o = odom['o'].pose.pose
            pose = (o.position.x, o.position.y, yaw_q(o.orientation))
            # 원기둥을 지름 크기 정사각형으로 근사 (보수적)
            gap = rect_gap(pose, (mx, my, 2 * args.radius, 2 * args.radius, 0.0))
            if gap < best[0]:
                best = (gap, (round(pose[0], 2), round(pose[1], 2)), (round(mx, 2), round(my, 2)))
            contacts += gap <= 0.005 and not touching
            touching = gap <= 0.005
    call(node, delete, DeleteEntity.Request(name=args.name), timeout=2.0)
    print(f'[sim_mover] 끝 (원기둥 삭제). 로봇과 최소 여유 {best[0]:.3f} m (로봇 {best[1]}, 원기둥 {best[2]}), '
          f'충돌 {contacts}회', flush=True)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
