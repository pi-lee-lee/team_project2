"""(sim) Moving obstacle: moves a cylinder (about the size of a person's leg) back and forth between two points. For testing tracking/Frenet.

    source scripts/env.sh
    python3 tools/sim_mover.py --from 1.0 -1.6 --to 2.1 -1.6 --speed 0.2 [--radius 0.12] [--seconds 120]

The world needs the gazebo_ros_state plugin (/set_entity_state) (worlds/maze_room.world has it).
When finished (Ctrl-C or --seconds) the cylinder is deleted and the minimum clearance between the robot (ground-truth odom) and the cylinder and the number of collisions are printed.
The cylinder's position is set directly (teleported), so if the robot stands in its way it gets pushed harder than by a real person.
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
    stop = []   # so the cylinder is deleted and the summary printed even on Ctrl-C
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
            # approximate the cylinder as a square of its diameter (conservative)
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
