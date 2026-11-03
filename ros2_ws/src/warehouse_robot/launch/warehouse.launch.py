"""Launch the A* planner + server bridge.

    ros2 launch warehouse_robot warehouse.launch.py server_url:=ws://<server IP>:8000/ws/robot/tb3/

Multiple robots: give each robot its own robot_name (name used to connect to the server, shown in the web robot list) and namespace
    ros2 launch warehouse_robot warehouse.launch.py robot_name:=tb3_1 namespace:=tb3_1
    -> nodes/topics/services go under /tb3_1/ (cmd_vel, scan, odom, astar/..., yolo/...), TF on /tb3_1/tf, /tb3_1/tf_static
      (frame names stay map/odom/base_footprint; TF topics are split per robot). An empty server_url is built from robot_name
"""
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def namespaced_params(params, namespace):
    """The node-name keys in params.yaml (astar_planner:, warehouse_bridge:) only match nodes directly under /,
    so with a namespace, use a copy whose keys are rewritten to /<ns>/<node>."""
    with open(params, encoding='utf-8') as f:
        data = yaml.safe_load(f)
    data = {f'/{namespace}/{key.strip("/")}': value for key, value in data.items()}
    out = tempfile.NamedTemporaryFile('w', suffix=f'_{namespace}_params.yaml', delete=False, encoding='utf-8')
    with out:
        yaml.safe_dump(data, out, allow_unicode=True)
    return out.name


def launch_nodes(context):
    params = os.path.join(get_package_share_directory('warehouse_robot'), 'config', 'params.yaml')
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    robot_name = arg('robot_name')
    namespace = arg('namespace').strip('/')
    if namespace:
        params = namespaced_params(params, namespace)
    server_url = arg('server_url') or f'ws://localhost:8000/ws/robot/{robot_name}/'
    use_sim_time = arg('use_sim_time') == 'true'
    # with a namespace, use only that robot's TF (/tf -> /<ns>/tf); otherwise plain /tf
    remaps = [('/tf', 'tf'), ('/tf_static', 'tf_static')] if namespace else []
    nodes = [
        Node(
            package='warehouse_robot', executable='astar_planner', name='astar_planner', namespace=namespace,
            output='screen', remappings=remaps,
            parameters=[params, {'use_sim_time': use_sim_time, 'allow_unknown': arg('allow_unknown') == 'true'}]),
        Node(
            package='warehouse_robot', executable='bridge', name='warehouse_bridge', namespace=namespace,
            output='screen', remappings=remaps,
            parameters=[params, {'use_sim_time': use_sim_time, 'server_url': server_url,
                                 'map_name': arg('map_name'), 'mode': arg('mode')}]),
    ]
    # YOLO component detector: starts with the bridge. enabled (inference + detection image) starts false; the web "YOLO start" button sets it true
    # with a namespace the detector goes under it too (/<ns>/component_detector_switch, /<ns>/csi/image_raw -> /<ns>/yolo/counts)
    yolo = arg('yolo')
    if yolo:
        cmd = [yolo] + (['-r', f'__ns:=/{namespace}'] if namespace else [])
        nodes.append(ExecuteProcess(cmd=cmd, name='yolo', output='screen'))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('robot_name', default_value='tb3',
                              description='서버에 접속하는 로봇 이름 (/ws/robot/<이름>/, 웹 로봇 선택 목록)'),
        DeclareLaunchArgument('namespace', default_value='',
                              description='ROS 네임스페이스 (로봇 여러 대일 때 예: tb3_1). 비우면 지금처럼 / 아래'),
        DeclareLaunchArgument('server_url', default_value='',
                              description='비우면 ws://localhost:8000/ws/robot/<robot_name>/'),
        DeclareLaunchArgument('map_name', default_value='default'),
        DeclareLaunchArgument('mode', default_value='map', description='map (기존 맵 + AMCL) | slam (맵 만들기)'),
        DeclareLaunchArgument('yolo', default_value='',
                              description='YOLO 검출기 실행 스크립트 (scripts/start_yolo.sh 경로, start_bridge.sh 가 넘김). 비우면 실행 안 함'),
        DeclareLaunchArgument('allow_unknown', default_value='false',
                              description='A*가 미탐색 칸을 지나갈 수 있는지 (SLAM 모드에서 true)'),
        OpaqueFunction(function=launch_nodes),
    ])
