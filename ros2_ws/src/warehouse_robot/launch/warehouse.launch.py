"""A* 플래너 + 서버 브리지 실행.

    ros2 launch warehouse_robot warehouse.launch.py server_url:=ws://<서버IP>:8000/ws/robot/tb3/

로봇 여러 대: robot_name (서버에 접속하는 이름, 웹 로봇 선택 목록에 나옴) 과 namespace 를 로봇마다 다르게
    ros2 launch warehouse_robot warehouse.launch.py robot_name:=tb3_1 namespace:=tb3_1
    → 노드/토픽/서비스가 /tb3_1/ 아래로 (cmd_vel, scan, odom, astar/..., yolo/...), TF 는 /tb3_1/tf, /tb3_1/tf_static
      (프레임 이름은 map/odom/base_footprint 그대로, 로봇마다 TF 토픽을 나눔). server_url 을 비우면 robot_name 으로 만든다
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
    """params.yaml 의 노드 이름 키(astar_planner:, warehouse_bridge:)는 / 바로 아래 노드에만 맞으므로,
    네임스페이스를 주면 키를 /<ns>/<노드> 로 바꾼 사본을 만들어 쓴다."""
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
    # 네임스페이스를 주면 TF 도 그 로봇 것만 (/tf → /<ns>/tf). 안 주면 지금처럼 /tf
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
                                 'planner': arg('planner'), 'controller': arg('controller'),
                                 'map_name': arg('map_name'), 'mode': arg('mode'),
                                 'sim_scan.enabled': arg('scan_noise') == 'true'}]),
    ]
    # YOLO 부품 검출기: 브리지와 같이 뜬다. enabled(추론+검출 영상) 는 false 로 시작, 웹 "YOLO 시작" 버튼이 true 로
    # 네임스페이스를 주면 검출기도 그 아래로 (/<ns>/component_detector_switch, /<ns>/csi/image_raw → /<ns>/yolo/counts)
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
        DeclareLaunchArgument('planner', default_value='astar', description='astar | nav2'),
        DeclareLaunchArgument('controller', default_value='builtin', description='builtin | nav2 | dwa'),
        DeclareLaunchArgument('map_name', default_value='default'),
        DeclareLaunchArgument('mode', default_value='map', description='map (기존 맵 + AMCL) | slam (맵 만들기)'),
        DeclareLaunchArgument('scan_noise', default_value='false',
                              description='시뮬레이션 LiDAR 를 실물 LDS-02 처럼 (빔 개수 변동, 흔들림, 값 빠짐)'),
        DeclareLaunchArgument('yolo', default_value='',
                              description='YOLO 검출기 실행 스크립트 (scripts/start_yolo.sh 경로, start_bridge.sh 가 넘김). 비우면 실행 안 함'),
        DeclareLaunchArgument('allow_unknown', default_value='false',
                              description='A*가 미탐색 칸을 지나갈 수 있는지 (SLAM 모드에서 true)'),
        OpaqueFunction(function=launch_nodes),
    ])
