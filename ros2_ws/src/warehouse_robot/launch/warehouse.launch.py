"""A* 플래너 + 서버 브리지 실행.

    ros2 launch warehouse_robot warehouse.launch.py server_url:=ws://<서버IP>:8000/ws/robot/tb3/
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import LaunchConfigurationNotEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('warehouse_robot'), 'config', 'params.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')
    server_url = LaunchConfiguration('server_url')
    planner = LaunchConfiguration('planner')
    map_name = LaunchConfiguration('map_name')
    mode = LaunchConfiguration('mode')
    allow_unknown = LaunchConfiguration('allow_unknown')
    controller = LaunchConfiguration('controller')
    scan_noise = LaunchConfiguration('scan_noise')
    yolo = LaunchConfiguration('yolo')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('server_url', default_value='ws://localhost:8000/ws/robot/tb3/'),
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
        Node(
            package='warehouse_robot', executable='astar_planner', name='astar_planner',
            output='screen', parameters=[params, {'use_sim_time': use_sim_time, 'allow_unknown': allow_unknown}]),
        Node(
            package='warehouse_robot', executable='bridge', name='warehouse_bridge',
            output='screen',
            parameters=[params, {'use_sim_time': use_sim_time, 'server_url': server_url,
                                 'planner': planner, 'controller': controller,
                                 'map_name': map_name, 'mode': mode,
                                 'sim_scan.enabled': scan_noise}]),
        # YOLO 부품 검출기: 브리지와 같이 뜬다. enabled(추론+검출 영상) 는 false 로 시작, 웹 "검출 켜기" 버튼이 true 로
        ExecuteProcess(cmd=[yolo], name='yolo', output='screen',
                       condition=LaunchConfigurationNotEquals('yolo', '')),
    ])
