"""A* 플래너 + 서버 브리지 실행.

    ros2 launch warehouse_robot warehouse.launch.py server_url:=ws://<서버IP>:8000/ws/robot/tb3/
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('server_url', default_value='ws://localhost:8000/ws/robot/tb3/'),
        DeclareLaunchArgument('planner', default_value='astar', description='astar | nav2'),
        DeclareLaunchArgument('controller', default_value='builtin', description='builtin | nav2'),
        DeclareLaunchArgument('map_name', default_value='default'),
        DeclareLaunchArgument('mode', default_value='map', description='map (기존 맵 + AMCL) | slam (맵 만들기)'),
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
                                 'map_name': map_name, 'mode': mode}]),
    ])
