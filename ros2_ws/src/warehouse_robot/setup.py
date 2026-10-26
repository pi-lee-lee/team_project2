from glob import glob

from setuptools import setup

package_name = 'warehouse_robot'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abc',
    maintainer_email='wlgh8449@gmail.com',
    description='TurtleBot3 물류창고 로봇: A* 플래너와 Django 서버 브리지',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'astar_planner = warehouse_robot.astar_planner_node:main',
            'bridge = warehouse_robot.bridge_node:main',
        ],
    },
)
