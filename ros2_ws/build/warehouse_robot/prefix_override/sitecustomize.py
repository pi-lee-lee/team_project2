import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/abc/webserver/ros2_ws/install/warehouse_robot'
