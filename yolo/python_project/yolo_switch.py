#!/usr/bin/env python3
"""Independent YOLO launcher; toggle /component_detector_switch enabled."""
import os
import sys
from yolo_runtime import PROJECT, PYTHON, ros_environment

if __name__ == '__main__':
    env = ros_environment()
    os.execve(str(PYTHON), [str(PYTHON), '-u', str(PROJECT / 'infer_ros_switch.py'),
                           *sys.argv[1:]], env)
