#!/usr/bin/env python3
"""Run the fine-tuned detector: /csi/image_raw -> /yolo/image_raw.

Run this file from VS Code or with python3. ROS/venv setup is automatic.
"""
from yolo_runtime import run

if __name__ == '__main__':
    run(preview=False)
