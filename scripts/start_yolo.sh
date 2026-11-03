#!/bin/bash
# Start the YOLO component detector (yolo/yolo_training/infer_ros_switch.py, the version that can be switched on/off).
#   Usage: scripts/start_yolo.sh [ROS args...]   e.g. scripts/start_yolo.sh -p imgsz:=640
#   Input: /csi/image_raw (yolo/camera.sh on the Pi), output: /yolo/image_raw (detection image), /yolo/counts (per-class count text)
#   Normally start_bridge.sh (warehouse.launch.py) starts it with the bridge. Run directly only to start it separately
#   The on/off toggle of the web "카메라 검출 (YOLO)" card = this node's enabled parameter. Default false (off), true when the button is pressed
#   Domain follows env.sh (real 44, sim 77)
source "$(dirname "$0")/env.sh"
ROOT="$(cd "$(dirname "$0")/.." && pwd)/yolo/yolo_training"
# Use the virtualenv from yolo/README.md if present, otherwise system python (needs ultralytics and torch)
PY="$ROOT/.venv/bin/python"
[ -x "$PY" ] || PY=/usr/bin/python3
export YOLO_CONFIG_DIR="$ROOT/.config"
echo "[start_yolo] 도메인 $ROS_DOMAIN_ID, python $PY"
exec "$PY" -u "$ROOT/infer_ros_switch.py" --ros-args -p enabled:=false "$@"
