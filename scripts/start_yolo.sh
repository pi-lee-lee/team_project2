#!/bin/bash
# YOLO 부품 검출기 (yolo/yolo_training/infer_ros_switch.py, 켜기/끄기 가능한 버전) 실행.
#   사용: scripts/start_yolo.sh [ROS 인자...]   예) scripts/start_yolo.sh -p imgsz:=640
#   입력: /csi/image_raw (Pi 에서 yolo/camera.sh), 출력: /yolo/image_raw (검출 영상), /yolo/counts (클래스별 개수 글자)
#   보통은 start_bridge.sh (warehouse.launch.py) 가 브리지와 같이 띄운다. 따로 띄울 때만 직접 실행
#   웹 "카메라 검출 (YOLO)" 카드의 켜기/끄기 = 이 노드의 enabled 파라미터. 기본값 false (꺼짐), 버튼을 누르면 true
#   도메인은 env.sh 를 따른다 (실물 44, sim 77)
source "$(dirname "$0")/env.sh"
ROOT="$(cd "$(dirname "$0")/.." && pwd)/yolo/yolo_training"
# yolo/README.md 대로 만든 가상환경이 있으면 그것, 없으면 시스템 python (ultralytics, torch 가 설치돼 있어야 함)
PY="$ROOT/.venv/bin/python"
[ -x "$PY" ] || PY=/usr/bin/python3
export YOLO_CONFIG_DIR="$ROOT/.config"
echo "[start_yolo] 도메인 $ROS_DOMAIN_ID, python $PY"
exec "$PY" -u "$ROOT/infer_ros_switch.py" --ros-args -p enabled:=false "$@"
