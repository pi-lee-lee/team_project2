#!/usr/bin/env bash
# Pi 카메라 노드 (v4l2_camera). Pi 에서 실행한다. 실물 run.sh 는 이 파일을 Pi 의 ~/warehouse_camera.sh 로 복사해 자동으로 띄움
#   사용: bash camera.sh [장치]   기본 /dev/video0 (장치 번호는 v4l2-ctl --list-devices 로 확인)
#   출력: /csi/image_raw (원본, Pi 안에서만 쓰기), /csi/image_raw/compressed (JPEG, PC 의 YOLO 검출기가 받는 것)
#   원본 320x240 rgb8 15fps 는 약 28 Mbit/s 라 Wi-Fi 로 받으면 LiDAR/오도메트리까지 늦어져 연결이 끊겼다 붙었다 함
#   CAMERA_FPS (기본 15), CAMERA_SIZE (기본 "[640, 480]", 겹친 물건 개수 판정 때문에 320x240 에서 올림) 로 바꿀 수 있음
set -e
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID="${YOLO_DOMAIN_ID:-44}"
DEVICE="${1:-/dev/video0}"
v4l2-ctl -d "$DEVICE" --set-parm="${CAMERA_FPS:-15}" >/dev/null
exec ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -p video_device:="$DEVICE" \
  -p pixel_format:=YUYV \
  -p image_size:="${CAMERA_SIZE:-[640, 480]}" \
  -p output_encoding:=rgb8 \
  -r __ns:=/csi
