#!/bin/bash
# Pi 카메라 노드(yolo/camera.sh) 켜기/끄기. 실물 로봇 전용.
#   사용: scripts/pi_camera.sh on|off|status
#
# 주행 중에는 끈다: 카메라가 돌면 (영상을 PC 로 안 보내도) Pi 의 Wi-Fi 가 불안정해져 오도메트리/LiDAR 가
# 0.3~0.5초씩 끊기고(위치 TF 지연 정지), PC 의 정지 명령이 9초 동안 안 들어가 벽에 부딪힌 적이 있음 (실물 2026-10-26).
# 브리지가 YOLO 를 켤 때(주문 섹션/사람 위치 도착, 웹 YOLO 시작) on, 끌 때 off 를 부른다.
# on 은 저장소의 yolo/camera.sh 를 Pi 의 ~/warehouse_camera.sh 로 복사해서 띄운다 (이미 떠 있으면 그대로).
PI_HOST="${WAREHOUSE_PI_HOST:-test@192.168.0.181}"
HOME_DIR="$(cd "$(dirname "$0")/.." && pwd)"
SSH=(ssh -o BatchMode=yes -o ConnectTimeout=5 "$PI_HOST")

case "$1" in
  on)
    "${SSH[@]}" 'cat > ~/warehouse_camera.sh' < "$HOME_DIR/yolo/camera.sh" || exit 1
    "${SSH[@]}" 'pgrep -f "[v]4l2_camera_node" >/dev/null && echo "카메라: 이미 실행 중" || {
      setsid nohup bash ~/warehouse_camera.sh > ~/warehouse_camera.log 2>&1 < /dev/null &
      sleep 4; pgrep -f "[v]4l2_camera_node" >/dev/null && echo "카메라 시작됨" || { echo "카메라 확인 필요: Pi ~/warehouse_camera.log"; exit 1; }; }'
    ;;
  off)
    # pkill -f 패턴에 [v] 를 써서 이 원격 셸 자신(명령 줄에 패턴이 들어 있음)은 안 죽게
    "${SSH[@]}" 'pkill -f "[v]4l2_camera_node"; sleep 1; pgrep -f "[v]4l2_camera_node" >/dev/null && { echo "카메라가 아직 실행 중"; exit 1; } || echo "카메라 꺼짐"'
    ;;
  status)
    "${SSH[@]}" 'l=$(pgrep -af "[v]4l2_camera_node" | grep -o "image_size:=\[[0-9, ]*\]" | head -1)
      [ -n "$l" ] && echo "카메라 실행 중 $l" || echo "카메라 꺼짐"'
    ;;
  *)
    sed -n '2,8p' "$0"; exit 2 ;;
esac
