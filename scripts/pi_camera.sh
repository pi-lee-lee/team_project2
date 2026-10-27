#!/bin/bash
# Pi 카메라 노드(yolo/camera.sh) 켜기/끄기. 실물 로봇 전용.
#   사용: scripts/pi_camera.sh on|off|status
#
# 실물 run.sh 가 시작할 때 on (기본, 계속 켜 둠) / --no-camera 면 off 로 부른다. 수동 확인/재시작용으로도 씀.
# (한때 YOLO 를 켤 때만 카메라를 켰지만 켜는 데 실제 10~15초 걸려 섹션에서 YOLO 를 못 썼고, 측정상 카메라는 Wi-Fi 끊김 원인이 아니어서 되돌림)
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
