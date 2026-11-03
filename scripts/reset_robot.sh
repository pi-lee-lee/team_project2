#!/bin/bash
# Reset robot state (run by the bridge from the web "상태 초기화" button; can also be run directly)
#   - Stop command for 1 s
#   - Simulation: Gazebo /reset_world puts the robot back upright at the start pose (if it tipped over or got stuck)
#     Gazebo odometry follows the true pose, so the SLAM map/AMCL pose don't get corrupted.
#     In existing-map mode the initial pose is re-aligned around the start pose.
#   - Real robot: stop only (stand a tipped robot up by hand, then use "초기 위치 지정" on the web)
source "$(dirname "$0")/env.sh"

echo "[reset] 정지 명령"
timeout 5 ros2 topic pub --times 10 -r 10 /cmd_vel geometry_msgs/msg/Twist "{}" >/dev/null 2>&1

if [ "$WAREHOUSE_ROBOT" != sim ]; then
  echo "[reset] 실물 로봇: 정지 완료. 넘어졌다면 직접 세운 뒤 웹에서 '초기 위치 지정'을 하세요"
  exit 0
fi

echo "[reset] Gazebo 월드 초기화 (/reset_world)"
if ! timeout 15 ros2 service call /reset_world std_srvs/srv/Empty "{}" >/dev/null 2>&1; then
  echo "[reset] /reset_world 호출 실패 (Gazebo 가 실행 중인지 확인)"
  exit 1
fi
sleep 2

if [ "$WAREHOUSE_MODE" = map ]; then
  X="${WAREHOUSE_SPAWN_X:-0.0}"; Y="${WAREHOUSE_SPAWN_Y:-0.0}"
  echo "[reset] 초기 위치 다시 맞추기 ($X, $Y)"
  python3 "$WAREHOUSE_HOME/tools/localize.py" "$X" "$Y" 0 --range 0.3 --yaw-range 20 2>&1 \
    | grep -E "가장 잘 맞는|경고" | sed 's/^/[reset] /'
fi
echo "[reset] 완료: 로봇이 출발 위치로 돌아갔습니다"
