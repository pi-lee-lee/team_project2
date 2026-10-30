#!/bin/bash
# Start the A* planner + server bridge.
#   Usage: scripts/start_bridge.sh [map_name] [server_url]
#     map_name: map name stored in the server DB (registered locations are tied to it). Always slam_live in SLAM mode
#   Multiple robots: give each robot its own name and ROS namespace (select the robot in the web top bar)
#     WAREHOUSE_ROBOT_NAME=tb3_1 WAREHOUSE_NAMESPACE=tb3_1 scripts/start_bridge.sh <map_name>
#     WAREHOUSE_ROBOT_NAME: name used to connect to the server (default tb3), WAREHOUSE_NAMESPACE: topic/TF namespace (default none = /)
source "$(dirname "$0")/env.sh"
if [ "$WAREHOUSE_MODE" = slam ]; then
  MAP_NAME=slam_live   # So the map being built doesn't overwrite saved maps and their locations
  ALLOW_UNKNOWN=true   # Early SLAM maps are mostly unknown, so unknown cells must be passable for the robot to move
else
  MAP_NAME="${1:-maze_map}"
  ALLOW_UNKNOWN=false
fi
# Also start the YOLO component detector (scripts/start_yolo.sh, starts with enabled=false). WAREHOUSE_YOLO=0 to skip
ROBOT_NAME="${WAREHOUSE_ROBOT_NAME:-tb3}"
echo "[start_bridge] 로봇 $ROBOT_NAME, 네임스페이스 ${WAREHOUSE_NAMESPACE:-(없음)}, 모드 $WAREHOUSE_MODE, 맵 이름 $MAP_NAME, 시뮬레이션 시간 $WAREHOUSE_SIM_TIME, 도메인 $ROS_DOMAIN_ID"
# ros2 launch rejects empty arguments (namespace:=), so pass it only when set (otherwise the launch default '')
EXTRA=()
[ -n "$WAREHOUSE_NAMESPACE" ] && EXTRA+=(namespace:="$WAREHOUSE_NAMESPACE")
[ "${WAREHOUSE_YOLO:-1}" = 1 ] && EXTRA+=(yolo:="$(cd "$(dirname "$0")" && pwd)/start_yolo.sh")
exec ros2 launch warehouse_robot warehouse.launch.py map_name:="$MAP_NAME" \
  mode:="$WAREHOUSE_MODE" use_sim_time:="$WAREHOUSE_SIM_TIME" allow_unknown:="$ALLOW_UNKNOWN" \
  robot_name:="$ROBOT_NAME" server_url:="${2:-ws://localhost:8000/ws/robot/$ROBOT_NAME/}" "${EXTRA[@]}"
