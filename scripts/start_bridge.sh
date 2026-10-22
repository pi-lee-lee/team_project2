#!/bin/bash
# A* 플래너 + 서버 브리지 실행.
#   사용: scripts/start_bridge.sh [map_name] [server_url]
#     map_name: 서버 DB에 저장될 맵 이름 (등록 위치가 이 이름에 묶임). SLAM 모드에서는 slam_live 고정
#   로봇 여러 대: 로봇마다 이름과 ROS 네임스페이스를 다르게 (웹 위쪽 막대에서 로봇을 골라 조작)
#     WAREHOUSE_ROBOT_NAME=tb3_1 WAREHOUSE_NAMESPACE=tb3_1 scripts/start_bridge.sh <맵이름>
#     WAREHOUSE_ROBOT_NAME: 서버 접속 이름 (기본 tb3), WAREHOUSE_NAMESPACE: 토픽/TF 네임스페이스 (기본 없음 = 지금처럼 /)
source "$(dirname "$0")/env.sh"
if [ "$WAREHOUSE_MODE" = slam ]; then
  MAP_NAME=slam_live   # 만드는 중인 맵이 저장된 맵과 등록 위치를 덮어쓰지 않게
  ALLOW_UNKNOWN=true   # SLAM 초반 맵은 대부분 미탐색이라, 미탐색 칸도 지나가게 해야 움직일 수 있다
else
  MAP_NAME="${1:-maze_map}"
  ALLOW_UNKNOWN=false
fi
# YOLO 부품 검출기도 같이 실행 (scripts/start_yolo.sh, enabled=false 로 시작). 끄려면 WAREHOUSE_YOLO=0
# 경로 계획/추종 방식 (run.sh 의 --planner/--controller, 기본: 직접 구현한 A* + 내장 추종기)
PLANNER="${WAREHOUSE_PLANNER:-astar}"
CONTROLLER="${WAREHOUSE_CONTROLLER:-dwa}"
ROBOT_NAME="${WAREHOUSE_ROBOT_NAME:-tb3}"
echo "[start_bridge] 로봇 $ROBOT_NAME, 네임스페이스 ${WAREHOUSE_NAMESPACE:-(없음)}, 모드 $WAREHOUSE_MODE, 맵 이름 $MAP_NAME, planner=$PLANNER, controller=$CONTROLLER, 시뮬레이션 시간 $WAREHOUSE_SIM_TIME, LiDAR 노이즈 ${WAREHOUSE_SCAN_NOISE:-false}, 도메인 $ROS_DOMAIN_ID"
exec ros2 launch warehouse_robot warehouse.launch.py map_name:="$MAP_NAME" \
  mode:="$WAREHOUSE_MODE" use_sim_time:="$WAREHOUSE_SIM_TIME" allow_unknown:="$ALLOW_UNKNOWN" \
  planner:="$PLANNER" controller:="$CONTROLLER" scan_noise:="${WAREHOUSE_SCAN_NOISE:-false}" \
  robot_name:="$ROBOT_NAME" namespace:="${WAREHOUSE_NAMESPACE:-}" \
  server_url:="${2:-ws://localhost:8000/ws/robot/$ROBOT_NAME/}" \
  yolo:="$( [ "${WAREHOUSE_YOLO:-1}" = 1 ] && echo "$(cd "$(dirname "$0")" && pwd)/start_yolo.sh" )"
