#!/bin/bash
# 전체 실행 (백그라운드). 로그는 logs/ 에, 끌 때는 scripts/stop_ros.sh --all
#
#   scripts/run.sh --robot real|sim --mode slam|map [옵션]
#
#   --robot real   실물 TurtleBot3 (Pi bringup 을 SSH 로 실행, 도메인 44)
#   --robot sim    Gazebo 시뮬레이션 (도메인 77)
#   --mode slam    맵 만들기: SLAM 실행. 끝나면 scripts/save_map.sh <이름> 또는 웹 "맵 저장"
#   --mode map     기존 맵 사용: map_server + AMCL
#
#   --map FILE     (map) 맵 yaml. sim + turtlebot3_world 는 생략 시 TurtleBot3 기본 맵 사용
#   --name NAME    (map) 서버 DB 맵 이름 (등록 위치가 묶임). 기본: 파일 이름에서 _straight/_clean 뺀 것
#   --slam ENGINE  (slam) slam_toolbox(기본) | cartographer
#   --world WORLD  (sim) turtlebot3_world(기본) | turtlebot3_house | empty_world ...
#   --gui          (sim) Gazebo 화면 표시
#   --nav2         Nav2 주행 노드(planner/controller_server, bt_navigator)도 실행. slam/map 모두 가능
#                  (slam: SLAM 이 맵과 위치를 내므로 AMCL/map_server 없이 주행 노드만)
#   --planner P    astar(기본, 직접 구현한 A*) | nav2 (Nav2 가 계획+주행, NavigateToPose)
#   --controller C builtin(기본, 내장 pure pursuit) | nav2 (A* 경로를 Nav2 controller_server 가 추종)
#                  planner/controller 를 nav2 로 주면 --nav2 는 자동으로 켜짐
#
# 예)
#   scripts/run.sh --robot sim --mode slam --world turtlebot3_house --gui
#   scripts/run.sh --robot sim --mode map
#   scripts/run.sh --robot real --mode map --map ~/maps/backup/maze_map_straight.yaml
#   scripts/run.sh --robot real --mode slam
#   scripts/run.sh --robot sim --mode slam --nav2 --planner nav2     (SLAM + Nav2 로 탐색 주행)
set -e
DIR="$(cd "$(dirname "$0")" && pwd)"
HOME_DIR="$(cd "$DIR/.." && pwd)"
LOGS="$HOME_DIR/logs"
PI_HOST="test@192.168.0.181"

ROBOT=""; MODE=""; MAP=""; NAME=""; ENGINE=slam_toolbox; WORLD=turtlebot3_world; GUI=""; NAV2=""
PLANNER=astar; CONTROLLER=builtin
while [ $# -gt 0 ]; do
  case "$1" in
    --robot) ROBOT="$2"; shift ;;
    --mode) MODE="$2"; shift ;;
    --map) MAP="$2"; shift ;;
    --name) NAME="$2"; shift ;;
    --slam) ENGINE="$2"; shift ;;
    --world) WORLD="$2"; shift ;;
    --gui) GUI=--gui ;;
    --nav2) NAV2=1 ;;
    --planner) PLANNER="$2"; shift ;;
    --controller) CONTROLLER="$2"; shift ;;
    -h|--help) sed -n '2,31p' "$0"; exit 0 ;;
    *) echo "알 수 없는 옵션: $1 (scripts/run.sh --help)"; exit 1 ;;
  esac
  shift
done
[[ "$ROBOT" =~ ^(real|sim)$ ]] || { echo "--robot real|sim 을 지정하세요"; exit 1; }
[[ "$MODE" =~ ^(slam|map)$ ]] || { echo "--mode slam|map 을 지정하세요"; exit 1; }
[[ "$PLANNER" =~ ^(astar|nav2)$ ]] || { echo "--planner astar|nav2"; exit 1; }
[[ "$CONTROLLER" =~ ^(builtin|nav2)$ ]] || { echo "--controller builtin|nav2"; exit 1; }
if [ "$PLANNER" = nav2 ] || [ "$CONTROLLER" = nav2 ]; then NAV2=1; fi

if [ "$MODE" = map ]; then
  if [ -z "$MAP" ] && [ "$ROBOT" = sim ] && [ "$WORLD" = turtlebot3_world ]; then
    source /opt/ros/humble/setup.bash; source "$HOME/turtlebot3_ws/install/setup.bash"
    MAP="$(ros2 pkg prefix turtlebot3_navigation2)/share/turtlebot3_navigation2/map/map.yaml"
    NAME="${NAME:-sim_turtlebot3_world}"
  fi
  [ -n "$MAP" ] || { echo "--mode map 에는 --map <yaml> 이 필요합니다"; exit 1; }
  MAP="$(realpath "$MAP")"
  [ -f "$MAP" ] || { echo "맵 파일 없음: $MAP"; exit 1; }
  if [ -z "$NAME" ]; then
    NAME="$(basename "$MAP" .yaml)"
    NAME="${NAME%_straight}"; NAME="${NAME%_clean}"
  fi
fi

# 시뮬레이션 출발 위치 (상태 초기화/초기 위치 자동 지정에 사용)
case "$WORLD" in turtlebot3_world|turtlebot3_house) SPAWN_X=-2.0; SPAWN_Y=-0.5 ;; *) SPAWN_X=0.0; SPAWN_Y=0.0 ;; esac

# 이미 떠 있는 노드/시뮬레이션은 먼저 정리 (서버는 유지)
"$DIR/stop_ros.sh" --sim >/dev/null || true

mkdir -p "$LOGS"
cat > "$LOGS/run_mode.env" <<EOF
# scripts/run.sh 가 기록한 현재 실행 모드 ($(date '+%F %T'))
WAREHOUSE_ROBOT=$ROBOT
WAREHOUSE_MODE=$MODE
WAREHOUSE_MAP=$MAP
WAREHOUSE_MAP_NAME=$NAME
WAREHOUSE_WORLD=$WORLD
WAREHOUSE_NAV2=${NAV2:-0}
WAREHOUSE_SLAM_ENGINE=$ENGINE
WAREHOUSE_SPAWN_X=$SPAWN_X
WAREHOUSE_SPAWN_Y=$SPAWN_Y
WAREHOUSE_PLANNER=$PLANNER
WAREHOUSE_CONTROLLER=$CONTROLLER
EOF
source "$DIR/env.sh"
echo "== 로봇 $ROBOT / 모드 $MODE / Nav2 주행 노드 $([ -n "$NAV2" ] && echo 켬 || echo 끔) / planner $PLANNER / controller $CONTROLLER / 도메인 $ROS_DOMAIN_ID"

start() {  # start <로그이름> <명령...>
  local log="$LOGS/$1.log"; shift
  setsid "$@" > "$log" 2>&1 < /dev/null &
  echo "   시작: $* (로그 logs/$(basename "$log"))"
}

# 1) 웹 서버
if curl -s -o /dev/null --max-time 2 http://localhost:8000/; then
  echo "1) 웹 서버: 이미 실행 중"
else
  echo "1) 웹 서버"; start server "$DIR/start_server.sh"; sleep 3
fi

# 2) 로봇
if [ "$ROBOT" = sim ]; then
  echo "2) Gazebo 시뮬레이션 ($WORLD)"; start sim "$DIR/start_sim.sh" "$WORLD" $GUI
  sleep 12
else
  echo "2) 실물 로봇 bringup ($PI_HOST)"
  if ! ssh -o BatchMode=yes -o ConnectTimeout=5 "$PI_HOST" true 2>/dev/null; then
    echo "   Pi 에 접속할 수 없습니다. 로봇 전원/Wi-Fi 를 확인하세요."; exit 1
  fi
  ssh -o BatchMode=yes "$PI_HOST" 'pgrep -f "[t]urtlebot3_ros" >/dev/null && echo "   이미 실행 중" || {
    source /opt/ros/humble/setup.bash; source ~/turtlebot3_ws/install/setup.bash
    export ROS_DOMAIN_ID=44 LDS_MODEL=LDS-02 TURTLEBOT3_MODEL=waffle_pi
    setsid nohup ros2 launch turtlebot3_bringup robot.launch.py > ~/warehouse_bringup.log 2>&1 < /dev/null &
    sleep 12; grep -q "Run!" ~/warehouse_bringup.log && echo "   bringup 시작됨" || echo "   bringup 확인 필요: Pi ~/warehouse_bringup.log"; }'
fi

# 3) 위치 추정 또는 SLAM
if [ "$MODE" = slam ]; then
  echo "3) SLAM ($ENGINE)"; start slam "$DIR/start_slam.sh" "$ENGINE"
  if [ -n "$NAV2" ]; then
    echo "   + Nav2 주행 노드 (map TF 가 나오면 시작)"; start nav2 "$DIR/start_nav2.sh" --navigation-only
  fi
else
  echo "3) 위치 추정 (맵 $MAP)$([ -n "$NAV2" ] && echo ' + Nav2 주행 노드 (초기 위치 지정 후 시작)')"
  start nav2 "$DIR/start_nav2.sh" "$MAP" ${NAV2:+--with-navigation}
fi
sleep 8

# 4) A* + 브리지
echo "4) A* 플래너 + 브리지"; start bridge "$DIR/start_bridge.sh" "$NAME"
sleep 8

# 5) 시뮬레이션 + 기존 맵: 출발 위치를 알고 있으므로 초기 위치 자동 지정
if [ "$ROBOT" = sim ] && [ "$MODE" = map ]; then
  echo "5) 초기 위치 자동 지정 ($SPAWN_X, $SPAWN_Y)"
  python3 "$HOME_DIR/tools/localize.py" "$SPAWN_X" "$SPAWN_Y" 0 --range 0.3 --yaw-range 20 2>&1 | grep -v -i warn | sed 's/^/   /' || true
elif [ "$MODE" = map ]; then
  echo "5) 웹에서 '초기 위치 지정'을 하거나: python3 tools/localize.py <x> <y> <방향°>"
else
  echo "5) SLAM: 로봇을 천천히 움직여 맵을 만드세요. 끝나면 웹 '맵 저장' 또는 scripts/save_map.sh <이름>"
fi
echo "== 완료. 웹: http://$(hostname -I | awk '{print $1}'):8000/   끄기: scripts/stop_ros.sh --all"
