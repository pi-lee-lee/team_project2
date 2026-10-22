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
#   --world WORLD  (sim) turtlebot3_world(기본) | turtlebot3_house | empty_world | maze_room(worlds/, 실제 방 복제) | lecture_hall(강당)
#                  | no_roof_small_warehouse (AWS 창고, ~/sim_ws 에 받아야 함) ...
#   --gui          (sim) Gazebo 화면 표시
#   --nav2         Nav2 주행 노드(planner/controller_server, bt_navigator)도 실행. slam/map 모두 가능
#                  (slam: SLAM 이 맵과 위치를 내므로 AMCL/map_server 없이 주행 노드만)
#   --planner P    astar(기본, 직접 구현한 A*) | nav2 (Nav2 가 계획+주행, NavigateToPose)
#   --controller C hybrid(기본, 평소 pure pursuit, 경로 앞에 지도에 없는 장애물이 가까우면 DWA 로 피함)
#                  | dwa (항상 DWA, 추적한 움직이는 장애물의 미래 위치를 피함. tb3_nav 에서 이식)
#                  | builtin (항상 pure pursuit, 앞쪽 장애물이면 정지 후 대기) | nav2 (A* 경로를 Nav2 controller_server 가 추종)
#                  planner/controller 를 nav2 로 주면 --nav2 는 자동으로 켜짐
#   --scan-noise   (sim) LiDAR 를 실물 LDS-02 처럼: 빔 개수 238~244 변동, 거리 흔들림, 먼 곳 값 빠짐 (scan_noise.py)
#                  실물과 비슷한 강당은 --world lecture_hall_real (책상 앞판 없음, 의자 무작위 배치)
#   --amcl A       (map) 위치 추정 설정: default(기본, config/amcl_waffle_pi.yaml, 작은 방) | hall(config/amcl_hall.yaml,
#                  책상/의자가 흩어진 강당·교실: 지도와 안 맞는 빔에 너그럽게) | yaml 파일 경로
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
PLANNER=astar; CONTROLLER=hybrid; AMCL=default; SCAN_NOISE=false
while [ $# -gt 0 ]; do
  case "$1" in
    --robot) ROBOT="$2"; shift ;;
    --mode) MODE="$2"; shift ;;
    --map) MAP="$2"; shift ;;
    --name) NAME="$2"; shift ;;
    --slam) ENGINE="$2"; shift ;;
    --world) WORLD="$2"; WORLD_SET=1; shift ;;
    --gui) GUI=--gui ;;
    --nav2) NAV2=1 ;;
    --planner) PLANNER="$2"; shift ;;
    --controller) CONTROLLER="$2"; shift ;;
    --amcl) AMCL="$2"; shift ;;
    --scan-noise) SCAN_NOISE=true ;;
    -h|--help) sed -n '2,33p' "$0"; exit 0 ;;
    *) echo "알 수 없는 옵션: $1 (scripts/run.sh --help)"; exit 1 ;;
  esac
  shift
done
[[ "$ROBOT" =~ ^(real|sim)$ ]] || { echo "--robot real|sim 을 지정하세요"; exit 1; }
[[ "$MODE" =~ ^(slam|map)$ ]] || { echo "--mode slam|map 을 지정하세요"; exit 1; }
[[ "$PLANNER" =~ ^(astar|nav2)$ ]] || { echo "--planner astar|nav2"; exit 1; }
[[ "$CONTROLLER" =~ ^(hybrid|builtin|nav2|dwa)$ ]] || { echo "--controller hybrid|dwa|builtin|nav2"; exit 1; }
if [ "$PLANNER" = nav2 ] || [ "$CONTROLLER" = nav2 ]; then NAV2=1; fi
case "$AMCL" in
  default) AMCL_FILE="$HOME_DIR/config/amcl_waffle_pi.yaml" ;;
  hall) AMCL_FILE="$HOME_DIR/config/amcl_hall.yaml" ;;
  *) AMCL_FILE="$(realpath "$AMCL" 2>/dev/null || true)" ;;
esac
[ -f "$AMCL_FILE" ] || { echo "AMCL 설정 파일 없음: $AMCL (default | hall | yaml 경로)"; exit 1; }

if [ "$MODE" = map ]; then
  if [ -z "$MAP" ] && [ "$ROBOT" = sim ] && [ "$WORLD" = turtlebot3_world ]; then
    source /opt/ros/humble/setup.bash; [ -f "$HOME/turtlebot3_ws/install/setup.bash" ] && source "$HOME/turtlebot3_ws/install/setup.bash"
    MAP="$(ros2 pkg prefix turtlebot3_navigation2)/share/turtlebot3_navigation2/map/map.yaml"
    NAME="${NAME:-sim_turtlebot3_world}"
  fi
  [ -n "$MAP" ] || { echo "--mode map 에는 --map <yaml> 이 필요합니다"; exit 1; }
  MAP="$(realpath "$MAP")"
  [ -f "$MAP" ] || { echo "맵 파일 없음: $MAP"; exit 1; }
  # 시뮬레이션: --world 를 안 주면 맵 파일에 기록된 월드(# world: ...)로 띄운다.
  # 다른 월드에서 그 맵으로 위치를 추정하면 AMCL 좌표가 완전히 꼬인다.
  MAP_WORLD=$(sed -n 's/^# world: *//p' "$MAP" | head -1)
  if [ "$ROBOT" = sim ] && [ -z "$WORLD_SET" ] && [ -n "$MAP_WORLD" ]; then
    WORLD="$MAP_WORLD"
    echo "맵에 기록된 월드 사용: $WORLD"
  elif [ "$ROBOT" = sim ] && [ -n "$MAP_WORLD" ] && [ "$MAP_WORLD" != "$WORLD" ]; then
    echo "경고: 맵은 '$MAP_WORLD' 월드에서 만들어졌는데 '$WORLD' 월드로 띄웁니다 (위치 추정이 맞지 않을 수 있음)"
  fi
  if [ -z "$NAME" ]; then
    NAME="$(basename "$MAP" .yaml)"
    NAME="${NAME%_straight}"; NAME="${NAME%_clean}"
  fi
fi

# 시뮬레이션 출발 위치 (상태 초기화/초기 위치 자동 지정에 사용)
case "$WORLD" in
  turtlebot3_world|turtlebot3_house) SPAWN_X=-2.0; SPAWN_Y=-0.5 ;;
  maze_room) SPAWN_X=0.2; SPAWN_Y=0.0 ;;   # 실제 로봇을 놓던 자리 (worlds/maze_room.world 는 실제 방 맵 좌표 그대로)
  no_roof_small_warehouse|small_warehouse) SPAWN_X=-3.3; SPAWN_Y=-2.0 ;;   # AWS 창고: 가운데(0,0)는 3.5m 안에 아무것도 없어 LiDAR 가 못 봄 → 선반 근처 빈 통로
  *) SPAWN_X=0.0; SPAWN_Y=0.0 ;;
esac

# 동시에 두 번 실행되면(웹 실행 + 터미널 등) 둘 다 정리 후 각자 띄워 SLAM/브리지가 두 개씩 뜬다
# → 같은 /map, map->odom 을 두 SLAM 이 번갈아 내서 지도가 망가짐. 실행 중이면 바로 거절.
# (잠금은 이 스크립트가 끝나면 풀림. 띄운 노드에는 넘기지 않음: start() 의 9>&-)
mkdir -p "$LOGS"
exec 9> "$LOGS/run.lock"
if ! flock -n 9; then
  echo "다른 run.sh 가 실행 중입니다. 끝난 뒤 다시 실행하세요 (웹 실행 모드 또는 다른 터미널)"
  exit 1
fi

# 이미 떠 있는 노드/시뮬레이션은 먼저 정리 (서버는 유지)
"$DIR/stop_ros.sh" --sim >/dev/null 9>&- || true

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
WAREHOUSE_AMCL=$AMCL_FILE
WAREHOUSE_SCAN_NOISE=$([ "$ROBOT" = sim ] && echo "$SCAN_NOISE" || echo false)
EOF
source "$DIR/env.sh"
echo "== 로봇 $ROBOT / 모드 $MODE$([ "$SCAN_NOISE" = true ] && [ "$ROBOT" = sim ] && echo ' / LiDAR 노이즈 켬') / Nav2 주행 노드 $([ -n "$NAV2" ] && echo 켬 || echo 끔) / planner $PLANNER / controller $CONTROLLER / 도메인 $ROS_DOMAIN_ID"

start() {  # start <로그이름> <명령...>
  local log="$LOGS/$1.log"; shift
  setsid "$@" > "$log" 2>&1 < /dev/null 9>&- &
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
  echo "2) Gazebo 시뮬레이션 ($WORLD)"; start sim "$DIR/start_sim.sh" "$WORLD" $GUI --x "$SPAWN_X" --y "$SPAWN_Y"
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

# 2-1) 실물 + 맵 만들기: SLAM 지도 원점 = 오도메트리 원점(= bringup 을 켠 자리)이라, bringup 을 켜 둔 채 로봇을
#      옮기면 원점이 로봇에서 떨어지고 x 축이 로봇 정면과 틀어진다 (예: 4.9m, -19°).
#      SLAM 을 켜기 직전에 오도메트리를 0 으로 → 원점 = 지금 로봇 자리, x 축 = 로봇 정면.
#      (turtlebot3_node 의 /reset_odometry: x, y, 방향을 0 으로. 방향은 IMU 변화량을 더하므로 초기화 후에도 맞음)
#      Pi 에서 부른다: PC(Cyclone DDS)에서 부르면 요청은 가지만 응답이 돌아오지 않았음 (Pi 는 Fast DDS)
if [ "$ROBOT" = real ] && [ "$MODE" = slam ]; then
  if ssh -o BatchMode=yes "$PI_HOST" 'source /opt/ros/humble/setup.bash; source ~/turtlebot3_ws/install/setup.bash
       export ROS_DOMAIN_ID=44; timeout 20 ros2 service call /reset_odometry std_srvs/srv/Trigger "{}"' 2>/dev/null \
     | grep -q "success=True"; then
    echo "   오도메트리 초기화: SLAM 원점 = 지금 로봇 위치, x 축 = 로봇 정면"
  else
    echo "   경고: 오도메트리 초기화 실패 (/reset_odometry 응답 없음). 원점이 로봇과 어긋날 수 있음"
  fi
fi

# 3) 위치 추정 또는 SLAM
if [ "$MODE" = slam ]; then
  echo "3) SLAM ($ENGINE)"; start slam "$DIR/start_slam.sh" "$ENGINE"
  if [ -n "$NAV2" ]; then
    echo "   + Nav2 주행 노드 (map TF 가 나오면 시작)"; start nav2 "$DIR/start_nav2.sh" --navigation-only
  fi
else
  echo "3) 위치 추정 (맵 $MAP, AMCL 설정 $(basename "$AMCL_FILE"))$([ -n "$NAV2" ] && echo ' + Nav2 주행 노드 (초기 위치 지정 후 시작)')"
  start nav2 "$DIR/start_nav2.sh" "$MAP" ${NAV2:+--with-navigation}
fi
sleep 8

# 4) A* + 브리지
echo "4) A* 플래너 + 브리지"; start bridge "$DIR/start_bridge.sh" "$NAME"
sleep 8

# 5) 시뮬레이션 + 기존 맵: 출발 위치를 알고 있으므로 초기 위치 자동 지정
if [ "$ROBOT" = sim ] && [ "$MODE" = map ]; then
  echo "5) 초기 위치 자동 지정 ($SPAWN_X, $SPAWN_Y)"
  # 맵 좌표계가 월드와 다를 수 있으므로(Cartographer 맵 등) 잘 안 맞으면 맵 전체에서 다시 찾는다
  python3 "$HOME_DIR/tools/localize.py" "$SPAWN_X" "$SPAWN_Y" 0 --range 0.3 --yaw-range 20 --fallback-global 2>&1 \
    | grep -v -i warn | sed 's/^/   /' || true
elif [ "$MODE" = map ]; then
  echo "5) 웹에서 '초기 위치 지정'을 하거나: python3 tools/localize.py <x> <y> <방향°>"
else
  echo "5) SLAM: 로봇을 천천히 움직여 맵을 만드세요. 끝나면 웹 '맵 저장' 또는 scripts/save_map.sh <이름>"
fi
echo "== 완료. 웹: http://$(hostname -I | awk '{print $1}'):8000/   끄기: scripts/stop_ros.sh --all"
