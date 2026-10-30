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
#                  | map2_room (worlds/, 실물 map2 방 복제. ~/maps/map2_clean.yaml 에 기록돼 있어 --map 만 주면 자동)
#                  | no_roof_small_warehouse (AWS 창고, ~/sim_ws 에 받아야 함) ...
#                  실물과 비슷한 강당은 --world lecture_hall_real (책상 앞판 없음, 의자 무작위 배치)
#   --gui          (sim) Gazebo 화면 표시
#   --no-camera    (real) Pi 카메라(yolo/camera.sh)를 띄우지 않음 (기본은 bringup 과 같이 띄워 계속 켜 둠, 이미 떠 있으면 그대로).
#                  YOLO 검출기가 꺼져 있으면 영상을 구독하지 않아 주행 중 Wi-Fi 로 오지 않음
#   --amcl A       (map) 위치 추정 설정: default(기본, config/amcl_waffle_pi.yaml, 작은 방) | hall(config/amcl_hall.yaml,
#                  책상/의자가 흩어진 강당·교실: 지도와 안 맞는 빔에 너그럽게) | yaml 파일 경로
#
# 예)
#   scripts/run.sh --robot sim --mode slam --world turtlebot3_house --gui
#   scripts/run.sh --robot sim --mode map
#   scripts/run.sh --robot real --mode map --map ~/maps/backup/maze_map_straight.yaml
#   scripts/run.sh --robot real --mode slam
#
# 주행: 브리지의 A* + Frenet 로컬 플래너 (경로를 기준선으로 옆으로 비키며 장애물을 피함)
set -e
DIR="$(cd "$(dirname "$0")" && pwd)"
HOME_DIR="$(cd "$DIR/.." && pwd)"
LOGS="$HOME_DIR/logs"
PI_HOST="test@192.168.0.181"

ROBOT=""; MODE=""; MAP=""; NAME=""; ENGINE=slam_toolbox; WORLD=turtlebot3_world; GUI=""
AMCL=default; CAMERA=1
while [ $# -gt 0 ]; do
  case "$1" in
    --robot) ROBOT="$2"; shift ;;
    --mode) MODE="$2"; shift ;;
    --map) MAP="$2"; shift ;;
    --name) NAME="$2"; shift ;;
    --slam) ENGINE="$2"; shift ;;
    --world) WORLD="$2"; WORLD_SET=1; shift ;;
    --gui) GUI=--gui ;;
    --amcl) AMCL="$2"; shift ;;
    --camera) CAMERA=1 ;;      # Accepted for compatibility (same as the default)
    --no-camera) CAMERA=0 ;;
    -h|--help) sed -n '2,29p' "$0"; exit 0 ;;
    *) echo "알 수 없는 옵션: $1 (scripts/run.sh --help)"; exit 1 ;;
  esac
  shift
done
[[ "$ROBOT" =~ ^(real|sim)$ ]] || { echo "--robot real|sim 을 지정하세요"; exit 1; }
[[ "$MODE" =~ ^(slam|map)$ ]] || { echo "--mode slam|map 을 지정하세요"; exit 1; }
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
  # Sim: without --world, launch the world recorded in the map file (# world: ...).
  # Localizing with that map in a different world completely breaks AMCL coordinates.
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

# Sim start pose (used for state reset / automatic initial pose)
case "$WORLD" in
  turtlebot3_world|turtlebot3_house) SPAWN_X=-2.0; SPAWN_Y=-0.5 ;;
  maze_room) SPAWN_X=0.2; SPAWN_Y=0.0 ;;   # Where the real robot is placed (worlds/maze_room.world uses the real room's map coordinates)
  map2_room) SPAWN_X=0.0; SPAWN_Y=0.0 ;;   # map2's home location (worlds/map2_room.world uses map2 coordinates)
  no_roof_small_warehouse|small_warehouse) SPAWN_X=-3.3; SPAWN_Y=-2.0 ;;   # AWS warehouse: nothing is within LiDAR range of the center (0,0) -> free aisle near the shelves
  *) SPAWN_X=0.0; SPAWN_Y=0.0 ;;
esac

# If run twice at once (web + terminal etc.), both clean up and each starts its own SLAM/bridge
# -> two SLAMs alternately publish /map and map->odom and corrupt the map. Refuse immediately if already running.
# (The lock is released when this script exits. It is not inherited by started nodes: 9>&- in start())
mkdir -p "$LOGS"
exec 9> "$LOGS/run.lock"
if ! flock -n 9; then
  echo "다른 run.sh 가 실행 중입니다. 끝난 뒤 다시 실행하세요 (웹 실행 모드 또는 다른 터미널)"
  exit 1
fi

# Stop already running nodes/simulation first (the server is kept)
"$DIR/stop_ros.sh" --sim >/dev/null 9>&- || true

cat > "$LOGS/run_mode.env" <<EOF
# scripts/run.sh 가 기록한 현재 실행 모드 ($(date '+%F %T'))
WAREHOUSE_ROBOT=$ROBOT
WAREHOUSE_MODE=$MODE
WAREHOUSE_MAP=$MAP
WAREHOUSE_MAP_NAME=$NAME
WAREHOUSE_WORLD=$WORLD
WAREHOUSE_SLAM_ENGINE=$ENGINE
WAREHOUSE_SPAWN_X=$SPAWN_X
WAREHOUSE_SPAWN_Y=$SPAWN_Y
WAREHOUSE_AMCL=$AMCL_FILE
EOF
source "$DIR/env.sh"
echo "== 로봇 $ROBOT / 모드 $MODE / 도메인 $ROS_DOMAIN_ID"

start() {  # start <log_name> <command...>
  local log="$LOGS/$1.log"; shift
  setsid "$@" > "$log" 2>&1 < /dev/null 9>&- &
  echo "   시작: $* (로그 logs/$(basename "$log"))"
}

# 1) Web server
if curl -s -o /dev/null --max-time 2 http://localhost:8000/; then
  echo "1) 웹 서버: 이미 실행 중"
else
  echo "1) 웹 서버"; start server "$DIR/start_server.sh"; sleep 3
fi

# 2) Robot
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
  # Cmd watchdog (scripts/pi_cmd_watchdog.py): forwards the bridge's cmd_vel_pc to cmd_vel and stops the robot after a gap over 0.5 s.
  # Without it the bridge's commands never reach the real robot (it won't move). Restarted every time so edits take effect
  ssh -o BatchMode=yes "$PI_HOST" 'cat > ~/warehouse_cmd_watchdog.py' < "$HOME_DIR/scripts/pi_cmd_watchdog.py"
  # Stop and start separately: if the file name is on the same command line, pkill -f also kills that remote shell
  ssh -o BatchMode=yes "$PI_HOST" 'pkill -f "[w]arehouse_cmd_watchdog.py"; sleep 0.5'
  ssh -o BatchMode=yes "$PI_HOST" 'source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=44
    setsid nohup python3 ~/warehouse_cmd_watchdog.py > ~/warehouse_cmd_watchdog.log 2>&1 < /dev/null &'
  ssh -o BatchMode=yes "$PI_HOST" 'sleep 3; pgrep -f "[w]arehouse_cmd_watchdog.py" >/dev/null && echo "   명령 감시 시작됨 (통신 0.5초 끊기면 정지)" \
      || echo "   명령 감시 확인 필요: Pi ~/warehouse_cmd_watchdog.log (없으면 로봇이 움직이지 않음)"'
  # Camera (YOLO detector input /csi/image_raw/compressed). Kept running: images only cross Wi-Fi while the detector is enabled
  # (stopped at an order section/person location). It is not toggled per stop because starting the camera takes too long.
  # Manual on/off/status: scripts/pi_camera.sh on|off|status. stop_ros.sh does not touch the camera
  if [ "$CAMERA" = 1 ]; then
    echo "   $(WAREHOUSE_PI_HOST="$PI_HOST" "$HOME_DIR/scripts/pi_camera.sh" on)"
  else
    echo "   $(WAREHOUSE_PI_HOST="$PI_HOST" "$HOME_DIR/scripts/pi_camera.sh" off) (--no-camera)"
  fi
fi

# 2-1) Real robot + map building: the SLAM map origin = odometry origin (= where bringup was started), so moving the robot
#      with bringup running puts the origin away from the robot and rotates the x axis off the robot's heading.
#      Reset odometry to 0 right before starting SLAM -> origin = current robot position, x axis = robot heading.
#      (turtlebot3_node /reset_odometry zeroes x, y and heading; heading adds IMU deltas so it stays correct after the reset)
#      Called on the Pi: called from the PC (Cyclone DDS) the request goes out but no response comes back (the Pi uses Fast DDS)
if [ "$ROBOT" = real ] && [ "$MODE" = slam ]; then
  if ssh -o BatchMode=yes "$PI_HOST" 'source /opt/ros/humble/setup.bash; source ~/turtlebot3_ws/install/setup.bash
       export ROS_DOMAIN_ID=44; timeout 20 ros2 service call /reset_odometry std_srvs/srv/Trigger "{}"' 2>/dev/null \
     | grep -q "success=True"; then
    echo "   오도메트리 초기화: SLAM 원점 = 지금 로봇 위치, x 축 = 로봇 정면"
  else
    echo "   경고: 오도메트리 초기화 실패 (/reset_odometry 응답 없음). 원점이 로봇과 어긋날 수 있음"
  fi
fi

# 3) Localization or SLAM
if [ "$MODE" = slam ]; then
  echo "3) SLAM ($ENGINE)"; start slam "$DIR/start_slam.sh" "$ENGINE"
else
  echo "3) 위치 추정 (맵 $MAP, AMCL 설정 $(basename "$AMCL_FILE"))"
  start nav2 "$DIR/start_nav2.sh" "$MAP"
fi
sleep 8

# 4) A* + bridge
echo "4) A* 플래너 + 브리지"; start bridge "$DIR/start_bridge.sh" "$NAME"
sleep 8

# 5) Sim + existing map: the start pose is known, so set the initial pose automatically
if [ "$ROBOT" = sim ] && [ "$MODE" = map ]; then
  echo "5) 초기 위치 자동 지정 ($SPAWN_X, $SPAWN_Y)"
  # The map frame may differ from the world (Cartographer maps etc.), so search the whole map if it doesn't fit well
  python3 "$HOME_DIR/tools/localize.py" "$SPAWN_X" "$SPAWN_Y" 0 --range 0.3 --yaw-range 20 --fallback-global 2>&1 \
    | grep -v -i warn | sed 's/^/   /' || true
elif [ "$MODE" = map ]; then
  echo "5) 웹에서 '초기 위치 지정'을 하거나: python3 tools/localize.py <x> <y> <방향°>"
else
  echo "5) SLAM: 로봇을 천천히 움직여 맵을 만드세요. 끝나면 웹 '맵 저장' 또는 scripts/save_map.sh <이름>"
fi
echo "== 완료. 웹: http://$(hostname -I | awk '{print $1}'):8000/   끄기: scripts/stop_ros.sh --all"
