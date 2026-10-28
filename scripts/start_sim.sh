#!/bin/bash
# Gazebo 시뮬레이션 (TurtleBot3 Waffle Pi). 실물 로봇의 bringup 대신 /scan, /odom, /tf 를 낸다.
# 실물과 같은 사양·같은 처리 경로로: 로봇은 worlds/models/turtlebot3_waffle_pi_real/model.sdf (LiDAR = 실물 LDS-02 측정값),
# scripts/sim_real_io.py 가 스캔을 실물 드라이버 형식으로, 오도메트리를 실물처럼 바퀴 엔코더 + IMU 로 내고,
# 실물 Pi 와 같은 명령 감시(scripts/pi_cmd_watchdog.py)가 브리지의 cmd_vel_pc 를 cmd_vel 로 넘긴다 (0.5초 끊기면 정지).
#   사용: scripts/start_sim.sh [월드] [--gui] [--x X --y Y]
#     월드: turtlebot3_world(기본) | turtlebot3_house | empty_world | 그 밖의 turtlebot3_gazebo/worlds/*.world
#           | 이 저장소 worlds/*.world (예: maze_room = 실제 방 복제, tools/map_to_world.py 로 생성)
#           | no_roof_small_warehouse / small_warehouse (AWS 창고, ~/sim_ws 에 받아 둔 경우)
#     --gui: Gazebo 화면도 띄움 (기본은 화면 없이 gzserver 만)
# 도메인 77 에서 실행되도록 WAREHOUSE_ROBOT=sim 으로 env.sh 를 읽는다.
export WAREHOUSE_ROBOT=sim
source "$(dirname "$0")/env.sh"

WORLD=turtlebot3_world
GUI=0
X=""
Y=""
while [ $# -gt 0 ]; do
  case "$1" in
    --gui) GUI=1 ;;
    --x) X="$2"; shift ;;
    --y) Y="$2"; shift ;;
    *) WORLD="$1" ;;
  esac
  shift
done
# 월드별 TurtleBot3 기본 출발 위치
if [ -z "$X" ]; then
  case "$WORLD" in
    turtlebot3_world|turtlebot3_house) X=-2.0; Y=-0.5 ;;
    *) X=0.0; Y=0.0 ;;
  esac
fi

GZ=$(ros2 pkg prefix turtlebot3_gazebo)/share/turtlebot3_gazebo
WORLD_FILE="$WAREHOUSE_HOME/worlds/$WORLD.world"
[ -f "$WORLD_FILE" ] || WORLD_FILE="$AWS_WAREHOUSE_DIR/worlds/$WORLD/$WORLD.world"
[ -f "$WORLD_FILE" ] || WORLD_FILE="$GZ/worlds/$WORLD.world"
[ -f "$WORLD_FILE" ] || { echo "[start_sim] 월드 파일 없음: $WORLD_FILE"; exit 1; }
# 모델 위치를 서비스로 바꿀 수 있게(/set_entity_state, tools/sim_mover.py) gazebo_ros_state 플러그인이 없으면 넣은 사본을 쓴다
if ! grep -q gazebo_ros_state "$WORLD_FILE"; then
  COPY="$WAREHOUSE_HOME/logs/world_$WORLD.world"
  sed '0,/<world name="[^"]*">/s//&\n    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so"><ros><namespace>\/<\/namespace><\/ros><update_rate>20.0<\/update_rate><\/plugin>/' \
    "$WORLD_FILE" > "$COPY"
  WORLD_FILE="$COPY"
fi
echo "[start_sim] 월드 $WORLD, 출발 위치 ($X, $Y), 화면 $([ $GUI = 1 ] && echo 표시 || echo 없음), 도메인 $ROS_DOMAIN_ID"

GZROS=$(ros2 pkg prefix gazebo_ros)/share/gazebo_ros/launch
ros2 launch "$GZROS/gzserver.launch.py" world:="$WORLD_FILE" &
PIDS=($!)
if [ "$GUI" = 1 ]; then
  DISPLAY="${DISPLAY:-:0}" ros2 launch "$GZROS/gzclient.launch.py" &
  PIDS+=($!)
fi
ros2 launch "$GZ/launch/robot_state_publisher.launch.py" use_sim_time:=true &
PIDS+=($!)
# 원본 spawn_turtlebot3.launch.py 와 같은 방식(이름 waffle_pi, 높이 0.01)이고 모델 파일만 실물 사양 사본
ros2 run gazebo_ros spawn_entity.py -entity "$TURTLEBOT3_MODEL" \
  -file "$WAREHOUSE_HOME/worlds/models/turtlebot3_waffle_pi_real/model.sdf" -x "$X" -y "$Y" -z 0.01 &
PIDS+=($!)
python3 "$WAREHOUSE_HOME/scripts/sim_real_io.py" &
PIDS+=($!)
python3 "$WAREHOUSE_HOME/scripts/pi_cmd_watchdog.py" --ros-args -p use_sim_time:=true &
PIDS+=($!)
trap 'kill -INT "${PIDS[@]}" 2>/dev/null; wait' INT TERM
wait
