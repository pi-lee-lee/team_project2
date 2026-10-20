#!/bin/bash
# Gazebo 시뮬레이션 (TurtleBot3 Waffle Pi). 실물 로봇의 bringup 대신 /scan, /odom, /tf 를 낸다.
#   사용: scripts/start_sim.sh [월드] [--gui] [--x X --y Y]
#     월드: turtlebot3_world(기본) | turtlebot3_house | empty_world | 그 밖의 turtlebot3_gazebo/worlds/*.world
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
WORLD_FILE="$GZ/worlds/$WORLD.world"
[ -f "$WORLD_FILE" ] || { echo "[start_sim] 월드 파일 없음: $WORLD_FILE"; exit 1; }
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
ros2 launch "$GZ/launch/spawn_turtlebot3.launch.py" x_pose:="$X" y_pose:="$Y" &
PIDS+=($!)
trap 'kill -INT "${PIDS[@]}" 2>/dev/null; wait' INT TERM
wait
