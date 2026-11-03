#!/bin/bash
# Gazebo simulation (TurtleBot3 Waffle Pi). Publishes /scan, /odom, /tf in place of the real robot's bringup.
# Same spec and same processing path as the real robot: the robot is worlds/models/turtlebot3_waffle_pi_real/model.sdf (LiDAR = measured LDS-02 values),
# scripts/sim_real_io.py publishes scans in the real driver's format and odometry from wheel encoders + IMU like the real robot,
# and the same cmd watchdog as on the Pi (scripts/pi_cmd_watchdog.py) forwards the bridge's cmd_vel_pc to cmd_vel (stops after a 0.5 s gap).
#   Usage: scripts/start_sim.sh [world] [--gui] [--x X --y Y]
#     world: turtlebot3_world (default) | turtlebot3_house | empty_world | any other turtlebot3_gazebo/worlds/*.world
#            | this repo's worlds/*.world (e.g. maze_room = replica of the real room, generated with tools/map_to_world.py)
#            | no_roof_small_warehouse / small_warehouse (AWS warehouse, if downloaded to ~/sim_ws)
#     --gui: also show the Gazebo window (default: headless gzserver only)
# Source env.sh with WAREHOUSE_ROBOT=sim so everything runs in domain 77.
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
# Default TurtleBot3 start pose per world
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
# So model poses can be set via service (/set_entity_state, tools/sim_mover.py): if the world lacks the gazebo_ros_state plugin, use a copy with it added
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
# Same as the stock spawn_turtlebot3.launch.py (name waffle_pi, height 0.01), only the model file is the real-spec copy
ros2 run gazebo_ros spawn_entity.py -entity "$TURTLEBOT3_MODEL" \
  -file "$WAREHOUSE_HOME/worlds/models/turtlebot3_waffle_pi_real/model.sdf" -x "$X" -y "$Y" -z 0.01 &
PIDS+=($!)
python3 "$WAREHOUSE_HOME/scripts/sim_real_io.py" &
PIDS+=($!)
python3 "$WAREHOUSE_HOME/scripts/pi_cmd_watchdog.py" --ros-args -p use_sim_time:=true &
PIDS+=($!)
trap 'kill -INT "${PIDS[@]}" 2>/dev/null; wait' INT TERM
wait
