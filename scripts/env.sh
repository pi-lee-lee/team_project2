# Before running this project's ROS 2 nodes on the PC: source scripts/env.sh
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export WAREHOUSE_HOME="$(cd "$SCRIPT_DIR/.." && pwd)"
source /opt/ros/humble/setup.bash
[ -f "$HOME/turtlebot3_ws/install/setup.bash" ] && source "$HOME/turtlebot3_ws/install/setup.bash"
# WAREHOUSE_ROS_INSTALL: use an install built elsewhere (Docker: /opt/warehouse/ros2_ws/install inside the container)
source "${WAREHOUSE_ROS_INSTALL:-$WAREHOUSE_HOME/ros2_ws/install}/setup.bash"
export TURTLEBOT3_MODEL=waffle_pi

# Run mode (written by scripts/run.sh to logs/run_mode.env).
#   WAREHOUSE_ROBOT=real|sim, WAREHOUSE_MODE=map|slam, WAREHOUSE_SIM_TIME=true|false
# Real robot uses domain 44, simulation 77 (so they never mix even with the Pi on)
[ -f "$WAREHOUSE_HOME/logs/run_mode.env" ] && source "$WAREHOUSE_HOME/logs/run_mode.env"
export WAREHOUSE_ROBOT="${WAREHOUSE_ROBOT:-real}"
export WAREHOUSE_MODE="${WAREHOUSE_MODE:-map}"
if [ "$WAREHOUSE_ROBOT" = sim ]; then
  export ROS_DOMAIN_ID=77
  export WAREHOUSE_SIM_TIME=true
  # Add the Gazebo default model path (sun, ground_plane) first and disable the online model database
  # (otherwise gzserver waits to download from models.gazebosim.org and spawning the robot fails)
  [ -f /usr/share/gazebo/setup.sh ] && source /usr/share/gazebo/setup.sh
  export GAZEBO_MODEL_DATABASE_URI=""
  export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH:+$GAZEBO_MODEL_PATH:}$(ros2 pkg prefix turtlebot3_gazebo 2>/dev/null)/share/turtlebot3_gazebo/models"
  # AWS RoboMaker Small Warehouse (optional: ~/sim_ws/src/aws-robomaker-small-warehouse-world, ros2 branch)
  export AWS_WAREHOUSE_DIR="$HOME/sim_ws/src/aws-robomaker-small-warehouse-world"
  if [ -d "$AWS_WAREHOUSE_DIR/models" ]; then
    export GAZEBO_MODEL_PATH="$GAZEBO_MODEL_PATH:$AWS_WAREHOUSE_DIR/models"
    # The warehouse models reference meshes via relative file://models/... paths, so the package folder must be on the
    # resource path for collision shapes to exist (otherwise LiDAR rays pass straight through the shelves)
    export GAZEBO_RESOURCE_PATH="${GAZEBO_RESOURCE_PATH:+$GAZEBO_RESOURCE_PATH:}$AWS_WAREHOUSE_DIR"
  fi
else
  export ROS_DOMAIN_ID=44
  export WAREHOUSE_SIM_TIME=false
fi

# The PC uses Cyclone DDS (the Pi stays on Fast DDS; they interoperate)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# Settings used when switching back to Fast DDS (shared memory off, UDP only)
export FASTRTPS_DEFAULT_PROFILES_FILE="$SCRIPT_DIR/fastdds_udp.xml"
