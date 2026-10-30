#!/bin/bash
# Start SLAM (when building a new map). Publishes /map and the map->odom TF. Must not run together with AMCL (start_nav2.sh).
#   Usage: scripts/start_slam.sh [slam_toolbox|cartographer]
#     slam_toolbox (default): map update about every 5 s, stable loop closure
#     cartographer          : TurtleBot3 official example setup, map update about every 1 s
# Save the map: scripts/save_map.sh <name>  (or "맵 저장" on the web)
source "$(dirname "$0")/env.sh"
ENGINE="${1:-slam_toolbox}"
echo "[start_slam] $ENGINE, 시뮬레이션 시간 $WAREHOUSE_SIM_TIME, 도메인 $ROS_DOMAIN_ID"

case "$ENGINE" in
  slam_toolbox)
    # config/slam_toolbox.yaml: only scans taken while level (/scan_slam, published by the bridge, inf beams -> free space)
    # Same settings for real and sim: the sim robot also has the real LDS-02 spec (10 m, about 1.45°, no-return = NaN), so nothing is overridden
    PARAMS="$WAREHOUSE_HOME/config/slam_toolbox.yaml"
    grep -E '^\s*(max_laser_range|minimum_travel_distance|minimum_travel_heading|scan_buffer_size|loop_match_minimum_chain_size):' "$PARAMS" \
      | sed 's/#.*//; s/^\s*/[start_slam] /'
    exec ros2 launch slam_toolbox online_async_launch.py use_sim_time:="$WAREHOUSE_SIM_TIME" \
      slam_params_file:="$PARAMS"
    ;;
  cartographer)
    exec ros2 launch turtlebot3_cartographer cartographer.launch.py \
      use_sim_time:="$WAREHOUSE_SIM_TIME" use_rviz:=false
    ;;
  *)
    echo "[start_slam] 알 수 없는 SLAM: $ENGINE (slam_toolbox | cartographer)"
    exit 1
    ;;
esac
