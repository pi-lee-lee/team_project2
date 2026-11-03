#!/bin/bash
# Start localization (existing map): Nav2 map_server + AMCL (localization_launch.py). Publishes /map and the map->odom TF.
#   Usage: scripts/start_nav2.sh <map.yaml>
# Driving is done by the bridge itself (A* + Frenet local planner). Nav2 navigation nodes are not used.
source "$(dirname "$0")/env.sh"
MAP="$1"
# AMCL uses this project's settings (TurtleBot3 defaults are inaccurate in a small room, see config/amcl_waffle_pi.yaml)
# With run.sh --amcl hall, WAREHOUSE_AMCL in logs/run_mode.env = config/amcl_hall.yaml (halls/classrooms). NAV2_PARAMS selects another file
LOC_PARAMS="${NAV2_PARAMS:-${WAREHOUSE_AMCL:-$WAREHOUSE_HOME/config/amcl_waffle_pi.yaml}}"
LAUNCH="$(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/launch"

# Without a map file map_server silently stays unconfigured and looks fine, so check up front
if [ -z "$MAP" ]; then
  echo "[start_nav2] 맵 파일을 지정하세요: scripts/start_nav2.sh ~/maps/<이름>.yaml"
  exit 1
fi
if [ ! -f "$MAP" ]; then
  echo "[start_nav2] 맵 파일 없음: $MAP"
  exit 1
fi
echo "[start_nav2] 위치 추정: 맵 $MAP, 시뮬레이션 시간 $WAREHOUSE_SIM_TIME, 도메인 $ROS_DOMAIN_ID"
exec ros2 launch "$LAUNCH/localization_launch.py" map:="$MAP" params_file:="$LOC_PARAMS" \
  use_sim_time:="$WAREHOUSE_SIM_TIME" use_composition:=False
