#!/bin/bash
# SLAM 실행 (맵을 새로 만들 때). /map 과 map->odom TF 를 낸다. AMCL(start_nav2.sh)과 같이 띄우면 안 된다.
#   사용: scripts/start_slam.sh [slam_toolbox|cartographer]
#     slam_toolbox (기본): 맵 갱신 약 5초, 루프 클로저 안정적
#     cartographer       : TurtleBot3 공식 예제 방식, 맵 갱신 약 1초
# 만든 맵 저장: scripts/save_map.sh <이름>  (또는 웹의 "맵 저장")
source "$(dirname "$0")/env.sh"
ENGINE="${1:-slam_toolbox}"
echo "[start_slam] $ENGINE, 시뮬레이션 시간 $WAREHOUSE_SIM_TIME, 도메인 $ROS_DOMAIN_ID"

case "$ENGINE" in
  slam_toolbox)
    # config/slam_toolbox.yaml: 수평일 때 스캔(/scan_slam, 브리지가 발행, inf 빔 → 빈 공간)만 사용
    # 시뮬레이션 LiDAR 는 3.5 m 까지 (inf → 3.45 로 바뀜) → max_laser_range 를 그보다 작은 3.4 로 (5.0 이면 3.45 를 벽으로 찍음)
    # 시뮬레이션은 스캔 개수로 정해지는 두 값을 5배로 (실물은 그대로):
    #   slam_toolbox 의 scan_buffer_size(맞춤에 쓰는 최근 스캔 수)·loop_match_minimum_chain_size(루프 클로저 인정 연속 스캔 수)는
    #   '개수' 라, 스캔 반영 간격을 0.5 → 0.1 로 줄이면서 맞춤 기준 약 5 m → 1 m, 루프 클로저 인정 구간 약 7.5 m → 1.5 m 로 약해져
    #   LiDAR 3.5 m 인 넓은 house 에서 map→odom 이 한 번에 2~8° 씩 튀며 맵이 무너졌음 (2026-10-28).
    #   간격을 0.5 로 되돌리면 안 무너지지만 벽이 0.5 m 움직일 때마다만 들어가 가까이 가야 맵 벽이 됨 →
    #   간격은 0.1 그대로 두고 개수를 10 → 50, 15 → 75 로 해서 거리 기준을 0.5 시절과 같게
    PARAMS="$WAREHOUSE_HOME/config/slam_toolbox.yaml"
    if [ "$WAREHOUSE_ROBOT" = sim ]; then
      sed -e 's/^\(\s*max_laser_range:\s*\)[0-9.]*/\13.4/' \
          -e 's/^\(\s*scan_buffer_size:\s*\)[0-9]*/\150/' \
          -e 's/^\(\s*loop_match_minimum_chain_size:\s*\)[0-9]*/\175/' "$PARAMS" > "$WAREHOUSE_HOME/logs/slam_toolbox_run.yaml"
      PARAMS="$WAREHOUSE_HOME/logs/slam_toolbox_run.yaml"
    fi
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
