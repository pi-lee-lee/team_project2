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
    # 실물·시뮬레이션 같은 설정. 시뮬레이션 로봇도 실물 LDS-02 사양(10 m, 약 1.45°, 안 맞은 빔 NaN)이라 따로 바꾸지 않는다
    # (예전에는 시뮬레이션 LiDAR 가 3.5 m 라 max_laser_range 를 3.4 로, 스캔 개수 기준 값도 따로 바꿔 띄웠음)
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
