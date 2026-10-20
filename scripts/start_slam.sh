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
    # config/slam_toolbox.yaml: 수평일 때 스캔(/scan_level, 브리지가 발행)만 사용
    exec ros2 launch slam_toolbox online_async_launch.py use_sim_time:="$WAREHOUSE_SIM_TIME" \
      slam_params_file:="$WAREHOUSE_HOME/config/slam_toolbox.yaml"
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
