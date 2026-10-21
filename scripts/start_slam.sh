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
    # max_laser_range = LiDAR 최대 거리 - 0.1 (sim TB3 LiDAR 3.5m → 3.4, 실물 LDS-02 8m → 7.9. SLAM_MAX_RANGE 로 바꿀 수 있음)
    # 브리지가 못 맞힌 빔을 '최대 거리 - 0.01' 로 넘기면, Karto 가 이 값 이상 빔을 '여기까지 비어 있음' 으로 칠한다
    # (최대 거리 이상인 빔은 Karto 가 버리므로 max_laser_range 가 최대 거리와 같으면 아무것도 안 칠해짐)
    RANGE="${SLAM_MAX_RANGE:-$([ "$WAREHOUSE_ROBOT" = real ] && echo 7.9 || echo 3.4)}"
    PARAMS="$WAREHOUSE_HOME/logs/slam_toolbox_run.yaml"
    sed "s/^    max_laser_range: .*/    max_laser_range: $RANGE/" "$WAREHOUSE_HOME/config/slam_toolbox.yaml" > "$PARAMS"
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
