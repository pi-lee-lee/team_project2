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
    # 시뮬레이션은 스캔 반영 간격도 예전(마지막으로 정상 확인한 10-21) 값 0.5 m / 0.5 rad 로:
    #   0.1 은 실물 먼 벽 점선/탐색 갱신용인데, slam_toolbox 의 scan_buffer_size·loop_match_minimum_chain_size 는 '스캔 개수' 라
    #   간격을 줄이면 맞춤 기준(약 1 m)과 루프 클로저 인정 구간(약 1.5 m)도 같이 줄어, LiDAR 3.5 m 인 넓은 house 에서
    #   map→odom 이 한 번에 2~8° 씩 튀며 맵이 무너졌음 (2026-10-28). 실물 방에서는 0.1 로 정상이라 실물은 그대로
    PARAMS="$WAREHOUSE_HOME/config/slam_toolbox.yaml"
    if [ "$WAREHOUSE_ROBOT" = sim ]; then
      sed -e 's/^\(\s*max_laser_range:\s*\)[0-9.]*/\13.4/' \
          -e 's/^\(\s*minimum_travel_distance:\s*\)[0-9.]*/\10.5/' \
          -e 's/^\(\s*minimum_travel_heading:\s*\)[0-9.]*/\10.5/' "$PARAMS" > "$WAREHOUSE_HOME/logs/slam_toolbox_run.yaml"
      PARAMS="$WAREHOUSE_HOME/logs/slam_toolbox_run.yaml"
    fi
    grep -E '^\s*(max_laser_range|minimum_travel_distance|minimum_travel_heading)' "$PARAMS" | sed 's/#.*//; s/^\s*/[start_slam] /'
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
