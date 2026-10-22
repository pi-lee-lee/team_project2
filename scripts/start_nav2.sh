#!/bin/bash
# 위치 추정 실행 (기존 맵): Nav2 의 map_server + AMCL (localization_launch.py). /map 과 map->odom TF 를 낸다.
#   사용: scripts/start_nav2.sh <map.yaml>
# 주행은 브리지가 직접 한다 (A* + pure pursuit / 장애물 근처 DWA). Nav2 주행 노드는 쓰지 않는다.
source "$(dirname "$0")/env.sh"
MAP="$1"
# 위치 추정(AMCL)은 이 프로젝트 설정을 쓴다 (TurtleBot3 기본값은 작은 방에서 10cm 넘게 어긋남, config/amcl_waffle_pi.yaml 참고)
# run.sh --amcl hall 이면 logs/run_mode.env 의 WAREHOUSE_AMCL = config/amcl_hall.yaml (강당/교실용). NAV2_PARAMS 로 다른 파일
LOC_PARAMS="${NAV2_PARAMS:-${WAREHOUSE_AMCL:-$WAREHOUSE_HOME/config/amcl_waffle_pi.yaml}}"
LAUNCH="$(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/launch"

# 맵 파일이 없으면 map_server가 조용히 unconfigured 상태로 남아 겉보기엔 정상처럼 보이므로 미리 막는다
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
