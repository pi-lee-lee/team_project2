#!/bin/bash
# Nav2 실행.
#   사용: scripts/start_nav2.sh <map.yaml> [--with-navigation]   기존 맵: map_server + AMCL (+ 주행 노드)
#         scripts/start_nav2.sh --navigation-only                 SLAM 과 같이: 주행 노드만 (맵/위치는 SLAM 이 냄)
#
# 기본(controller=dwa, builtin 도 같음)은 브리지가 직접 주행하므로 위치 추정만 띄운다.
# 주행 노드(planner_server, controller_server, bt_navigator 등)는 브리지를
# planner:=nav2 또는 controller:=nav2 로 쓸 때 필요하다.
#   주의: 실물 로봇 환경에서 controller_server 안의 TF 수신이 멈춰 로봇 위치를 잘못 아는 문제가 있었다.
source "$(dirname "$0")/env.sh"
MAP=""
WITH_NAV=0
NAV_ONLY=0
for arg in "$@"; do
  case "$arg" in
    --with-navigation) WITH_NAV=1 ;;
    --navigation-only) WITH_NAV=1; NAV_ONLY=1 ;;
    *) MAP="$arg" ;;
  esac
done
# NAV2_PARAMS 로 다른 Nav2 설정 파일을 쓸 수 있음
PARAMS="${NAV2_PARAMS:-$(ros2 pkg prefix turtlebot3_navigation2)/share/turtlebot3_navigation2/param/humble/waffle_pi.yaml}"
# 위치 추정(AMCL)은 이 프로젝트 설정을 쓴다 (TurtleBot3 기본값은 작은 방에서 10cm 넘게 어긋남, config/amcl_waffle_pi.yaml 참고)
# run.sh --amcl hall 이면 logs/run_mode.env 의 WAREHOUSE_AMCL = config/amcl_hall.yaml (강당/교실용)
LOC_PARAMS="${NAV2_PARAMS:-${WAREHOUSE_AMCL:-$WAREHOUSE_HOME/config/amcl_waffle_pi.yaml}}"
LAUNCH="$(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/launch"

if [ "$NAV_ONLY" = 0 ]; then
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
  ros2 launch "$LAUNCH/localization_launch.py" map:="$MAP" params_file:="$LOC_PARAMS" \
    use_sim_time:="$WAREHOUSE_SIM_TIME" use_composition:=False &
  LOC_PID=$!
fi
trap 'kill -INT $LOC_PID $NAV_PID 2>/dev/null; wait' INT TERM

if [ "$WITH_NAV" = 1 ]; then
  # 주행 노드는 map->odom TF 가 나온 뒤에 띄운다 (먼저 띄우면 controller_server 가 TF 를 못 받는 경우가 있었음)
  if [ "$NAV_ONLY" = 1 ]; then
    echo "[start_nav2] map TF 대기 중... (SLAM 이 실행 중이어야 합니다)"
  else
    echo "[start_nav2] map TF 대기 중... 웹에서 '초기 위치 지정'을 하세요."
  fi
  until timeout 4 ros2 run tf2_ros tf2_echo map odom 2>/dev/null | grep -q "At time"; do sleep 1; done
  echo "[start_nav2] map TF 확인. 주행 노드를 시작합니다."
  ros2 launch "$LAUNCH/navigation_launch.py" params_file:="$PARAMS" \
    use_sim_time:="$WAREHOUSE_SIM_TIME" use_composition:=False &
  NAV_PID=$!
fi
wait
