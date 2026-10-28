#!/bin/bash
# PC 쪽 ROS 2 노드와 Gazebo 시뮬레이션을 한 번에 정리한다 (Pi bringup은 건드리지 않음).
#   사용: scripts/stop_ros.sh [--server] [--daemon] [--all]
#     --server : Django 서버도 끈다
#     --daemon : ROS 2 데몬도 끈다 (ros2 node list 에 죽은 노드가 남아 보일 때)
#     --all    : --server --daemon
#
# 노드를 하나씩 끄면 lifecycle_manager/amcl 같은 짝이 부모 없이 남기 쉬워서,
# 실행 스크립트 → ros2 launch → 노드 순서로 SIGINT → SIGTERM → SIGKILL 을 보낸다.

STOP_SERVER=0
STOP_DAEMON=0
for arg in "$@"; do
  case "$arg" in
    --server) STOP_SERVER=1 ;;
    --daemon) STOP_DAEMON=1 ;;
    --all) STOP_SERVER=1; STOP_DAEMON=1 ;;
    --sim) ;;  # 예전 호환 (Gazebo 는 항상 정리)
    -h|--help) sed -n '2,9p' "$0"; exit 0 ;;
    *) echo "알 수 없는 옵션: $arg"; exit 1 ;;
  esac
done

# 이 스크립트 자신과 겹치지 않도록 실행 파일 경로로 시작하는 패턴만 쓴다
PATTERNS=(
  '^/bin/bash (\S*/)?scripts/start_(nav2|bridge|slam|sim)\.sh( |$)'
  '^/usr/bin/python3 /opt/ros/humble/bin/ros2 launch'
  '^/opt/ros/humble/lib/nav2_'
  '^/opt/ros/humble/lib/rclcpp_components/component_container'
  '^/opt/ros/humble/lib/(cartographer_ros|slam_toolbox)/'
  '^/usr/bin/python3 .*/warehouse_robot/lib/warehouse_robot/'
  '^gz(server|client)( |$)'
  '^(/usr/bin/)?python3 /opt/ros/humble/lib/gazebo_ros/'
  '^/opt/ros/humble/lib/robot_state_publisher/robot_state_publisher'
  '^/opt/ros/humble/lib/nav2_map_server/map_saver_cli'
  '^\S*python\S* -u \S*/yolo/yolo_training/infer_ros'   # scripts/start_yolo.sh (YOLO 검출기)
  '^\S*python3 \S*/scripts/(sim_real_io|pi_cmd_watchdog)\.py'   # start_sim.sh (시뮬레이션 실물 형식 출력, 명령 감시)
)
if [ "$STOP_SERVER" = 1 ]; then
  PATTERNS+=('^/bin/bash (\S*/)?scripts/start_server\.sh( |$)' '^(\S*/)?\.venv/bin/python manage\.py runserver')
fi
REGEX=$(IFS='|'; echo "${PATTERNS[*]}")

# 이 프로젝트 노드만: ROS_DOMAIN_ID 가 44(실물)/77(시뮬레이션)이거나 없는(웹 서버 등) 프로세스.
# 같은 PC 의 다른 프로젝트 Gazebo/노드(다른 도메인)는 건드리지 않는다.
ours() {
  local d
  d=$(tr '\0' '\n' < "/proc/$1/environ" 2>/dev/null | sed -n 's/^ROS_DOMAIN_ID=//p')
  [ -z "$d" ] || [ "$d" = 44 ] || [ "$d" = 77 ]
}
list() { for p in $(pgrep -f -- "$REGEX"); do ours "$p" && echo "$p"; done; }
show() { ps -o pid=,ppid=,cmd= -p "$1" 2>/dev/null | cut -c1-110; }

PIDS=$(list)
if [ -z "$PIDS" ]; then
  echo "끌 프로세스가 없습니다."
else
  echo "종료 대상:"
  for p in $PIDS; do echo "  $(show "$p")"; done
  for sig in INT TERM KILL; do
    PIDS=$(list)
    [ -z "$PIDS" ] && break
    echo "SIG$sig 보냄 ($(echo $PIDS | wc -w)개)"
    kill -s "$sig" $PIDS 2>/dev/null
    for _ in 1 2 3 4 5; do
      sleep 1
      [ -z "$(list)" ] && break
    done
  done
fi

if [ "$STOP_DAEMON" = 1 ]; then
  source "$(dirname "$0")/env.sh"
  ros2 daemon stop >/dev/null 2>&1 && echo "ROS 2 데몬 종료"
fi

LEFT=$(list)
if [ -z "$LEFT" ]; then
  echo "완료: 남은 프로세스 없음"
else
  echo "아직 남은 프로세스:"
  for p in $LEFT; do echo "  $(show "$p")"; done
  exit 1
fi
