# PC에서 이 프로젝트의 ROS 2 노드를 실행할 때: source scripts/env.sh
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export WAREHOUSE_HOME="$(cd "$SCRIPT_DIR/.." && pwd)"
source /opt/ros/humble/setup.bash
[ -f "$HOME/turtlebot3_ws/install/setup.bash" ] && source "$HOME/turtlebot3_ws/install/setup.bash"
source "$WAREHOUSE_HOME/ros2_ws/install/setup.bash"
export TURTLEBOT3_MODEL=waffle_pi

# 실행 모드 (scripts/run.sh 가 logs/run_mode.env 에 기록).
#   WAREHOUSE_ROBOT=real|sim, WAREHOUSE_MODE=map|slam, WAREHOUSE_SIM_TIME=true|false
# 실물 로봇은 도메인 44, 시뮬레이션은 77 (Pi가 켜져 있어도 섞이지 않게)
[ -f "$WAREHOUSE_HOME/logs/run_mode.env" ] && source "$WAREHOUSE_HOME/logs/run_mode.env"
export WAREHOUSE_ROBOT="${WAREHOUSE_ROBOT:-real}"
export WAREHOUSE_MODE="${WAREHOUSE_MODE:-map}"
if [ "$WAREHOUSE_ROBOT" = sim ]; then
  export ROS_DOMAIN_ID=77
  export WAREHOUSE_SIM_TIME=true
  # Gazebo 기본 모델(sun, ground_plane) 경로를 먼저 넣고, 인터넷 모델 저장소 조회는 끈다
  # (빠지면 gzserver가 models.gazebosim.org 에서 받으려고 기다리느라 로봇 생성이 실패함)
  [ -f /usr/share/gazebo/setup.sh ] && source /usr/share/gazebo/setup.sh
  export GAZEBO_MODEL_DATABASE_URI=""
  export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH:+$GAZEBO_MODEL_PATH:}$(ros2 pkg prefix turtlebot3_gazebo 2>/dev/null)/share/turtlebot3_gazebo/models"
  # AWS RoboMaker Small Warehouse (선택 설치: ~/sim_ws/src/aws-robomaker-small-warehouse-world, ros2 브랜치)
  export AWS_WAREHOUSE_DIR="$HOME/sim_ws/src/aws-robomaker-small-warehouse-world"
  if [ -d "$AWS_WAREHOUSE_DIR/models" ]; then
    export GAZEBO_MODEL_PATH="$GAZEBO_MODEL_PATH:$AWS_WAREHOUSE_DIR/models"
    # 창고 모델은 메시를 file://models/... 상대 경로로 가리키므로 패키지 폴더를 리소스 경로에 넣어야
    # 충돌 형상이 생긴다 (없으면 LiDAR 광선이 선반을 그냥 통과함)
    export GAZEBO_RESOURCE_PATH="${GAZEBO_RESOURCE_PATH:+$GAZEBO_RESOURCE_PATH:}$AWS_WAREHOUSE_DIR"
  fi
else
  export ROS_DOMAIN_ID=44
  export WAREHOUSE_SIM_TIME=false
fi

# PC 쪽은 Cyclone DDS 사용. Fast DDS에서는 Nav2 노드가 다른 노드의 TF를 처음 몇 개만 받고
# 끊기는 문제가 있었다 (Pi는 Fast DDS 그대로 두어도 서로 통신된다)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# Fast DDS로 되돌릴 때 쓰는 설정 (공유 메모리 끄고 UDP만 사용)
export FASTRTPS_DEFAULT_PROFILES_FILE="$SCRIPT_DIR/fastdds_udp.xml"
