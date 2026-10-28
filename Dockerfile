# 시뮬레이션 전체를 컨테이너 하나로: Django 서버 + Gazebo(TurtleBot3 Waffle Pi) + SLAM/AMCL + A* + 브리지
#   빌드: docker build -t warehouse-sim .
#   실행: docker run --rm -it -p 8000:8000 -v warehouse-maps:/root/maps -v warehouse-data:/data warehouse-sim
#         (기본: --robot sim --mode map, turtlebot3_world). 다른 모드는 뒤에 run.sh 인자를 붙인다:
#         docker run ... warehouse-sim --mode slam --world turtlebot3_house
#         docker run ... warehouse-sim none   (서버만 띄우고 웹 "실행 모드" 카드에서 시작)
#   웹: http://localhost:8000/
# 실물 로봇(Pi SSH), YOLO(torch), Gazebo 화면(--gui), AWS 창고 월드는 넣지 않았다.
# Gazebo classic (gazebo_ros, turtlebot3_gazebo) 의 Humble 패키지는 amd64 만 있다 → amd64 고정
# (Ubuntu PC 는 그대로, Apple Silicon Mac 은 에뮬레이션으로 느리게 동작)
FROM --platform=linux/amd64 ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    TURTLEBOT3_MODEL=waffle_pi

# ROS 쪽 파이썬 라이브러리(numpy, scipy 등)는 apt 것을 쓴다 (서버 venv 와 따로)
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-rmw-cyclonedds-cpp \
      ros-humble-gazebo-ros-pkgs \
      ros-humble-turtlebot3-gazebo \
      ros-humble-turtlebot3-navigation2 \
      ros-humble-nav2-bringup \
      ros-humble-nav2-map-server \
      ros-humble-nav2-amcl \
      ros-humble-slam-toolbox \
      ros-humble-robot-state-publisher \
      ros-humble-tf2-ros-py \
      python3-colcon-common-extensions \
      python3-numpy python3-scipy python3-matplotlib python3-pil python3-websocket python3-yaml \
      python3-venv python3-pip \
      curl procps util-linux iproute2 tini xvfb \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY . /app
# .gitattributes 이전에 Windows 에서 받은 저장소면 스크립트가 CRLF 일 수 있으므로 LF 로 바꾸고 실행 권한을 준다
RUN find scripts docker tools -name '*.sh' -exec sed -i 's/\r$//' {} + \
    && chmod +x scripts/*.sh docker/*.sh

# Django 서버 가상환경 (start_server.sh 가 server/.venv/bin/python 을 씀). DB 는 SQLite 라 mysqlclient 는 뺀다
RUN python3 -m venv server/.venv \
    && grep -v '^mysqlclient' server/requirements.txt > /tmp/requirements.txt \
    && server/.venv/bin/pip install --no-cache-dir -r /tmp/requirements.txt

# ROS 패키지 빌드 (저장소의 build/install 은 PC 경로라 .dockerignore 로 빼고 여기서 새로 만든다)
RUN . /opt/ros/humble/setup.sh && cd ros2_ws && colcon build --symlink-install

# DB_ENGINE=sqlite: server/.env 의 MySQL 대신 SQLite (/data 볼륨). WAREHOUSE_YOLO=0: 시뮬레이션엔 카메라 검출기 없음
ENV DB_ENGINE=sqlite \
    SQLITE_PATH=/data/db.sqlite3 \
    WAREHOUSE_YOLO=0
VOLUME ["/root/maps", "/data"]
EXPOSE 8000

ENTRYPOINT ["/usr/bin/tini", "--", "/app/docker/entrypoint.sh"]
CMD ["--mode", "map"]
