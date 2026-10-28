# 시뮬레이션 실행 환경 이미지: ROS 2 Humble + Gazebo(TurtleBot3 Waffle Pi) + SLAM/AMCL + Django 서버용 파이썬.
# 소스는 이미지에 넣지 않는다. 실행할 때 프로젝트 폴더를 /app 으로 연결해 그 소스로 돈다 (고친 코드가 바로 반영).
#   docker compose up --build        (compose.yaml: 프로젝트 폴더 → /app, wsim/logs → /app/logs)
#   웹: http://localhost:8000/        자세한 것은 DOCKER_GUIDE.md
# 컨테이너 시작 때 docker/entrypoint.sh 가 ros2_ws 를 컨테이너 안(/opt/warehouse/ros2_ws)에 다시 빌드한다
# (프로젝트의 ros2_ws/build|install 은 Ubuntu PC 경로를 가리켜 컨테이너에서 쓸 수 없음).
# 실물 로봇(Pi SSH), YOLO(torch), Gazebo 화면(--gui), AWS 창고 월드는 넣지 않았다.
# Gazebo classic (gazebo_ros, turtlebot3_gazebo) 의 Humble 패키지는 amd64 만 있다 → amd64 고정
# (Ubuntu/Windows PC 는 그대로, Apple Silicon Mac 은 에뮬레이션으로 느리게 동작)
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

# Django 서버 가상환경. 프로젝트의 server/.venv(Ubuntu PC 것)와 겹치지 않게 /opt 에 둔다.
# 경로를 .../.venv/bin/python 으로 끝나게 해서 stop_ros.sh 가 서버 프로세스를 찾을 수 있게 함. DB 는 SQLite 라 mysqlclient 는 뺀다
COPY server/requirements.txt /tmp/requirements.txt
RUN python3 -m venv /opt/warehouse/.venv \
    && grep -v '^mysqlclient' /tmp/requirements.txt > /tmp/req.txt \
    && /opt/warehouse/.venv/bin/pip install --no-cache-dir -r /tmp/req.txt

# WAREHOUSE_SERVER_PY / WAREHOUSE_ROS_INSTALL: start_server.sh, env.sh 가 프로젝트 폴더 대신 컨테이너 것을 쓰게 함
# DB_ENGINE=sqlite: server/.env 의 MySQL 대신 SQLite (/data 볼륨). WAREHOUSE_YOLO=0: 시뮬레이션엔 카메라 검출기 없음
# PYTHONDONTWRITEBYTECODE: 연결한 프로젝트 폴더에 root 소유 __pycache__ 를 만들지 않게
ENV WAREHOUSE_SERVER_PY=/opt/warehouse/.venv/bin/python \
    WAREHOUSE_ROS_INSTALL=/opt/warehouse/ros2_ws/install \
    DB_ENGINE=sqlite \
    SQLITE_PATH=/data/db.sqlite3 \
    WAREHOUSE_YOLO=0 \
    PYTHONDONTWRITEBYTECODE=1

WORKDIR /app
VOLUME ["/root/maps", "/data"]
EXPOSE 8000

ENTRYPOINT ["/usr/bin/tini", "--", "/app/docker/entrypoint.sh"]
CMD ["--mode", "map"]
