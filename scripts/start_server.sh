#!/bin/bash
# Django 서버 실행 (HTTP + WebSocket, 0.0.0.0:8000)
# WAREHOUSE_SERVER_PY: 다른 가상환경의 python (Docker: /opt/warehouse/.venv/bin/python, stop_ros.sh 가 .venv/bin/python 으로 찾음)
cd "$(dirname "$0")/../server" && exec "${WAREHOUSE_SERVER_PY:-.venv/bin/python}" manage.py runserver 0.0.0.0:8000
