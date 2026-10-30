#!/bin/bash
# Start the Django server (HTTP + WebSocket, 0.0.0.0:8000)
# WAREHOUSE_SERVER_PY: python from another virtualenv (Docker: /opt/warehouse/.venv/bin/python; stop_ros.sh finds it by .venv/bin/python)
cd "$(dirname "$0")/../server" && exec "${WAREHOUSE_SERVER_PY:-.venv/bin/python}" manage.py runserver 0.0.0.0:8000
