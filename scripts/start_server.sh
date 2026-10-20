#!/bin/bash
# Django 서버 실행 (HTTP + WebSocket, 0.0.0.0:8000)
cd "$(dirname "$0")/../server" && exec .venv/bin/python manage.py runserver 0.0.0.0:8000
