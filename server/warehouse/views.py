import os

from django.http import Http404, HttpResponse
from django.shortcuts import render

from .models import Map

STATIC_DIR = os.path.join(os.path.dirname(__file__), 'static', 'warehouse')


def static_version(*files):
    # 정적 파일이 바뀌면 주소가 바뀌어 브라우저 캐시 대신 새 파일을 받도록
    return int(max(os.path.getmtime(os.path.join(STATIC_DIR, f)) for f in files))


def index(request):
    return render(request, 'warehouse/index.html', {'static_version': static_version('app.js', 'style.css')})


def robot_screen(request):
    """로봇 위에 올린 태블릿/휴대폰용 화면 (같은 와이파이에서 http://<PC>:8000/robot)."""
    return render(request, 'warehouse/robot.html', {'static_version': static_version('robot.js', 'robot.css')})


def map_image(request, map_id):
    m = Map.objects.filter(id=map_id).first()
    if m is None:
        raise Http404
    return HttpResponse(bytes(m.png), content_type='image/png')
