import os

from django.http import Http404, HttpResponse
from django.shortcuts import render

from .models import Map

STATIC_DIR = os.path.join(os.path.dirname(__file__), 'static', 'warehouse')


def index(request):
    # 정적 파일이 바뀌면 주소가 바뀌어 브라우저 캐시 대신 새 파일을 받도록
    version = int(max(os.path.getmtime(os.path.join(STATIC_DIR, f)) for f in ('app.js', 'style.css')))
    return render(request, 'warehouse/index.html', {'static_version': version})


def map_image(request, map_id):
    m = Map.objects.filter(id=map_id).first()
    if m is None:
        raise Http404
    return HttpResponse(bytes(m.png), content_type='image/png')
