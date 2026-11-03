import os

from django.http import Http404, HttpResponse
from django.shortcuts import render

from .models import Map

STATIC_DIR = os.path.join(os.path.dirname(__file__), 'static', 'warehouse')


def static_version(*files):
    # Changes the URL when a static file changes, so the browser fetches the new file instead of its cache
    return int(max(os.path.getmtime(os.path.join(STATIC_DIR, f)) for f in files))


def index(request):
    return render(request, 'warehouse/index.html', {'static_version': static_version('app.js', 'style.css')})


def robot_screen(request):
    """Screen for the tablet/phone mounted on the robot (http://<PC>:8000/robot on the same Wi-Fi)."""
    return render(request, 'warehouse/robot.html', {'static_version': static_version('robot.js', 'robot.css')})


def order_screen(request):
    """Screen for the orderer: pick your location and item counts, place an order and follow its progress (http://<PC>:8000/order)."""
    return render(request, 'warehouse/order.html', {'static_version': static_version('order.js', 'order.css')})


def map_image(request, map_id):
    m = Map.objects.filter(id=map_id).first()
    if m is None:
        raise Http404
    return HttpResponse(bytes(m.png), content_type='image/png')
