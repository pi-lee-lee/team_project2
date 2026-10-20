from django.urls import path

from . import consumers

websocket_urlpatterns = [
    path('ws/robot/<str:name>/', consumers.RobotConsumer.as_asgi()),
    path('ws/ui/', consumers.UIConsumer.as_asgi()),
]
