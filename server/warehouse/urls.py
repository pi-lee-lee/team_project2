from django.urls import path

from . import views

urlpatterns = [
    path('', views.index, name='index'),
    path('map/<int:map_id>.png', views.map_image, name='map_image'),
]
