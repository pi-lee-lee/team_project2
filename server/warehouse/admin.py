from django.contrib import admin

from .models import EventLog, Location, Map, Robot, Task, TaskStep


@admin.register(Map)
class MapAdmin(admin.ModelAdmin):
    list_display = ('name', 'width', 'height', 'resolution', 'is_active', 'updated_at')
    exclude = ('png',)


@admin.register(Location)
class LocationAdmin(admin.ModelAdmin):
    list_display = ('name', 'kind', 'map', 'x', 'y', 'yaw')
    list_filter = ('map', 'kind')


@admin.register(Robot)
class RobotAdmin(admin.ModelAdmin):
    list_display = ('name', 'state', 'message', 'battery_voltage', 'last_seen')


class TaskStepInline(admin.TabularInline):
    model = TaskStep
    extra = 0


@admin.register(Task)
class TaskAdmin(admin.ModelAdmin):
    list_display = ('id', 'robot', 'status', 'created_at', 'finished_at')
    inlines = [TaskStepInline]


@admin.register(EventLog)
class EventLogAdmin(admin.ModelAdmin):
    list_display = ('created_at', 'robot', 'message')
