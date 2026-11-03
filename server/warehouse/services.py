"""Task flow (state machine) logic.

Called from the consumers via database_sync_to_async. Each function changes the DB and
returns the messages to send in an Outbox; the consumer does the actual sending.

    Robot states: OFFLINE / IDLE / MOVING / WAITING_LOAD / ERROR
    Task flow: pickup locations (load) ... -> return home (move) -> done
    Order (start_order): home -> item sections (load, YOLO count check) -> dropoff (unload) -> home
"""
import base64
import math
import re
import time
import zlib
from collections import Counter
from datetime import timedelta

from django.db import transaction
from django.utils import timezone

from . import planning
from .live import LIVE
from .mapimage import encode_png, grid_to_rows
from .models import EventLog, Location, Map, Robot, Task, TaskStep, Zone

DEFAULT_ROBOT = 'tb3'


class ServiceError(Exception):
    """Error returned only to the requesting UI."""


class Outbox:
    def __init__(self):
        self.robot = []  # (robot_name, message)
        self.ui = []     # message (broadcast to all UIs)

    def to_robot(self, robot, msg):
        self.robot.append((robot.name, msg))

    def to_ui(self, msg):
        self.ui.append(msg)


# ---------------------------------------------------------------- common

def get_robot(name=None):
    robot, _ = Robot.objects.get_or_create(name=name or DEFAULT_ROBOT)
    return robot


ROBOT_NAME_RE = re.compile(r'^[A-Za-z0-9_-]{1,50}$')   # robot name = bridge URL /ws/robot/<name>/ (e.g. tb3, tb3_1)


def robot_list():
    """Web robot selection list: every robot in the DB (any name a bridge has ever connected with) + whether its bridge is connected."""
    return [{'name': r.name, 'state': r.state, 'message': r.message,
             'bridge': bool(LIVE.get(r.name, {}).get('health', {}).get('bridge'))}
            for r in Robot.objects.order_by('name')]


def known_robot(name):
    """The web-selected robot name if it is in the DB, else the default robot (so a previously selected robot that no longer exists doesn't create an empty one)."""
    return name if name and Robot.objects.filter(name=name).exists() else DEFAULT_ROBOT


def moving_robots():
    return list(Robot.objects.filter(state=Robot.STATE_MOVING).values_list('name', flat=True))


def active_map():
    return Map.objects.filter(is_active=True).first()


def active_task(robot):
    return robot.tasks.filter(status=Task.STATUS_RUNNING).first()


LOG_DEDUP_SEC = 10   # Log the same text only once within this time (so rapidly repeated failures don't flood the log)


def log(out, robot, message):
    since = timezone.now() - timedelta(seconds=LOG_DEDUP_SEC)
    if EventLog.objects.filter(message=message, created_at__gte=since).exists():
        return
    entry = EventLog.objects.create(robot=robot, message=message)
    out.to_ui({'type': 'log', 'log': entry.to_dict()})


# Prevent repeated failures at the same point: if moves to the same place (within the radius) fail this many times within this time, reject requests to that point for a while
REPEAT_FAIL_SEC, REPEAT_FAIL_COUNT, REPEAT_FAIL_RADIUS = 30.0, 3, 0.3
RECENT_FAILS = {}   # robot_name -> [(time, x, y)]


def recent_fails(robot_name, x, y):
    now = time.monotonic()
    fails = [f for f in RECENT_FAILS.get(robot_name, []) if now - f[0] < REPEAT_FAIL_SEC]
    RECENT_FAILS[robot_name] = fails
    return sum(1 for _, fx, fy in fails if math.hypot(fx - x, fy - y) < REPEAT_FAIL_RADIUS)


def check_goal_allowed(robot, x, y):
    """Reject goals inside keep-out zones or repeatedly failing points immediately, without sending them to the planner."""
    m = active_map()
    zone = next((z for z in m.zones.all() if planning.zone_contains(z.to_dict(), (x, y))), None) if m else None
    if zone is not None:
        raise ServiceError(f'목표가 금지 구역 "{zone.name}" 안에 있습니다. 구역 밖을 고르세요.')
    if recent_fails(robot.name, x, y) >= REPEAT_FAIL_COUNT:
        raise ServiceError(f'같은 지점({x:.2f}, {y:.2f})이 {REPEAT_FAIL_SEC:.0f}초 안에 {REPEAT_FAIL_COUNT}번 실패했습니다. '
                           '잠시 뒤 다시 시도하거나 다른 곳을 고르세요.')


def set_state(robot, state, message=''):
    robot.state = state
    robot.message = message
    robot.save(update_fields=['state', 'message'])


def push_state(out, robot):
    task = active_task(robot) or robot.tasks.first()
    out.to_ui({'type': 'state', 'robot': robot.to_dict(),
               'task': task.to_dict() if task else None})


def push_locations(out):
    m = active_map()
    locs = [l.to_dict() for l in m.locations.all()] if m else []
    out.to_ui({'type': 'locations', 'locations': locs})


def zones_of(m):
    return [z.to_dict() for z in m.zones.all()] if m else []


def push_zones(out, m=None):
    """Send keep-out zones to the UI and the robots (A* planner)."""
    m = m or active_map()
    zones = zones_of(m)
    out.to_ui({'type': 'zones', 'zones': zones})
    for robot in Robot.objects.all():   # to the A* planner of every robot using the same map
        out.to_robot(robot, {'type': 'zones', 'zones': zones})


def snapshot(robot_name=None):
    """Full state sent when a UI first connects."""
    robot = get_robot(robot_name)
    m = active_map()
    task = active_task(robot) or robot.tasks.first()
    return {
        'type': 'snapshot',
        'robot': robot.to_dict(),
        'task': task.to_dict() if task else None,
        'map': m.to_dict() if m else None,
        'locations': [l.to_dict() for l in m.locations.all()] if m else [],
        'zones': zones_of(m),
        'logs': [e.to_dict() for e in EventLog.objects.all()[:30]],
    }


def require_online(robot):
    if robot.state == Robot.STATE_OFFLINE:
        raise ServiceError('로봇이 연결되어 있지 않습니다.')


def send_goal(out, robot, step):
    step.status = TaskStep.STATUS_MOVING
    step.save(update_fields=['status'])
    # Record for travel time learning (start pose, path length/turning, predicted time)
    planning.record_start(active_map(), step, LIVE.get(robot.name, {}).get('pose'))
    out.to_robot(robot, {'type': 'goto', 'goal_id': step.goal_id,
                         'x': step.x, 'y': step.y, 'yaw': step.yaw})
    set_state(robot, Robot.STATE_MOVING, f'{step.name}(으)로 이동 중')
    log(out, robot, f'{step.name}(으)로 이동 시작')


def finish_task(out, robot, task, status, message):
    task.status = status
    task.finished_at = timezone.now()
    task.save(update_fields=['status', 'finished_at'])
    log(out, robot, message)
    if is_order(task):   # Turn off YOLO that an order turned on when the order ends (leave it if it was turned on with the web YOLO button)
        yolo_power(out, robot, False)


def yolo_power(out, robot, on):
    """Orders: YOLO is on only while stopped at a section/dropoff. It is off while driving so the camera stream doesn't use Wi-Fi
    (the detector subscribes to the image only while enabled) and the PC CPU is left to the driving computations."""
    out.to_robot(robot, {'type': 'yolo_set', 'enabled': bool(on)})


def advance(out, robot, task):
    """Advance to the next step. If no steps remain, the task is done."""
    step = task.current_step()
    if step is None:
        finish_task(out, robot, task, Task.STATUS_DONE, f'작업 #{task.id} 완료')
        set_state(robot, Robot.STATE_IDLE)
    else:
        send_goal(out, robot, step)


def create_task(robot, locations, with_home):
    task = Task.objects.create(robot=robot)
    order = 0
    for loc in locations:
        action = TaskStep.ACTION_MOVE if loc.kind == Location.KIND_HOME else TaskStep.ACTION_LOAD
        TaskStep.objects.create(task=task, order=order, action=action, location=loc,
                                name=loc.name, x=loc.x, y=loc.y, yaw=loc.yaw)
        order += 1
    if with_home:
        home = home_location()
        if home is None:
            raise ServiceError('홈 위치가 등록되어 있지 않습니다.')
        TaskStep.objects.create(task=task, order=order, action=TaskStep.ACTION_MOVE,
                                location=home, name=home.name,
                                x=home.x, y=home.y, yaw=home.yaw)
    return task


def home_location():
    m = active_map()
    return m.locations.filter(kind=Location.KIND_HOME).first() if m else None


def cancel_active(out, robot, reason):
    task = active_task(robot)
    if task is None:
        return
    out.to_robot(robot, {'type': 'cancel'})
    task.steps.filter(status__in=[TaskStep.STATUS_MOVING, TaskStep.STATUS_ARRIVED]) \
        .update(status=TaskStep.STATUS_FAILED)
    finish_task(out, robot, task, Task.STATUS_CANCELED, f'작업 #{task.id} 취소: {reason}')


# ---------------------------------------------------------------- robot -> server

@transaction.atomic
def robot_connected(name, active_goal_id, mode='map', session=''):
    out = Outbox()
    robot = get_robot(name)
    if mode == 'slam' and session:
        start_slam_session(out, robot, session)
    robot.last_seen = timezone.now()
    # Clear the battery values so a previous connection's values aren't shown.
    # The bridge fills them again when it receives /battery_state (simulation has no battery info, so they stay empty)
    robot.battery_voltage = None
    robot.battery_percentage = None
    robot.save(update_fields=['last_seen', 'battery_voltage', 'battery_percentage'])

    task = active_task(robot)
    step = task.current_step() if task else None
    if step is not None and step.status == TaskStep.STATUS_ARRIVED and step.items:
        yolo_power(out, robot, True)   # The detector starts disabled when the bridge restarts -> turn it back on if waiting
    if step is None:
        set_state(robot, Robot.STATE_IDLE)
    elif step.status == TaskStep.STATUS_ARRIVED and step.action == TaskStep.ACTION_UNLOAD:
        set_state(robot, Robot.STATE_WAITING_UNLOAD, f'{step.name} 수령 대기')
    elif step.status == TaskStep.STATUS_ARRIVED:
        set_state(robot, Robot.STATE_WAITING_LOAD, f'{step.name} 적재 대기')
    elif step.status == TaskStep.STATUS_MOVING and active_goal_id == step.goal_id:
        set_state(robot, Robot.STATE_MOVING, f'{step.name}(으)로 이동 중')
    elif task.steps.count() == 1:
        # A single move that lost its goal ends as failed so it doesn't block the next command
        step.status = TaskStep.STATUS_FAILED
        step.save(update_fields=['status'])
        finish_task(out, robot, task, Task.STATUS_FAILED, f'작업 #{task.id} 실패: 재연결로 이동이 끊김')
        set_state(robot, Robot.STATE_IDLE, '직전 이동이 연결 끊김으로 중단됨')
    else:
        set_state(robot, Robot.STATE_ERROR, '재연결됨: 진행 중이던 이동이 끊겼습니다. 재시도하세요.')
    log(out, robot, '로봇 연결됨')
    push_state(out, robot)
    return out


@transaction.atomic
def robot_health(name, robot_ok):
    """When the bridge is connected but the robot's (Pi) LiDAR/odometry drops out or comes back."""
    out = Outbox()
    robot = get_robot(name)
    if not robot_ok:
        if robot.state != Robot.STATE_OFFLINE:
            set_state(robot, Robot.STATE_OFFLINE, '로봇(Pi) 응답 없음: LiDAR/오도메트리가 들어오지 않습니다')
            log(out, robot, '로봇(Pi) 응답 없음')
    elif robot.state == Robot.STATE_OFFLINE:
        # Restore according to the running task's state (same handling as a bridge reconnect)
        goal_step = None
        task = active_task(robot)
        step = task.current_step() if task else None
        if step is not None and step.status == TaskStep.STATUS_MOVING:
            goal_step = step.goal_id
        return robot_connected(name, goal_step)
    push_state(out, robot)
    return out


@transaction.atomic
def robot_disconnected(name):
    out = Outbox()
    robot = get_robot(name)
    set_state(robot, Robot.STATE_OFFLINE, '연결 끊김')
    log(out, robot, '로봇 연결 끊김')
    push_state(out, robot)
    return out


@transaction.atomic
def start_slam_session(out, robot, session):
    """On a new SLAM run, delete the previous SLAM map and the locations registered on it.
    The SLAM map's origin is the robot pose when SLAM started, so the frame differs per run (world/start pose).
    Otherwise the previous run's locations would show up in the wrong places on the new map. Locations of saved maps were copied on save and remain.
    If the bridge merely reconnected (same run ID), leave it as is."""
    SLAM_SESSION[robot.name] = session
    m = Map.objects.filter(name=SLAM_MAP).first()
    if m is None or m.session == session:
        return
    n = m.locations.count()
    m.delete()
    out.to_ui({'type': 'map', 'map': None})
    out.to_ui({'type': 'locations', 'locations': []})
    log(out, robot, f'새 SLAM 실행: 이전 SLAM 맵{f"과 위치 {n}개" if n else ""}를 지웠습니다 (좌표계가 달라서)')


@transaction.atomic
def handle_map(name, msg):
    """Store the OccupancyGrid sent by the bridge as PNG and make it the active map."""
    out = Outbox()
    width, height = int(msg['width']), int(msg['height'])
    if width == 0 or height == 0:
        return out  # Don't store the empty map right after SLAM starts
    data = zlib.decompress(base64.b64decode(msg['data']))
    if len(data) != width * height:
        raise ServiceError('맵 데이터 크기가 맞지 않습니다.')
    png = encode_png(grid_to_rows(data, width, height), width, height)
    m = save_map(msg.get('name') or 'default', msg['resolution'],
                 msg['origin'][0], msg['origin'][1], width, height, png)
    if m.name == SLAM_MAP and SLAM_SESSION.get(name) and m.session != SLAM_SESSION[name]:
        m.session = SLAM_SESSION[name]
        m.save(update_fields=['session'])
    out.to_ui({'type': 'map', 'map': m.to_dict()})
    push_locations(out)
    # The bridge sends the map first on (re)connect, so send that map's keep-out zones to the robot here
    zones = zones_of(m)
    out.to_ui({'type': 'zones', 'zones': zones})
    out.to_robot(get_robot(name), {'type': 'zones', 'zones': zones})
    return out


def save_map(name, resolution, origin_x, origin_y, width, height, png, activate=True):
    m, _ = Map.objects.update_or_create(name=name, defaults=dict(
        resolution=resolution, origin_x=origin_x, origin_y=origin_y,
        width=width, height=height, png=png))
    if activate and not m.is_active:
        Map.objects.exclude(pk=m.pk).update(is_active=False)
        m.is_active = True
        m.save(update_fields=['is_active'])
    return m


@transaction.atomic
def handle_battery(name, voltage, percentage):
    out = Outbox()
    robot = get_robot(name)
    robot.battery_voltage = voltage
    robot.battery_percentage = percentage
    robot.last_seen = timezone.now()
    robot.save(update_fields=['battery_voltage', 'battery_percentage', 'last_seen'])
    push_state(out, robot)
    return out


@transaction.atomic
def handle_nav_status(name, goal_id, status, message=''):
    """status: succeeded / failed / canceled"""
    out = Outbox()
    robot = get_robot(name)
    task = active_task(robot)
    step = task.current_step() if task else None
    if step is None or step.goal_id != goal_id or step.status != TaskStep.STATUS_MOVING:
        return out  # Reply to an already canceled or old goal

    if status == 'succeeded':
        planning.record_arrival(step)
        if step.action == TaskStep.ACTION_LOAD:
            step.status = TaskStep.STATUS_ARRIVED
            step.save(update_fields=['status'])
            if step.items:
                yolo_power(out, robot, True)
                set_state(robot, Robot.STATE_WAITING_LOAD, f'{step.name} 도착, {items_text(step.items)} 실어 주세요')
                log(out, robot, f'{step.name} 도착 - {items_text(step.items)} 적재 대기 ("적재 완료" 또는 "다음 경유지로")')
            elif step.cargo:
                set_state(robot, Robot.STATE_WAITING_LOAD, f'{step.name} 도착, {step.cargo} 을(를) 실어 주세요')
                log(out, robot, f'{step.name} 도착 - {step.cargo} 적재 대기 (카메라 인식 또는 "실었어요")')
                # If already loaded (loaded before arrival), confirm right away
                if cargo_matches(LIVE.get(robot.name, {}).get('cargo'), step.cargo):
                    complete_cargo_step(out, robot, task, step, f'카메라: {step.cargo} 적재 확인')
            else:
                set_state(robot, Robot.STATE_WAITING_LOAD, f'{step.name} 도착, 적재 대기')
                log(out, robot, f'{step.name} 도착 - 적재 후 확인을 눌러주세요')
        elif step.action == TaskStep.ACTION_UNLOAD:
            step.status = TaskStep.STATUS_ARRIVED
            step.save(update_fields=['status'])
            what = items_text(step.items) if step.items else f'{step.cargo} 을(를)'
            if step.items:
                yolo_power(out, robot, True)
            set_state(robot, Robot.STATE_WAITING_UNLOAD, f'{step.name} 도착, {what} 꺼내 주세요')
            log(out, robot, f'{step.name} 도착 - {what} 수령 대기 ("꺼냈어요" 또는 "강제 보내기")')
        else:
            step.status = TaskStep.STATUS_DONE
            step.save(update_fields=['status'])
            log(out, robot, f'{step.name} 도착')
            advance(out, robot, task)
    elif status == 'failed':
        step.status = TaskStep.STATUS_FAILED
        step.save(update_fields=['status'])
        RECENT_FAILS.setdefault(robot.name, []).append((time.monotonic(), step.x, step.y))
        log(out, robot, f'{step.name} 이동 실패: {message}')
        if task.steps.count() == 1:
            # A single move (move-here, move to one location) ends as failed right away so it doesn't block the next command
            finish_task(out, robot, task, Task.STATUS_FAILED, f'작업 #{task.id} 실패')
            set_state(robot, Robot.STATE_IDLE, f'직전 이동 실패: {message}')
        else:
            # Multi-location tasks wait for retry/skip/cancel
            set_state(robot, Robot.STATE_ERROR, f'{step.name} 이동 실패: {message}')
    # canceled was requested by the server, so ignore it
    push_state(out, robot)
    return out


# ---------------------------------------------------------------- UI -> server

@transaction.atomic
def start_task(robot_name, location_ids, with_home=True):
    out = Outbox()
    robot = get_robot(robot_name)
    require_online(robot)
    if active_task(robot):
        raise ServiceError('진행 중인 작업이 있습니다. 먼저 취소하세요.')
    m = active_map()
    by_id = {l.id: l for l in Location.objects.filter(map=m, id__in=location_ids)}
    locations = [by_id[i] for i in location_ids if i in by_id]
    if not locations:
        raise ServiceError('이동할 위치를 선택하세요.')
    task = create_task(robot, locations, with_home)
    log(out, robot, f'작업 #{task.id} 시작: ' + ' → '.join(s.name for s in task.steps.all()))
    advance(out, robot, task)
    push_state(out, robot)
    return out


@transaction.atomic
def goto_point(robot_name, x, y, yaw, replace=False):
    """Move to an arbitrary unregistered point (done on arrival).
    replace: if the running task is a single point move, replace it with this goal. No cancel is sent to the robot,
    so it keeps going without stopping (the bridge drops the previous goal when it receives a new goto). Used by autonomous exploration."""
    out = Outbox()
    robot = get_robot(robot_name)
    require_online(robot)
    check_goal_allowed(robot, x, y)
    task = active_task(robot)
    if task:
        steps = list(task.steps.all())
        single_move = len(steps) == 1 and steps[0].action == TaskStep.ACTION_MOVE and steps[0].location_id is None
        if not (replace and single_move):
            raise ServiceError('진행 중인 작업이 있습니다. 먼저 취소하세요.')
        task.steps.filter(status__in=[TaskStep.STATUS_MOVING, TaskStep.STATUS_ARRIVED]) \
            .update(status=TaskStep.STATUS_FAILED)
        finish_task(out, robot, task, Task.STATUS_CANCELED, f'작업 #{task.id}: 다음 지점으로 이어 감')
    task = Task.objects.create(robot=robot)
    TaskStep.objects.create(task=task, order=0, action=TaskStep.ACTION_MOVE,
                            name=f'지정 지점({x:.2f}, {y:.2f})', x=x, y=y, yaw=yaw)
    advance(out, robot, task)
    push_state(out, robot)
    return out


@transaction.atomic
def go_home(robot_name):
    out = Outbox()
    robot = get_robot(robot_name)
    require_online(robot)
    if home_location() is None:
        raise ServiceError('홈 위치가 등록되어 있지 않습니다.')
    cancel_active(out, robot, '홈 복귀 요청')
    task = create_task(robot, [], with_home=True)
    advance(out, robot, task)
    push_state(out, robot)
    return out


@transaction.atomic
def confirm_load(robot_name):
    out = Outbox()
    robot = get_robot(robot_name)
    task = active_task(robot)
    step = task.current_step() if task else None
    if step is None or step.status != TaskStep.STATUS_ARRIVED:
        raise ServiceError('적재 대기 중인 위치가 없습니다.')
    if step.items and not order_check(out, robot, task, step):
        return out
    step.status = TaskStep.STATUS_DONE
    step.save(update_fields=['status'])
    log(out, robot, f'{step.name} 적재 완료')
    if step.items:
        yolo_power(out, robot, False)   # Leaving for the next destination: off while driving
    advance(out, robot, task)
    push_state(out, robot)
    return out


@transaction.atomic
def confirm_unload(robot_name):
    """Manual unload confirmation (the tablet's unload button, when the camera doesn't detect it)."""
    out = Outbox()
    robot = get_robot(robot_name)
    task = active_task(robot)
    step = task.current_step() if task else None
    if step is None or step.status != TaskStep.STATUS_ARRIVED or step.action != TaskStep.ACTION_UNLOAD:
        raise ServiceError('수령 대기 중인 배달이 없습니다.')
    if step.items and not order_check(out, robot, task, step):
        return out
    complete_cargo_step(out, robot, task, step, f'{step.name} 수령 완료')
    push_state(out, robot)
    return out


# ---------------------------------------------------------------- LED delivery
# Recipient requests an LED -> robot goes to the LED location and a person loads it (checked by camera) -> robot goes to the
# requested point and the recipient takes it out (disappears from the camera) -> home. One request at a time; new ones are rejected while running.

def cargo_matches(label, expected):
    return bool(label) and bool(expected) and label.strip().lower() == expected.strip().lower()


def complete_cargo_step(out, robot, task, step, message):
    step.status = TaskStep.STATUS_DONE
    step.save(update_fields=['status'])
    log(out, robot, message)
    if step.items:
        yolo_power(out, robot, False)   # Leaving for the next destination: off while driving
    advance(out, robot, task)


@transaction.atomic
def start_delivery(robot_name, location_id, x, y, yaw):
    """location_id: location where the LED is placed, (x, y, yaw): point the recipient clicked on the map."""
    out = Outbox()
    robot = get_robot(robot_name)
    require_online(robot)
    task = active_task(robot)
    if task:
        raise ServiceError(f'다른 요청(작업 #{task.id})을 처리 중입니다. 끝난 뒤 다시 요청하세요.')
    loc = Location.objects.filter(map=active_map(), id=location_id).first()
    if loc is None or loc.kind == Location.KIND_HOME:
        raise ServiceError('요청할 물건(LED) 위치를 고르세요.')
    check_goal_allowed(robot, x, y)
    if LIVE.get(robot.name, {}).get('cargo'):
        raise ServiceError(f'로봇에 이미 물건({LIVE[robot.name]["cargo"]})이 실려 있습니다. 먼저 꺼내 주세요.')
    label = loc.cargo_label()
    task = Task.objects.create(robot=robot)
    TaskStep.objects.create(task=task, order=0, action=TaskStep.ACTION_LOAD, location=loc,
                            name=loc.name, x=loc.x, y=loc.y, yaw=loc.yaw, cargo=label)
    TaskStep.objects.create(task=task, order=1, action=TaskStep.ACTION_UNLOAD,
                            name=f'요청 위치({x:.2f}, {y:.2f})', x=x, y=y, yaw=yaw, cargo=label)
    home = home_location()
    if home:
        TaskStep.objects.create(task=task, order=2, action=TaskStep.ACTION_MOVE, location=home,
                                name=home.name, x=home.x, y=home.y, yaw=home.yaw)
    log(out, robot, f'{label} 요청 (작업 #{task.id}): {loc.name} 에서 싣고 요청 위치로'
                    f'{" → " + home.name if home else ""}')
    advance(out, robot, task)
    push_state(out, robot)
    return out


@transaction.atomic
def handle_cargo(name, label):
    """When the camera detection result changes (label: visible item label, '' = nothing)."""
    out = Outbox()
    robot = get_robot(name)
    task = active_task(robot)
    step = task.current_step() if task else None
    if step is None or step.status != TaskStep.STATUS_ARRIVED or not step.cargo or step.items:
        return out  # Order steps advance only by button (YOLO counts are checked when the button is pressed)
    if step.action == TaskStep.ACTION_LOAD:
        if cargo_matches(label, step.cargo):
            complete_cargo_step(out, robot, task, step, f'카메라: {step.cargo} 적재 확인')
        elif label:
            log(out, robot, f'카메라: {label} 이(가) 실렸습니다. 요청한 것은 {step.cargo} 입니다')
    elif step.action == TaskStep.ACTION_UNLOAD and not label:
        complete_cargo_step(out, robot, task, step, f'카메라: {step.cargo} 수령 확인 (꺼냄)')
    push_state(out, robot)
    return out


# ---------------------------------------------------------------- orders (multiple items)
# Order = dropoff point + {label: count} -> for each label find the section holding that item (Location.cargo, several
# comma-separated labels), group by section and visit in the fastest order. At a section, the person there loads all of that
# section's items and presses load-done -> next if the YOLO counts match, otherwise "wrong load" (the tablet asks the person;
# if they confirm, force_next(confirmed) sends it on). At the dropoff the items are taken out and unload-done is pressed ->
# home if no ordered item is visible, otherwise asked the same way.

YOLO_WINDOW = 3.0   # On a button press, judge by the most frequent counts over the last this many seconds (guards against single-frame flicker)


def is_order(task):
    return task.steps.exclude(items={}).exists()


def items_text(items):
    return ', '.join(f'{k} {v}개' for k, v in items.items())


def parse_counts(text):
    """YOLO counts text "DHT11: 0 / led_red: 1 / ..." -> {'DHT11': 0, 'led_red': 1, ...}"""
    out = {}
    for part in (text or '').split('/'):
        name, sep, n = part.partition(':')
        if sep and name.strip() and n.strip().isdigit():
            out[name.strip()] = int(n.strip())
    return out


def yolo_counts(robot_name):
    """The most frequent counts text received in the last YOLO_WINDOW seconds (dict). None if none (YOLO off / no image)."""
    now = time.monotonic()
    texts = [t for at, t in LIVE.get(robot_name, {}).get('yolo_hist', []) if now - at <= YOLO_WINDOW and t]
    if not texts:
        return None
    return parse_counts(Counter(texts).most_common(1)[0][0])


def expected_counts(task, step):
    """Counts to check at this step. Load: only the items to load at this section (earlier sections' items are ignored, so an
    under-loaded earlier section doesn't affect this one), unload: all ordered labels are 0."""
    if step.action == TaskStep.ACTION_UNLOAD:
        return {k: 0 for k in step.items}
    return {k: int(v) for k, v in step.items.items()}


def check_counts(expected, counts):
    """The single place for the match rule: only ordered labels are checked and must match exactly (other items/excess are not checked).
    Returns (ok, 'label seen/expected ...')."""
    lower = {k.lower(): v for k, v in counts.items()}
    ok, parts = True, []
    for label, want in expected.items():
        have = lower.get(label.lower(), 0)
        ok &= have == want
        if want == 0:   # unload: list only what remains
            if have:
                parts.append(f'{label} {have}개 남음')
        else:
            parts.append(f'{label} {have}/{want}{"" if have == want else " ✗"}')
    return ok, ', '.join(parts) or '모두 꺼냄'


def order_check(out, robot, task, step):
    """Check the YOLO counts when the load-done/unload-done button is pressed. On mismatch report it in the state message and return False."""
    counts = yolo_counts(robot.name)
    is_load = step.action == TaskStep.ACTION_LOAD
    state = Robot.STATE_WAITING_LOAD if is_load else Robot.STATE_WAITING_UNLOAD
    if counts is None:
        msg = 'YOLO 개수를 못 받았습니다 (검출기 꺼짐 또는 영상 없음). 확인 후 강제로 보내세요'
        set_state(robot, state, msg)
        log(out, robot, f'{step.name}: {msg}')
        push_state(out, robot)
        return False
    ok, detail = check_counts(expected_counts(task, step), counts)
    if ok:
        log(out, robot, f'{step.name} YOLO 확인: {detail}')
        return True
    msg = f'{"적재 잘못" if is_load else "꺼내기 덜 됨"}: {detail}'
    set_state(robot, state, msg)
    log(out, robot, f'{step.name} {msg}')
    push_state(out, robot)
    return False


def area_labels(loc):
    """Item labels at a section (comma-separated in the detection label field; empty = the location name)."""
    return [x.strip() for x in loc.cargo_label().split(',') if x.strip()]


def find_area(m, label):
    """The section holding that item (the first one found if several; assumes sections don't share items)."""
    for loc in m.locations.filter(kind=Location.KIND_PICKUP):
        if label.strip().lower() in (x.lower() for x in area_labels(loc)):
            return loc
    return None


def order_route(robot, dropoff_id, items):
    """Validate an order and compute the visit order. Returns (dropoff, [(section, label, count) in order], optimize result, home)."""
    m = active_map()
    if m is None:
        raise ServiceError('맵이 없습니다.')
    try:
        items = {str(k).strip(): int(v) for k, v in (items or {}).items() if int(v) > 0}
    except (TypeError, ValueError):
        raise ServiceError('개수는 숫자로 입력하세요.')
    if not items:
        raise ServiceError('주문할 물건 개수를 1개 이상 입력하세요.')
    dropoff = Location.objects.filter(map=m, id=dropoff_id, kind=Location.KIND_DROPOFF).first()
    if dropoff is None:
        raise ServiceError('받을 사람 위치를 고르세요 (위치 종류 "사람 위치").')
    home = home_location()
    if home is None:
        raise ServiceError('홈 위치가 등록되어 있지 않습니다.')
    areas = {}
    for label in items:
        loc = find_area(m, label)
        if loc is None:
            raise ServiceError(f'{label} 이(가) 있는 섹션이 없습니다 (물류 위치의 인식 라벨에 {label} 를 넣으세요).')
        areas.setdefault(loc.id, (loc, {}))[1][label] = items[label]
    locs = [loc for loc, _ in areas.values()]
    try:
        r = planning.optimize(m, LIVE.get(robot.name, {}).get('pose'), locs, dropoff)
    except planning.PlanError as e:
        raise ServiceError(str(e))
    return dropoff, [(loc, areas[loc.id][1]) for loc in r['order']], r, home


def preview_order(robot_name, dropoff_id, items):
    """Order visit sequence and estimated time (does not change the DB). The dropoff -> home leg doesn't depend on the order, so it isn't counted."""
    robot = get_robot(robot_name)
    dropoff, route, r, home = order_route(robot, dropoff_id, items)
    names = [home.name] + [loc.name for loc, _ in route] + [dropoff.name, home.name]
    return {'type': 'order_preview', 'names': names, 'total': r['total'],
            'stops': [{'name': loc.name, 'items': it} for loc, it in route]}


@transaction.atomic
def start_order(robot_name, dropoff_id, items):
    out = Outbox()
    robot = get_robot(robot_name)
    require_online(robot)
    task = active_task(robot)
    if task:
        raise ServiceError(f'다른 작업(#{task.id})을 처리 중입니다. 끝난 뒤 다시 주문하세요.')
    dropoff, route, _, home = order_route(robot, dropoff_id, items)
    all_items = {}
    task = Task.objects.create(robot=robot)
    for order, (loc, it) in enumerate(route):
        TaskStep.objects.create(task=task, order=order, action=TaskStep.ACTION_LOAD, location=loc,
                                name=loc.name, x=loc.x, y=loc.y, yaw=loc.yaw, items=it)
        all_items.update(it)
    n = len(route)
    TaskStep.objects.create(task=task, order=n, action=TaskStep.ACTION_UNLOAD, location=dropoff,
                            name=dropoff.name, x=dropoff.x, y=dropoff.y, yaw=dropoff.yaw, items=all_items)
    TaskStep.objects.create(task=task, order=n + 1, action=TaskStep.ACTION_MOVE, location=home,
                            name=home.name, x=home.x, y=home.y, yaw=home.yaw)
    yolo_power(out, robot, False)   # Departing: turned on on arrival at a section
    if not (LIVE.get(robot.name, {}).get('yolo') or {}).get('available'):
        log(out, robot, '주의: YOLO 검출기가 없습니다. 개수 확인 없이 "다음 경유지로/강제 보내기"로만 진행됩니다')
    log(out, robot, f'주문 (작업 #{task.id}) {dropoff.name}: {items_text(all_items)} - '
                    + ' → '.join(s.name for s in task.steps.all()))
    advance(out, robot, task)
    push_state(out, robot)
    return out


@transaction.atomic
def force_next(robot_name, confirmed=False):
    """Advance an order step without the YOLO check. confirmed: a person checked after being asked about a YOLO mismatch;
    otherwise the next-stop button (YOLO off etc.). The counts at that moment are logged."""
    out = Outbox()
    robot = get_robot(robot_name)
    task = active_task(robot)
    step = task.current_step() if task else None
    if step is None or step.status != TaskStep.STATUS_ARRIVED:
        raise ServiceError('적재/수령 대기 중인 단계가 없습니다.')
    counts = yolo_counts(robot.name)
    detail = check_counts(expected_counts(task, step), counts)[1] if counts is not None and step.items else 'YOLO 개수 없음'
    how = 'YOLO 와 다름, 사람이 확인하고 진행' if confirmed else '확인 없이 진행'
    complete_cargo_step(out, robot, task, step, f'{step.name} {how} (YOLO: {detail})')
    push_state(out, robot)
    return out


@transaction.atomic
def retry_step(robot_name):
    out = Outbox()
    robot = get_robot(robot_name)
    require_online(robot)
    task = active_task(robot)
    step = task.current_step() if task else None
    if step is None or step.status not in (TaskStep.STATUS_FAILED, TaskStep.STATUS_MOVING):
        raise ServiceError('재시도할 이동이 없습니다.')
    send_goal(out, robot, step)
    push_state(out, robot)
    return out


@transaction.atomic
def skip_step(robot_name):
    out = Outbox()
    robot = get_robot(robot_name)
    require_online(robot)
    task = active_task(robot)
    step = task.current_step() if task else None
    if step is None:
        raise ServiceError('건너뛸 단계가 없습니다.')
    out.to_robot(robot, {'type': 'cancel'})
    step.status = TaskStep.STATUS_SKIPPED
    step.save(update_fields=['status'])
    log(out, robot, f'{step.name} 건너뜀')
    if step.items:
        yolo_power(out, robot, False)
    advance(out, robot, task)
    push_state(out, robot)
    return out


@transaction.atomic
def cancel_task(robot_name):
    out = Outbox()
    robot = get_robot(robot_name)
    if not active_task(robot):
        raise ServiceError('진행 중인 작업이 없습니다.')
    cancel_active(out, robot, '사용자 요청')
    if robot.state != Robot.STATE_OFFLINE:
        set_state(robot, Robot.STATE_IDLE)
    push_state(out, robot)
    return out


@transaction.atomic
def estop(robot_name):
    out = Outbox()
    robot = get_robot(robot_name)
    out.to_robot(robot, {'type': 'stop'})
    cancel_active(out, robot, '비상정지')
    if robot.state != Robot.STATE_OFFLINE:
        set_state(robot, Robot.STATE_ERROR, '비상정지됨')
    log(out, robot, '비상정지')
    push_state(out, robot)
    return out


@transaction.atomic
def reset_error(robot_name):
    out = Outbox()
    robot = get_robot(robot_name)
    if robot.state == Robot.STATE_ERROR and not active_task(robot):
        set_state(robot, Robot.STATE_IDLE)
    push_state(out, robot)
    return out


@transaction.atomic
def set_initial_pose(robot_name, x, y, yaw, precise=False):
    """precise: pose found by scan matching (auto localization) -> give AMCL a small uncertainty."""
    out = Outbox()
    robot = get_robot(robot_name)
    require_online(robot)
    out.to_robot(robot, {'type': 'initial_pose', 'x': float(x), 'y': float(y), 'yaw': float(yaw),
                         'precise': bool(precise)})
    log(out, robot, f'초기 위치 지정 ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f}°)')
    return out


@transaction.atomic
def save_location(data):
    """Update if data has an id, otherwise add."""
    out = Outbox()
    m = active_map()
    if m is None:
        raise ServiceError('먼저 맵을 불러오세요.')
    name = (data.get('name') or '').strip()
    if not name:
        raise ServiceError('위치 이름을 입력하세요.')
    kind = data.get('kind', Location.KIND_PICKUP)
    if kind not in (Location.KIND_PICKUP, Location.KIND_HOME, Location.KIND_DROPOFF):
        raise ServiceError('잘못된 위치 종류입니다.')
    fields = dict(name=name, kind=kind, x=float(data['x']), y=float(data['y']),
                  yaw=float(data.get('yaw', 0.0)))
    if 'cargo' in data:  # camera detection label (empty = location name)
        fields['cargo'] = str(data.get('cargo') or '').strip()[:200]
    if kind == Location.KIND_PICKUP:   # An item may be in only one section (so an order maps to a single section)
        old = Location.objects.filter(map=m, id=data['id']).first() if data.get('id') else None
        cargo = fields['cargo'] if 'cargo' in fields else (old.cargo if old else '')
        mine = {x.lower() for x in area_labels(Location(name=name, cargo=cargo))}
        for other in m.locations.filter(kind=Location.KIND_PICKUP).exclude(id=data.get('id') or None):
            dup = sorted(x for x in area_labels(other) if x.lower() in mine)
            if dup:
                raise ServiceError(f'{", ".join(dup)} 은(는) 이미 섹션 "{other.name}" 에 있습니다. 같은 물건은 한 섹션에만 둘 수 있습니다.')
    if data.get('id'):
        loc = Location.objects.filter(map=m, id=data['id']).first()
        if loc is None:
            raise ServiceError('위치를 찾을 수 없습니다.')
        for k, v in fields.items():
            setattr(loc, k, v)
        loc.save()
    else:
        loc = Location.objects.create(map=m, **fields)
    if kind == Location.KIND_HOME:  # only one home
        m.locations.filter(kind=Location.KIND_HOME).exclude(pk=loc.pk) \
            .update(kind=Location.KIND_PICKUP)
    push_locations(out)
    return out


@transaction.atomic
def set_home(x, y, yaw):
    """Set the home location to (x, y, yaw). If it exists, only move it."""
    out = Outbox()
    m = active_map()
    if m is None:
        raise ServiceError('먼저 맵을 불러오세요.')
    home = m.locations.filter(kind=Location.KIND_HOME).first()
    if home is None:
        home = Location(map=m, name='홈', kind=Location.KIND_HOME)
    home.x, home.y, home.yaw = float(x), float(y), float(yaw)
    home.save()
    log(out, None, f'홈 위치 지정 ({home.x:.2f}, {home.y:.2f}, {math.degrees(home.yaw):.0f}°)')
    push_locations(out)
    return out


@transaction.atomic
def delete_location(location_id):
    out = Outbox()
    Location.objects.filter(id=location_id).delete()
    push_locations(out)
    return out


@transaction.atomic
def save_zone(data):
    """Add a keep-out zone. data: {shape: rect|line, x1, y1, x2, y2, name?}"""
    out = Outbox()
    m = active_map()
    if m is None:
        raise ServiceError('먼저 맵을 불러오세요.')
    shape = data.get('shape', Zone.SHAPE_RECT)
    if shape not in (Zone.SHAPE_RECT, Zone.SHAPE_LINE):
        raise ServiceError('잘못된 금지 구역 모양입니다.')
    x1, y1, x2, y2 = (float(data[k]) for k in ('x1', 'y1', 'x2', 'y2'))
    if not all(math.isfinite(v) for v in (x1, y1, x2, y2)):
        raise ServiceError('잘못된 좌표입니다.')
    if math.hypot(x2 - x1, y2 - y1) < 0.1:
        raise ServiceError('금지 구역이 너무 작습니다 (10cm 이상 끌어서 그리세요).')
    n = m.zones.count() + 1
    name = (data.get('name') or '').strip()[:100] or f'{"가상 벽" if shape == Zone.SHAPE_LINE else "금지 구역"} {n}'
    Zone.objects.create(map=m, name=name, shape=shape, x1=x1, y1=y1, x2=x2, y2=y2)
    log(out, None, f'{name} 추가 ({x1:.2f}, {y1:.2f})-({x2:.2f}, {y2:.2f})')
    push_zones(out, m)
    return out


@transaction.atomic
def delete_zone(zone_id):
    out = Outbox()
    z = Zone.objects.filter(id=zone_id).first()
    if z is None:
        raise ServiceError('금지 구역을 찾을 수 없습니다.')
    m = z.map
    z.delete()
    log(out, None, f'{z.name} 삭제')
    push_zones(out, m)
    return out


# ---------------------------------------------------------------- map building (SLAM mode)

MAP_NAME_RE = re.compile(r'^[A-Za-z0-9_-]{1,50}$')
PENDING_SAVE = {}  # robot_name -> map name requested for saving
SLAM_MAP = 'slam_live'
SLAM_SESSION = {}  # robot_name -> SLAM run ID of the currently connected bridge


@transaction.atomic
def request_save_map(robot_name, name):
    out = Outbox()
    robot = get_robot(robot_name)
    require_online(robot)
    if not MAP_NAME_RE.match(name):
        raise ServiceError('맵 이름은 영문/숫자/_/- 만 쓸 수 있습니다 (예: office_1f).')
    out.to_robot(robot, {'type': 'save_map', 'name': name})
    PENDING_SAVE[robot.name] = name
    log(out, robot, f'맵 저장 요청: {name}')
    return out


@transaction.atomic
def map_saved(robot_name, ok, message):
    out = Outbox()
    robot = get_robot(robot_name)
    log(out, robot, ('맵 저장 완료: ' if ok else '맵 저장 실패: ') + message[:200])
    name = PENDING_SAVE.pop(robot.name, None)
    if ok and name:
        copied = carry_over_locations(name)
        if copied is not None:
            log(out, robot, f'SLAM 중 등록한 위치 {copied}개를 맵 "{name}" 으로 옮겼습니다')
    return out


def carry_over_locations(name):
    """Register a saved SLAM map file as a DB map (not activated) and copy the locations registered on slam_live.
    The map was saved in the same SLAM frame, so coordinates are reused as is; but if the map was rotated on save to straighten it
    (the yaml's '# rotation_deg: <deg>', tools/straighten_map.py), locations/zones are rotated about the origin as well."""
    import math
    import os
    import re
    from .management.commands.import_map import parse_yaml
    from .mapimage import read_pgm
    path = os.path.expanduser(f'~/maps/{name}_clean.yaml')
    if not os.path.exists(path):
        path = os.path.expanduser(f'~/maps/{name}.yaml')
    if not os.path.exists(path):
        return None
    conf = parse_yaml(path)
    with open(path, encoding='utf-8') as f:
        found = re.search(r'^#\s*rotation_deg:\s*([-+0-9.]+)', f.read(), re.M)
    t = math.radians(float(found.group(1))) if found else 0.0
    c, s = math.cos(t), math.sin(t)

    def rot(x, y):
        return c * x - s * y, s * x + c * y
    width, height, rows = read_pgm(os.path.join(os.path.dirname(path), conf['image'].strip('\'"')))
    origin = [float(v) for v in conf['origin'].strip('[]').split(',')]
    m = save_map(name, float(conf['resolution']), origin[0], origin[1], width, height,
                 encode_png(rows, width, height), activate=False)
    src = Map.objects.filter(name='slam_live').first()
    if src is None:
        return 0
    copied = 0
    for loc in src.locations.all():
        x, y = rot(loc.x, loc.y)
        yaw = math.atan2(math.sin(loc.yaw + t), math.cos(loc.yaw + t))
        _, created = Location.objects.update_or_create(
            map=m, name=loc.name, defaults=dict(kind=loc.kind, x=x, y=y, yaw=yaw, cargo=loc.cargo))
        copied += 1
    for z in src.zones.all():   # Zones share the frame. Rectangles are map-axis aligned, so use the box enclosing the four rotated corners
        if z.shape == Zone.SHAPE_RECT and t:
            xs, ys = zip(*(rot(x, y) for x in (z.x1, z.x2) for y in (z.y1, z.y2)))
            x1, y1, x2, y2 = min(xs), min(ys), max(xs), max(ys)
        else:
            (x1, y1), (x2, y2) = rot(z.x1, z.y1), rot(z.x2, z.y2)
        Zone.objects.update_or_create(map=m, name=z.name, defaults=dict(
            shape=z.shape, x1=x1, y1=y1, x2=x2, y2=y2))
    return copied


# ---------------------------------------------------------------- state reset

@transaction.atomic
def reset_robot(robot_name):
    """Cancel the running task, set state to idle, and ask the robot to reset (simulation returns it to the start pose)."""
    out = Outbox()
    robot = get_robot(robot_name)
    require_online(robot)
    cancel_active(out, robot, '상태 초기화')
    out.to_robot(robot, {'type': 'reset'})
    set_state(robot, Robot.STATE_IDLE, '상태 초기화 중...')
    log(out, robot, '상태 초기화 요청')
    push_state(out, robot)
    return out


@transaction.atomic
def reset_done(robot_name, ok, message):
    out = Outbox()
    robot = get_robot(robot_name)
    if robot.state != Robot.STATE_OFFLINE:
        set_state(robot, Robot.STATE_IDLE if ok else Robot.STATE_ERROR, message[:255])
    log(out, robot, ('상태 초기화 완료: ' if ok else '상태 초기화 실패: ') + message[:200])
    push_state(out, robot)
    return out


# ---------------------------------------------------------------- run mode (runner.py)

def log_runner(job, ok, message):
    """Called from the runner thread: log the result and notify the UI."""
    from asgiref.sync import async_to_sync
    from channels.layers import get_channel_layer
    entry = EventLog.objects.create(robot=get_robot(), message=f'{job} {"완료" if ok else "실패"}: {message}'[:255])
    async_to_sync(get_channel_layer().group_send)('ui', {'type': 'send.msg', 'msg': {'type': 'log', 'log': entry.to_dict()}})


# ---------------------------------------------------------------- visit order optimization (planning.py)

def plan_order(robot_name, location_ids, with_home):
    """Fastest order to visit the selected locations and its estimated time. Does not change the DB."""
    robot = get_robot(robot_name)
    m = active_map()
    if m is None:
        raise ServiceError('맵이 없습니다.')
    by_id = {l.id: l for l in Location.objects.filter(map=m, id__in=location_ids)}
    locations = [by_id[i] for i in location_ids if i in by_id]
    if not locations:
        raise ServiceError('위치를 선택하세요.')
    home = home_location() if with_home else None
    if with_home and home is None:
        raise ServiceError('홈 위치가 등록되어 있지 않습니다.')
    try:
        r = planning.optimize(m, LIVE.get(robot.name, {}).get('pose'), locations, home)
    except planning.PlanError as e:
        raise ServiceError(str(e))
    names = [l.name for l in r['order']] + ([home.name] if home else [])
    return {'type': 'plan_order', 'order': [l.id for l in r['order']], 'names': names,
            'legs': r['legs'], 'total': r['total'], 'original_total': r['original_total'],
            'model': {k: r['model'][k] for k in ('summary', 'n', 'outliers', 'mae', 'learned')}}


def travel_model_info():
    info = planning.travel_model()
    return {'type': 'travel_model', **{k: info[k] for k in ('summary', 'n', 'outliers', 'mae', 'learned', 'w')}}


def teleop_check(robot_name):
    """Whether web teleop is allowed. Returns the robot name if allowed, otherwise ServiceError."""
    robot = get_robot(robot_name)
    require_online(robot)
    task = active_task(robot)
    if task:
        raise ServiceError(f'작업 #{task.id} 진행 중에는 수동 조작을 할 수 없습니다. 먼저 작업을 취소하세요.')
    return robot.name
