"""작업 흐름(상태 머신) 로직.

consumer에서 database_sync_to_async로 호출한다. 각 함수는 DB를 바꾸고
보낼 메시지를 Outbox에 담아 돌려준다. 실제 전송은 consumer가 맡는다.

    로봇 상태: OFFLINE / IDLE / MOVING / WAITING_LOAD / ERROR
    작업 흐름: 물류 위치(load) ... → 홈 복귀(move) → 완료
"""
import base64
import math
import re
import time
import zlib
from datetime import timedelta

from django.db import transaction
from django.utils import timezone

from . import planning
from .live import LIVE
from .mapimage import encode_png, grid_to_rows
from .models import EventLog, Location, Map, Robot, Task, TaskStep, Zone

DEFAULT_ROBOT = 'tb3'


class ServiceError(Exception):
    """요청한 UI에만 돌려줄 오류."""


class Outbox:
    def __init__(self):
        self.robot = []  # (robot_name, message)
        self.ui = []     # message (모든 UI에 브로드캐스트)

    def to_robot(self, robot, msg):
        self.robot.append((robot.name, msg))

    def to_ui(self, msg):
        self.ui.append(msg)


# ---------------------------------------------------------------- 공통

def get_robot(name=None):
    robot, _ = Robot.objects.get_or_create(name=name or DEFAULT_ROBOT)
    return robot


def active_map():
    return Map.objects.filter(is_active=True).first()


def active_task(robot):
    return robot.tasks.filter(status=Task.STATUS_RUNNING).first()


LOG_DEDUP_SEC = 10   # 같은 문구는 이 시간 안에 한 번만 기록 (같은 실패가 빠르게 반복될 때 로그가 넘치지 않게)


def log(out, robot, message):
    since = timezone.now() - timedelta(seconds=LOG_DEDUP_SEC)
    if EventLog.objects.filter(message=message, created_at__gte=since).exists():
        return
    entry = EventLog.objects.create(robot=robot, message=message)
    out.to_ui({'type': 'log', 'log': entry.to_dict()})


# 같은 지점 반복 실패 막기: 이 시간 안에 같은 곳(반경 안) 이동이 이만큼 실패하면 그 지점 요청을 잠시 거절
REPEAT_FAIL_SEC, REPEAT_FAIL_COUNT, REPEAT_FAIL_RADIUS = 30.0, 3, 0.3
RECENT_FAILS = {}   # robot_name -> [(시각, x, y)]


def recent_fails(robot_name, x, y):
    now = time.monotonic()
    fails = [f for f in RECENT_FAILS.get(robot_name, []) if now - f[0] < REPEAT_FAIL_SEC]
    RECENT_FAILS[robot_name] = fails
    return sum(1 for _, fx, fy in fails if math.hypot(fx - x, fy - y) < REPEAT_FAIL_RADIUS)


def check_goal_allowed(robot, x, y):
    """금지 구역 안 목표, 같은 곳 반복 실패를 플래너까지 보내지 않고 바로 거절."""
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
    """금지 구역을 UI 와 로봇(A* 플래너)에 보낸다."""
    m = m or active_map()
    zones = zones_of(m)
    out.to_ui({'type': 'zones', 'zones': zones})
    out.to_robot(get_robot(), {'type': 'zones', 'zones': zones})


def snapshot(robot_name=None):
    """UI가 처음 접속했을 때 보낼 전체 상태."""
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
    # 이동 시간 학습용 기록 (출발 위치, 경로 길이/회전량, 예측 시간)
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


def advance(out, robot, task):
    """다음 단계로 진행. 남은 단계가 없으면 작업 완료."""
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


# ---------------------------------------------------------------- 로봇 → 서버

@transaction.atomic
def robot_connected(name, active_goal_id, mode='map', session=''):
    out = Outbox()
    robot = get_robot(name)
    if mode == 'slam' and session:
        start_slam_session(out, robot, session)
    robot.last_seen = timezone.now()
    # 이전 연결(예: 어제 실물 로봇)의 배터리 값이 남아 보이지 않도록 지운다.
    # 브리지가 /battery_state 를 받으면 다시 채운다 (시뮬레이션은 배터리 정보가 없어 비어 있음)
    robot.battery_voltage = None
    robot.battery_percentage = None
    robot.save(update_fields=['last_seen', 'battery_voltage', 'battery_percentage'])

    task = active_task(robot)
    step = task.current_step() if task else None
    if step is None:
        set_state(robot, Robot.STATE_IDLE)
    elif step.status == TaskStep.STATUS_ARRIVED:
        set_state(robot, Robot.STATE_WAITING_LOAD, f'{step.name} 적재 대기')
    elif step.status == TaskStep.STATUS_MOVING and active_goal_id == step.goal_id:
        set_state(robot, Robot.STATE_MOVING, f'{step.name}(으)로 이동 중')
    elif task.steps.count() == 1:
        # 단일 이동은 목표를 잃었으면 실패로 끝내서 다음 명령을 막지 않는다
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
    """브리지는 연결돼 있지만 로봇(Pi)의 LiDAR/오도메트리가 끊기거나 돌아왔을 때."""
    out = Outbox()
    robot = get_robot(name)
    if not robot_ok:
        if robot.state != Robot.STATE_OFFLINE:
            set_state(robot, Robot.STATE_OFFLINE, '로봇(Pi) 응답 없음: LiDAR/오도메트리가 들어오지 않습니다')
            log(out, robot, '로봇(Pi) 응답 없음')
    elif robot.state == Robot.STATE_OFFLINE:
        # 진행 중이던 작업 상태에 맞춰 복귀 (브리지 재연결과 같은 처리)
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
    """새 SLAM 실행이면 이전 SLAM 맵과 그 위에 등록한 위치를 지운다.
    SLAM 맵의 원점은 SLAM 을 시작한 순간의 로봇 위치라 실행(월드/출발 위치)마다 좌표계가 다르다.
    지우지 않으면 이전 실행의 위치가 새 지도 위 엉뚱한 곳에 보인다. 저장한 맵의 위치는 저장할 때 복사되어 남아 있다.
    브리지가 재연결만 한 것(같은 실행 번호)이면 그대로 둔다."""
    SLAM_SESSION[robot.name] = session
    m = Map.objects.filter(name=SLAM_MAP).first()
    if m is not None and m.session == session:
        return
    # 새 실행: 첫 위치를 받으면 로봇 정면이 화면 위쪽이 되게 화면 각도를 정한다 (set_slam_view)
    SLAM_VIEW.pop(robot.name, None)
    SLAM_VIEW_PENDING.add(robot.name)
    if m is None:
        return
    n = m.locations.count()
    m.delete()
    out.to_ui({'type': 'map', 'map': None})
    out.to_ui({'type': 'locations', 'locations': []})
    log(out, robot, f'새 SLAM 실행: 이전 SLAM 맵{f"과 위치 {n}개" if n else ""}를 지웠습니다 (좌표계가 달라서)')


@transaction.atomic
def handle_map(name, msg):
    """브리지가 보낸 OccupancyGrid를 PNG로 저장하고 활성 맵으로 지정."""
    out = Outbox()
    width, height = int(msg['width']), int(msg['height'])
    if width == 0 or height == 0:
        return out  # SLAM 시작 직후의 빈 맵은 저장하지 않는다
    data = zlib.decompress(base64.b64decode(msg['data']))
    if len(data) != width * height:
        raise ServiceError('맵 데이터 크기가 맞지 않습니다.')
    png = encode_png(grid_to_rows(data, width, height), width, height)
    m = save_map(msg.get('name') or 'default', msg['resolution'],
                 msg['origin'][0], msg['origin'][1], width, height, png)
    if m.name == SLAM_MAP and SLAM_SESSION.get(name) and m.session != SLAM_SESSION[name]:
        m.session = SLAM_SESSION[name]
        m.save(update_fields=['session'])
    if m.name == SLAM_MAP and name in SLAM_VIEW and m.view_yaw != SLAM_VIEW[name]:
        m.view_yaw = SLAM_VIEW[name]
        m.save(update_fields=['view_yaw'])
    out.to_ui({'type': 'map', 'map': m.to_dict()})
    push_locations(out)
    # 브리지가 (재)연결하면 맵부터 보내므로, 여기서 그 맵의 금지 구역을 로봇에 알린다
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
        return out  # 이미 취소됐거나 지난 목표에 대한 응답

    if status == 'succeeded':
        planning.record_arrival(step)
        if step.action == TaskStep.ACTION_LOAD:
            step.status = TaskStep.STATUS_ARRIVED
            step.save(update_fields=['status'])
            if step.cargo:
                set_state(robot, Robot.STATE_WAITING_LOAD, f'{step.name} 도착, {step.cargo} 을(를) 실어 주세요')
                log(out, robot, f'{step.name} 도착 - {step.cargo} 적재 대기 (카메라 인식 또는 "실었어요")')
                # 이미 실려 있으면 (도착 전에 실음) 바로 확인
                if cargo_matches(LIVE.get(robot.name, {}).get('cargo'), step.cargo):
                    complete_cargo_step(out, robot, task, step, f'카메라: {step.cargo} 적재 확인')
            else:
                set_state(robot, Robot.STATE_WAITING_LOAD, f'{step.name} 도착, 적재 대기')
                log(out, robot, f'{step.name} 도착 - 적재 후 확인을 눌러주세요')
        elif step.action == TaskStep.ACTION_UNLOAD:
            step.status = TaskStep.STATUS_ARRIVED
            step.save(update_fields=['status'])
            set_state(robot, Robot.STATE_WAITING_UNLOAD, f'{step.name} 도착, {step.cargo} 을(를) 꺼내 주세요')
            log(out, robot, f'{step.name} 도착 - {step.cargo} 수령 대기 (카메라 인식 또는 "꺼냈어요")')
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
            # 단일 이동("여기로 이동", 위치 하나로 이동)은 바로 실패로 끝내서 다음 명령을 막지 않는다
            finish_task(out, robot, task, Task.STATUS_FAILED, f'작업 #{task.id} 실패')
            set_state(robot, Robot.STATE_IDLE, f'직전 이동 실패: {message}')
        else:
            # 여러 위치 작업은 재시도/건너뛰기/취소를 기다린다
            set_state(robot, Robot.STATE_ERROR, f'{step.name} 이동 실패: {message}')
    # canceled는 서버가 요청한 경우이므로 무시
    push_state(out, robot)
    return out


# ---------------------------------------------------------------- UI → 서버

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
    """등록하지 않은 임의의 지점으로 이동 (도착하면 바로 완료).
    replace: 진행 중인 작업이 '지정 지점' 이동 하나뿐이면 그것을 이 목표로 바꾼다. 로봇에는 취소를 보내지 않으므로
    멈추지 않고 이어서 간다 (브리지는 새 goto 를 받으면 이전 목표를 버림). 자동 탐색이 씀."""
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
    step.status = TaskStep.STATUS_DONE
    step.save(update_fields=['status'])
    log(out, robot, f'{step.name} 적재 완료')
    advance(out, robot, task)
    push_state(out, robot)
    return out


@transaction.atomic
def confirm_unload(robot_name):
    """수동 수령 확인 (카메라가 인식하지 못할 때 태블릿의 "꺼냈어요")."""
    out = Outbox()
    robot = get_robot(robot_name)
    task = active_task(robot)
    step = task.current_step() if task else None
    if step is None or step.status != TaskStep.STATUS_ARRIVED or step.action != TaskStep.ACTION_UNLOAD:
        raise ServiceError('수령 대기 중인 배달이 없습니다.')
    complete_cargo_step(out, robot, task, step, f'{step.name} 수령 완료')
    push_state(out, robot)
    return out


# ---------------------------------------------------------------- LED 배달
# 받는 사람이 LED 를 요청 → 로봇이 LED 위치로 가서 사람이 실어 줌(카메라로 확인) → 요청 위치로 가서
# 받는 사람이 꺼냄(카메라에서 사라짐) → 홈. 한 번에 한 건만 받고, 진행 중이면 새 요청은 거절한다.

def cargo_matches(label, expected):
    return bool(label) and bool(expected) and label.strip().lower() == expected.strip().lower()


def complete_cargo_step(out, robot, task, step, message):
    step.status = TaskStep.STATUS_DONE
    step.save(update_fields=['status'])
    log(out, robot, message)
    advance(out, robot, task)


@transaction.atomic
def start_delivery(robot_name, location_id, x, y, yaw):
    """location_id: LED 가 놓인 위치, (x, y, yaw): 받을 사람이 지도에서 찍은 자리."""
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
    """카메라 인식 결과가 바뀌었을 때 (label: 보이는 물건 라벨, '' = 아무것도 없음)."""
    out = Outbox()
    robot = get_robot(name)
    task = active_task(robot)
    step = task.current_step() if task else None
    if step is None or step.status != TaskStep.STATUS_ARRIVED or not step.cargo:
        return out
    if step.action == TaskStep.ACTION_LOAD:
        if cargo_matches(label, step.cargo):
            complete_cargo_step(out, robot, task, step, f'카메라: {step.cargo} 적재 확인')
        elif label:
            log(out, robot, f'카메라: {label} 이(가) 실렸습니다. 요청한 것은 {step.cargo} 입니다')
    elif step.action == TaskStep.ACTION_UNLOAD and not label:
        complete_cargo_step(out, robot, task, step, f'카메라: {step.cargo} 수령 확인 (꺼냄)')
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
    """precise: 스캔 매칭으로 찾은 위치 (자동 위치 추정) → AMCL 불확실성을 작게 준다."""
    out = Outbox()
    robot = get_robot(robot_name)
    require_online(robot)
    out.to_robot(robot, {'type': 'initial_pose', 'x': float(x), 'y': float(y), 'yaw': float(yaw),
                         'precise': bool(precise)})
    log(out, robot, f'초기 위치 지정 ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f}°)')
    return out


@transaction.atomic
def save_location(data):
    """data에 id가 있으면 수정, 없으면 추가."""
    out = Outbox()
    m = active_map()
    if m is None:
        raise ServiceError('먼저 맵을 불러오세요.')
    name = (data.get('name') or '').strip()
    if not name:
        raise ServiceError('위치 이름을 입력하세요.')
    kind = data.get('kind', Location.KIND_PICKUP)
    if kind not in (Location.KIND_PICKUP, Location.KIND_HOME):
        raise ServiceError('잘못된 위치 종류입니다.')
    fields = dict(name=name, kind=kind, x=float(data['x']), y=float(data['y']),
                  yaw=float(data.get('yaw', 0.0)))
    if 'cargo' in data:  # 카메라 인식 라벨 (비우면 위치 이름)
        fields['cargo'] = str(data.get('cargo') or '').strip()[:50]
    if data.get('id'):
        loc = Location.objects.filter(map=m, id=data['id']).first()
        if loc is None:
            raise ServiceError('위치를 찾을 수 없습니다.')
        for k, v in fields.items():
            setattr(loc, k, v)
        loc.save()
    else:
        loc = Location.objects.create(map=m, **fields)
    if kind == Location.KIND_HOME:  # 홈은 하나만
        m.locations.filter(kind=Location.KIND_HOME).exclude(pk=loc.pk) \
            .update(kind=Location.KIND_PICKUP)
    push_locations(out)
    return out


@transaction.atomic
def set_home(x, y, yaw):
    """홈 위치를 (x, y, yaw)로 지정. 이미 있으면 위치만 바꾼다."""
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


def set_slam_view(name, yaw):
    """새 SLAM 실행의 첫 로봇 위치(yaw): 로봇 정면이 웹 화면 위쪽(90°)이 되도록 맵 화면 각도를 정한다."""
    out = Outbox()
    if name not in SLAM_VIEW_PENDING:
        return out
    SLAM_VIEW_PENDING.discard(name)
    view = math.atan2(math.sin(math.pi / 2 - yaw), math.cos(math.pi / 2 - yaw))
    SLAM_VIEW[name] = view
    m = Map.objects.filter(name=SLAM_MAP).first()
    if m is not None and m.view_yaw != view:
        m.view_yaw = view
        m.save(update_fields=['view_yaw'])
        out.to_ui({'type': 'map', 'map': m.to_dict()})
    out.to_ui({'type': 'view_yaw', 'view_yaw': view})   # 맵이 아직 없을 때(격자 화면)도 바로 돌림
    return out


@transaction.atomic
def save_zone(data):
    """금지 구역 추가. data: {shape: rect|line, x1, y1, x2, y2, name?}"""
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


# ---------------------------------------------------------------- 맵 만들기 (SLAM 모드)

MAP_NAME_RE = re.compile(r'^[A-Za-z0-9_-]{1,50}$')
PENDING_SAVE = {}  # robot_name -> 저장 요청한 맵 이름
SLAM_MAP = 'slam_live'
SLAM_SESSION = {}  # robot_name -> 지금 연결된 브리지의 SLAM 실행 번호
SLAM_VIEW_PENDING = set()  # 새 SLAM 실행: 첫 로봇 위치를 받으면 화면 각도(view_yaw)를 정할 로봇
SLAM_VIEW = {}             # robot_name -> 이번 SLAM 실행의 화면 각도 (rad)


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
    """저장한 SLAM 맵 파일을 DB 맵으로 등록하고(활성화는 안 함), slam_live 에 등록한 위치를 복사한다.
    같은 SLAM 좌표계에서 저장한 맵이므로 좌표를 그대로 쓸 수 있다 (--straighten 으로 돌린 맵은 제외)."""
    import os
    from .management.commands.import_map import parse_yaml
    from .mapimage import read_pgm
    path = os.path.expanduser(f'~/maps/{name}_clean.yaml')
    if not os.path.exists(path):
        path = os.path.expanduser(f'~/maps/{name}.yaml')
    if not os.path.exists(path):
        return None
    conf = parse_yaml(path)
    width, height, rows = read_pgm(os.path.join(os.path.dirname(path), conf['image'].strip('\'"')))
    origin = [float(v) for v in conf['origin'].strip('[]').split(',')]
    m = save_map(name, float(conf['resolution']), origin[0], origin[1], width, height,
                 encode_png(rows, width, height), activate=False)
    src = Map.objects.filter(name='slam_live').first()
    if src is None:
        return 0
    m.view_yaw = src.view_yaw   # 저장한 맵도 SLAM 때와 같은 방향으로 보이게
    m.save(update_fields=['view_yaw'])
    copied = 0
    for loc in src.locations.all():
        _, created = Location.objects.update_or_create(
            map=m, name=loc.name, defaults=dict(kind=loc.kind, x=loc.x, y=loc.y, yaw=loc.yaw, cargo=loc.cargo))
        copied += 1
    for z in src.zones.all():   # 금지 구역도 같은 좌표계라 그대로 복사
        Zone.objects.update_or_create(map=m, name=z.name, defaults=dict(
            shape=z.shape, x1=z.x1, y1=z.y1, x2=z.x2, y2=z.y2))
    return copied


# ---------------------------------------------------------------- 상태 초기화

@transaction.atomic
def reset_robot(robot_name):
    """진행 중인 작업 취소, 상태를 대기로, 로봇에 초기화 요청 (시뮬레이션은 출발 위치로 되돌림)."""
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


# ---------------------------------------------------------------- 실행 모드 (runner.py)

def log_runner(job, ok, message):
    """runner 스레드에서 호출: 결과를 기록에 남기고 UI 에 알린다."""
    from asgiref.sync import async_to_sync
    from channels.layers import get_channel_layer
    entry = EventLog.objects.create(robot=get_robot(), message=f'{job} {"완료" if ok else "실패"}: {message}'[:255])
    async_to_sync(get_channel_layer().group_send)('ui', {'type': 'send.msg', 'msg': {'type': 'log', 'log': entry.to_dict()}})


# ---------------------------------------------------------------- 방문 순서 최적화 (planning.py)

def plan_order(robot_name, location_ids, with_home):
    """선택한 위치들을 가장 빨리 도는 순서와 예상 시간. DB 를 바꾸지 않는다."""
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
    """웹 수동 조작 허용 여부. 허용이면 로봇 이름, 아니면 ServiceError."""
    robot = get_robot(robot_name)
    require_online(robot)
    task = active_task(robot)
    if task:
        raise ServiceError(f'작업 #{task.id} 진행 중에는 수동 조작을 할 수 없습니다. 먼저 작업을 취소하세요.')
    return robot.name
