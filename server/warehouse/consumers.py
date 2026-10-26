"""WebSocket 엔드포인트.

/ws/robot/<name>/ : ROS 2 브리지 노드가 접속 (name = 로봇 이름, 예: tb3, tb3_1, tb3_2)
/ws/ui/[?robot=<name>] : 웹 브라우저가 접속. 연결마다 '선택한 로봇' 이 있어, 그 로봇의 위치/경로/상태만 받고
                         요청(이동, 정지 등)도 그 로봇으로 간다 (요청에 robot_name 을 주면 그 로봇). 기본 tb3
"""
import logging
import time
from urllib.parse import parse_qs

from channels.db import database_sync_to_async
from channels.generic.websocket import AsyncJsonWebsocketConsumer

from . import runner, services

logger = logging.getLogger(__name__)

UI_GROUP = 'ui'

# 위치·경로처럼 자주 오는 값은 DB에 저장하지 않고 메모리에만 둔다 (live.py)
from .live import LIVE  # noqa: E402


def robot_group(name):
    return f'robot_{name}'


def robot_of(msg):
    """메시지가 어느 로봇 것인지 (pose 등은 'robot': 이름, state/snapshot 은 'robot': {name, ...}). 없으면 None."""
    r = msg.get('robot')
    if isinstance(r, dict):
        return r.get('name')
    return r if isinstance(r, str) else None


async def broadcast_robots(channel_layer):
    """웹 로봇 선택 목록 갱신 (브리지 접속/끊김 때)."""
    robots = await database_sync_to_async(services.robot_list)()
    await channel_layer.group_send(UI_GROUP, {'type': 'send.msg', 'msg': {'type': 'robots', 'robots': robots}})


async def dispatch(channel_layer, out):
    for robot_name, msg in out.robot:
        await channel_layer.group_send(robot_group(robot_name), {'type': 'send.msg', 'msg': msg})
    for msg in out.ui:
        await channel_layer.group_send(UI_GROUP, {'type': 'send.msg', 'msg': msg})


class RobotConsumer(AsyncJsonWebsocketConsumer):
    async def connect(self):
        name = self.scope['url_route']['kwargs']['name']
        if not services.ROBOT_NAME_RE.match(name):
            logger.warning('robot name rejected: %r', name)
            self.robot_name = None
            await self.close()
            return
        self.robot_name = name
        await self.channel_layer.group_add(robot_group(self.robot_name), self.channel_name)
        await self.accept()
        logger.info('robot %s connected', self.robot_name)
        await broadcast_robots(self.channel_layer)

    async def disconnect(self, code):
        if self.robot_name is None:
            return
        await self.channel_layer.group_discard(robot_group(self.robot_name), self.channel_name)
        LIVE.pop(self.robot_name, None)
        await self.broadcast({'type': 'pose', 'robot': self.robot_name, 'lost': True})
        await self.broadcast({'type': 'path', 'robot': self.robot_name, 'points': []})
        await self.broadcast({'type': 'health', 'robot': self.robot_name, 'bridge': False})
        await self.run(services.robot_disconnected, self.robot_name)
        await broadcast_robots(self.channel_layer)

    async def broadcast(self, msg):
        await self.channel_layer.group_send(UI_GROUP, {'type': 'send.msg', 'msg': msg})

    async def send_msg(self, event):
        await self.send_json(event['msg'])

    async def run(self, func, *args):
        try:
            out = await database_sync_to_async(func)(*args)
        except services.ServiceError as e:
            logger.warning('robot %s: %s', self.robot_name, e)
            return
        await dispatch(self.channel_layer, out)

    async def receive_json(self, msg, **kwargs):
        kind = msg.get('type')
        live = LIVE.setdefault(self.robot_name, {})
        if kind == 'pose':
            live['pose'] = {'x': msg['x'], 'y': msg['y'], 'yaw': msg['yaw']}
            await self.channel_layer.group_send(UI_GROUP, {'type': 'send.msg', 'msg': {
                'type': 'pose', 'robot': self.robot_name, **live['pose']}})
        elif kind == 'pose_lost':
            if live.pop('pose', None) is not None:
                await self.broadcast({'type': 'pose', 'robot': self.robot_name, 'lost': True})
        elif kind == 'health':
            live['mode'] = msg.get('mode', live.get('mode', 'map'))
            health = {'scan_ok': msg.get('scan_ok'), 'odom_ok': msg.get('odom_ok'),
                      'localized': msg.get('localized'), 'bridge': True, 'mode': live['mode'],
                      'tipped': bool(msg.get('tipped')), 'tilt_deg': msg.get('tilt_deg'),
                      'footprint': msg.get('footprint')}
            live['health'] = health
            await self.broadcast({'type': 'health', 'robot': self.robot_name, **health})
            # 로봇(Pi)이 응답할 때/안 할 때가 바뀔 때만 DB 상태를 바꾼다
            robot_ok = bool(msg.get('robot_ok'))
            if live.get('robot_ok') != robot_ok:
                live['robot_ok'] = robot_ok
                await self.run(services.robot_health, self.robot_name, robot_ok)
        elif kind == 'path':
            live['path'] = msg.get('points', [])
            await self.channel_layer.group_send(UI_GROUP, {'type': 'send.msg', 'msg': {
                'type': 'path', 'robot': self.robot_name, 'points': live['path']}})
        elif kind == 'hello':
            live['mode'] = msg.get('mode', 'map')
            await self.run(services.robot_connected, self.robot_name, msg.get('active_goal_id'),
                           live['mode'], str(msg.get('session', '')))
        elif kind == 'reset_done':
            await self.broadcast({'type': 'reset_done', 'ok': bool(msg.get('ok')), 'message': msg.get('message', '')})
            await self.run(services.reset_done, self.robot_name, bool(msg.get('ok')), msg.get('message', ''))
        elif kind == 'map_saved':
            await self.broadcast({'type': 'map_saved', 'ok': bool(msg.get('ok')), 'message': msg.get('message', '')})
            await self.run(services.map_saved, self.robot_name, bool(msg.get('ok')), msg.get('message', ''))
        elif kind == 'map':
            await self.run(services.handle_map, self.robot_name, msg)
        elif kind == 'cargo':
            # 카메라 인식 결과 (브리지가 1초 이상 같을 때만 보냄). '' = 아무것도 안 보임
            live['cargo'] = str(msg.get('label') or '')
            await self.broadcast({'type': 'cargo', 'robot': self.robot_name, 'label': live['cargo']})
            await self.run(services.handle_cargo, self.robot_name, live['cargo'])
        elif kind == 'yolo':
            # YOLO 부품 검출기 상태 (available, enabled, counts 글자) - 화면 표시용, DB 에 저장하지 않음
            live['yolo'] = {k: msg.get(k) for k in ('available', 'enabled', 'counts', 'age')}
            # 주문 적재 확인용: 최근 개수 글자 (services.yolo_counts 가 가장 많이 나온 값을 씀)
            now = time.monotonic()
            hist = [h for h in live.get('yolo_hist', []) if now - h[0] <= 10.0]
            hist.append((now, str(msg.get('counts') or '')))
            live['yolo_hist'] = hist
            await self.broadcast({'type': 'yolo', 'robot': self.robot_name, **live['yolo']})
        elif kind == 'params':
            # 실행 중에 바꿀 수 있는 브리지 파라미터 (이름, 지금 값, 시작 값) - 웹 '주행 파라미터' 카드, DB 에 저장하지 않음
            live['params'] = msg.get('items', [])
            await self.broadcast({'type': 'params', 'robot': self.robot_name, 'items': live['params']})
        elif kind == 'param_result':
            await self.broadcast({'type': 'param_result', 'robot': self.robot_name, 'name': msg.get('name'),
                                  'ok': bool(msg.get('ok')), 'reason': msg.get('reason', '')})
        elif kind == 'scan':
            # 실시간 LiDAR (지도 좌표 점, 초당 4번) - 화면 표시용, DB 에 저장하지 않음
            await self.broadcast({'type': 'scan', 'robot': self.robot_name, 'origin': msg.get('origin'),
                                  'inc': msg.get('inc'), 'points': msg.get('points', [])})
        elif kind == 'obstacles':
            # 추적 중인 움직이는 장애물 (map 좌표, 위치+속도) - 화면 표시용, DB 에 저장하지 않음
            live['obstacles'] = msg.get('obstacles', [])
            await self.broadcast({'type': 'obstacles', 'robot': self.robot_name, 'obstacles': live['obstacles']})
        elif kind == 'local_plan':
            # Frenet 로컬 플래너가 고른 궤적 (map 좌표) + 명령 속도/상태 (초당 5번) - 화면 표시용, 저장/스냅샷 안 함
            await self.broadcast({'type': 'local_plan', 'robot': self.robot_name, 'traj': msg.get('traj', []),
                                  'v': msg.get('v'), 'w': msg.get('w'), 'state': msg.get('state')})
        elif kind == 'memory':
            # A* 가 기억 중인 장애물: remembered = 맵에 없는 LiDAR 장애물 칸, bumps = 부딪힌 자리
            live['memory'] = {'remembered': msg.get('remembered', []), 'bumps': msg.get('bumps', [])}
            await self.broadcast({'type': 'memory', 'robot': self.robot_name, **live['memory']})
        elif kind == 'battery':
            await self.run(services.handle_battery, self.robot_name,
                           msg.get('voltage'), msg.get('percentage'))
        elif kind == 'nav_status':
            await self.run(services.handle_nav_status, self.robot_name,
                           msg.get('goal_id'), msg.get('status'), msg.get('message', ''))
        else:
            logger.warning('robot %s: unknown message type %r', self.robot_name, kind)


class UIConsumer(AsyncJsonWebsocketConsumer):
    async def connect(self):
        query = parse_qs(self.scope.get('query_string', b'').decode())
        name = (query.get('robot') or [''])[0]
        self.robot_name = name if services.ROBOT_NAME_RE.match(name) else services.DEFAULT_ROBOT
        await self.channel_layer.group_add(UI_GROUP, self.channel_name)
        await self.accept()
        await self.send_snapshot()

    async def send_snapshot(self):
        """선택한 로봇의 전체 상태 (처음 접속, 로봇을 바꿨을 때)."""
        self.robot_name = await database_sync_to_async(services.known_robot)(self.robot_name)
        snap = await database_sync_to_async(services.snapshot)(self.robot_name)
        snap['robots'] = await database_sync_to_async(services.robot_list)()
        live = LIVE.get(snap['robot']['name'], {})
        snap['pose'] = live.get('pose')
        snap['cargo'] = live.get('cargo')   # None = 인식 노드 없음
        snap['memory'] = live.get('memory')
        snap['yolo'] = live.get('yolo')     # None = 브리지 없음
        snap['params'] = live.get('params')  # None = 브리지 없음
        snap['obstacles'] = live.get('obstacles', [])
        snap['path'] = live.get('path', [])
        snap['health'] = live.get('health', {'bridge': False})
        snap['runner'] = {'maps': runner.list_maps(), 'worlds': runner.list_worlds(),
                          'mode': runner.current_mode(), 'busy': runner.busy(), 'elapsed': runner.busy_elapsed()}
        await self.send_json(snap)

    async def disconnect(self, code):
        await self.channel_layer.group_discard(UI_GROUP, self.channel_name)

    async def handle_runner(self, action, msg):
        """실행 모드 전환 (서버가 scripts/run.sh 등을 실행)."""
        try:
            if action == 'list_maps':
                await self.send_json({'type': 'runner_info', 'maps': runner.list_maps(), 'worlds': runner.list_worlds(),
                                      'mode': runner.current_mode(), 'busy': runner.busy(), 'elapsed': runner.busy_elapsed()})
                return
            if action in ('runner_start', 'runner_stop'):
                # 노드를 다시 띄우기 전에 주행 중인 로봇은 모두 멈춘다
                for name in await database_sync_to_async(services.moving_robots)():
                    await dispatch(self.channel_layer, await database_sync_to_async(services.estop)(name))
            if action == 'runner_start':
                await database_sync_to_async(runner.start_mode)(
                    str(msg.get('robot')), str(msg.get('mode')), msg.get('map') or None, msg.get('world') or None,
                    msg.get('amcl') or 'default')
            elif action == 'runner_stop':
                await database_sync_to_async(runner.stop_all)()
            elif action == 'explore_start':
                if LIVE.get(self.robot_name, {}).get('mode') != 'slam':
                    raise runner.RunnerError('자동 탐색은 "맵 만들기 (SLAM)" 모드에서 쓸 수 있습니다.')
                await database_sync_to_async(runner.explore)()
            elif action == 'explore_stop':
                await database_sync_to_async(runner.cancel)(runner.EXPLORE_JOB)
                # 탐색이 보낸 이동도 멈춘다
                robot = await database_sync_to_async(services.get_robot)(self.robot_name)
                if robot.state == 'MOVING':
                    await dispatch(self.channel_layer, await database_sync_to_async(services.cancel_task)(robot.name))
            elif action == 'auto_localize':
                if LIVE.get(self.robot_name, {}).get('mode') != 'map':
                    raise runner.RunnerError('위치 자동 찾기는 "저장된 맵으로 위치 추정(AMCL)" 모드에서 쓸 수 있습니다.')
                # 로봇이 제자리에서 돌고 조금 움직이므로, 작업 중이면 하지 않는다
                robot = await database_sync_to_async(services.get_robot)(self.robot_name)
                if robot.state not in ('IDLE', 'ERROR'):
                    raise runner.RunnerError('작업 중에는 위치 자동 찾기를 할 수 없습니다. 작업을 끝내거나 취소하세요.')
                await database_sync_to_async(runner.auto_localize)()
        except runner.RunnerError as e:
            await self.send_json({'type': 'error', 'message': str(e)})

    async def send_msg(self, event):
        msg = event['msg']
        name = robot_of(msg)
        if name and name != self.robot_name:
            # 선택하지 않은 로봇의 위치/경로 등은 보내지 않는다. 상태 변화만 목록 표시용으로 줄여서
            if msg.get('type') == 'state':
                await self.send_json({'type': 'robot_summary', 'robot': msg['robot']})
            return
        await self.send_json(msg)

    async def receive_json(self, msg, **kwargs):
        action = msg.get('action')
        if action == 'select_robot':
            name = str(msg.get('robot_name', ''))
            if not services.ROBOT_NAME_RE.match(name):
                await self.send_json({'type': 'error', 'message': f'잘못된 로봇 이름: {name}'})
                return
            self.robot_name = name
            self.teleop_checked_until = 0.0   # 수동 조작 허용 여부를 새 로봇으로 다시 확인
            await self.send_snapshot()
            return
        # 요청 대상 로봇: robot_name 을 주면 그 로봇, 아니면 이 화면에서 선택한 로봇 ('robot' 은 실행 모드의 real/sim)
        robot = str(msg.get('robot_name') or self.robot_name)
        if not services.ROBOT_NAME_RE.match(robot):
            await self.send_json({'type': 'error', 'message': f'잘못된 로봇 이름: {robot}'})
            return
        if action in ('runner_start', 'runner_stop', 'auto_localize', 'list_maps', 'explore_start', 'explore_stop'):
            await self.handle_runner(action, msg)
            return
        if action == 'teleop':
            # 웹 수동 조작: 누르는 동안 0.1초마다 온다. DB 에 쓰지 않고 브리지로 바로 보낸다.
            # 허용 여부(연결됨, 작업 없음)는 1초에 한 번만 DB 로 확인
            now = time.monotonic()
            if now > getattr(self, 'teleop_checked_until', 0.0):
                try:
                    self.teleop_robot = await database_sync_to_async(services.teleop_check)(robot)
                except services.ServiceError as e:
                    self.teleop_checked_until = now + 1.0
                    self.teleop_robot = None
                    await self.send_json({'type': 'error', 'message': str(e)})
                    return
                self.teleop_checked_until = now + 1.0
            if self.teleop_robot:
                await self.channel_layer.group_send(robot_group(self.teleop_robot), {'type': 'send.msg', 'msg': {
                    'type': 'teleop', 'v': float(msg.get('v', 0.0)), 'w': float(msg.get('w', 0.0))}})
            return
        if action == 'yolo_set':
            # 웹 YOLO 버튼: 검출기 enabled 켜기/끄기, 장면 캡처. DB 에 쓰지 않고 브리지로 바로 보낸다
            target = robot
            if not LIVE.get(target, {}).get('health', {}).get('bridge'):
                await self.send_json({'type': 'error', 'message': '브리지가 연결되어 있지 않습니다.'})
                return
            out = {'type': 'yolo_set'}
            if 'enabled' in msg:
                out['enabled'] = bool(msg['enabled'])
            if msg.get('capture'):
                out['capture'] = True
            await self.channel_layer.group_send(robot_group(target), {'type': 'send.msg', 'msg': out})
            return
        if action == 'param_set':
            # 웹 '주행 파라미터' 카드: 브리지가 ROS 파라미터로 바꾸고 결과(param_result)와 새 목록(params)을 보낸다
            if not LIVE.get(robot, {}).get('health', {}).get('bridge'):
                await self.send_json({'type': 'error', 'message': '브리지가 연결되어 있지 않습니다.'})
                return
            await self.channel_layer.group_send(robot_group(robot), {'type': 'send.msg', 'msg': {
                'type': 'param_set', 'name': str(msg.get('name', '')), 'value': msg.get('value')}})
            return
        if action in ('plan_order', 'travel_model', 'preview_order'):
            # 계산 결과는 요청한 화면에만 돌려준다 (DB 를 바꾸지 않음)
            try:
                if action == 'plan_order':
                    reply = await database_sync_to_async(services.plan_order)(
                        robot, [int(i) for i in msg.get('location_ids', [])], bool(msg.get('with_home', True)))
                elif action == 'preview_order':
                    reply = await database_sync_to_async(services.preview_order)(
                        robot, int(msg['dropoff_id']), dict(msg.get('items') or {}))
                else:
                    reply = await database_sync_to_async(services.travel_model_info)()
                await self.send_json(reply)
            except services.ServiceError as e:
                await self.send_json({'type': 'error', 'message': str(e)})
            except (KeyError, ValueError, TypeError) as e:
                await self.send_json({'type': 'error', 'message': f'잘못된 요청: {e}'})
            return
        if action == 'set_home_here':
            # 로봇의 현재 위치는 DB가 아니라 메모리(LIVE)에 있으므로 여기서 꺼내 넘긴다
            pose = LIVE.get(robot, {}).get('pose')
            if pose is None:
                await self.send_json({'type': 'error', 'message': '로봇 위치를 모릅니다. 초기 위치를 먼저 지정하세요.'})
                return
            msg = {'action': 'set_home', 'x': pose['x'], 'y': pose['y'], 'yaw': pose['yaw']}
            action = 'set_home'
        if action == 'set_initial_pose' and LIVE.get(robot, {}).get('mode') == 'slam':
            await self.send_json({'type': 'error', 'message': 'SLAM 모드에서는 초기 위치 지정이 필요 없습니다 (SLAM이 위치를 잡습니다).'})
            return
        handlers = {
            'start_task': lambda: (services.start_task, robot, list(map(int, msg.get('location_ids', []))),
                                   bool(msg.get('with_home', True))),
            'goto_location': lambda: (services.start_task, robot, [int(msg['location_id'])], False),
            'goto_point': lambda: (services.goto_point, robot,
                                   float(msg['x']), float(msg['y']), float(msg['yaw']), bool(msg.get('replace'))),
            'go_home': lambda: (services.go_home, robot),
            'confirm_load': lambda: (services.confirm_load, robot),
            'confirm_unload': lambda: (services.confirm_unload, robot),
            'start_delivery': lambda: (services.start_delivery, robot, int(msg['location_id']),
                                       float(msg['x']), float(msg['y']), float(msg.get('yaw', 0.0))),
            'start_order': lambda: (services.start_order, robot, int(msg['dropoff_id']), dict(msg.get('items') or {})),
            'force_next': lambda: (services.force_next, robot, bool(msg.get('confirmed'))),
            'retry_step': lambda: (services.retry_step, robot),
            'skip_step': lambda: (services.skip_step, robot),
            'cancel_task': lambda: (services.cancel_task, robot),
            'estop': lambda: (services.estop, robot),
            'reset_error': lambda: (services.reset_error, robot),
            'set_initial_pose': lambda: (services.set_initial_pose, robot,
                                         float(msg['x']), float(msg['y']), float(msg['yaw']),
                                         bool(msg.get('precise'))),
            'reset_robot': lambda: (services.reset_robot, robot),
            'save_map': lambda: (services.request_save_map, robot, str(msg.get('name', ''))),
            'set_home': lambda: (services.set_home, float(msg['x']), float(msg['y']), float(msg['yaw'])),
            'save_location': lambda: (services.save_location, msg.get('location', {})),
            'delete_location': lambda: (services.delete_location, int(msg['location_id'])),
            'save_zone': lambda: (services.save_zone, msg.get('zone', {})),
            'delete_zone': lambda: (services.delete_zone, int(msg['zone_id'])),
        }
        if action not in handlers:
            await self.send_json({'type': 'error', 'message': f'알 수 없는 요청: {action}'})
            return
        try:
            func, *args = handlers[action]()
            out = await database_sync_to_async(func)(*args)
        except services.ServiceError as e:
            await self.send_json({'type': 'error', 'message': str(e)})
            return
        except (KeyError, ValueError, TypeError) as e:
            await self.send_json({'type': 'error', 'message': f'잘못된 요청: {e}'})
            return
        await dispatch(self.channel_layer, out)
