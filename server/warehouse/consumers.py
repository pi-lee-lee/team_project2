import time
"""WebSocket 엔드포인트.

/ws/robot/<name>/ : ROS 2 브리지 노드가 접속
/ws/ui/           : 웹 브라우저가 접속
"""
import logging

from channels.db import database_sync_to_async
from channels.generic.websocket import AsyncJsonWebsocketConsumer

from . import runner, services

logger = logging.getLogger(__name__)

UI_GROUP = 'ui'

# 위치·경로처럼 자주 오는 값은 DB에 저장하지 않고 메모리에만 둔다 (live.py)
from .live import LIVE  # noqa: E402


def robot_group(name):
    return f'robot_{name}'


async def dispatch(channel_layer, out):
    for robot_name, msg in out.robot:
        await channel_layer.group_send(robot_group(robot_name), {'type': 'send.msg', 'msg': msg})
    for msg in out.ui:
        await channel_layer.group_send(UI_GROUP, {'type': 'send.msg', 'msg': msg})


class RobotConsumer(AsyncJsonWebsocketConsumer):
    async def connect(self):
        self.robot_name = self.scope['url_route']['kwargs']['name']
        await self.channel_layer.group_add(robot_group(self.robot_name), self.channel_name)
        await self.accept()
        logger.info('robot %s connected', self.robot_name)

    async def disconnect(self, code):
        await self.channel_layer.group_discard(robot_group(self.robot_name), self.channel_name)
        LIVE.pop(self.robot_name, None)
        await self.broadcast({'type': 'pose', 'robot': self.robot_name, 'lost': True})
        await self.broadcast({'type': 'path', 'robot': self.robot_name, 'points': []})
        await self.broadcast({'type': 'health', 'robot': self.robot_name, 'bridge': False})
        await self.run(services.robot_disconnected, self.robot_name)

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
                      'tipped': bool(msg.get('tipped')), 'tilt_deg': msg.get('tilt_deg')}
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
        elif kind == 'obstacles':
            # 추적 중인 움직이는 장애물 (map 좌표, 위치+속도) - 화면 표시용, DB 에 저장하지 않음
            live['obstacles'] = msg.get('obstacles', [])
            await self.broadcast({'type': 'obstacles', 'robot': self.robot_name, 'obstacles': live['obstacles']})
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
        await self.channel_layer.group_add(UI_GROUP, self.channel_name)
        await self.accept()
        snap = await database_sync_to_async(services.snapshot)()
        live = LIVE.get(snap['robot']['name'], {})
        snap['pose'] = live.get('pose')
        snap['cargo'] = live.get('cargo')   # None = 인식 노드 없음
        snap['memory'] = live.get('memory')
        snap['obstacles'] = live.get('obstacles', [])
        snap['path'] = live.get('path', [])
        snap['health'] = live.get('health', {'bridge': False})
        snap['runner'] = {'maps': runner.list_maps(), 'worlds': runner.list_worlds(),
                          'mode': runner.current_mode(), 'busy': runner.busy()}
        await self.send_json(snap)

    async def disconnect(self, code):
        await self.channel_layer.group_discard(UI_GROUP, self.channel_name)

    async def handle_runner(self, action, msg):
        """실행 모드 전환 (서버가 scripts/run.sh 등을 실행)."""
        try:
            if action == 'list_maps':
                await self.send_json({'type': 'runner_info', 'maps': runner.list_maps(), 'worlds': runner.list_worlds(),
                                      'mode': runner.current_mode(), 'busy': runner.busy()})
                return
            if action in ('runner_start', 'runner_stop'):
                # 노드를 다시 띄우기 전에 주행 중이면 멈춘다
                robot = await database_sync_to_async(services.get_robot)()
                if robot.state == 'MOVING':
                    await dispatch(self.channel_layer, await database_sync_to_async(services.estop)(None))
            if action == 'runner_start':
                await database_sync_to_async(runner.start_mode)(
                    str(msg.get('robot')), str(msg.get('mode')), msg.get('map') or None, msg.get('world') or None)
            elif action == 'runner_stop':
                await database_sync_to_async(runner.stop_all)()
            elif action == 'explore_start':
                if LIVE.get(services.DEFAULT_ROBOT, {}).get('mode') != 'slam':
                    raise runner.RunnerError('자동 탐색은 "맵 만들기 (SLAM)" 모드에서 쓸 수 있습니다.')
                await database_sync_to_async(runner.explore)()
            elif action == 'explore_stop':
                await database_sync_to_async(runner.cancel)(runner.EXPLORE_JOB)
                # 탐색이 보낸 이동도 멈춘다
                robot = await database_sync_to_async(services.get_robot)()
                if robot.state == 'MOVING':
                    await dispatch(self.channel_layer, await database_sync_to_async(services.cancel_task)(None))
            elif action == 'auto_localize':
                if LIVE.get(services.DEFAULT_ROBOT, {}).get('mode') != 'map':
                    raise runner.RunnerError('위치 자동 찾기는 "저장된 맵으로 위치 추정(AMCL)" 모드에서 쓸 수 있습니다.')
                await database_sync_to_async(runner.auto_localize)()
        except runner.RunnerError as e:
            await self.send_json({'type': 'error', 'message': str(e)})

    async def send_msg(self, event):
        await self.send_json(event['msg'])

    async def receive_json(self, msg, **kwargs):
        action = msg.get('action')
        robot = msg.get('robot')
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
        if action in ('plan_order', 'travel_model'):
            # 계산 결과는 요청한 화면에만 돌려준다 (DB 를 바꾸지 않음)
            try:
                if action == 'plan_order':
                    reply = await database_sync_to_async(services.plan_order)(
                        robot, [int(i) for i in msg.get('location_ids', [])], bool(msg.get('with_home', True)))
                else:
                    reply = await database_sync_to_async(services.travel_model_info)()
                await self.send_json(reply)
            except services.ServiceError as e:
                await self.send_json({'type': 'error', 'message': str(e)})
            return
        if action == 'set_home_here':
            # 로봇의 현재 위치는 DB가 아니라 메모리(LIVE)에 있으므로 여기서 꺼내 넘긴다
            pose = LIVE.get(robot or services.DEFAULT_ROBOT, {}).get('pose')
            if pose is None:
                await self.send_json({'type': 'error', 'message': '로봇 위치를 모릅니다. 초기 위치를 먼저 지정하세요.'})
                return
            msg = {'action': 'set_home', 'x': pose['x'], 'y': pose['y'], 'yaw': pose['yaw']}
            action = 'set_home'
        if action == 'set_initial_pose' and LIVE.get(robot or services.DEFAULT_ROBOT, {}).get('mode') == 'slam':
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
