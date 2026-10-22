"""웹에서 실행 모드를 바꾸기 위해 scripts/*.sh 와 tools/*.py 를 실행한다.

서버는 ROS 노드와 같은 PC 에서 돌기 때문에 run.sh / stop_ros.sh 를 직접 실행할 수 있다.
ROS 가 전부 꺼져 있어도 웹에서 켤 수 있도록 브리지가 아니라 서버가 실행한다.
작업은 한 번에 하나만, 별도 스레드에서 돌리고 결과는 UI 그룹에 알린다.
"""
import glob
import os
import re
import signal
import subprocess
import threading
import time

from asgiref.sync import async_to_sync
from channels.layers import get_channel_layer
from django.conf import settings

HOME = os.path.dirname(settings.BASE_DIR)          # 저장소 최상위 (server/ 의 부모)
SCRIPTS = os.path.join(HOME, 'scripts')
MAP_DIR = os.path.expanduser('~/maps')
RUN_MODE_FILE = os.path.join(HOME, 'logs', 'run_mode.env')
# 시뮬레이션 월드 (scripts/start_sim.sh 가 찾는 곳과 같음)
AWS_WORLDS = os.path.expanduser('~/sim_ws/src/aws-robomaker-small-warehouse-world/worlds')
TB3_WORLDS = os.path.expanduser('~/turtlebot3_ws/install/turtlebot3_gazebo/share/turtlebot3_gazebo/worlds')
if not os.path.isdir(TB3_WORLDS):  # 소스 빌드 대신 apt 로 설치한 PC
    TB3_WORLDS = '/opt/ros/humble/share/turtlebot3_gazebo/worlds'
WORLD_LABELS = {
    'maze_room': '미로 방 (실제 방 복제)',
    'lecture_hall': '강당 (강단 + 복도 6×5, 책상 20·의자 40)',
    'lecture_hall_real': '강당 실물형 (책상 앞판 없음, 좌석 줄 사이 0.25m 로 못 지나감, 의자 무작위)',
    'turtlebot3_world': 'TurtleBot3 기본 (육각형)',
    'turtlebot3_house': 'TurtleBot3 집',
    'empty_world': '빈 공간',
    'no_roof_small_warehouse': 'AWS 소형 창고 (지붕 없음)',
    'small_warehouse': 'AWS 소형 창고',
}

_lock = threading.Lock()
_running = None  # 실행 중인 작업 이름
_proc = None     # 실행 중인 프로세스 (중지용)
_started_at = None  # 실행 중인 작업 시작 시각 (monotonic)
EXPLORE_JOB = '자동 탐색'


class RunnerError(Exception):
    pass


def list_maps():
    """~/maps 아래 맵 yaml 목록 (미리보기 등 제외). 경로는 ~/maps 기준 상대 경로.
    world: 시뮬레이션에서 만든 맵이면 만든 월드 (yaml 의 '# world:' 줄, scripts/save_map.sh 가 기록)."""
    paths = glob.glob(os.path.join(MAP_DIR, '**', '*.yaml'), recursive=True)
    out = []
    for p in sorted(paths):
        rel = os.path.relpath(p, MAP_DIR)
        world = ''
        try:
            with open(p, encoding='utf-8') as f:
                for line in f:
                    if line.startswith('# world:'):
                        world = line.split(':', 1)[1].strip()
        except OSError:
            pass
        out.append({'path': rel, 'name': os.path.basename(p)[:-5], 'world': world})
    return out


def list_worlds():
    """시뮬레이션 월드 목록 [{'name', 'label'}]. 이 저장소 worlds/, AWS 창고, TurtleBot3 기본 월드."""
    names = [os.path.basename(p)[:-6] for p in sorted(glob.glob(os.path.join(HOME, 'worlds', '*.world')))]
    names += [os.path.basename(d) for d in sorted(glob.glob(os.path.join(AWS_WORLDS, '*')))
              if os.path.isfile(os.path.join(d, os.path.basename(d) + '.world'))]
    names += [n for n in ('turtlebot3_world', 'turtlebot3_house', 'empty_world')
              if os.path.isfile(os.path.join(TB3_WORLDS, n + '.world'))]
    seen, out = set(), []
    for n in names:
        if n not in seen:
            seen.add(n)
            out.append({'name': n, 'label': WORLD_LABELS.get(n, n)})
    return out


def current_mode():
    """logs/run_mode.env 의 KEY=VALUE."""
    mode = {}
    try:
        with open(RUN_MODE_FILE, encoding='utf-8') as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith('#') and '=' in line:
                    k, v = line.split('=', 1)
                    mode[k.replace('WAREHOUSE_', '').lower()] = v
    except FileNotFoundError:
        pass
    return mode


def busy():
    return _running


def busy_elapsed():
    """진행 중인 job 이 시작된 뒤 지난 초 (없으면 None). 페이지를 새로 열어도 대기 화면 시간이 이어지게"""
    return round(time.monotonic() - _started_at, 1) if _running and _started_at else None


def _broadcast(msg):
    async_to_sync(get_channel_layer().group_send)('ui', {'type': 'send.msg', 'msg': msg})


def _start(job, cmd, timeout, parse=None):
    """cmd 를 스레드에서 실행하고 끝나면 {'type': 'runner', ...} 를 UI 에 알린다."""
    global _running
    with _lock:
        if _running:
            raise RunnerError(f'"{_running}" 이(가) 아직 진행 중입니다.')
        _running = job
    global _started_at
    _started_at = time.monotonic()
    _broadcast({'type': 'runner', 'job': job, 'state': 'started'})

    def run():
        global _running
        global _proc
        ok, message = False, ''
        try:
            _proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                                     cwd=HOME, start_new_session=True, bufsize=1)
            proc = _proc
            timed_out = []

            def kill():
                timed_out.append(1)
                try:
                    os.killpg(proc.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
            timer = threading.Timer(timeout, kill)
            timer.start()
            # 출력을 한 줄씩 읽어 웹 대기 화면에 진행 상황으로 보낸다 (run.sh 의 "1) 웹 서버", "2) Gazebo ..." = 단계)
            lines, last_sent = [], 0.0
            for line in proc.stdout:
                line = line.rstrip()
                lines.append(line)
                text = line.strip()
                if not text or 'WARN' in text or 'warn' in text:
                    continue
                step = bool(re.match(r'^(\d\)|==)', text))
                now = time.monotonic()
                if step or now - last_sent > 0.5:
                    last_sent = now
                    _broadcast({'type': 'runner', 'job': job, 'state': 'progress', 'line': text[:200], 'step': step,
                                'elapsed': round(now - _started_at, 1)})
            proc.wait()
            timer.cancel()
            output = '\n'.join(lines)
            if timed_out:
                output += f'\n{timeout}초 안에 끝나지 않아 중지했습니다.'
            output = output.strip()
            ok = _proc.returncode == 0
            message = parse(output, ok) if parse else _last_lines(output)
            if _proc.returncode in (-signal.SIGINT, -signal.SIGTERM, 130):
                ok, message = False, '중지됨. ' + message
        except Exception as e:  # 실행 파일 없음 등
            message = str(e)
        finally:
            with _lock:
                _running = None
                _proc = None
        _broadcast({'type': 'runner', 'job': job, 'state': 'done', 'ok': ok, 'message': message,
                    'mode': current_mode(), 'elapsed': round(time.monotonic() - _started_at, 1)})
        from . import services  # 순환 import 방지
        services.log_runner(job, ok, message)
    threading.Thread(target=run, daemon=True).start()


def _last_lines(output, n=2):
    lines = [l for l in output.splitlines() if l.strip() and 'WARN' not in l and 'warn' not in l]
    return ' / '.join(lines[-n:])[:300]


def start_mode(robot, mode, map_path=None, world=None, amcl='default', scan_noise=False):
    """robot: sim|real, mode: map|slam. map_path: ~/maps 기준 상대 경로 (map 모드).
    world: 시뮬레이션 월드. 비우면 SLAM 은 turtlebot3_world, 맵 모드는 맵에 기록된 월드 (run.sh).
    amcl: default | hall (run.sh --amcl, 강당/교실용 위치 추정 설정). scan_noise: 시뮬레이션 LiDAR 를 실물처럼."""
    if robot not in ('sim', 'real') or mode not in ('map', 'slam') or amcl not in ('default', 'hall'):
        raise RunnerError('잘못된 모드입니다.')
    cmd = [os.path.join(SCRIPTS, 'run.sh'), '--robot', robot, '--mode', mode, '--amcl', amcl]
    if robot == 'sim' and scan_noise:
        cmd.append('--scan-noise')
    if robot == 'sim' and world:
        if world not in {w['name'] for w in list_worlds()}:
            raise RunnerError(f'월드를 찾을 수 없습니다: {world}')
        cmd += ['--world', world]
    if mode == 'map':
        if map_path:
            full = os.path.realpath(os.path.join(MAP_DIR, map_path))
            if not full.startswith(os.path.realpath(MAP_DIR) + os.sep) or not os.path.isfile(full):
                raise RunnerError('맵 파일을 찾을 수 없습니다.')
            cmd += ['--map', full]
        elif robot != 'sim':
            raise RunnerError('맵을 선택하세요.')
    label = (f'{"시뮬레이션" if robot == "sim" else "실물"}{f" ({world})" if robot == "sim" and world else ""} '
             f'{"맵 만들기(SLAM)" if mode == "slam" else "맵 사용(AMCL)"}')

    def parse(output, ok):
        done = [l for l in output.splitlines() if l.startswith('==') or '가장 잘 맞는' in l or '실패' in l
                or '없습니다' in l or '접속할 수 없습니다' in l or '월드' in l]
        return ' / '.join(l.strip('= ') for l in done[-3:])[:300] or _last_lines(output)
    _start(label, cmd, timeout=240, parse=parse)


def stop_all():
    _start('전체 정지', [os.path.join(SCRIPTS, 'stop_ros.sh')], timeout=60)


def auto_localize():
    """맵 전체에서 스캔과 가장 잘 맞는 위치를 찾아 초기 위치로 지정 (AMCL 모드).
    로봇이 제자리에서 돌고, 모호하면 조금 이동하며 스캔을 모은다 (tools/localize.py --global, 1~3분)."""
    cmd = ['bash', '-c', f'source "{SCRIPTS}/env.sh" && python3 -W ignore "{HOME}/tools/localize.py" --global']

    def parse(output, ok):
        m = re.search(r'가장 잘 맞는 위치: .*', output)
        warn = [l for l in output.splitlines() if l.startswith('경고')]
        if not m:
            return _last_lines(output)
        return (m.group(0) + (' / ' + warn[0] if warn else ''))[:300]
    _start('위치 자동 찾기', cmd, timeout=300, parse=parse)


def explore(max_minutes=30):
    """SLAM 모드 자동 탐색 (tools/explore.py). cancel() 로 중지."""
    cmd = ['bash', '-c', f'source "{SCRIPTS}/env.sh" && exec python3 -u -W ignore "{HOME}/tools/explore.py" '
                         f'--max-minutes {int(max_minutes)}']

    def parse(output, ok):
        lines = [l.replace('[explore] ', '') for l in output.splitlines() if l.startswith('[explore]')]
        return ' / '.join(lines[-2:])[:300] or _last_lines(output)
    _start(EXPLORE_JOB, cmd, timeout=int(max_minutes * 60) + 120, parse=parse)


def cancel(job):
    """실행 중인 job 이 맞으면 프로세스 그룹에 SIGINT."""
    with _lock:
        if _running != job or _proc is None:
            raise RunnerError(f'"{job}" 이(가) 실행 중이 아닙니다.')
        os.killpg(_proc.pid, signal.SIGINT)
