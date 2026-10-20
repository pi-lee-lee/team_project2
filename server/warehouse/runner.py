"""웹에서 실행 모드를 바꾸기 위해 scripts/*.sh 와 tools/*.py 를 실행한다.

서버는 ROS 노드와 같은 PC 에서 돌기 때문에 run.sh / stop_ros.sh 를 직접 실행할 수 있다.
ROS 가 전부 꺼져 있어도 웹에서 켤 수 있도록 브리지가 아니라 서버가 실행한다.
작업은 한 번에 하나만, 별도 스레드에서 돌리고 결과는 UI 그룹에 알린다.
"""
import glob
import os
import re
import subprocess
import threading

from asgiref.sync import async_to_sync
from channels.layers import get_channel_layer
from django.conf import settings

HOME = os.path.dirname(settings.BASE_DIR)          # 저장소 최상위 (server/ 의 부모)
SCRIPTS = os.path.join(HOME, 'scripts')
MAP_DIR = os.path.expanduser('~/maps')
RUN_MODE_FILE = os.path.join(HOME, 'logs', 'run_mode.env')

_lock = threading.Lock()
_running = None  # 실행 중인 작업 이름


class RunnerError(Exception):
    pass


def list_maps():
    """~/maps 아래 맵 yaml 목록 (미리보기 등 제외). 경로는 ~/maps 기준 상대 경로."""
    paths = glob.glob(os.path.join(MAP_DIR, '**', '*.yaml'), recursive=True)
    out = []
    for p in sorted(paths):
        rel = os.path.relpath(p, MAP_DIR)
        out.append({'path': rel, 'name': os.path.basename(p)[:-5]})
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


def _broadcast(msg):
    async_to_sync(get_channel_layer().group_send)('ui', {'type': 'send.msg', 'msg': msg})


def _start(job, cmd, timeout, parse=None):
    """cmd 를 스레드에서 실행하고 끝나면 {'type': 'runner', ...} 를 UI 에 알린다."""
    global _running
    with _lock:
        if _running:
            raise RunnerError(f'"{_running}" 이(가) 아직 진행 중입니다.')
        _running = job
    _broadcast({'type': 'runner', 'job': job, 'state': 'started'})

    def run():
        global _running
        ok, message = False, ''
        try:
            r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout, cwd=HOME,
                               start_new_session=True)
            output = (r.stdout + r.stderr).strip()
            ok = r.returncode == 0
            message = parse(output, ok) if parse else _last_lines(output)
        except subprocess.TimeoutExpired:
            message = f'{timeout}초 안에 끝나지 않았습니다.'
        except Exception as e:  # 실행 파일 없음 등
            message = str(e)
        finally:
            with _lock:
                _running = None
        _broadcast({'type': 'runner', 'job': job, 'state': 'done', 'ok': ok, 'message': message,
                    'mode': current_mode()})
        from . import services  # 순환 import 방지
        services.log_runner(job, ok, message)
    threading.Thread(target=run, daemon=True).start()


def _last_lines(output, n=2):
    lines = [l for l in output.splitlines() if l.strip() and 'WARN' not in l and 'warn' not in l]
    return ' / '.join(lines[-n:])[:300]


def start_mode(robot, mode, map_path=None):
    """robot: sim|real, mode: map|slam. map_path: ~/maps 기준 상대 경로 (map 모드)."""
    if robot not in ('sim', 'real') or mode not in ('map', 'slam'):
        raise RunnerError('잘못된 모드입니다.')
    cmd = [os.path.join(SCRIPTS, 'run.sh'), '--robot', robot, '--mode', mode]
    if mode == 'map':
        if map_path:
            full = os.path.realpath(os.path.join(MAP_DIR, map_path))
            if not full.startswith(os.path.realpath(MAP_DIR) + os.sep) or not os.path.isfile(full):
                raise RunnerError('맵 파일을 찾을 수 없습니다.')
            cmd += ['--map', full]
        elif robot != 'sim':
            raise RunnerError('맵을 선택하세요.')
    label = f'{"시뮬레이션" if robot == "sim" else "실물"} {"맵 만들기(SLAM)" if mode == "slam" else "위치 추정(AMCL)"}'

    def parse(output, ok):
        done = [l for l in output.splitlines() if l.startswith('==') or '가장 잘 맞는' in l or '실패' in l
                or '없습니다' in l or '접속할 수 없습니다' in l]
        return ' / '.join(l.strip('= ') for l in done[-3:])[:300] or _last_lines(output)
    _start(label, cmd, timeout=240, parse=parse)


def stop_all():
    _start('전체 정지', [os.path.join(SCRIPTS, 'stop_ros.sh')], timeout=60)


def auto_localize():
    """맵 전체에서 스캔과 가장 잘 맞는 위치를 찾아 초기 위치로 지정 (AMCL 모드)."""
    cmd = ['bash', '-c', f'source "{SCRIPTS}/env.sh" && python3 -W ignore "{HOME}/tools/localize.py" --global']

    def parse(output, ok):
        m = re.search(r'가장 잘 맞는 위치: .*', output)
        warn = [l for l in output.splitlines() if l.startswith('경고')]
        if not m:
            return _last_lines(output)
        return (m.group(0) + (' / ' + warn[0] if warn else ''))[:300]
    _start('위치 자동 찾기', cmd, timeout=120, parse=parse)
