"""Runs scripts/*.sh and tools/*.py to switch the run mode from the web.

The server runs on the same PC as the ROS nodes, so it can run run.sh / stop_ros.sh directly.
The server (not the bridge) runs them so that modes can be started from the web even when all of ROS is down.
One job at a time, run in a separate thread; the result is announced to the UI group.
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

HOME = os.path.dirname(settings.BASE_DIR)          # Repository root (parent of server/)
SCRIPTS = os.path.join(HOME, 'scripts')
MAP_DIR = os.path.expanduser('~/maps')
RUN_MODE_FILE = os.path.join(HOME, 'logs', 'run_mode.env')
# Simulation worlds (same places scripts/start_sim.sh looks)
AWS_WORLDS = os.path.expanduser('~/sim_ws/src/aws-robomaker-small-warehouse-world/worlds')
TB3_WORLDS = os.path.expanduser('~/turtlebot3_ws/install/turtlebot3_gazebo/share/turtlebot3_gazebo/worlds')
if not os.path.isdir(TB3_WORLDS):  # PC with turtlebot3 installed via apt instead of a source build
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
_running = None  # Name of the running job
_proc = None     # Running process (for stopping)
_started_at = None  # Start time of the running job (monotonic)
EXPLORE_JOB = '자동 탐색'


class RunnerError(Exception):
    pass


def list_maps():
    """List of map yamls under ~/maps (excluding previews etc.). Paths are relative to ~/maps.
    world: for a map built in simulation, the world it was built in (the yaml's '# world:' line, written by scripts/save_map.sh)."""
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
    """Simulation worlds [{'name', 'label'}]: this repo's worlds/, the AWS warehouse, TurtleBot3 default worlds."""
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
    """KEY=VALUE pairs from logs/run_mode.env."""
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
    """Seconds since the running job started (None if none), so the waiting screen's timer continues after a page reload"""
    return round(time.monotonic() - _started_at, 1) if _running and _started_at else None


def _broadcast(msg):
    async_to_sync(get_channel_layer().group_send)('ui', {'type': 'send.msg', 'msg': msg})


def _start(job, cmd, timeout, parse=None):
    """Run cmd in a thread and announce {'type': 'runner', ...} to the UI when it ends."""
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
            # Read output line by line and send it as progress to the web waiting screen (run.sh lines like "1) ...", "2) ..." = steps)
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
        except Exception as e:  # e.g. executable not found
            message = str(e)
        finally:
            with _lock:
                _running = None
                _proc = None
        _broadcast({'type': 'runner', 'job': job, 'state': 'done', 'ok': ok, 'message': message,
                    'mode': current_mode(), 'elapsed': round(time.monotonic() - _started_at, 1)})
        from . import services  # avoid circular import
        services.log_runner(job, ok, message)
    threading.Thread(target=run, daemon=True).start()


def _last_lines(output, n=2):
    lines = [l for l in output.splitlines() if l.strip() and 'WARN' not in l and 'warn' not in l]
    return ' / '.join(lines[-n:])[:300]


def start_mode(robot, mode, map_path=None, world=None, amcl='default'):
    """robot: sim|real, mode: map|slam. map_path: path relative to ~/maps (map mode).
    world: simulation world. Empty = turtlebot3_world for SLAM, the world recorded in the map for map mode (run.sh).
    amcl: default | hall (run.sh --amcl, localization settings for a lecture hall/classroom)."""
    if robot not in ('sim', 'real') or mode not in ('map', 'slam') or amcl not in ('default', 'hall'):
        raise RunnerError('잘못된 모드입니다.')
    cmd = [os.path.join(SCRIPTS, 'run.sh'), '--robot', robot, '--mode', mode, '--amcl', amcl]
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
    """Find the pose on the whole map that best matches the scan and set it as the initial pose (AMCL mode).
    The robot turns in place and, if ambiguous, moves a little to collect scans (tools/localize.py --global, 1-3 min)."""
    cmd = ['bash', '-c', f'source "{SCRIPTS}/env.sh" && python3 -W ignore "{HOME}/tools/localize.py" --global']

    def parse(output, ok):
        m = re.search(r'가장 잘 맞는 위치: .*', output)
        warn = [l for l in output.splitlines() if l.startswith('경고')]
        if not m:
            return _last_lines(output)
        return (m.group(0) + (' / ' + warn[0] if warn else ''))[:300]
    _start('위치 자동 찾기', cmd, timeout=300, parse=parse)


def explore(max_minutes=30):
    """Autonomous exploration in SLAM mode (tools/explore.py). Stop with cancel()."""
    cmd = ['bash', '-c', f'source "{SCRIPTS}/env.sh" && exec python3 -u -W ignore "{HOME}/tools/explore.py" '
                         f'--max-minutes {int(max_minutes)}']

    def parse(output, ok):
        lines = [l.replace('[explore] ', '') for l in output.splitlines() if l.startswith('[explore]')]
        return ' / '.join(lines[-2:])[:300] or _last_lines(output)
    _start(EXPLORE_JOB, cmd, timeout=int(max_minutes * 60) + 120, parse=parse)


def cancel(job):
    """If job is the running one, send SIGINT to its process group."""
    with _lock:
        if _running != job or _proc is None:
            raise RunnerError(f'"{job}" 이(가) 실행 중이 아닙니다.')
        os.killpg(_proc.pid, signal.SIGINT)
