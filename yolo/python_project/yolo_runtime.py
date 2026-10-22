"""Shared runtime setup for ROS publishing and the VS Code preview launcher."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

PROJECT = Path(__file__).resolve().parents[1] / 'yolo_training'
WEIGHTS = PROJECT / 'best_components7_finetuned.pt'
PYTHON = PROJECT / '.venv/bin/python'


def ros_environment():
    if not PYTHON.is_file() or not WEIGHTS.is_file():
        raise RuntimeError(f'가상환경 또는 모델을 찾을 수 없습니다: {PROJECT}')
    domain = os.environ.get('YOLO_DOMAIN_ID', '44')
    if not domain.isdigit() or not 0 <= int(domain) <= 232:
        raise RuntimeError('YOLO_DOMAIN_ID는 0~232 정수여야 합니다.')
    result = subprocess.run(
        ['/bin/bash', '-c', 'source /opt/ros/humble/setup.bash && env -0'],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=True,
    )
    environment = {}
    for entry in result.stdout.split(b'\0'):
        if b'=' in entry:
            key, value = entry.split(b'=', 1)
            environment[os.fsdecode(key)] = os.fsdecode(value)
    environment['ROS_DOMAIN_ID'] = domain
    environment['PYTHONUNBUFFERED'] = '1'
    environment['YOLO_CONFIG_DIR'] = str(PROJECT / '.config')
    # Keep debugger bootstrap variables out of explicitly managed subprocesses.
    environment.pop('PYTHONHOME', None)
    return environment


def terminate(process):
    if process.poll() is not None:
        return
    for sig, wait in [(signal.SIGINT, 4), (signal.SIGTERM, 2), (signal.SIGKILL, 2)]:
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            break
        try:
            process.wait(timeout=wait)
            break
        except subprocess.TimeoutExpired:
            continue


def run(preview=False):
    if '--help' in sys.argv[1:]:
        print('실행: python3 ' + Path(sys.argv[0]).name)
        print('모델: ' + str(WEIGHTS))
        print('입력: /csi/image_raw | 도메인: 44 (YOLO_DOMAIN_ID로 변경 가능)')
        print('종료: Ctrl+C' + (' 또는 영상 창 닫기' if preview else ''))
        return
    env = ros_environment()
    command = [str(PYTHON), '-u', str(PROJECT / 'infer_ros.py')]
    if preview:
        command += ['--ros-args', '-r', '__node:=component_detector_preview',
                    '-r', '/yolo/image_raw:=/yolo_preview/image_raw']
    command += sys.argv[1:]
    commands = [command]
    if preview:
        commands.append(['/opt/ros/humble/lib/rqt_image_view/rqt_image_view',
                         '/yolo_preview/image_raw'])
    processes = []
    previous_handlers = {}

    def interrupted(signum, frame):
        raise KeyboardInterrupt

    try:
        for sig in (signal.SIGINT, signal.SIGTERM):
            previous_handlers[sig] = signal.signal(sig, interrupted)
        for args in commands:
            processes.append(subprocess.Popen(args, env=env, cwd=PROJECT,
                                               start_new_session=True))
        print(f"도메인 {env['ROS_DOMAIN_ID']} | 추가 학습 모델 | 입력 /csi/image_raw")
        if preview:
            print('미리보기 창을 열었습니다. 종료: 창 닫기 또는 Ctrl+C')
        else:
            print('검출 결과: /yolo/image_raw | 종료: Ctrl+C')
        while all(p.poll() is None for p in processes):
            time.sleep(0.2)
        failures = [p.returncode for p in processes if p.returncode not in (None, 0)]
        if failures:
            raise RuntimeError(f'실행 프로세스가 오류로 종료됐습니다: {failures}')
    except KeyboardInterrupt:
        print('\nYOLO 실행을 종료합니다.')
    finally:
        # Prevent a second Ctrl+C from interrupting cleanup.
        for sig in previous_handlers:
            signal.signal(sig, signal.SIG_IGN)
        for process in reversed(processes):
            terminate(process)
        for sig, handler in previous_handlers.items():
            signal.signal(sig, handler)
