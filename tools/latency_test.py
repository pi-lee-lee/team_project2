"""명령 → 실제 속도 지연을 잰다: cmd_vel 계단 명령에 대한 odom(·IMU) 속도 반응을 '순수 지연 L + 1차 지연 τ' 로 맞춘다.

    source scripts/env.sh
    python3 tools/latency_test.py [--v 0.13] [--w 1.0] [--trials 5] [--hold 2.0] [--rest 1.5] [--axis both|v|w]
    python3 tools/latency_test.py --analyze logs/latency_YYYYmmdd_HHMMSS.csv     # 저장한 기록만 다시 계산

주의: 로봇이 직접 움직인다. 직진은 앞으로 v x hold 갔다가 같은 만큼 뒤로 돌아오고(시행마다 약 0.26m 왕복),
회전은 왼쪽으로 돌았다가 오른쪽으로 되돌아온다. 주변이 비어 있고 작업(웹 이동/자동 탐색)이 없을 때만 실행.
출발 전과 직진마다 LiDAR 로 앞/뒤 여유를 보고 모자라면 멈춘다. Ctrl+C 면 정지 명령을 보내고 끝낸다.

모델 (명령이 a → b 로 바뀐 시각 t0 기준):
    y(t) = a                                   (t < t0 + L)
    y(t) = a + (b - a)(1 - e^(-(t - t0 - L)/τ))  (t ≥ t0 + L)
  L: 명령을 보낸 뒤 반응이 시작될 때까지 (PC→Pi 전송 + OpenCR + odom 이 PC 로 돌아오는 시간)
  τ: 반응이 시작된 뒤 63% 까지 (모터/가속 프로파일)
시각은 전부 PC 시계(보낸 시각, 받은 시각)로 잰다. odom header.stamp 는 Pi 시계라 PC 와 수십 ms 어긋날 수 있음.
그래서 결과는 '브리지가 명령을 내고 그 효과를 odom 으로 보기까지' 의 왕복 지연 = 지연 보상에 필요한 값.
odom 은 약 30Hz 라 한 번 측정의 분해능은 ~33ms. 여러 번(--trials) 반복해 중앙값을 본다.

결과: 출발(0→목표)/정지(목표→0) 별 L, τ 중앙값과 범위, 실제 도달 속도 비율. 원본 기록은 logs/latency_*.csv
"""
import argparse
import csv
import math
import os
import threading
import time

import numpy as np

HOME = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CMD_RATE = 20.0          # 계단 명령을 이 주기로 계속 보냄 (Hz, 브리지 CmdMux 와 같음)
FRONT_HALF = math.radians(30)
CLEAR_EXTRA = 0.40       # 직진 거리 + 차체 끝 + 이만큼 앞/뒤가 비어 있어야 출발 (m)
# LiDAR(base_scan, x=-0.064) 에서 차체 끝까지: 앞 0.069 + 0.064, 뒤(적재함 포함 -0.277) 0.277 - 0.064
BODY_FRONT = 0.133
BODY_REAR = 0.213
L_GRID = np.arange(0.0, 0.801, 0.005)
TAU_GRID = np.arange(0.005, 1.001, 0.005)


# ------------------------------------------------------------------ 분석 (ROS 없음)
def fit_fopdt(t, y):
    """정규화된 계단 반응 (t: 명령 바뀐 뒤 초, y: 0→1) 에 L, τ 를 격자 탐색으로 맞춤. 반환 (L, τ, rmse)."""
    t = np.asarray(t, dtype=float)[None, None, :]
    y = np.asarray(y, dtype=float)
    L = L_GRID[:, None, None]
    tau = TAU_GRID[None, :, None]
    model = np.where(t < L, 0.0, 1.0 - np.exp(-(t - L) / tau))
    sse = ((model - y[None, None, :]) ** 2).sum(axis=2)
    i, j = np.unravel_index(np.argmin(sse), sse.shape)
    return float(L_GRID[i]), float(TAU_GRID[j]), float(math.sqrt(sse[i, j] / y.size))


def edges(cmds, axis):
    """명령 기록 [(t, v, w)] 에서 axis(1=v, 2=w) 값이 바뀐 시각 [(t0, a, b, 다음 변화 시각)]."""
    out = []
    prev = None
    for row in cmds:
        val = row[axis]
        if prev is not None and abs(val - prev[1]) > 1e-9:
            out.append([row[0], prev[1], val, None])
        if prev is None or abs(val - prev[1]) > 1e-9:
            prev = (row[0], val)
    for k in range(len(out)):
        out[k][3] = out[k + 1][0] if k + 1 < len(out) else cmds[-1][0]
    return [tuple(e) for e in out]


def analyze(cmds, odom, imu, axis_name):
    """axis_name 'v' | 'w' 의 모든 계단 변화를 맞춰 [(종류, 신호원, a, b, L, τ, 도달 비율, rmse)]."""
    axis = 1 if axis_name == 'v' else 2
    sources = [('odom', odom, axis)] + ([('imu', imu, 1)] if axis_name == 'w' and len(imu) else [])
    res = []
    for t0, a, b, t_next in edges(cmds, axis):
        if abs(b - a) < 1e-6:
            continue
        kind = '출발' if abs(a) < 1e-6 else ('정지' if abs(b) < 1e-6 else '변경')
        for name, rows, idx in sources:
            seg = [(r[0] - t0, r[idx]) for r in rows if t0 - 0.2 <= r[0] < t_next]
            if len(seg) < 8:
                continue
            ts = np.array([s[0] for s in seg])
            ys = np.array([s[1] for s in seg])
            base = float(np.mean(ys[ts < 0])) if np.any(ts < 0) else a
            tail = ys[ts > 0.7 * ts.max()]
            final = float(np.mean(tail)) if len(tail) else b
            if abs(final - base) < 0.2 * abs(b - a):
                continue   # 거의 안 움직임 (막힘 등)
            keep = ts >= 0
            yn = (ys[keep] - base) / (final - base)
            L, tau, err = fit_fopdt(ts[keep], yn)
            ratio = (final - a) / (b - a) if kind != '정지' else (base - b) / (a - b)
            res.append((kind, name, a, b, L, tau, ratio, err))
    return res


def report(results, unit):
    groups = {}
    for kind, src, a, b, L, tau, ratio, err in results:
        groups.setdefault((kind, src), []).append((L, tau, ratio, err))
    for (kind, src), rows in sorted(groups.items()):
        arr = np.array(rows)
        med = np.median(arr, axis=0)
        print(f'  {kind} ({src}, {len(rows)}회): L = {med[0] * 1000:4.0f} ms '
              f'[{arr[:, 0].min() * 1000:.0f}~{arr[:, 0].max() * 1000:.0f}], '
              f'τ = {med[1] * 1000:4.0f} ms [{arr[:, 1].min() * 1000:.0f}~{arr[:, 1].max() * 1000:.0f}], '
              f'L+τ = {(med[0] + med[1]) * 1000:4.0f} ms, 도달 {med[2] * 100:.0f}%, 맞춤 오차 {med[3]:.3f}', flush=True)
    if groups:
        allv = np.array([[r[4], r[5]] for r in results if r[1] == 'odom'])
        if len(allv):
            print(f'  → 지연 보상용 (odom 기준 전체 중앙값): L ≈ {np.median(allv[:, 0]):.3f} s, '
                  f'τ ≈ {np.median(allv[:, 1]):.3f} s  [{unit}]', flush=True)


def save_csv(path, cmds, odom, imu):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w', newline='', encoding='utf-8') as f:
        wr = csv.writer(f)
        wr.writerow(['kind', 't', 'v', 'w'])
        for kind, rows in (('cmd', cmds), ('odom', odom)):
            for r in rows:
                wr.writerow([kind, f'{r[0]:.6f}', f'{r[1]:.6f}', f'{r[2]:.6f}'])
        for r in imu:
            wr.writerow(['imu', f'{r[0]:.6f}', '', f'{r[1]:.6f}'])


def load_csv(path):
    cmds, odom, imu = [], [], []
    with open(path, encoding='utf-8') as f:
        for row in csv.DictReader(f):
            t = float(row['t'])
            if row['kind'] == 'cmd':
                cmds.append((t, float(row['v']), float(row['w'])))
            elif row['kind'] == 'odom':
                odom.append((t, float(row['v']), float(row['w'])))
            else:
                imu.append((t, float(row['w'])))
    return cmds, odom, imu


def print_results(cmds, odom, imu, axes):
    if 'v' in axes:
        print('[latency] 직진 (m/s)', flush=True)
        report(analyze(cmds, odom, imu, 'v'), '직진')
    if 'w' in axes:
        print('[latency] 회전 (rad/s)', flush=True)
        report(analyze(cmds, odom, imu, 'w'), '회전')


# ------------------------------------------------------------------ 측정 (ROS)
def run(args):
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu, LaserScan

    rclpy.init()
    node = Node('latency_test', parameter_overrides=[
        rclpy.parameter.Parameter('use_sim_time', value=os.environ.get('WAREHOUSE_SIM_TIME') == 'true')])
    pub = node.create_publisher(Twist, 'cmd_vel', 1)
    cmds, odom, imu = [], [], []
    scan = {}
    lock = threading.Lock()

    def on_odom(m):
        with lock:
            odom.append((time.monotonic(), m.twist.twist.linear.x, m.twist.twist.angular.z))

    def on_imu(m):
        with lock:
            imu.append((time.monotonic(), m.angular_velocity.z))

    node.create_subscription(Odometry, 'odom', on_odom, qos_profile_sensor_data)
    node.create_subscription(Imu, 'imu', on_imu, qos_profile_sensor_data)
    node.create_subscription(LaserScan, 'scan', lambda m: scan.__setitem__('msg', m), qos_profile_sensor_data)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    def send(v, w):
        t = Twist()
        t.linear.x, t.angular.z = float(v), float(w)
        pub.publish(t)
        with lock:
            cmds.append((time.monotonic(), float(v), float(w)))

    def hold(v, w, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            send(v, w)
            time.sleep(1.0 / CMD_RATE)

    def clearance(back=False):
        """LiDAR 앞(back 이면 뒤) ±30° 안 가장 가까운 거리 (m). 스캔 없으면 None."""
        m = scan.get('msg')
        if m is None:
            return None
        r = np.asarray(m.ranges, dtype=float)
        ang = m.angle_min + np.arange(len(r)) * m.angle_increment
        center = math.pi if back else 0.0
        off = np.abs(np.arctan2(np.sin(ang - center), np.cos(ang - center)))
        ok = (off <= FRONT_HALF) & np.isfinite(r) & (r >= m.range_min) & (r < m.range_max)
        return float(r[ok].min()) if np.any(ok) else float('inf')

    def check_room(back):
        need = abs(args.v) * args.hold + CLEAR_EXTRA + (BODY_REAR if back else BODY_FRONT)
        c = clearance(back)
        if c is None:
            print('[latency] /scan 이 안 들어와 앞/뒤 여유를 볼 수 없습니다 (--no-scan-check 로 건너뜀)', flush=True)
            return args.no_scan_check
        if c < need:
            print(f'[latency] {"뒤" if back else "앞"}쪽 여유 {c:.2f} m < 필요 {need:.2f} m: 멈춥니다', flush=True)
            return False
        return True

    out = args.out or os.path.join(HOME, 'logs', time.strftime('latency_%Y%m%d_%H%M%S.csv'))
    try:
        if node.count_publishers('cmd_vel') > 1:
            print('[latency] 주의: cmd_vel 을 내는 다른 노드가 있습니다 (브리지 등). 웹 이동/자동 탐색이 없는지 확인하세요',
                  flush=True)
        t_wait = time.monotonic() + 5.0
        while time.monotonic() < t_wait and (not odom or (not args.no_scan_check and 'msg' not in scan)):
            time.sleep(0.1)
        if not odom:
            print('[latency] /odom 이 안 들어옵니다 (source scripts/env.sh, bringup 확인)', flush=True)
            return
        hold(0.0, 0.0, 1.0)
        for k in range(args.trials):
            print(f'[latency] 시행 {k + 1}/{args.trials}', flush=True)
            if args.axis in ('both', 'v'):
                for sign in (1, -1):          # 앞으로 갔다가 같은 만큼 뒤로 (제자리로)
                    if not args.no_scan_check and not check_room(back=sign < 0):
                        raise KeyboardInterrupt
                    hold(sign * args.v, 0.0, args.hold)
                    hold(0.0, 0.0, args.rest)
            if args.axis in ('both', 'w'):
                for sign in (1, -1):          # 왼쪽으로 돌았다가 되돌아옴
                    hold(0.0, sign * args.w, args.hold)
                    hold(0.0, 0.0, args.rest)
    except KeyboardInterrupt:
        print('[latency] 중지', flush=True)
    finally:
        for _ in range(10):
            send(0.0, 0.0)
            time.sleep(0.05)
        time.sleep(0.3)
        with lock:
            c, o, i = list(cmds), list(odom), list(imu)
        node.destroy_node()
        rclpy.try_shutdown()
    save_csv(out, c, o, i)
    print(f'[latency] 기록 저장: {out} (명령 {len(c)}, odom {len(o)}, imu {len(i)})', flush=True)
    if len(o) > 1:
        dt = np.diff([r[0] for r in o])
        print(f'[latency] odom 수신 간격 중앙값 {np.median(dt) * 1000:.0f} ms (최대 {dt.max() * 1000:.0f} ms)', flush=True)
    print_results(c, o, i, 'vw' if args.axis == 'both' else args.axis)


def main():
    ap = argparse.ArgumentParser(description='cmd_vel → odom 지연(L) / 시정수(τ) 측정')
    ap.add_argument('--v', type=float, default=0.13, help='직진 계단 크기 (m/s), 기본 = dwa.max_v')
    ap.add_argument('--w', type=float, default=1.0, help='회전 계단 크기 (rad/s)')
    ap.add_argument('--trials', type=int, default=5)
    ap.add_argument('--hold', type=float, default=2.0, help='계단 유지 시간 (s)')
    ap.add_argument('--rest', type=float, default=1.5, help='계단 사이 정지 시간 (s)')
    ap.add_argument('--axis', choices=('both', 'v', 'w'), default='both')
    ap.add_argument('--no-scan-check', action='store_true', help='LiDAR 앞/뒤 여유 확인을 건너뜀')
    ap.add_argument('--out', default='', help='원본 기록 csv (기본 logs/latency_<시각>.csv)')
    ap.add_argument('--analyze', default='', help='저장한 csv 만 다시 분석 (로봇을 움직이지 않음)')
    args = ap.parse_args()
    if args.analyze:
        print_results(*load_csv(args.analyze), 'vw')
        return
    run(args)


if __name__ == '__main__':
    main()
