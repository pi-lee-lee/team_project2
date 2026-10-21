"""실물 LiDAR(LDS-02) 측정 범위 확인. 로봇을 세워 둔 채로 스캔을 모아, 거리대별로 얼마나 안정적으로 돌아오는지 본다.
AMCL laser_max_range / slam_toolbox max_laser_range 를 3.5m 보다 늘려도 되는지 판단할 때 쓴다.

    source scripts/env.sh
    python3 tools/scan_check.py [초=20] [--topic scan]

  - 로봇은 움직이지 말고, 강당 가운데처럼 여러 거리에 벽/물체가 보이는 곳에 두세요 (사람이 지나가지 않게)
  - 1° 방향마다 스캔 사이 값을 비교: 응답률(유효 값이 돌아온 비율)과 흔들림(표준편차)을 거리대별로 집계
  - 추천 최대 거리 = 응답률 80% 이상, 흔들림 5cm 이하인 가장 먼 거리대의 끝 (드라이버 range_max 를 넘지 않게)
"""
import argparse
import math
import time

import numpy as np

BAND = 0.5          # m, 거리대 폭
MIN_RETURN = 0.8    # 추천 기준: 응답률
MAX_STD = 0.05      # 추천 기준: 흔들림 (m)


def analyze(scans):
    """scans: [(angle_min, angle_increment, ranges, range_min, range_max), ...] (로봇 정지).
    반환: dict (빔 개수, 무응답 종류 비율, 거리대별 [끝 거리, 방향 수, 응답률, 흔들림], 추천 최대 거리)."""
    counts = [len(s[2]) for s in scans]
    all_r = np.concatenate([np.asarray(s[2], dtype=float) for s in scans])
    range_max = max(s[4] for s in scans)
    info = {
        'scans': len(scans), 'beams_min': min(counts), 'beams_max': max(counts),
        'range_min': min(s[3] for s in scans), 'range_max': range_max,
        'inf': float(np.isinf(all_r).mean()), 'nan': float(np.isnan(all_r).mean()),
        'zero': float((all_r == 0).mean()),
    }
    # 1° 방향 칸마다 스캔별 값 (빔 개수가 스캔마다 달라도 같은 방향끼리 비교)
    per_dir = [[] for _ in range(360)]
    for amin, inc, ranges, rmin, rmax in scans:
        r = np.asarray(ranges, dtype=float)
        ang = np.degrees(amin + np.arange(len(r)) * inc) % 360.0
        idx = np.floor(ang).astype(int) % 360
        valid = np.isfinite(r) & (r > rmin) & (r < rmax)
        for d in range(360):
            sel = idx == d
            if sel.any():
                v = r[sel & valid]
                per_dir[d].append(float(np.min(v)) if len(v) else np.nan)
    n_bands = int(math.ceil(range_max / BAND))
    bands = [[0, 0.0, []] for _ in range(n_bands)]   # [방향 수, 응답률 합, 흔들림 목록]
    for vals in per_dir:
        v = np.array(vals)
        ok = np.isfinite(v)
        if ok.sum() < max(3, 0.2 * len(v)):
            continue   # 대부분 무응답인 방향은 어느 거리대인지 모름
        med = float(np.median(v[ok]))
        b = min(int(med / BAND), n_bands - 1)
        bands[b][0] += 1
        bands[b][1] += ok.mean()
        if ok.sum() >= 3:
            bands[b][2].append(float(np.std(v[ok])))
    table, best = [], None
    for i, (n, ret, stds) in enumerate(bands):
        if n == 0:
            continue
        rate = ret / n
        std = float(np.median(stds)) if stds else float('nan')
        end = min((i + 1) * BAND, range_max)
        table.append((end, n, rate, std))
        if rate >= MIN_RETURN and std <= MAX_STD:
            best = end
    info['bands'] = table
    info['recommend'] = best
    return info


def report(info):
    print(f'스캔 {info["scans"]}개, 빔 개수 {info["beams_min"]}~{info["beams_max"]}, '
          f'드라이버 범위 {info["range_min"]:.2f}~{info["range_max"]:.2f} m')
    print(f'무응답 값: inf {info["inf"] * 100:.1f}%, NaN {info["nan"] * 100:.1f}%, 0 {info["zero"] * 100:.1f}%')
    print('거리대(m)    방향 수   응답률   흔들림(cm)')
    for end, n, rate, std in info['bands']:
        mark = ' ✓' if rate >= MIN_RETURN and std <= MAX_STD else ''
        print(f'{max(0.0, end - BAND):4.1f}~{end:<4.1f}  {n:8d}  {rate * 100:6.0f}%  {std * 100:9.1f}{mark}')
    rec = info['recommend']
    if rec is None:
        print('추천: 기준(응답률 80%, 흔들림 5cm)을 만족하는 거리대가 없습니다. 벽이 보이는 곳에서 다시 재 보세요.')
        return
    print(f'추천 최대 거리: {rec:.1f} m  (지금 설정: AMCL laser_max_range 3.5, slam_toolbox max_laser_range 3.4)')
    if rec > 3.5:
        print(f'  → config/amcl_hall.yaml 의 laser_max_range 를 {rec:.1f} 로, '
              f'config/slam_toolbox.yaml 의 max_laser_range 를 {rec - 0.1:.1f} 로 바꿔 볼 수 있습니다')
        print('    (먼 벽이 실제로 보이는지 웹 지도에서 스캔 점도 같이 확인하세요)')
    else:
        print('  → 3.5 m 보다 늘리지 마세요 (지금 설정 유지)')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('seconds', nargs='?', type=float, default=20.0)
    ap.add_argument('--topic', default='scan')
    args = ap.parse_args()

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    rclpy.init()
    node = Node('scan_check')
    scans, stamps = [], []

    def on_scan(m):
        scans.append((m.angle_min, m.angle_increment, list(m.ranges), m.range_min, m.range_max))
        stamps.append(time.monotonic())

    node.create_subscription(LaserScan, args.topic, on_scan, qos_profile_sensor_data)
    print(f'/{args.topic} 를 {args.seconds:.0f}초 동안 모읍니다. 로봇을 움직이지 마세요...')
    end = time.monotonic() + args.seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.try_shutdown()
    if len(scans) < 5:
        print(f'스캔을 {len(scans)}개밖에 못 받았습니다. Pi bringup 과 도메인(source scripts/env.sh)을 확인하세요.')
        return
    print(f'스캔 주기 {(len(stamps) - 1) / (stamps[-1] - stamps[0]):.1f} Hz')
    report(analyze(scans))


if __name__ == '__main__':
    main()
