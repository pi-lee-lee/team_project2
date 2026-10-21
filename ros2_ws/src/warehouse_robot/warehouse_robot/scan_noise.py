"""시뮬레이션 LiDAR 를 실물 LDS-02 처럼 만들기 (ROS 의존성 없음, numpy만 사용).

Gazebo 스캔은 360개 빔이 늘 같은 각도로, 거의 흔들림 없이 들어오지만 실물 LDS-02 는
  - 빔 개수가 스캔마다 238~244 개로 바뀌고 (slam_toolbox 가 스캔을 버리던 원인, bridge_node.to_slam_scan 참고)
  - 거리에 따라 흔들림이 커지고
  - 반사가 약하면(먼 곳, 검은 물체, 비스듬한 면) 값이 0 으로 빠진다.
브리지가 시뮬레이션에서 sim_scan.enabled 일 때 /scan 을 받자마자 이 함수로 바꿔서, 안전 정지/추적/AMCL/SLAM 모두
같은 '실물 같은' 스캔을 쓰게 한다. 실물 로봇에서는 쓰지 않는다.
"""
import math
from dataclasses import dataclass

import numpy as np


@dataclass
class ScanNoiseConfig:
    beams_min: int = 238        # 한 바퀴 빔 개수 범위 (실물 LDS-02)
    beams_max: int = 244
    sigma0: float = 0.01        # 거리 흔들림 표준편차 = sigma0 + sigma_k * 거리 (m)
    sigma_k: float = 0.01
    drop_base: float = 0.02     # 가까워도 값이 빠지는 비율
    drop_far: float = 0.35      # far_start ~ max_range 사이에서 추가로 빠지는 비율 (거리에 비례해 늘어남)
    far_start: float = 2.5      # m
    max_range: float = 3.5      # m, 이보다 먼 값은 응답 없음(inf). 실물 측정값(tools/scan_check.py)으로 맞출 것


def degrade(ranges, angle_min, angle_increment, range_min, rng, cfg=None):
    """반환: (새 ranges 배열, 새 angle_min, 새 angle_increment). 응답 없는 빔은 0 (실물처럼)."""
    cfg = cfg or ScanNoiseConfig()
    src = np.asarray(ranges, dtype=float)
    n_src = len(src)
    if n_src == 0:
        return src, angle_min, angle_increment
    n = int(rng.integers(cfg.beams_min, cfg.beams_max + 1))
    inc = 2.0 * math.pi / n
    start = angle_min + float(rng.uniform(0.0, inc))      # 시작 각도도 스캔마다 조금씩 다름
    ang = start + np.arange(n) * inc
    # 가장 가까운 원래 빔 값을 쓴다 (한 바퀴를 넘으면 앞에서 다시)
    idx = np.round((ang - angle_min) / angle_increment).astype(int) % n_src
    r = src[idx].copy()
    hit = np.isfinite(r) & (r > range_min)
    sigma = cfg.sigma0 + cfg.sigma_k * np.where(hit, r, 0.0)
    r = np.where(hit, r + rng.normal(0.0, 1.0, n) * sigma, r)
    far = np.clip((r - cfg.far_start) / max(cfg.max_range - cfg.far_start, 1e-6), 0.0, 1.0)
    drop = hit & (rng.random(n) < cfg.drop_base + cfg.drop_far * far)
    r[drop] = 0.0
    r[hit & ~drop & (r >= cfg.max_range)] = np.inf
    return r, start, inc
