"""LiDAR 점 군집화 + 칼만필터 다중 객체 추적. ROS 의존성 없음.
(~/tb3_nav_ws/src/tb3_nav/tb3_nav/core/tracker.py 에서 가져옴)

흐름: 스캔 점(map frame) -> 지도 벽에 붙은 점 제거 -> 연속 점끼리 묶기(segment)
      -> 각 덩어리의 중심 = 관측(detection) -> 기존 track 과 매칭 -> KF 갱신
"""
from dataclasses import dataclass, field
from itertools import count

import numpy as np


def segment_points(points, gap):
    """스캔 순서대로 정렬된 점 (N,2) 를 인접 거리 gap 기준으로 끊어 덩어리 리스트로."""
    if len(points) == 0:
        return []
    jumps = np.linalg.norm(np.diff(points, axis=0), axis=1) > gap
    segments = np.split(points, np.nonzero(jumps)[0] + 1)
    # 360도 스캔은 마지막 점과 첫 점이 이웃이다
    if len(segments) > 1 and np.linalg.norm(points[-1] - points[0]) <= gap:
        segments[0] = np.vstack([segments[-1], segments[0]])
        segments.pop()
    return segments


def detect_clusters(points, gap, min_points, max_diameter, min_radius, sensor_xy=None):
    """덩어리 -> (cx, cy, radius) 리스트. 너무 큰 덩어리(벽 등)는 버린다.

    라이다는 물체의 앞면(호)만 보므로 점 평균은 실제 중심보다 센서 쪽으로 치우친다.
    sensor_xy 를 주면 중심을 센서 반대 방향으로 반지름만큼 밀어 보정한다.
    """
    detections = []
    for seg in segment_points(points, gap):
        if len(seg) < min_points:
            continue
        diameter = np.linalg.norm(seg.max(axis=0) - seg.min(axis=0))
        if diameter > max_diameter:
            continue
        center = seg.mean(axis=0)
        radius = max(min_radius, float(np.max(np.linalg.norm(seg - center, axis=1))))
        if sensor_xy is not None:
            u = center - np.asarray(sensor_xy, dtype=float)
            center = center + u / max(np.linalg.norm(u), 1e-6) * radius
        detections.append((float(center[0]), float(center[1]), radius))
    return detections


_ids = count(1)


@dataclass
class Track:
    x: np.ndarray                # [px, py, vx, vy]
    P: np.ndarray                # 4x4 공분산
    radius: float
    id: int = field(default_factory=lambda: next(_ids))
    hits: int = 1
    misses: int = 0

    @property
    def position(self):
        return self.x[:2]


class MultiObjectTracker:
    """등속(CV) 모델 칼만필터 + 최근접(greedy) 매칭."""

    H = np.array([[1.0, 0, 0, 0], [0, 1.0, 0, 0]])

    def __init__(self, gate=0.5, min_hits=3, max_misses=5,
                 accel_noise=1.0, meas_noise=0.05, max_speed=1.5):
        self.gate = gate
        self.min_hits = min_hits
        self.max_misses = max_misses
        self.accel_noise = accel_noise
        self.R = np.eye(2) * meas_noise ** 2
        self.max_speed = max_speed
        self.tracks = []
        self.last_time = None

    def _predict(self, dt):
        F = np.eye(4)
        F[0, 2] = F[1, 3] = dt
        G = np.array([[0.5 * dt * dt, 0], [0, 0.5 * dt * dt], [dt, 0], [0, dt]])
        Q = G @ G.T * self.accel_noise ** 2
        for t in self.tracks:
            t.x = F @ t.x
            t.P = F @ t.P @ F.T + Q

    def _correct(self, track, z, radius):
        y = np.asarray(z) - self.H @ track.x
        S = self.H @ track.P @ self.H.T + self.R
        K = track.P @ self.H.T @ np.linalg.inv(S)
        track.x = track.x + K @ y
        track.P = (np.eye(4) - K @ self.H) @ track.P
        # 비현실적인 속도는 자른다 (매칭 실수로 튀는 것 방지)
        speed = np.linalg.norm(track.x[2:])
        if speed > self.max_speed:
            track.x[2:] *= self.max_speed / speed
        track.radius = 0.7 * track.radius + 0.3 * radius

    def update(self, detections, stamp):
        """detections: [(cx, cy, r)], stamp: 초 단위 시간. 확정된 track 리스트 반환."""
        dt = 0.0 if self.last_time is None else max(0.0, stamp - self.last_time)
        self.last_time = stamp
        if dt > 0.0:
            self._predict(dt)

        # greedy 매칭: 거리 작은 쌍부터
        pairs = []
        for ti, t in enumerate(self.tracks):
            for di, (cx, cy, _) in enumerate(detections):
                d = np.hypot(cx - t.x[0], cy - t.x[1])
                if d < self.gate:
                    pairs.append((d, ti, di))
        pairs.sort()
        used_t, used_d = set(), set()
        for _, ti, di in pairs:
            if ti in used_t or di in used_d:
                continue
            used_t.add(ti)
            used_d.add(di)
            t = self.tracks[ti]
            cx, cy, r = detections[di]
            self._correct(t, (cx, cy), r)
            t.hits += 1
            t.misses = 0

        for ti, t in enumerate(self.tracks):
            if ti not in used_t:
                t.misses += 1
        self.tracks = [t for t in self.tracks if t.misses <= self.max_misses]

        for di, (cx, cy, r) in enumerate(detections):
            if di not in used_d:
                P = np.diag([self.R[0, 0], self.R[1, 1], 1.0, 1.0])
                self.tracks.append(Track(np.array([cx, cy, 0.0, 0.0]), P, r))

        return [t for t in self.tracks if t.hits >= self.min_hits]
