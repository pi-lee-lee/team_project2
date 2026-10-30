"""LiDAR point clustering + Kalman filter multi-object tracking. No ROS dependency.
(taken from ~/tb3_nav_ws/src/tb3_nav/tb3_nav/core/tracker.py)

Flow: scan points (map frame) -> remove points on map walls -> group consecutive points (segment)
      -> each cluster's center = detection -> match with existing tracks -> KF update
"""
from dataclasses import dataclass, field
from itertools import count

import numpy as np


def segment_points(points, gap):
    """Split points (N,2) in scan order into a list of clusters wherever neighbors are more than gap apart."""
    if len(points) == 0:
        return []
    jumps = np.linalg.norm(np.diff(points, axis=0), axis=1) > gap
    segments = np.split(points, np.nonzero(jumps)[0] + 1)
    # in a 360-degree scan the last and first points are neighbors
    if len(segments) > 1 and np.linalg.norm(points[-1] - points[0]) <= gap:
        segments[0] = np.vstack([segments[-1], segments[0]])
        segments.pop()
    return segments


def detect_clusters(points, gap, min_points, max_diameter, min_radius, sensor_xy=None):
    """Clusters -> list of (cx, cy, radius). Clusters that are too large (walls etc.) are dropped.

    The LiDAR only sees the front arc of an object, so the point mean is biased toward the sensor.
    If sensor_xy is given, the center is pushed away from the sensor by the radius to correct this.
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
    P: np.ndarray                # 4x4 covariance
    radius: float
    id: int = field(default_factory=lambda: next(_ids))
    hits: int = 1
    misses: int = 0

    @property
    def position(self):
        return self.x[:2]


class MultiObjectTracker:
    """Constant-velocity (CV) Kalman filter + greedy nearest matching."""

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
        # clip unrealistic speeds (prevents jumps from mismatches)
        speed = np.linalg.norm(track.x[2:])
        if speed > self.max_speed:
            track.x[2:] *= self.max_speed / speed
        track.radius = 0.7 * track.radius + 0.3 * radius

    def update(self, detections, stamp):
        """detections: [(cx, cy, r)], stamp: time in seconds. Returns the list of confirmed tracks."""
        dt = 0.0 if self.last_time is None else max(0.0, stamp - self.last_time)
        self.last_time = stamp
        if dt > 0.0:
            self._predict(dt)

        # greedy matching: closest pairs first
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
