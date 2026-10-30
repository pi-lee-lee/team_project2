"""Tracking (tracker.py) tests. Taken from ~/tb3_nav_ws/src/tb3_nav/test/test_core.py."""
import numpy as np

from warehouse_robot.tracker import MultiObjectTracker, detect_clusters


def test_tracker_estimates_velocity():
    tr = MultiObjectTracker(min_hits=3)
    tracks = []
    for k in range(20):
        t = k * 0.1
        cx = 1.0 + 0.3 * t
        arc = np.array([[cx - 0.1 + 0.02 * i, 2.0 - 0.05 * np.sin(i / 10 * np.pi)]
                        for i in range(11)])
        tracks = tr.update(detect_clusters(arc, 0.15, 3, 0.8, 0.08), t)
    assert len(tracks) == 1
    assert abs(tracks[0].x[2] - 0.3) < 0.05   # state [x, y, vx, vy]
    assert abs(tracks[0].x[3]) < 0.05


def test_cluster_center_corrected_away_from_sensor():
    # front arc of a radius-0.12 cylinder (center (1.0, 0)) as seen by a sensor at the origin
    ang = np.linspace(np.pi - 0.9, np.pi + 0.9, 15)
    arc = np.column_stack([1.0 + 0.12 * np.cos(ang), 0.12 * np.sin(ang)])
    raw = detect_clusters(arc, 0.15, 3, 0.8, 0.08)[0]
    fixed = detect_clusters(arc, 0.15, 3, 0.8, 0.08, sensor_xy=(0.0, 0.0))[0]
    assert abs(fixed[0] - 1.0) < abs(raw[0] - 1.0)
    assert abs(fixed[0] - 1.0) < 0.04
