import cv2
import numpy as np

from warehouse_robot.cargo_detect import detect, parse_ranges

RANGES = parse_ranges(['A', 'B', 'C'], [[170, 120, 180, 10, 255, 255],
                                        [45, 120, 180, 85, 255, 255],
                                        [100, 120, 180, 130, 255, 255]])


def scene(color_bgr=None, size=40):
    """어두운 적재 칸 영상에 밝은 LED 원 하나."""
    img = np.full((240, 320, 3), 40, np.uint8)
    if color_bgr is not None:
        cv2.circle(img, (160, 120), size // 2, color_bgr, -1)
    return img


def test_empty_tray_is_empty():
    assert detect(scene(), RANGES)[0] == ''


def test_each_led_color():
    assert detect(scene((0, 0, 255)), RANGES)[0] == 'A'      # 빨강 (색상 0 근처, 범위가 0 을 넘어감)
    assert detect(scene((0, 255, 0)), RANGES)[0] == 'B'      # 초록
    assert detect(scene((255, 0, 0)), RANGES)[0] == 'C'      # 파랑


def test_tiny_blob_ignored():
    assert detect(scene((0, 255, 0), size=4), RANGES)[0] == ''


def test_roi_excludes_outside():
    img = scene()
    cv2.circle(img, (20, 20), 15, (0, 255, 0), -1)          # 적재 칸 밖 (왼쪽 위)
    assert detect(img, RANGES)[0] == 'B'
    assert detect(img, RANGES, roi=(0.3, 0.3, 1.0, 1.0))[0] == ''
