"""카메라 영상에서 LED(물건)를 색으로 찾는 임시 인식기. ROS 의존성 없음 (단위 테스트 대상).

학습 모델(YOLO 등)이 준비되기 전에 배달 흐름을 시험하기 위한 것이다.
켜진 LED 는 밝고 채도가 높으므로, 라벨마다 HSV 범위를 주고 그 범위에 드는 픽셀 덩어리 중
가장 큰 것을 고른다. 학습 모델로 바꿀 때는 같은 라벨을 cargo/detected 토픽으로 내기만 하면 된다.
"""
import cv2
import numpy as np


def parse_ranges(labels, ranges):
    """labels: ['A', 'B'], ranges: 라벨별 [h_min, s_min, v_min, h_max, s_max, v_max] (OpenCV H 는 0~179).
    h_min > h_max 이면 빨강처럼 0 을 넘어가는 범위로 본다."""
    out = {}
    for label, r in zip(labels, ranges):
        if len(r) != 6:
            raise ValueError(f'{label}: HSV 범위는 값 6개여야 합니다 ({r})')
        out[label] = [int(v) for v in r]
    return out


def color_mask(hsv, r):
    h0, s0, v0, h1, s1, v1 = r
    if h0 <= h1:
        return cv2.inRange(hsv, (h0, s0, v0), (h1, s1, v1))
    # 빨강: h0~179 와 0~h1
    return cv2.inRange(hsv, (h0, s0, v0), (179, s1, v1)) | cv2.inRange(hsv, (0, s0, v0), (h1, s1, v1))


def detect(bgr, ranges, min_area_ratio=0.002, roi=None):
    """bgr: 영상 (H, W, 3). ranges: parse_ranges 결과. roi: (x0, y0, x1, y1) 비율 (0~1), 적재 칸만 볼 때.
    반환: (라벨 또는 '', 라벨별 가장 큰 덩어리 면적 비율 dict)."""
    h, w = bgr.shape[:2]
    if roi is not None:
        x0, y0, x1, y1 = roi
        bgr = bgr[int(y0 * h):int(y1 * h), int(x0 * w):int(x1 * w)]
        h, w = bgr.shape[:2]
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    kernel = np.ones((3, 3), np.uint8)
    areas = {}
    for label, r in ranges.items():
        mask = cv2.morphologyEx(color_mask(hsv, r), cv2.MORPH_OPEN, kernel)
        n, _, stats, _ = cv2.connectedComponentsWithStats(mask)
        areas[label] = float(stats[1:, cv2.CC_STAT_AREA].max()) / (h * w) if n > 1 else 0.0
    best = max(areas, key=areas.get) if areas else None
    if best is None or areas[best] < min_area_ratio:
        return '', areas
    return best, areas
