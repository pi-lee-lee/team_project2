"""A* 경로를 따라가는 pure pursuit 추종기 (ROS 의존성 없음).

매 주기 현재 위치를 넣으면 (선속도, 각속도, 상태)를 돌려준다.
    상태: 'following' / 'rotating' / 'done' / 'off_path'
"""
import math


def normalize(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class PurePursuit:
    def __init__(self, points, goal_yaw, max_v=0.15, max_w=1.0, lookahead=0.3,
                 xy_tolerance=0.08, yaw_tolerance=0.1, rotate_threshold=0.8, max_offset=0.5):
        if not points:
            raise ValueError('빈 경로')
        self.points = [tuple(p) for p in points]
        self.goal_yaw = goal_yaw
        self.max_v = max_v
        self.max_w = max_w
        self.lookahead = lookahead
        self.xy_tolerance = xy_tolerance
        self.yaw_tolerance = yaw_tolerance
        self.rotate_threshold = rotate_threshold  # 목표 방향과 이 각도 이상 차이 나면 제자리 회전
        self.max_offset = max_offset              # 경로에서 이만큼 벗어나면 off_path
        self.index = 0                            # 지나온 경로 점 (뒤로 가지 않음)
        self.at_goal = False

    def distance_to_goal(self, x, y):
        gx, gy = self.points[-1]
        return math.hypot(gx - x, gy - y)

    def _advance_index(self, x, y):
        """현재 index부터 앞쪽 일정 구간에서 가장 가까운 점으로 index를 갱신. 그 거리를 반환."""
        best_i, best_d = self.index, math.inf
        for i in range(self.index, min(len(self.points), self.index + 40)):
            d = math.hypot(self.points[i][0] - x, self.points[i][1] - y)
            if d < best_d:
                best_i, best_d = i, d
        self.index = best_i
        return best_d

    def _lookahead_point(self, x, y):
        for i in range(self.index, len(self.points)):
            px, py = self.points[i]
            if math.hypot(px - x, py - y) >= self.lookahead:
                return px, py
        return self.points[-1]

    def _turn(self, error):
        w = max(-self.max_w, min(self.max_w, 2.0 * error))
        # 아주 작은 각속도로는 실제 로봇이 돌지 않으므로 최소값 보장
        if abs(w) < 0.15:
            w = math.copysign(0.15, error)
        return w

    def compute(self, x, y, yaw):
        # 목표 위치에 도착한 뒤에는 방향만 맞춘다
        if self.at_goal or self.distance_to_goal(x, y) <= self.xy_tolerance:
            self.at_goal = True
            error = normalize(self.goal_yaw - yaw)
            if abs(error) <= self.yaw_tolerance:
                return 0.0, 0.0, 'done'
            return 0.0, self._turn(error), 'rotating'

        offset = self._advance_index(x, y)
        if offset > self.max_offset:
            return 0.0, 0.0, 'off_path'

        lx, ly = self._lookahead_point(x, y)
        alpha = normalize(math.atan2(ly - y, lx - x) - yaw)
        if abs(alpha) > self.rotate_threshold:
            return 0.0, self._turn(alpha), 'rotating'

        dist_goal = self.distance_to_goal(x, y)
        # 방향 차이가 클수록, 목표에 가까울수록 감속
        v = self.max_v * (1.0 - 0.5 * abs(alpha) / self.rotate_threshold)
        v = min(v, 0.6 * dist_goal + 0.03)
        ld = max(math.hypot(lx - x, ly - y), 1e-3)
        w = v * 2.0 * math.sin(alpha) / ld
        w = max(-self.max_w, min(self.max_w, w))
        return v, w, 'following'
