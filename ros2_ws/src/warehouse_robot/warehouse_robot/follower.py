"""A* 경로를 따라가는 pure pursuit 추종기 (ROS 의존성 없음).

매 주기 현재 위치를 넣으면 (선속도, 각속도, 상태)를 돌려준다.
    상태: 'following' / 'rotating' / 'done' / 'off_path'

속도는 점진적으로 바꾼다:
    출발/가속: 한 주기에 max_acc * dt 까지만 올림 (지수 램프는 출발 순간 가속이 가장 커서 쓰지 않음)
    정지/감속: 남은 거리 d 에서 v <= sqrt(2 * max_acc * d) (일정 감속으로 딱 맞게 멈춤.
              예전 v = 0.6 d + 0.03 은 남은 거리가 지수로 줄어 끝에서 늘어졌음)
    제자리 회전도 같은 방식 (max_acc_w)
"""
import math
import time


def normalize(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class PurePursuit:
    def __init__(self, points, goal_yaw, max_v=0.15, max_w=1.0, lookahead=0.3,
                 xy_tolerance=0.08, yaw_tolerance=0.1, rotate_threshold=0.8, max_offset=0.5,
                 max_acc=0.3, max_acc_w=1.5, min_v=0.03, period=0.05):
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
        self.max_acc = max_acc                    # m/s², 출발 가속과 정지 감속
        self.max_acc_w = max_acc_w                # rad/s², 제자리 회전
        self.min_v = min_v                        # 목표 바로 앞에서도 이 속도는 냄 (너무 느리면 실제 로봇이 안 움직임)
        self.period = period                      # 첫 주기/공백 뒤의 dt (브리지 제어 주기)
        self.v = 0.0                              # 직전 명령 (램프 기준)
        self.w = 0.0
        self.last_t = None

    def set_speed(self, v, w):
        """추종기 명령 대신 다른 속도가 나갔을 때 알림 (앞이 막혀 정지 → 0, DWA 에서 넘겨받음 → 현재 속도)."""
        self.v, self.w, self.last_t = v, w, None

    def _dt(self, now):
        if self.last_t is None:
            dt = self.period
        else:
            dt = now - self.last_t
            if dt > 10 * self.period:             # 한동안 안 불림 (물러나기/회복 동작 중) → 서 있었다고 봄
                self.v = self.w = 0.0
                dt = self.period
            elif dt <= 0.0:                       # 시각이 거꾸로 (다른 시계로 불림)
                dt = self.period
        self.last_t = now
        return dt

    @staticmethod
    def _limit(prev, target, rate, dt):
        """prev 에서 target 으로 한 주기만큼. 커질 때는 rate, 줄 때는 정지 프로필을 늦지 않게 따르도록 2배."""
        up = abs(target) > abs(prev) and target * prev >= 0.0
        step = rate * dt * (1.0 if up else 2.0)
        return prev + max(-step, min(step, target - prev))

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

    def _turn(self, error, dt):
        # 남은 각도를 일정 각감속으로 딱 맞게 멈출 수 있는 속도 이하, 각가속도 한도 안에서
        w = min(self.max_w, 2.0 * abs(error), math.sqrt(2.0 * self.max_acc_w * abs(error)))
        w = self._limit(self.w, math.copysign(w, error), self.max_acc_w, dt)
        # 아주 작은 각속도로는 실제 로봇이 돌지 않으므로 최소값 보장 (반대쪽으로 줄어드는 중이면 그대로)
        if abs(w) < 0.15 and w * error >= 0.0:
            w = math.copysign(0.15, error)
        return w

    def _stop(self, state):
        self.v = self.w = 0.0
        return 0.0, 0.0, state

    def compute(self, x, y, yaw, now=None):
        """now: 단조 시각 (s). 생략하면 time.monotonic()"""
        dt = self._dt(time.monotonic() if now is None else now)
        # 목표 위치에 도착한 뒤에는 방향만 맞춘다
        if self.at_goal or self.distance_to_goal(x, y) <= self.xy_tolerance:
            self.at_goal = True
            error = normalize(self.goal_yaw - yaw)
            if abs(error) <= self.yaw_tolerance:
                return self._stop('done')
            self.v, self.w = 0.0, self._turn(error, dt)
            return 0.0, self.w, 'rotating'

        offset = self._advance_index(x, y)
        if offset > self.max_offset:
            return self._stop('off_path')

        lx, ly = self._lookahead_point(x, y)
        alpha = normalize(math.atan2(ly - y, lx - x) - yaw)
        if abs(alpha) > self.rotate_threshold:
            # 달리던 중이면 선속도는 감속 한도로 줄이며 돈다
            self.v = max(0.0, self.v - 2.0 * self.max_acc * dt)
            self.w = self._turn(alpha, dt)
            return self.v, self.w, 'rotating'

        dist_goal = self.distance_to_goal(x, y)
        # 방향 차이가 클수록 감속, 목표 앞에서는 허용 오차 경계에서 멈추는 일정 감속 프로필 이하
        v = self.max_v * (1.0 - 0.5 * abs(alpha) / self.rotate_threshold)
        v = min(v, max(self.min_v, math.sqrt(2.0 * self.max_acc * max(dist_goal - self.xy_tolerance, 0.0))))
        v = self._limit(self.v, v, self.max_acc, dt)
        ld = max(math.hypot(lx - x, ly - y), 1e-3)
        # 각속도는 경로 곡률 x 선속도라 선속도 램프를 그대로 따라간다
        w = v * 2.0 * math.sin(alpha) / ld
        w = max(-self.max_w, min(self.max_w, w))
        self.v, self.w = v, w
        return v, w, 'following'
