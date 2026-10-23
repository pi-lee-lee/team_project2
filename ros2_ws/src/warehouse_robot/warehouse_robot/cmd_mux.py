"""로봇 속도 명령 선택기. ROS 의존성 없음 (단위 테스트 대상).

브리지만 /cmd_vel 에 내보내고, 여러 곳의 명령을 우선순위대로 하나로 고른다:
  정지(비상정지, 작업 취소 직후)  >  수동 조작(웹 / 외부 키보드 cmd_vel_teleop)  >  자동 이동(Frenet 추종기/물러나기)
예전에는 외부 teleop_keyboard 와 브리지가 같은 /cmd_vel 에 번갈아 내보내 로봇이 끊기듯 움직였다.

수동 조작은 가속을 제한해 부드럽게 출발/정지한다 (사람이 버튼/키를 누르면 목표 속도가 계단처럼 바뀌므로).
자동 이동 명령은 추종기가 이미 속도를 계산하므로 그대로 보낸다.
"""


class CmdMux:
    def __init__(self, manual_acc_v=0.4, manual_dec_v=0.8, manual_acc_w=2.0,
                 manual_timeout=0.4, auto_timeout=0.3):
        self.manual_acc_v = manual_acc_v     # m/s² 수동 가속
        self.manual_dec_v = manual_dec_v     # m/s² 수동 감속 (멈출 때는 더 빨리)
        self.manual_acc_w = manual_acc_w     # rad/s²
        self.manual_timeout = manual_timeout  # 이 시간 넘게 수동 명령이 없으면 멈춤 (버튼에서 손 뗌, 연결 끊김)
        self.auto_timeout = auto_timeout
        self._auto = None      # (v, w, 시각)
        self._manual = None    # (v, w, 시각)
        self._stop_until = 0.0
        self.out = (0.0, 0.0)  # 마지막으로 내보낸 명령
        self.source = 'idle'   # stop | manual | auto | idle
        self._idle_sent = True
        self._last = None

    def auto(self, v, w, now):
        self._auto = (float(v), float(w), now)

    def manual(self, v, w, now):
        self._manual = (float(v), float(w), now)

    def stop(self, now, hold=0.0):
        """즉시 정지, hold 초 동안 다른 명령 무시."""
        self._stop_until = max(self._stop_until, now + hold)
        self._manual = None
        self._auto = None
        self.out = (0.0, 0.0)
        self.source = 'stop'
        self._idle_sent = False

    def manual_active(self, now):
        return self._manual is not None and now - self._manual[2] <= self.manual_timeout

    @staticmethod
    def _toward(cur, target, up, down, dt):
        """cur 에서 target 으로, 커질 때 up, 작아질 때(0 쪽) down 만큼씩."""
        lim = (down if abs(target) < abs(cur) or cur * target < 0 else up) * dt
        return cur + max(-lim, min(lim, target - cur))

    def step(self, now):
        """내보낼 (v, w) 또는 None (보낼 것 없음: 계속 멈춰 있음)."""
        dt = 0.05 if self._last is None else min(0.2, max(0.0, now - self._last))
        self._last = now
        if now < self._stop_until:
            self.out, self.source = (0.0, 0.0), 'stop'
            return self.out
        if self.manual_active(now):
            v, w, _ = self._manual
            cv, cw = self.out
            self.out = (self._toward(cv, v, self.manual_acc_v, self.manual_dec_v, dt),
                        self._toward(cw, w, self.manual_acc_w, self.manual_acc_w * 2, dt))
            self.source = 'manual'
            self._idle_sent = False
            return self.out
        if self.source in ('manual', 'manual_stop') and self.out != (0.0, 0.0):
            # 수동 조작이 끝남: 부드럽게 멈춤
            self._manual = None
            self.out = (self._toward(self.out[0], 0.0, 0.0, self.manual_dec_v, dt),
                        self._toward(self.out[1], 0.0, 0.0, self.manual_acc_w * 2, dt))
            self.source = 'manual_stop'
            return self.out
        if self._auto is not None and now - self._auto[2] <= self.auto_timeout:
            self.out = self._auto[:2]
            self.source = 'auto'
            self._idle_sent = False
            return self.out
        # 아무 명령도 없음: 한 번만 0 을 보내고 조용히
        self.out, self.source = (0.0, 0.0), 'idle'
        if self._idle_sent:
            return None
        self._idle_sent = True
        return self.out
