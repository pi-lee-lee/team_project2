"""Robot velocity command selector. No ROS dependency (unit tested).

Only the bridge publishes to /cmd_vel; commands from several sources are reduced to one by priority:
  stop (emergency stop, right after task cancel)  >  manual (web / external keyboard cmd_vel_teleop)  >  auto (Frenet follower/back-off)
This keeps an external teleop_keyboard and the bridge from alternating on the same /cmd_vel, which makes the robot stutter.

Manual commands are acceleration-limited for smooth start/stop (button/key presses change the target speed in steps).
Auto commands are sent as is, since the follower already computes the speed.
"""


class CmdMux:
    def __init__(self, manual_acc_v=0.4, manual_dec_v=0.8, manual_acc_w=2.0,
                 manual_timeout=0.4, auto_timeout=0.3):
        self.manual_acc_v = manual_acc_v     # m/s² manual acceleration
        self.manual_dec_v = manual_dec_v     # m/s² manual deceleration (stopping is faster)
        self.manual_acc_w = manual_acc_w     # rad/s²
        self.manual_timeout = manual_timeout  # stop if no manual command for longer than this (button released, connection lost)
        self.auto_timeout = auto_timeout
        self._auto = None      # (v, w, time)
        self._manual = None    # (v, w, time)
        self._stop_until = 0.0
        self.out = (0.0, 0.0)  # last command sent
        self.source = 'idle'   # stop | manual | auto | idle
        self._idle_sent = True
        self._last = None

    def auto(self, v, w, now):
        self._auto = (float(v), float(w), now)

    def manual(self, v, w, now):
        self._manual = (float(v), float(w), now)

    def stop(self, now, hold=0.0):
        """Stop immediately and ignore other commands for hold seconds."""
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
        """Step cur toward target by up when growing, by down when shrinking (toward 0)."""
        lim = (down if abs(target) < abs(cur) or cur * target < 0 else up) * dt
        return cur + max(-lim, min(lim, target - cur))

    def step(self, now):
        """(v, w) to send, or None (nothing to send: still stopped)."""
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
            # manual control ended: stop smoothly
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
        # no commands at all: send 0 once, then stay quiet
        self.out, self.source = (0.0, 0.0), 'idle'
        if self._idle_sent:
            return None
        self._idle_sent = True
        return self.out
