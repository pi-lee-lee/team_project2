from warehouse_robot.cmd_mux import CmdMux


def run(mux, t0, t1, dt=0.05):
    out = []
    t = t0
    while t < t1 - 1e-9:
        out.append(mux.step(t))
        t += dt
    return out


def test_manual_ramps_up_smoothly():
    m = CmdMux(manual_acc_v=0.4)
    for k in range(10):           # 0.5초 동안 0.2 m/s 요청
        m.manual(0.2, 0.0, k * 0.05)
        v, w = m.step(k * 0.05)
        assert v <= 0.4 * (k + 1) * 0.05 + 1e-9
    assert 0.15 <= v <= 0.2


def test_manual_release_stops_smoothly_then_idle():
    m = CmdMux(manual_acc_v=10.0, manual_dec_v=0.8, manual_timeout=0.4)
    for k in range(5):
        m.manual(0.2, 0.0, k * 0.05)
        m.step(k * 0.05)
    outs = run(m, 0.25 + 0.4, 2.0)     # 손 뗀 뒤
    vs = [o[0] for o in outs if o is not None]
    assert vs[0] < 0.2 and vs[-1] == 0.0
    assert all(b <= a + 1e-9 for a, b in zip(vs, vs[1:]))   # 점점 줄어듦
    assert outs[-1] is None                                   # 다 멈추면 더 보내지 않음


def test_priority_stop_over_manual_over_auto():
    m = CmdMux(manual_acc_v=100.0, manual_acc_w=100.0)
    m.auto(0.1, 0.5, 0.0)
    assert m.step(0.0) == (0.1, 0.5) and m.source == 'auto'
    m.manual(-0.05, 0.0, 0.05)
    m.auto(0.1, 0.5, 0.05)
    v, _ = m.step(0.05)
    assert m.source == 'manual' and v < 0.1                   # 수동이 자동보다 먼저 (부드럽게 줄어듦)
    m.stop(0.1, hold=0.5)
    m.manual(0.2, 0.0, 0.15)
    assert m.step(0.15) == (0.0, 0.0) and m.source == 'stop'  # 정지 중에는 수동도 무시


def test_auto_timeout_sends_single_zero():
    m = CmdMux(auto_timeout=0.3)
    m.auto(0.1, 0.0, 0.0)
    m.step(0.0)
    assert m.step(0.5) == (0.0, 0.0)
    assert m.step(0.55) is None
