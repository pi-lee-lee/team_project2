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
    for k in range(10):           # request 0.2 m/s for 0.5 s
        m.manual(0.2, 0.0, k * 0.05)
        v, w = m.step(k * 0.05)
        assert v <= 0.4 * (k + 1) * 0.05 + 1e-9
    assert 0.15 <= v <= 0.2


def test_manual_release_stops_smoothly_then_idle():
    m = CmdMux(manual_acc_v=10.0, manual_dec_v=0.8, manual_timeout=0.4)
    for k in range(5):
        m.manual(0.2, 0.0, k * 0.05)
        m.step(k * 0.05)
    outs = run(m, 0.25 + 0.4, 2.0)     # after release
    vs = [o[0] for o in outs if o is not None]
    assert vs[0] < 0.2 and vs[-1] == 0.0
    assert all(b <= a + 1e-9 for a, b in zip(vs, vs[1:]))   # decreasing
    assert outs[-1] is None                                   # nothing more is sent once stopped


def test_priority_stop_over_manual_over_auto():
    m = CmdMux(manual_acc_v=100.0, manual_acc_w=100.0)
    m.auto(0.1, 0.5, 0.0)
    assert m.step(0.0) == (0.1, 0.5) and m.source == 'auto'
    m.manual(-0.05, 0.0, 0.05)
    m.auto(0.1, 0.5, 0.05)
    v, _ = m.step(0.05)
    assert m.source == 'manual' and v < 0.1                   # manual takes priority over auto (ramping down smoothly)
    m.stop(0.1, hold=0.5)
    m.manual(0.2, 0.0, 0.15)
    assert m.step(0.15) == (0.0, 0.0) and m.source == 'stop'  # manual is ignored too while stopped


def test_auto_timeout_sends_single_zero():
    m = CmdMux(auto_timeout=0.3)
    m.auto(0.1, 0.0, 0.0)
    m.step(0.0)
    assert m.step(0.5) == (0.0, 0.0)
    assert m.step(0.55) is None
