from race_vehicle_interface.pad_v6_autonomy import PadV6Autonomy


def test_disarmed_or_invalid_always_stops():
    state = PadV6Autonomy()
    assert state.update(True, .15, 0, 1) == [b"x"]
    state.arm(True)
    assert state.update(False, .15, 0, 1) == [b"x"]
    assert not state.moving


def test_first_valid_command_uses_only_slowest_firmware_step():
    state = PadV6Autonomy()
    state.arm(True)
    assert state.update(True, .15, 0, 1) == [b"w", b"G484\n"]
    assert state.update(True, .15, 0, 1) == []
    assert state.moving


def test_steering_degree_to_adc_and_clamp():
    state = PadV6Autonomy()
    state.arm(True)
    assert state.update(True, .15, 10, 1)[-1] == b"G304\n"
    assert state.update(True, .15, -100, 1) == [b"G798\n"]
    assert state.update(True, .15, 100, 1) == [b"G158\n"]


def test_zero_speed_or_curvature_stop_resets_motion():
    state = PadV6Autonomy()
    state.arm(True)
    state.update(True, .15, 0, 1)
    assert state.update(True, 0, 0, 1) == [b"x"]
    assert not state.moving
    state.update(True, .15, 0, 1)
    assert state.update(True, .15, 0, 0) == [b"x"]
