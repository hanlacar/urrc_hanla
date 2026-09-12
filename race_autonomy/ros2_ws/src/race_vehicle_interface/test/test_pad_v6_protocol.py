import pytest

from race_vehicle_interface.pad_v6_protocol import PadV6CommandState


def test_starts_disarmed_and_drive_requires_enable():
    state = PadV6CommandState()
    assert state.key("w") is None
    assert state.key("s") is None
    assert state.key("e") == b"x"
    assert state.armed
    assert state.key("w") == b"w"
    assert state.estimated_pwm == 40
    assert state.key("w") == b"w"
    assert state.estimated_pwm == 50


def test_direction_change_steps_through_zero_like_attached_controller():
    state = PadV6CommandState()
    state.key("e")
    state.key("w")
    assert state.key("s") == b"s"
    assert state.estimated_pwm == 30
    for expected in (20, 10, 0):
        assert state.key("s") == b"s"
        assert state.estimated_pwm == expected
    assert state.key("s") == b"s"
    assert state.estimated_pwm == -40


def test_stop_keeps_arm_but_toggle_disarms():
    state = PadV6CommandState()
    state.key("e")
    state.key("w")
    assert state.key("x") == b"x"
    assert state.armed and state.estimated_pwm == 0
    assert state.key("e") == b"x"
    assert not state.armed


def test_adc_steering_and_status_commands():
    state = PadV6CommandState()
    assert state.key("a") == b"G548\n"
    assert state.key("d") == b"G484\n"
    assert state.key("c") == b"G484\n"
    assert state.key("i") == b"S\n"
    for _ in range(20):
        left = state.key("a")
    assert left == b"G798\n"
    for _ in range(20):
        right = state.key("d")
    assert right == b"G158\n"


@pytest.mark.parametrize("args", [(484, 158, 798, 0), (100, 158, 798, 64)])
def test_invalid_configuration_rejected(args):
    with pytest.raises(ValueError):
        PadV6CommandState(*args)
