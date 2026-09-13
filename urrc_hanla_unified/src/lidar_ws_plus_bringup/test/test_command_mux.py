import math

from lidar_ws_plus_bringup.command_mux import (
    SourceState, choose_output, select_source, valid_drive)


def fresh_source(now=10.0):
    return SourceState(
        drive=1.0, wheel=-12, stop=False,
        drive_time=now, wheel_time=now, stop_time=now,
        drive_valid=True, wheel_valid=True)


def test_mode_contract_selects_only_configured_source():
    assert select_source('T_PARK', ['T_PARK', 'PARALLEL_PARK'], ['5']) == 'parking'
    assert select_source('parallel_park', ['T_PARK', 'PARALLEL_PARK'], ['5']) == 'parking'
    assert select_source('5', ['T_PARK', 'PARALLEL_PARK'], ['5']) == 'avoidance'
    assert select_source('NORMAL', ['T_PARK', 'PARALLEL_PARK'], ['5']) is None


def test_drive_contract_is_exact_discrete_mcu_set():
    for value in (-1.0, 0.0, 1.0, 2.0, 3.0):
        assert valid_drive(value)
    for value in (-2.0, 0.5, 4.0, math.nan, math.inf):
        assert not valid_drive(value)


def test_fresh_active_command_passes_without_sign_change():
    assert choose_output(fresh_source(), 10.1, 0.5) == (
        1.0, -12, False, 'ACTIVE')


def test_stop_overrides_drive_but_preserves_safe_steering_command():
    source = fresh_source()
    source.stop = True
    assert choose_output(source, 10.1, 0.5) == (
        0.0, -12, True, 'SOURCE_STOP')


def test_watchdog_is_fail_closed():
    assert choose_output(fresh_source(), 10.6, 0.5) == (
        0.0, 0, True, 'ACTIVE_SOURCE_TIMEOUT_OR_INVALID')


def test_forced_stop_has_highest_priority():
    assert choose_output(fresh_source(), 10.1, 0.5, True) == (
        0.0, 0, True, 'GLOBAL_STOP')


def test_non_lidar_mode_is_zero_and_non_latching():
    assert choose_output(None, 10.0, 0.5) == (
        0.0, 0, False, 'NON_LIDAR_MODE')
