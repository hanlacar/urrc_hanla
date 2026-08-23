from mcu_manager.section_entry_stop import (
    HOLD, RELEASED, WAIT_SPEED, WAIT_STOP, SectionEntryStop,
)


def test_section_change_waits_for_actual_stop_then_holds_two_seconds():
    gate = SectionEntryStop(0.05, 0.2, 2.0)
    assert gate.set_section(1)
    assert gate.update(0.0, False, 0.0) == (True, WAIT_SPEED)
    assert gate.update(0.1, True, 0.2) == (True, WAIT_STOP)
    assert gate.update(1.0, True, 0.04) == (True, WAIT_STOP)
    assert gate.update(1.19, True, 0.04) == (True, WAIT_STOP)
    assert gate.update(1.2, True, 0.04) == (True, HOLD)
    assert gate.update(3.19, True, 0.0) == (True, HOLD)
    assert gate.update(3.2, True, 0.0) == (False, RELEASED)


def test_motion_during_confirmation_restarts_stop_detection():
    gate = SectionEntryStop(0.05, 0.2, 2.0)
    gate.set_section(2)
    gate.update(1.0, True, 0.0)
    assert gate.update(1.1, True, 0.1) == (True, WAIT_STOP)
    assert gate.update(1.2, True, 0.0) == (True, WAIT_STOP)
    assert gate.update(1.4, True, 0.0) == (True, HOLD)


def test_same_section_does_not_rearm_but_new_section_does():
    gate = SectionEntryStop(0.05, 0.0, 0.0)
    assert gate.set_section(4)
    gate.update(0.0, True, 0.0)
    gate.update(0.0, True, 0.0)
    assert not gate.set_section(4)
    assert gate.set_section(5)
