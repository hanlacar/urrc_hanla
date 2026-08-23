from typing import Dict, Iterable, Tuple

from .command_state import SourceState, make_source_states


class InputManager:
    """Stores the latest validated drive and wheel values per source."""

    def __init__(self, source_names: Iterable[str]):
        names = [str(name).strip() for name in source_names if str(name).strip()]
        if not names:
            raise ValueError("at least one command source is required")
        if len(set(names)) != len(names):
            raise ValueError("duplicate command source name")
        self.states: Dict[str, SourceState] = make_source_states(names)

    def update_drive(self, source: str, value: float, now: float, valid: bool, reason: str) -> None:
        self.states[source].drive.update(value, now, valid, reason)

    def update_wheel(self, source: str, value: int, now: float, valid: bool, reason: str) -> None:
        self.states[source].wheel.update(float(value), now, valid, reason)

    def drive_status(self, source: str, now: float, timeout_s: float) -> Tuple[bool, str, float]:
        state = self.states[source].drive
        if not state.valid:
            return False, state.reason, state.value
        if not state.is_fresh(now, timeout_s):
            return False, "timeout", state.value
        return True, "ok", state.value

    def wheel_status(self, source: str, now: float, timeout_s: float) -> Tuple[bool, str, int]:
        state = self.states[source].wheel
        if not state.valid:
            return False, state.reason, int(round(state.value))
        if not state.is_fresh(now, timeout_s):
            return False, "timeout", int(round(state.value))
        return True, "ok", int(round(state.value))
