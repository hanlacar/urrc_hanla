from dataclasses import dataclass, field
from typing import Dict, Optional


@dataclass
class ChannelState:
    """Latest value and validity for one command channel."""

    value: float = 0.0
    received_at: Optional[float] = None
    valid: bool = False
    reason: str = "never_received"

    def update(self, value: float, now: float, valid: bool, reason: str = "ok") -> None:
        self.value = value
        self.received_at = now
        self.valid = valid
        self.reason = reason

    def is_fresh(self, now: float, timeout_s: float) -> bool:
        if self.received_at is None:
            return False
        return (now - self.received_at) <= timeout_s


@dataclass
class SourceState:
    """Independent drive/wheel state for a command source."""

    drive: ChannelState = field(default_factory=ChannelState)
    wheel: ChannelState = field(default_factory=ChannelState)


@dataclass
class ArbitrationStatus:
    mode: str = "IDLE"
    drive_source: str = "stop"
    wheel_source: str = "stop"
    safety_state: str = "IDLE"
    ready: bool = False


def make_source_states(source_names) -> Dict[str, SourceState]:
    return {name: SourceState() for name in source_names}
