from typing import Dict, Iterable, Tuple


STOP_SOURCE = "none"


class CommandSelector:
    """Maps a vehicle mode to independent drive and wheel sources."""

    def __init__(self, source_names: Iterable[str], mode_map_entries: Iterable[str]):
        self.source_names = set(source_names)
        self.mode_map: Dict[str, Tuple[str, str]] = {}
        for entry in mode_map_entries:
            text = str(entry).strip()
            if not text:
                continue
            parts = [part.strip() for part in text.split(":")]
            if len(parts) != 3:
                raise ValueError(
                    "mode_map entry must be MODE:DRIVE_SOURCE:WHEEL_SOURCE: %s" % text
                )
            mode, drive_source, wheel_source = parts
            mode = mode.upper()
            drive_source = drive_source.lower()
            wheel_source = wheel_source.lower()
            self._validate_source(drive_source, text)
            self._validate_source(wheel_source, text)
            self.mode_map[mode] = (drive_source, wheel_source)

        if "IDLE" not in self.mode_map:
            self.mode_map["IDLE"] = (STOP_SOURCE, STOP_SOURCE)

    def _validate_source(self, source: str, entry: str) -> None:
        if source != STOP_SOURCE and source not in self.source_names:
            raise ValueError("unknown source '%s' in mode_map entry '%s'" % (source, entry))

    def select(self, mode: str) -> Tuple[str, str, bool]:
        normalized = str(mode).strip().upper()
        if normalized not in self.mode_map:
            return STOP_SOURCE, STOP_SOURCE, False
        drive_source, wheel_source = self.mode_map[normalized]
        return drive_source, wheel_source, True
