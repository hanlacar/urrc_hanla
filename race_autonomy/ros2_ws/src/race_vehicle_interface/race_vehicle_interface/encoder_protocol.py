def parse_encoder_frame(line):
    """Parse E,count,delta,direction,edge_age_ms,invalid_transitions,valid."""
    if isinstance(line, (bytes, bytearray, memoryview)):
        try:
            line = bytes(line).decode("ascii")
        except UnicodeDecodeError:
            return None
    fields = str(line).strip().split(",")
    if len(fields) != 7 or fields[0] != "E":
        return None
    try:
        count = int(fields[1])
        delta = int(fields[2])
        direction = int(fields[3])
        edge_age_ms = int(fields[4])
        invalid_transitions = int(fields[5])
        signal_valid = int(fields[6])
    except ValueError:
        return None
    if direction not in (-1, 0, 1):
        return None
    if edge_age_ms < 0 or invalid_transitions < 0:
        return None
    if signal_valid not in (0, 1):
        return None
    if not -(2**63) <= count < 2**63:
        return None
    if not -(2**31) <= delta < 2**31:
        return None
    return (
        count,
        delta,
        direction,
        edge_age_ms,
        invalid_transitions,
        bool(signal_valid),
    )
