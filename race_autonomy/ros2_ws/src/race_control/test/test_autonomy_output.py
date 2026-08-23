from race_control.autonomy_output_node import traffic_light_code
from race_interfaces.msg import AutonomyObservation


def test_traffic_light_codes():
    assert traffic_light_code("RED") == AutonomyObservation.TRAFFIC_RED
    assert traffic_light_code(" green ") == AutonomyObservation.TRAFFIC_GREEN
    assert traffic_light_code("missing") == AutonomyObservation.TRAFFIC_UNKNOWN
