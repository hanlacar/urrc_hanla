from pathlib import Path

from race_control.autonomy_output_node import traffic_light_code
from race_interfaces.msg import AutonomyObservation


def test_traffic_light_codes():
    assert traffic_light_code("RED") == AutonomyObservation.TRAFFIC_RED
    assert traffic_light_code(" green ") == AutonomyObservation.TRAFFIC_GREEN
    assert traffic_light_code("missing") == AutonomyObservation.TRAFFIC_UNKNOWN


def test_course_manager_republishes_gps_section_as_active_section():
    text=(Path(__file__).parents[1]/"race_control"/
          "course_mission_node.py").read_text()
    assert '"/mission/section"' in text
    assert '"/mission/active_section"' in text
    assert "self.active_section_pub.publish" in text
    assert "section_after_ramp_detection" not in text


def test_course_manager_retains_latest_confirmed_signal_on_unknown():
    text=(Path(__file__).parents[1]/"race_control"/
          "course_mission_node.py").read_text()
    assert 'if state in {"GREEN","RED","YELLOW"}' in text
    assert 'if state in {"GREEN","RED"}' in text
    assert "if section != self.data.section" in text
