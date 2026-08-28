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


def test_course_manager_publishes_external_mcu_camera_interface():
    text=(Path(__file__).parents[1]/"race_control"/
          "course_mission_node.py").read_text()
    assert '"/camera_drive"' in text
    assert '"/camera_wheel"' in text
    assert '"/camera_stop"' in text
    assert "Bool(data=bool(output.stage == 0))" in text
    assert '"/vehicle_mode"' in text
    assert '1: "START"' in text
    assert '11: "FINISH"' in text


def test_external_mcu_launch_does_not_start_a_local_bridge():
    text=(Path(__file__).parents[1]/"launch"/
          "mcu_ws_autonomy.launch.py").read_text()
    assert '"use_internal_cmd_mux": "false"' in text
    assert '"vehicle_speed_topic": "/mcu/speed_mps"' in text
    assert "arduino_serial_bridge_node" not in text
