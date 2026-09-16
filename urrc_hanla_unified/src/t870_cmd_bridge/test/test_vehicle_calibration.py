from pathlib import Path

import yaml


def test_bridge_uses_confirmed_steering_calibration():
    package = Path(__file__).parents[1]
    config = yaml.safe_load(
        (package / "config" / "bridge.yaml").read_text())
    params = config["t870_cmd_bridge"]["ros__parameters"]
    assert params["steer_center_adc"] == 496
    assert params["steer_counts_per_deg"] == 18.0
    assert params["max_steer_deg"] == 22
    assert params["drive_topic"] == "/mcu/cmd_drive"
    assert params["wheel_topic"] == "/mcu/cmd_wheel"
    assert params["stop_topic"] == "/mcu/cmd_stop"


def test_bridge_has_independent_stop_subscription_and_watchdog():
    source = (Path(__file__).parents[1] / "t870_cmd_bridge" /
              "bridge_node.py").read_text()
    assert "self.create_subscription(Bool, self.stop_topic" in source
    assert "fresh and not self.stop_active" in source
    assert "self.send('X')" in source
