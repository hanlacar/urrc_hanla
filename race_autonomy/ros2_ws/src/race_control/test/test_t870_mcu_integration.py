from pathlib import Path


LAUNCH = (
    Path(__file__).resolve().parents[1]
    / "launch"
    / "mcu_camera_vehicle.launch.py"
).read_text(encoding="utf-8")


def test_unified_t870_manager_and_bridge_are_launched():
    assert LAUNCH.count('package="t870_mcu"') == 2
    assert 'executable="manager"' in LAUNCH
    assert 'executable="bridge"' in LAUNCH


def test_slope_mode_uses_existing_camera_mission_topics():
    assert '"SLOPE:camera:camera|center"' in LAUNCH


def test_measured_encoder_scale_reaches_bridge():
    assert '"counts_per_meter": 1073.4' in LAUNCH


def test_manager_and_bridge_can_be_safely_tested_separately():
    assert 'LaunchConfiguration("manager")' in LAUNCH
    assert 'LaunchConfiguration("bridge")' in LAUNCH
