from t870_mcu.protocol import speed_mps_to_stage


def test_speed_to_firmware_stage():
    assert speed_mps_to_stage(0.0) == 0
    assert speed_mps_to_stage(0.23) == 1
    assert speed_mps_to_stage(0.46) == 2
    assert speed_mps_to_stage(0.70) == 3
    assert speed_mps_to_stage(-0.23) == -1
