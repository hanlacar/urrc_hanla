from mcu_manager.gps_camera_transition import (
    CAMERA_CONTROL, GPS_CONTROL, WAIT_2SEC, WAIT_CAMERA, WAIT_FIRST_GPS,
    GpsCameraTransition,
)


def test_gps_loss_stops_two_seconds_then_latches_camera():
    transition = GpsCameraTransition(wait_sec=2.0)
    assert transition.update("INTERSECTION", 10.0, True, True) == (
        "gps", GPS_CONTROL)
    assert transition.update("INTERSECTION", 10.1, False, True) == (
        "none", WAIT_2SEC)
    assert transition.update("INTERSECTION", 12.09, True, True) == (
        "none", WAIT_2SEC)
    assert transition.update("INTERSECTION", 12.1, True, True) == (
        "camera", CAMERA_CONTROL)
    assert transition.update("INTERSECTION", 13.0, True, True) == (
        "camera", CAMERA_CONTROL)


def test_intersection_waits_for_first_gps_instead_of_falling_back():
    transition = GpsCameraTransition(wait_sec=2.0)
    assert transition.update("INTERSECTION", 1.0, False, True) == (
        "none", WAIT_FIRST_GPS)
    assert transition.update("INTERSECTION", 10.0, False, True) == (
        "none", WAIT_FIRST_GPS)
    assert transition.update("INTERSECTION", 10.1, True, True) == (
        "gps", GPS_CONTROL)


def test_handoff_waits_after_two_seconds_until_camera_is_fresh():
    transition = GpsCameraTransition(wait_sec=2.0)
    transition.update("INTERSECTION", 0.0, True, False)
    transition.update("INTERSECTION", 1.0, False, False)
    assert transition.update("INTERSECTION", 3.0, False, False) == (
        "none", WAIT_CAMERA)
    assert transition.update("INTERSECTION", 4.0, False, True) == (
        "camera", CAMERA_CONTROL)


def test_leaving_intersection_rearms_gps_for_next_intersection():
    transition = GpsCameraTransition(wait_sec=2.0)
    transition.update("INTERSECTION", 1.0, False, True)
    transition.update("INTERSECTION", 3.0, False, True)
    assert transition.update("NORMAL", 4.0, False, True) == (None, "INACTIVE")
    assert transition.update("INTERSECTION", 5.0, True, True) == (
        "gps", GPS_CONTROL)
