from pathlib import Path

import pytest

from race_control.route_waypoints import RouteEvents, load_waypoints


ROUTES = Path(__file__).resolve().parents[1] / "routes"


def route():
    return load_waypoints(ROUTES / "hand_gps_aaaa.csv",
                          ROUTES / "hand_gps_aaaa_segments.yaml")


def status(index=165, **overrides):
    point = route()[index]
    data = dict(route_index=index, mode=point.mode, segment_id=point.segment,
                state="TRACKING", gps_healthy=True, imu_healthy=True)
    data.update(overrides)
    return data


def test_reference_stops_and_every_section_boundary():
    points = route()
    assert [(p.index, p.section) for p in points if p.stop_line] == [
        (65, 1), (165, 2), (702, 4), (1261, 6), (1976, 8), (2283, 8), (2961, 11)]
    events = RouteEvents(points)
    for index, section in [(106, 1), (107, 2), (250, 2), (251, 3),
                           (642, 4), (862, 5), (1115, 6), (1367, 7),
                           (1579, 8), (2293, 9), (2668, 10), (2942, 11)]:
        assert events.update(status(index))[0].section == section


def test_nearest_index_is_not_arrival_and_waiting_latches_identity():
    events = RouteEvents(route())
    assert events.update(status())[1] is None
    assert events.update(status(state="APPROACH_STOP_LINE"))[1] is None
    assert events.update(status(164, state="STOPPED_AT_STOP_LINE",
                                reason="stop line reached index=165"))[1] == 165
    assert events.update(status(164, state="STOPPED_AT_STOP_LINE",
                                reason="waiting stop line release"))[1] == 165
    assert events.update(status(166))[1] is None


def test_two_stop_lines_in_same_section():
    events = RouteEvents(route())
    for index in [1976, 2283]:
        events.update(status(index))
        assert events.update(status(index, state="STOPPED_AT_STOP_LINE",
                                    stop_line_index=index))[1] == index


@pytest.mark.parametrize("override", [dict(route_index=-1), dict(route_index=True),
    dict(route_index=99999), dict(mode="3"), dict(segment_id="OTHER"),
    dict(gps_healthy=False), dict(imu_healthy=False), dict(state="FAULT"),
    dict(state="REJOINING"), dict(state="GOAL_REACHED"),
    dict(state="STOPPED_AT_STOP_LINE", stop_line_index=164),
    dict(state="STOPPED_AT_STOP_LINE", stop_line_index=65)])
def test_invalid_status_rejected(override):
    with pytest.raises(ValueError):
        RouteEvents(route()).update(status(**override))


def test_late_subscriber_does_not_guess_arrival_from_tracker_index():
    with pytest.raises(ValueError, match="missing stop-line identity"):
        RouteEvents(route()).update(status(state="STOPPED_AT_STOP_LINE",
                                           reason="waiting stop line release"))


def test_loader_rejects_non_finite_and_bad_sections(tmp_path):
    path = tmp_path / "route.csv"
    for mode, x in [("12", "0"), ("1", "nan"), ("NORMAL", "0")]:
        path.write_text("index,mode,x_m,y_m,latitude,longitude,event\n"
                        f"0,{mode},{x},0,37,127,STOP_LINE\n")
        with pytest.raises(ValueError):
            load_waypoints(path)
