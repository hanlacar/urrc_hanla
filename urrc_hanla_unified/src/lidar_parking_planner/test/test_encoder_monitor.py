import ast
import inspect
import textwrap

import pytest
from std_msgs.msg import Int32

from lidar_parking_planner import encoder_monitor_node
from lidar_parking_planner.encoder_monitor_node import EncoderDeltaCalculator


def test_first_count_only_initializes_previous_sample():
    calculator = EncoderDeltaCalculator()

    update = calculator.update(100, 1.0)

    assert update.encoder_count == 100
    assert update.delta_count is None
    assert update.dt_sec is None
    assert update.count_rate is None


def test_successive_counts_produce_expected_deltas():
    calculator = EncoderDeltaCalculator()
    calculator.update(100, 1.0)

    assert calculator.update(110, 1.2).delta_count == 10
    assert calculator.update(125, 1.4).delta_count == 15


def test_decreasing_count_produces_negative_delta():
    calculator = EncoderDeltaCalculator()
    calculator.update(125, 1.0)

    update = calculator.update(110, 1.2)

    assert update.delta_count == -15


def test_normal_dt_produces_count_rate_in_counts_per_second():
    calculator = EncoderDeltaCalculator()
    calculator.update(100, 1.0)

    update = calculator.update(115, 1.2)

    assert update.dt_sec == pytest.approx(0.2)
    assert update.count_rate == pytest.approx(75.0)


@pytest.mark.parametrize('dt_sec', [0.0, 0.0005, -0.1])
def test_too_small_or_negative_dt_suppresses_count_rate(dt_sec):
    calculator = EncoderDeltaCalculator(min_dt_sec=0.001)
    calculator.update(100, 1.0)

    update = calculator.update(110, 1.0 + dt_sec)

    assert update.delta_count == 10
    assert update.count_rate is None


def test_drive_subscription_uses_int32_type_and_required_callback():
    source = inspect.getsource(encoder_monitor_node.EncoderMonitor.__init__)
    tree = ast.parse(textwrap.dedent(source))
    subscriptions = [
        node for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == 'create_subscription'
    ]

    assert encoder_monitor_node.Int32 is Int32
    assert len(subscriptions) == 1
    call = subscriptions[0]
    assert isinstance(call.args[0], ast.Name) and call.args[0].id == 'Int32'
    assert isinstance(call.args[1], ast.Constant) and call.args[1].value == '/drive'
    assert isinstance(call.args[2], ast.Attribute)
    assert call.args[2].attr == 'encoder_callback'


def test_encoder_implementation_has_no_meter_conversion_logic():
    source = inspect.getsource(encoder_monitor_node)

    assert 'counts_per_meter' not in source
    assert 'distance_m' not in source
    assert 'speed_mps' not in source
