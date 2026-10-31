import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "deployment"))
from minimum_snap import MinimumSnapTrajectory, interpolation_settings, validate_controller_rate


def test_single_segment_matches_analytic_minimum_snap():
    trajectory = MinimumSnapTrajectory([0, 0.1], [[0], [1]])
    for u in np.linspace(0, 1, 21):
        expected = 35 * u**4 - 84 * u**5 + 70 * u**6 - 20 * u**7
        assert trajectory.at(u * 0.1)[0] == pytest.approx(expected, abs=1e-12)
    for order in range(1, 4):
        np.testing.assert_allclose(trajectory.at(0, order), 0, atol=1e-6)
        np.testing.assert_allclose(trajectory.at(0.1, order), 0, atol=1e-6)
    np.testing.assert_allclose(trajectory.at(1), [1])
    np.testing.assert_allclose(trajectory.at(1, 1), [0])


def test_multijoint_interpolation_hits_waypoints_and_is_c3():
    times = np.arange(31) / 30
    points = np.sin(times[:, None] * np.arange(1, 15)[None, :]) * 0.01
    initial = np.full((3, 14), 0.001)
    trajectory = MinimumSnapTrajectory(times, points, initial)
    for t, point in zip(times, points):
        np.testing.assert_allclose(trajectory.at(t), point, atol=1e-10)
    for order in range(1, 4):
        np.testing.assert_allclose(trajectory.at(0, order), initial[order - 1], atol=1e-7)
        np.testing.assert_allclose(trajectory.at(times[-1], order), 0, atol=1e-7)
        for t in times[1:-1]:
            np.testing.assert_allclose(trajectory.at(t - 1e-11, order),
                                       trajectory.at(t + 1e-11, order), atol=1e-4)


def test_seventh_degree_polynomial_is_recovered():
    # A global polynomial with these fixed boundary derivatives is already
    # stationary for the snap objective and must survive adding waypoints.
    times = np.linspace(0, 1, 15)
    values = 35 * times**4 - 84 * times**5 + 70 * times**6 - 20 * times**7
    trajectory = MinimumSnapTrajectory(times, values[:, None])
    for t in np.linspace(0, 1, 101):
        expected = 35 * t**4 - 84 * t**5 + 70 * t**6 - 20 * t**7
        assert trajectory.at(t)[0] == pytest.approx(expected, abs=1e-10)


@pytest.mark.parametrize("times,points", [
    ([0], [[0]]), ([0, 0], [[0], [1]]), ([1, 2], [[0], [1]]),
    ([0, float("nan")], [[0], [1]]), ([0, 1], [[0], [float("inf")]]),
])
def test_invalid_trajectories_are_rejected(times, points):
    with pytest.raises(ValueError):
        MinimumSnapTrajectory(times, points)


@pytest.mark.parametrize("rate", [0, 15, float("nan"), float("inf")])
def test_invalid_output_rate_is_rejected(rate):
    with pytest.raises(ValueError):
        interpolation_settings({"interpolation": {"enabled": True, "publish_rate_hz": rate}}, 30)


def test_controller_bottleneck_is_rejected():
    config = {"TransLoopCtlInte": {
        "robot_ctlNode_loop_ctl": {"TempTime": 1 / 300},
        "dual_arm_ctl_node_loop_ctl": {"TempTime": 1 / 50},
        "servo_node_loop_ctl": {"TempTime": 1 / 1000},
    }}
    with pytest.raises(ValueError, match="dual_arm_ctl_node_loop_ctl"):
        validate_controller_rate(config, 300)
    config["TransLoopCtlInte"]["dual_arm_ctl_node_loop_ctl"]["TempTime"] = 1 / 300
    validate_controller_rate(config, 300)
