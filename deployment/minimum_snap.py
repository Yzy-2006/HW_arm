"""Minimum-integrated-squared-snap interpolation for joint position targets."""

from __future__ import annotations

import numpy as np
from scipy.interpolate import make_interp_spline, PPoly


JOINT_INDICES = np.r_[0:7, 8:15]


def interpolation_settings(config: dict, source_rate: float) -> tuple[bool, float]:
    settings = config.get("interpolation", {})
    enabled = settings.get("enabled", False)
    if not isinstance(enabled, bool):
        raise ValueError("interpolation.enabled must be a boolean")
    rate = float(settings.get("publish_rate_hz", 50.0)) if enabled else source_rate
    if not np.isfinite(rate) or not np.isfinite(source_rate) or rate < source_rate or source_rate <= 0:
        raise ValueError("interpolation.publish_rate_hz must be finite and >= the source rate")
    return enabled, rate


def validate_controller_rate(config: dict, publish_rate: float) -> None:
    loops = config["TransLoopCtlInte"]
    for name in ("robot_ctlNode_loop_ctl", "dual_arm_ctl_node_loop_ctl", "servo_node_loop_ctl"):
        period = float(loops[name]["TempTime"])
        if not np.isfinite(period) or period <= 0 or round(1.0 / period) < publish_rate - 1e-6:
            raise ValueError(f"{name}.TempTime must support interpolation publication at {publish_rate:g} Hz")


class MinimumSnapTrajectory:
    """Seventh-degree interpolant with fixed endpoint velocity/acceleration/jerk.

    Simple interior knots give C6 continuity. Together with fixed waypoint
    positions and endpoint derivatives, these are the variational conditions
    for minimizing integral ||q''''(t)||^2 dt. SciPy uses a banded solve, so long
    replay episodes do not require a dense trajectory optimization matrix.
    """

    def __init__(self, times, positions, initial_derivatives=None):
        times = np.asarray(times, dtype=np.float64)
        positions = np.asarray(positions, dtype=np.float64)
        if (times.ndim != 1 or len(times) < 2 or not np.isfinite(times).all()
                or times[0] != 0 or np.any(np.diff(times) <= 0)):
            raise ValueError("Trajectory times must start at zero and strictly increase")
        if (positions.ndim != 2 or positions.shape[0] != len(times)
                or not np.isfinite(positions).all()):
            raise ValueError("Trajectory positions must be a finite (knots, joints) array")
        zeros = np.zeros((3, positions.shape[1]))
        initial = zeros if initial_derivatives is None else np.asarray(initial_derivatives, dtype=float)
        if initial.shape != zeros.shape or not np.isfinite(initial).all():
            raise ValueError("Initial derivatives must contain velocity, acceleration and jerk")
        # Normalize time for numerical stability; derivative values scale with T^d.
        self.duration = float(times[-1])
        boundary = (
            [(d, initial[d - 1] * self.duration ** d) for d in range(1, 4)],
            [(d, zeros[d - 1]) for d in range(1, 4)],
        )
        self.spline = make_interp_spline(times / self.duration, positions, k=7, bc_type=boundary)

    def at(self, elapsed: float, derivative: int = 0) -> np.ndarray:
        if not np.isfinite(elapsed) or derivative not in range(4):
            raise ValueError("Invalid trajectory sample time or derivative")
        if derivative and (elapsed < 0 or elapsed > self.duration):
            return np.zeros(self.spline.c.shape[1:])
        value = self.spline(np.clip(elapsed / self.duration, 0.0, 1.0), nu=derivative)
        return value / self.duration ** derivative

    @classmethod
    def rest_to_rest(cls, start, target, max_velocity, max_acceleration, min_duration=1.0):
        """Time a standalone two-point segment using its exact derivative peaks."""
        positions = np.asarray([start, target], dtype=float)
        limits = np.asarray([max_acceleration, min_duration], dtype=float)
        if (positions.ndim != 2 or positions.shape[1] == 0 or not np.isfinite(positions).all()
                or not np.isfinite(limits).all() or np.any(limits <= 0)):
            raise ValueError("Invalid approach positions or limits")
        speeds = np.broadcast_to(np.asarray(max_velocity, dtype=float), positions.shape[1:])
        if np.isnan(speeds).any() or np.any(speeds <= 0):
            raise ValueError("Invalid approach speed limits")
        distances = np.abs(positions[1] - positions[0])
        distance = float(np.max(distances))
        # q(u) = 35u^4 - 84u^5 + 70u^6 - 20u^7 has peak |q'|=35/16
        # and peak |q''|=84*sqrt(5)/25 on [0,1]. Leave 5% timing margin.
        duration = max(float(min_duration), 1.05 * max(
            (35 / 16) * float(np.max(distances / speeds)),
            np.sqrt((84 * np.sqrt(5) / 25) * distance / max_acceleration),
        ))
        return cls([0, duration], positions)

    def validate_limits(self, lower, upper, max_velocity, max_acceleration):
        """Check every polynomial interval, including extrema between waypoints."""
        from motion_safety import MotionFault
        # None disables the acceleration bound, but not finite-value validation.
        acceleration_limit = np.inf if max_acceleration is None else max_acceleration
        speed_limits = np.broadcast_to(np.asarray(max_velocity, dtype=float), (self.spline.c.shape[1],))
        if np.isnan(speed_limits).any() or np.any(speed_limits <= 0):
            raise ValueError("Invalid trajectory speed limits")
        for joint in range(self.spline.c.shape[1]):
            polynomial = PPoly.from_spline((self.spline.t, self.spline.c[:, joint], self.spline.k))
            for order, lo, hi in ((0, lower[joint], upper[joint]),
                                  (1, -speed_limits[joint], speed_limits[joint]),
                                  (2, -acceleration_limit, acceleration_limit)):
                derivative = polynomial.derivative(order)
                roots = derivative.derivative().roots(extrapolate=False)
                points = np.r_[0., 1., roots[np.isfinite(roots) & (roots >= 0) & (roots <= 1)]]
                values = derivative(points) / self.duration ** order
                if not np.isfinite(values).all() or np.min(values) < lo - 1e-8 or np.max(values) > hi + 1e-8:
                    raise MotionFault(f"Trajectory joint {joint + 1} derivative {order} exceeds limits; "
                                      "reduce replay speed or revise the trajectory")


class ChunkInterpolator:
    """Plan from the last emitted state; start playback on first publication."""

    def __init__(self, chunk, now: float, current, initial_derivatives=None):
        initial = np.asarray(current, dtype=float)[JOINT_INDICES]
        period = 1.0 / chunk.rate_hz
        if len(chunk.actions) == 1:
            times = np.array([period])
            actions = chunk.actions
        else:
            times = chunk.observed_at + np.arange(len(chunk.actions)) * period - now
            keep = times >= period * 0.5
            times, actions = times[keep], chunk.actions[keep]
            if not len(times):
                times, actions = np.array([period]), chunk.actions[-1:]
        self.planned_at = now
        self.started_at = None
        self.trajectory = MinimumSnapTrajectory(
            np.r_[0.0, times], np.vstack([initial, actions[:, JOINT_INDICES]]), initial_derivatives,
        )

    def start(self, now: float):
        if self.started_at is None:
            self.started_at = now

    def source_action(self, chunk, now: float):
        elapsed = 0.0 if self.started_at is None else now - self.started_at
        # Shift playback (including grippers), but preserve actual observation age.
        return chunk.at(now, timeline_time=self.planned_at + elapsed)

    def at(self, now: float, raw_action) -> np.ndarray:
        if self.started_at is None:
            raise ValueError("Chunk playback has not started")
        action = np.array(raw_action, dtype=float, copy=True)
        action[JOINT_INDICES] = self.trajectory.at(now - self.started_at)
        return action
