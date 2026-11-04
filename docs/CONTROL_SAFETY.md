# Control Safety Changes

All gravity, inertia, Coriolis and friction compensation switches are **off** in
`ros2_ws/src/ctl_msgs_hw/config/controllerConfig.yaml`. The installed configuration
is a symlink to that file. Historical incident evidence is unchanged.

These changes have offline tests and compilation checks, not physical validation.
The damaged hardware must be inspected before motion testing. RViz checks geometry;
it does not validate dynamics, collision forces, load support or bus behavior.

## Fault Behavior

The servo checks every arm joint before encoding the output packet. It latches on
non-finite commands, position/speed/feedforward limits, feedforward rate, persistent
tracking errors, motor error bits, missing complete ACK1 feedback or command timeout.
All joints are checked before any are encoded, so a fault on the last joint also
inhibits the first joint in that cycle. Invalid WKC, wrong motor IDs and partial
feedback packets do not refresh the per-motor feedback time.

Fault output is MIT velocity damping: Kp=0, target velocity=0, feedforward=0,
Kd=`armSafety.fault_damping` (12 by default). Grippers use Kd=2.2 and Kp=0.
**This is not a brake or a load-holding guarantee. Arms and held objects can fall.**
The command still depends on working motor controllers and communication; software
cannot deliver damping across a broken bus or replace a hardware emergency stop.

Limits in `armSafety` are conservative software trip thresholds, not certified
safe actuator limits. Feedforward limits refer to the protocol input field, not
measured output torque. Gateway firmware that repeats a cached complete ACK1
without a sequence number is not distinguishable from a new identical frame.

The servo publishes `/towel/safety/state` at 20 Hz: `waiting`, `ready`, or
`fault:<reason>`. Replay and deployment require a fresh ready heartbeat. Missing
heartbeat after readiness, or no readiness within five seconds, latches a client
fault. `/towel/safety/stop` (std_msgs/String) requests a servo fault.

Faults do not clear when feedback returns. `/towel/safety/reset`
(std_srvs/Trigger) requires upstream arm controllers to have stopped sending for
at least one second, all joint feedback to be fresh and within range, and all
joint velocities below 0.05 rad/s. Support and inspect the mechanism before reset.
After reset, restart upper controllers and replay/deployment from fresh feedback;
existing client fault latches deliberately do not resume old trajectories.

## Trajectory and Compensation

`BridgeCtlMsg.target_period_s` is the source sample interval. Upper forwarding
preserves the original sample timestamp; repeated forwarding does not restart
mid-level interpolation. A 15 Hz source interval is approximately 66.7 ms even
when the upper and middle controllers run at 300 Hz.

Minimum-snap samples carry analytic velocity and acceleration. The middle layer
uses that same acceleration for inertia compensation when enabled with target
inputs. The servo preserves the sampled position/velocity pair without adding
independent filters or a startup zero-position segment. Legacy trajectory filters
are reinitialized when leaving sampled-trajectory mode. Servo interpolation length
uses the declared middle period divided by the actual servo period.

Both the mid-level acceleration/feedback Kalman filter and the separate servo
output Kalman filter now use F*P*F^T, two-state updates and Joseph covariance
updates. Their time step comes from the actual control period. Low-pass updates
use alpha for the new input; middle and servo sample rates follow their loops.

Gravity, inertia and Coriolis switches are independent; enabling gravity alone
does not enable acceleration-dependent compensation. Friction remains disabled.
Position/velocity filtering switches are separate from compensation switches.

## Replay and Deployment

`bridge.motion_safety` sets command velocity, acceleration and persistent tracking
error checks independently from `max_joint_step_rad * slow_factor`. The latter is
only a feedback-relative target allowance. Invalid feedback is rejected; clipping
uses the intersection of the allowance and absolute joint bounds.

Before motion, minimum-snap trajectories are checked at polynomial extrema for
position overshoot, speed and acceleration. Unsafe trajectories are rejected, not
silently retimed or clipped into a different trajectory. Reduce replay `--speed`
or revise the trajectory following offline review. A slower source requires
interpolation to maintain the command heartbeat. Additional command checks apply
at publication, including the transition from current feedback to the first command.

Replay writes a JSON status alongside its CSV/plot, including `completed`,
`interrupted`, or `fault`, the reason and number of samples. Even rejection before
the first sample writes a status file. Faulted replay exits nonzero.

VR takeover is allowed only while servo safety is ready. A fault invalidates policy
results, blocks both handoff directions and enters `fault` intervention state. A
recording already started is finalized, including when its start acknowledgement
arrives after the fault. Fault recovery cannot automatically enable either source.

## Offline Verification

The ROS message definitions changed. Rebuild `ctl_msgs_hw`, `dual_arm_control_hw`,
`robot_control_hw`, and `servo_control_hw` together and restart consumers before use.

```bash
source scripts/env.sh
.venv-robot/bin/python -m pytest tests/test_motion_safety.py tests/test_minimum_snap.py tests/test_replay_dataset.py tests/test_pi05_deployment.py tests/test_intervention.py
```

Native tests run long filter sequences, covariance checks and fault injection.
The transport test constructs and inspects PDO memory without opening an EtherCAT
interface. These tests do not reconstruct the accident's physical closed loop.
