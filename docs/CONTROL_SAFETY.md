# Control Safety Changes

All gravity, inertia, Coriolis and friction compensation switches are **off** in
`ros2_ws/src/ctl_msgs_hw/config/controllerConfig.yaml`. The installed configuration
is a symlink to that file. Historical incident evidence is unchanged.

The active servo mode is `armSafety.protection_mode: speed_only`. This disables
servo position bounds, running feedback/command age checks, motor-error-bit trips,
feedforward magnitude/rate trips, persistent tracking trips and controller packet
timestamp trips. Homing still checks trajectory speed but does not enforce settling:
it reports trajectory finished / arrival NOT verified instead of claiming arrival.
`full` restores the checks subject to the individual position switches below.

`armSafety.speed_limit_enabled_per_arm` currently contains
`[true, true, true, true, false, false, false]`, applying to J1-J7 on each arm.
Only the shoulder joints and elbow (J1-J4) have software speed bounds; wrist
joints J5-J7 have no software overspeed trip, command velocity check or
position-increment speed check. Servo takeover uses the same mask. Python policy
and replay use the matching `bridge.motion_safety.speed_limit_enabled_per_arm`
for command checks, minimum-snap validation and approach timing. Keep both YAML
masks synchronized; omitted masks enable all seven joints. Non-finite values,
motor protocol bounds, first-frame settling speed and fault-reset stationarity
requirements remain enforced on all joints. Approach/takeover acceleration
smoothing remains active on all joints as configured.

For enabled joints, command velocity, feedback velocity and position increments converted to velocity
remain checked against `max_speed_rad_s` (currently 5.0 rad/s). Command speed violations latch damping immediately.
Feedback overspeed latches after `feedback_speed_trip_samples` consecutive updates
for the same joint (currently 3; omit or set 1 for immediate trips). A normal sample,
including exactly the speed limit, clears that joint's pending count. Rechecking
the same received timestamp does not increment it. A gap exceeding
`feedback_timeout_s` starts a new streak; full mode still faults on stale feedback.
Pending finite overspeed alone does not invalidate the safety heartbeat; malformed
feedback still follows the existing checks. Fault reset clears the counts, and
normal feedback cannot clear a latched fault. These are driver feedback updates,
not dataset frames: the PDO does not expose a motor sample sequence number, so
repeated values in newly received PDOs cannot be identified as cached by the gateway.
Debouncing delays overspeed shutdown and does not establish that a spike is noise;
this is an overspeed trip, not a guarantee of physical speed control. Motor errors
and stale cached feedback no longer stop movement in this mode, and absent new
feedback can therefore hide real overspeed. Initial motor feedback is still required
to construct homing; finite values, array dimensions, valid timing/configuration,
protocol encoding bounds and explicit stop requests remain enforced. Reset still
requires stopping upstream commands and stationary feedback under the active mode.
Python replay/policy and upper-controller protections are unchanged.

`armSafety.check_command_position_limits` is currently **false**: servo target
position bounds and the same check in homing preflight are disabled. Set it to
`true` and restart the servo to restore them (a missing key defaults to true).
`armSafety.check_feedback_position_limits` is also **false**, disabling servo
feedback position bounds in readiness, fault detection and reset eligibility.
It is independent of the command switch and also defaults to true if omitted.
In `full` mode, feedback freshness, motor errors, command position-step limits,
finite-value checks, speed/feedforward limits and tracking checks remain enabled. Python replay/policy
position checks are separate and unchanged.

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
heartbeat after arming, or failure to acquire readiness during the initial five-second wait, latches a client
fault. `/towel/safety/stop` (std_msgs/String) requests a servo fault.

The deployment bridge uses three executor threads and a separate mutually
exclusive callback group for safety reception, independent of control and joint
feedback callbacks. A lock protects heartbeat state and fault latching. The
receiver immediately latches reported faults but defers trajectory/VR cleanup to
the control callback group; final policy and VR-gripper publication is serialized
with fault reception. Control computation does not hold that lock. The 0.2-second
heartbeat deadline and five-second initial safety timeout are unchanged.
For deployment, observing a single `ready` does not arm the running watchdog:
before the first command, it requires at least three ready messages spanning
0.2 seconds, with no inter-message gap above 0.2 seconds. A startup gap restarts
this stability window and blocks publication; if readiness is still unavailable
after the initial five-second allowance it latches a fault. The first actual
policy/hold/VR-gripper publication arms the running watchdog atomically with the
final readiness check. After that, heartbeat loss trips at 0.2 seconds even when
paused or switching control source. Explicit servo faults always latch, including
during startup. Replay retains its existing readiness/arming behavior.
Diagnostics distinguish never-received, expired and non-ready heartbeats and
include the last state, heartbeat age, joint/gripper feedback ages and elapsed
time since the last successful camera observation check. Executor scheduling and
Python execution can still be delayed; this is not a real-time guarantee.

Before its first inference request, deployment waits for ready safety, fresh
joint/gripper feedback and a successful camera snapshot. The interpolation timer
does not publish or invalidate a pending request during this startup wait. Policy
resume repeats the input gate. Existing runtime feedback-loss latching and
observation-expiry hold behavior remain active after startup.

Deployment handles SIGINT/SIGTERM by requesting executor exit while keeping the
ROS context alive, draining executor callbacks, closing camera/worker/recorder
resources and destroying nodes, then shutting down ROS. This avoids destroying
entities underneath queued callbacks when the operator presses Ctrl+C.

Faults do not clear when feedback returns. `/towel/safety/reset`
(std_srvs/Trigger) requires upstream arm controllers to have stopped sending for
at least one second, all joint feedback to be fresh and valid under the configured checks, and all
joint velocities below 0.05 rad/s. Support and inspect the mechanism before reset.
After reset, restart upper controllers and replay/deployment from fresh feedback;
existing client fault latches deliberately do not resume old trajectories.

## Startup Homing

### Temporary Motor Coordinate Offset

`motorPositionOffsets.left_shoulder_roll_deg: 0.0` in `controllerConfig.yaml`
disables the temporary position offset for CAN 0, motor ID 2. Restart the servo
after changing this setting. A missing setting defaults
to zero. This does not write the motor's internal zero or modify saved trajectories.

The transform is `motor_q = direction * joint_q + offset`, with inverse feedback
`joint_q = direction * (motor_q - offset)`. It is applied once at the transport
boundary for MIT and position-mode commands, covering homing, replay, policy and
VR commands. Both ACK1 and legacy queried position feedback use the inverse, so
interpolation and tracking compare values in joint coordinates. Velocity, torque,
grippers, other joints and fault damping packets are unchanged. MIT commands
outside the protocol position range after transformation are rejected, not clipped.

No modulo or shortest-path wrapping is applied. With zero offset, commands and
feedback only use the configured direction compensation. Verify the actual
reference before motion; angles one turn apart are not assumed equivalent.

### Query Feedback Without Homing

For internal zeroing of exactly one selected arm motor, see [Single-Motor Internal Zero](MOTOR_ZERO.md).

Stop other servo/EtherCAT master processes, then run `make read-motors`.
This starts `servo_node --ros-args -p feedback_only:=true`. It initializes the
gateway, then only sends motor parameter queries (`0xE0`, parameters 1/2/3).
It skips homing, calibration, the control loop and ROS control callbacks. It does
not publish control-ready status or populate the normal control feedback topics.
Ctrl-C exits the query loop; no shutdown motion command is sent.

The terminal prints all configured CAN channel/motor ID pairs, including grippers,
at about 3 Hz. `q(rad)` applies the inverse offset and configured direction to the queried motor angle;
`motor(deg)` is the raw angle. `v(raw)` and `I(raw)` preserve protocol float values
without assuming firmware-specific units. Missing/stale fields display `nan` after
0.5 seconds. `err` is the OR of the cached ACK5 error fields. Wrong IDs, incomplete
replies and invalid WKC do not refresh the cache. Gateway-cached repeated replies
without a sequence counter cannot be distinguished from new identical replies.

Query mode does not disable motors, provide load support or cancel old commands.
Do not run it alongside another EtherCAT master. No hardware validation has been
performed; packet generation/decoding is tested using in-memory PDOs.

### Normal Startup

Normal startup first exchanges damping commands to acquire complete, fresh ACK1
feedback for all 14 joints (up to one second). A nonzero angle is not a feedback
validity test. The servo reports `waiting` and ignores upstream arm/gripper targets
until startup homing finishes; safety stop remains active and reset is rejected
during this phase. Start replay/deployment after homing succeeds.

Before the first homing position command, the controller checks the start, configured
zero pose, PD gains, and every generated sample for finite values and speed limits,
including speed inferred from consecutive positions. Position bounds are checked
only when `check_command_position_limits` is true; in that mode even a slightly
out-of-bounds start is rejected. Feedback position bounds are controlled separately
by `check_feedback_position_limits`; other feedback checks apply in both modes.

Command violations include the zero-based joint index, joint name, failed field,
q/v/feedforward/gains and their limits. Preflight errors identify the start, goal
or sample index. Faults immediately abort homing and request damping; the log says
`Homing aborted; zero position NOT confirmed` rather than claiming arrival.

After the trajectory ends, all fresh feedback must remain within `ZeroPos_Threshold`
(currently 0.02 rad) and below or equal to 0.05 rad/s for 0.2 seconds. Failure to
settle within two seconds latches `homing_settle_timeout`. The already-near-home
path also requires this sustained feedback check. A fault, stale feedback or
shutdown cannot produce the homing success message. Compensation stays off.

After homing, the servo preserves the last emitted endpoint (including gains)
while waiting for the first upstream command. It does not automatically start the
legacy interpolator or replace that target with a potentially lagging feedback
position. The first legacy interpolation starts from the held output. Startup
takeover for both legacy and sampled trajectories uses a critically damped target
filter, integrates the emitted velocity into the position target, and limits
velocity changes. `homingHandoff` in `controllerConfig.yaml` sets takeover speed
(0.3 rad/s, also capped by the safety speed limit), acceleration (0.5 rad/s^2),
Kp slew (20/s), Kd slew (10/s), and feedforward slew (1/s). Existing nonzero
command velocity decelerates within the acceleration budget rather than jumping
to the lower takeover speed cap. These are command limits, not physical speed or
torque guarantees. Compensation switches remain off.

The takeover lasts at least one second and releases only after q/v/gains and
feedforward align with the upstream command for 0.2 seconds. The final handover
also respects the discrete position/velocity change budgets. Repeated packets do
not restart this stage. A moving upstream target may prolong takeover; wait for
`Controller handoff complete` with the controller holding its startup pose before
starting a replay/policy trajectory. During takeover the sampled trajectory is
reshaped; normal sampled q/v pass through after completion. Explicit damping and
current-mode requests bypass the ramp, and a fault is never cleared by it.

The first upstream packet logs the largest change from the held target and the
largest held-target/feedback error, with zero-based joint indices. Completion
means command alignment, not confirmation of physical arrival. Normal
position-step checks remain enabled and log previous/target positions, delta,
allowed step and timing. Fault reset starts from damping and uses feedback as the
initial position anchor; it does not resume a pre-fault trajectory.

Holding the endpoint is not confirmation of physical arrival. In speed-only mode,
holding continues without a tracking-error trip until an upstream command arrives.

### Upper Controller Takeover

Replay holds its first frame after the approach until `/towel/control/handoff`
confirms completion for a command sent during this settling stage. The original
bridge header is forwarded through `CtlInformation.trajectory_header` and echoed
in `HandoffStatus.source_header`, preventing an old completion status from opening
the gate. The separate status topic leaves the existing safety-ready heartbeat
unchanged while the servo is transitioning. The status also reports the last
commanded joint positions and handoff epoch.

Replay additionally requires fresh position and velocity feedback within 0.1 rad
of the first frame and below 0.05 rad/s continuously for 0.2 seconds. It keeps
sending the static first frame with the same command mode and retains the starting
gripper state; it does not reset the stream using a hold-mode message. After ten
seconds without confirmation it trips with a diagnostic reason. CSV phase is
`settling` until the dataset clock starts. These settings live in
`bridge.replay_approach`. Approach and servo handoff acceleration are now 1.0 rad/s^2;
their speed caps are 5 rad/s for enabled joints only. Runtime acceleration trips remain disabled as configured.

`make arms` starts the middle controller; `make robot` starts a separate upper
controller later. Completing the first handoff does not cover the latter event.
`CtlInformation.handoff_epoch` now carries a persistent generation counter. The
middle controller increments it on the first upper command, an upper publisher
identity change (including a restart), or a change in control mode, interpolation
mode, bridge/derivative mode, or either arm's rigidity modes. Repeated trajectory
samples keep the same generation. The servo restarts the same configured handoff
from its current emitted q/v/gains/feedforward when that generation changes, and
resets legacy interpolation/filter state. It does not jump back to the original
homing endpoint or to lagging feedback. Manual stop and overspeed trips remain
active. A fault cannot be cleared by a generation change.

`make robot` now compares `feedback - configured_zero_pose` against its existing
0.2 rad startup tolerance. It previously compared absolute joint angles to zero,
incorrectly requesting homing even at a nonzero configured home pose. Real
deviation from that pose still requests homing; this fix does not assert arrival.

The ROS message definition changed: rebuild `ctl_msgs_hw`, `dual_arm_control_hw`,
`robot_control_hw`, and `servo_control_hw`, and restart all three running control
nodes together before using this version. Do not mix old and new message types.
Servo rate sleeps now tolerate context shutdown during Ctrl-C; unrelated runtime
errors while the context remains active still propagate.

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

The active Python mode is `bridge.motion_safety.protection_mode: speed_only`,
matching the already active servo `armSafety.protection_mode: speed_only`.
The model bridge sets `policy_disable_all_speed_checks: true` in
`bridge.motion_safety`. This disables policy velocity preflight, successive-command
and analytic velocity checks, and carries a source-scoped override through the
controllers to disable native command, position-step and feedback speed checks.
The override requires a model command with an original timestamp less than 0.25 s
old; forwarding the same packet does not extend its lifetime. Hold commands,
VR/replay commands or source expiry restore the configured native speed mask.
Startup homing retains its configured limits. Non-finite data, protocol bounds
and explicit stops remain enforced; the override cannot clear a latched fault.
`policy_command_speed_check: false` only disables Python policy speed checks when
the all-speed override is false. Set both switches to restore policy checking:
`policy_disable_all_speed_checks: false`, `policy_command_speed_check: true`.
Restart the model bridge after changing these settings. After upgrading the ROS
messages, rebuild and restart all four control packages before deployment.
Python now disables safety-heartbeat readiness/age gating, feedback and camera
age/skew checks, observation-check expiry, command-interval expiry, absolute
position and acceleration trips, persistent tracking error, feedback-relative
target clipping, and replay first-frame handoff/arrival confirmation. Model
result age rejection is disabled; accepted chunks start their playback timeline on first publication
instead of being discarded because inference consumed their source horizon.
Outside the model override, the configured speed mask remains active in planning
and publication. Shoulder and elbow feedback overspeed requires three consecutive
updates at the servo.

Initial finite feedback and images are still required to construct observations
and approach trajectories. NaN/Inf, shapes, positive time intervals, protocol
encoding bounds, explicit stop/hold commands and reported servo faults remain
enforced. Transport timeouts and stream exhaustion still cause waiting/holding,
not fabricated actions; the upper controller's command-stream hold and stale
derivative zeroing are retained. Gravity/other compensation remains off. Approach
and takeover smoothing remains, but replay starts after the planned approach
without confirming physical arrival. Cached feedback/images can now be used even
when their sources stop updating. `slow_factor` no longer reduces target offset
in this mode because that feedback-relative allowance is disabled.

Set the Python mode to `full` and restart deployment/replay to restore the
configured Python checks described below and the heartbeat/startup behavior
above. Servo checks have their own mode and require restarting servo. Threshold
values are preserved in YAML for restoration; changing the mode does not change
the individual position/acceleration switches already disabled there.

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

Policy chunk planning and extrema validation do not consume playback time. A new
interpolator remains pending until its first publication, which samples elapsed
time zero. Subsequent chunks use the last successfully published q/v/a/jerk as
their initial boundary, not the old polynomial extrapolated to wall-clock time.
Gripper frame selection shifts with joint playback. Full mode still evaluates
observation age using wall time, so deferring playback does not make old inputs
fresh. A hold/pause clears the saved derivative boundary before reanchoring.
Successive-command speed faults name the arm/joint and report previous and target
positions, position delta, elapsed time, computed velocity/acceleration and the
specific exceeded bound. Finite-value and configured speed checks remain active.

Replay writes a JSON status alongside its CSV/plot, including `completed`,
`interrupted`, or `fault`, the reason and number of samples. Even rejection before
the first sample writes a status file. Faulted replay exits nonzero.

Manual VR takeover uses `/dexbotCtl/vr_override` after the policy bridge exits.
The controller checks servo safety and anchors VR to fresh finite FK feedback
within configured VR bounds. A latched fault blocks either handoff direction.
Recording is a separate `make record` process; saving does not resume policy.
See [Manual VR takeover](POLICY_INTERVENTION.md) for the terminal sequence.

## Offline Verification

The ROS message definitions changed. Rebuild `ctl_msgs_hw`, `dual_arm_control_hw`,
`robot_control_hw`, and `servo_control_hw` together and restart consumers before use.

```bash
source scripts/env.sh
.venv-robot/bin/python -m pytest tests/test_motion_safety.py tests/test_minimum_snap.py tests/test_replay_dataset.py tests/test_pi05_deployment.py tests/test_vr_debug.py tests/test_ros_shutdown.py
```

Native tests run long filter sequences, covariance checks and fault injection.
The transport test constructs and inspects PDO memory without opening an EtherCAT
interface. These tests do not reconstruct the accident's physical closed loop.
