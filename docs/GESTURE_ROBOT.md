# Open-palm teleoperation on the robot

`make gesture` replaces `make vr` in the existing hardware workflow. It starts
the hand tracker and hardware input adapter, using the same command topics,
controller IK, gripper calibration and episode recorder as VR. It does not
start servo/arm/upper controllers. `make gesture-rviz` remains isolated simulation.
The tracker defaults to the bundled
[gesture_teleoperation/](../gesture_teleoperation/README.md), including recognition
code, model, camera/palm calibration and environment setup. A separate checkout
outside this repository is not required.

## Start

Use the controller startup and shutdown sequence in
[the collection workflow](DATA_COLLECTION_WORKFLOW.md). Stop the VR publisher
and any policy bridge before starting gesture control. Close `gesture-rviz`
or a standalone hand tracker so the gesture camera is available.

For an already running, compatible controller stack, start with:

```bash
cd /home/yanziyu/projects/HW_arm/towel_task
make gesture-check
```

This opens the hand camera and publishes diagnostics only. It has no arm,
gripper or home command publishers and makes no takeover/recording service
calls. Check the camera's Left/Right labels and the `deltas` in
`make watch WATCH_TOPIC=/gesture/status`. A held open palm at startup requires
curling the fingers first. Stop the check with Ctrl+C, then run:

```bash
make gesture
```

Wait for `Gesture takeover: VR anchored to feedback` and `ready`. Curl fingers
for at least 0.2 seconds, then fully open one palm and make a small movement.
Taking ownership anchors the controller to actual robot FK feedback; the
first palm pose is also an anchor and sends no displacement.

If starting the complete stack, use separate terminals in this order:

| Terminal | Command |
| --- | --- |
| 1 | `make servo` |
| 2 | `make arms` |
| 3 | `make robot` |
| 4, when collecting | `make record` |
| 5 | `make gesture` |

Do not launch duplicate controllers over an already running stack. The live
adapter requires this workspace's `/towel/safety/state`, fresh joint/FK/upper
feedback and `/dexbotCtl/vr_override` service. Older controller processes
missing these interfaces must be stopped through the existing shutdown
sequence and replaced with this workspace's build. A missing interface is
not bypassed by the adapter. No build is needed for the new Python files
themselves when the current ROS interfaces are already installed.

## Controls

| Gesture | Action |
| --- | --- |
| Fully open left/right palm, OPEN_PALM ID 4 | Increment that arm's position and orientation |
| Leave OPEN_PALM | Hold the arm; reopening establishes a new reference |
| Left/right fist, FIST ID 1 | Toggle that gripper using its calibrated endpoints |
| Left OK | Start/stop the real episode through `/towel/record` |
| Right OK | Return both arms to the hardware joint zero through `/vr/zero_return_cmd` |

Fist and OK need 0.2 seconds of confirmation. Release to a known different
gesture for 0.2 seconds before repeating.
Holding a fist does not repeatedly toggle the gripper. Five-tip pinch has no
gripper action. Making a fist while moving also stops the corresponding arm.
Unknown gestures pause movement; missing poses and brief input gaps re-anchor
when tracking resumes. Loss
longer than 1 second requires curling and reopening. Input packets expire
after 0.25 seconds; old motion is never replayed.

Recording uses `make record`, its three cameras and its configured dataset
directory. Left OK has no effect without the recorder. Right OK is blocked
while recording or while a recording request is pending. It uses the running
controller's joint zero, not an independent Cartesian target. The adapter
waits for the joint-return mode, then Cartesian mode and fresh stationary
joint feedback at the installed `zeropos.yaml` target. Palm increments remain
zero during the return. Curl and reopen after it completes. A timeout or loss
of controller feedback during return latches an input fault requiring inspection
and an adapter restart. Exiting the input process does not cancel a return
already executing in the controller.

## Coordinates and settings

`config/gesture_robot.yaml` shares the facing-camera rotation with simulation:
toward the camera maps to robot +X, the operator's left maps to +Y, and upward
maps to +Z. Display mirroring does not affect transmitted poses. Camera
orientation must match the assumptions documented in
[the RViz guide](GESTURE_RVIZ.md#facing-the-camera).

Edit both multipliers in `config/gesture_robot.yaml`, then restart
`make gesture` to apply them:

```yaml
position_scale: 1.0
orientation_scale: 1.0
position_speed_mps: null
orientation_speed_radps: null
```

Translation and rotation are scaled without input speed caps (`null`).
Servo fault guards are separate.
Pose jump rejection, tracking timeouts and controller feedback checks remain
active. A positive numeric speed value can re-enable an input cap later.
`swap_hands` switches pose ownership and all gesture actions together.

### Controller smoothing

The current `ros2_ws/src/ctl_msgs_hw/config/controllerConfig.yaml` enables
the middle controller's position/velocity output lowpass at 20 Hz
(`gravComp.ifUsePosVelOutpFtr: 1`). The servo uses B-spline interpolation with
its 5 Hz position/velocity lowpass enabled. Output position/velocity Kalman
filters remain disabled:

```yaml
servoCommandSmoothing:
  interpolation_mode: 2
  lowpass_enabled: true
  output_kalman_enabled: false
```

These settings affect ordinary arm commands, including VR/gesture commands.
Sampled policy trajectories keep their existing path. Torque filtering,
IK joint-jump protection, takeover ramps and servo fault guards remain active.
The IK joint step remains limited to 0.015 rad per update. Restoring spline
and lowpass smoothing does not change the pre-existing servo `speed_only`
mode, disabled position-bound checks or per-joint speed-check mask.

Rebuild after C++ changes, then restart controllers through the normal shutdown
and startup sequence. YAML-only changes need a restart of `make arms` and
`make servo`; restarting `make gesture` alone does not reload controller settings.

When the calibration file exists, the launcher uses RGB-D plus RGB/PnP
fallback. The tracker selects the calibration's camera serial automatically
(currently `349522071600`), separate from the three recording cameras. For
explicit RGB-D mode, select the serial as well in a multi-camera setup:

```bash
make gesture ARGS="-- --mode rgbd --serial 349522071600"
make gesture ARGS="-- --delegate cpu"
make gesture-check ARGS="--no-camera --duration 5"
```

Live input uses the current `ROS_DOMAIN_ID` (default 0), UDP 9874/9875, and
the existing ROS discovery configuration. `--domain-id`, `--port`,
`--rgb-port`, `--config` and the detector options after `--` remain available.

## Diagnostics and verification

`/gesture/status` reports readiness, blocking reason, service availability,
takeover, tracking, recording, homing and the mapped increments. Competing
publishers on arm/gripper/home command topics block output. Feedback older
than 0.2 seconds also blocks it; recovery requires a new feedback-based
takeover and a fresh release gesture. A latched servo fault cannot be cleared
by a hand gesture. Ctrl+C stops the tracker and input adapter; it is not a
physical emergency stop.

The mock integration test runs in isolated ROS domain 87 with synthetic UDP
and mock hardware feedback/services. It never starts hardware controllers:

```bash
source .venv-robot/bin/activate
export TOWEL_OPENROBOTS_PREFIX=/opt/openrobots
source scripts/env.sh
python -m pytest -q tests/test_gesture_teleop.py tests/test_grippers.py
python tests/gesture_robot_runtime.py
```

It checks dry-run isolation, takeover rejection/success, Cartesian and Euler
conventions, no increment replay, grippers, recording, home, stale feedback,
re-arming, competing publishers and loss of feedback during homing. Physical
motion and camera/operator alignment still require the operator's live trial.
