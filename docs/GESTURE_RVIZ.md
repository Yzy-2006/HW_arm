# Hand gesture teleoperation in RViz

Run from `towel_task`:

```bash
make gesture-rviz
```

This starts the bundled `gesture_teleoperation/scripts/track_hands.py`, the
simulation UDP receiver, `robot_state_publisher`, and RViz. It uses the
MediaPipe Conda Python for perception and `.venv-robot` for ROS/Pinocchio.
The recognition source, tests, model, calibration and environment specification
are included under [gesture_teleoperation/](../gesture_teleoperation/README.md);
the external sibling checkout is no longer needed.
No rebuild is required if the documented robot environment is already installed.
Close an OpenCV/RViz window or press Ctrl+C to stop the whole process group.
Do not run another camera tracker or camera recorder at the same time.

## Controls

| Gesture | Action |
| --- | --- |
| Hold left/right fully open palm (OPEN_PALM, ID 4) | Send that arm's incremental palm position and rotation |
| Leave OPEN_PALM | Hold the current arm pose; the next open palm starts from a new reference |
| Left/right fist (FIST, ID 1) | Toggle that arm's gripper open/closed once |
| Left OK | Start/stop a simulation episode once |
| Right OK | Return both arms to the configured hardware joint zero |

Begin with curled fingers or fists tracked for at least 0.15 seconds, then fully
open the palms to move. A palm already open at startup cannot start motion until
it has left OPEN_PALM once. For a second toggle, release to another
known gesture for at least 0.15 seconds, then perform the gesture again.
The upstream recognizer already confirms gestures for 0.15 seconds; the
receiver adds 0.10 seconds of confirmation for fist and OK events.
Holding a fist does not repeat the toggle; five-tip pinch no longer controls grippers.
UNKNOWN stops incremental motion and does not re-arm an event. Gestures can
still toggle grippers/recording/home when depth pose is unavailable.

If a palm pose disappears, the affected motion stops. A continuously recognized
open palm retains its confirmed clutch release; the first recovered pose establishes
a new reference, and subsequent poses resume movement without integrating the
missing interval. The same re-anchoring applies when switching pose sources.
The receiver stops on UNKNOWN immediately and marks input stale after 0.35
seconds without fresh packets. It retains the confirmed clutch for a 1.0-second
`resume_grace_s` window. Brief recognition/network gaps re-anchor on recovery;
their missing movement is never replayed. A frame gap above 0.20 seconds also
discards the previous anchor without immediately cancelling the clutch.
An implausible pose jump pauses motion in RECOVER until consecutive poses stay
within the jump thresholds for `recovery_hold_s` (0.15 seconds). It then starts
a new zero-increment anchor. Stream loss, UNKNOWN, or continuous pose jumps
lasting beyond the 1.0-second recovery window require curling the fingers and
opening the palm again.
Release confirmation uses the gesture even when depth pose is unavailable.
After homing, curl the fingers on both hands before reopening them to resume. Homing completes its bounded
return even if the hand stream disappears; gripper commands are preserved.

The RViz status text shows left/right gestures, motion state, gripper command,
REC/IDLE, and homing/tracking state. The names refer to detector labels by
default. Check one hand at a time; set `swap_hands: true` if your anatomical
left hand appears as Right in the detector. This switches pose and gesture
ownership together, including the two OK actions.

| Motion state | Meaning |
| --- | --- |
| MOVE | Valid OPEN_PALM poses are driving incremental motion |
| ANCHOR | First pose after engagement/recovery/source switch; zero increment |
| NO_POSE | OPEN_PALM recognized but neither available pose estimator has a valid pose |
| RELEASE | Curl fingers or make a fist, then fully open the palm to re-arm motion |
| UNKNOWN | Gesture identity is unknown or transitioning |
| RECOVER | Pose jump rejected; waiting for consistent poses to re-anchor |
| HOME / STALE / HOLD | Homing / expired input / non-open-palm hold |

`/gesture_sim/status` includes `release_reason`, and the terminal reports the
reason on entering RELEASE: `startup`, `stream_lost`, `gesture_lost`,
`pose_unstable`, `home` or `ik_failed`. The recovery window affects how long the
clutch remains armed, not how long motion continues on missing input.

When `gesture_teleoperation/calibration/hand_geometry.json` is present, the
launcher defaults to the existing detector's `compare` mode: a single camera
and inference pass produce both RGB-D and calibrated RGB/PnP poses. Only packets
from the same capture timestamp and sequence are combined. Each hand prefers
RGB-D, falls back to valid PnP when depth pose is missing, and keeps PnP through
the held open palm to avoid repeated source switching. Either stream may be delayed
or lost; the receiver waits at most 50 ms for its partner. No previous pose is
replayed to fill missing measurements. If both methods reject a hand pose, motion
stays at NO_POSE. Calibration must match the operator and camera.

## Facing the camera

Input positions and quaternions use the **unmirrored camera optical frame**:
X right, Y down, Z away from the camera. The default assumes a level camera
facing the operator, and maps the operator's directions to robot +X forward,
+Y left, +Z up:

| Operator movement | Camera delta | Robot delta |
| --- | --- | --- |
| Forward, toward camera | -Z | +X |
| Operator's left | +X | +Y |
| Up | -Y | +Z |

The rotation matrix in `config/gesture_rviz.yaml` is:

```text
R = [ 0  0 -1
      1  0  0
      0 -1  0 ]
delta_p_robot = R * (p_current - p_previous)
delta_R_robot = R * R_hand_current * R_hand_previous^T * R^T
R_tool_next = delta_R_robot * R_tool_current
```

This is a proper rotation (determinant +1), not a reflection. The OpenCV
mirror switch affects only display and needs no additional pose reflection.
Relative rotations cancel each hand's different fixed palm-axis convention;
the absolute palm orientation is never assigned directly to the robot tool.
Camera translation cancels in position increments. The camera must remain
fixed. A tilted/moved camera requires updating the rotation matrix; the
default is an operator-relative mapping, not measured camera/robot calibration.

## Simulation and recording

The model and grasp-point TCPs come from `dual_arm/` and
`simulation/robot_model.py`, matching the existing offline viewer/Newton model.
Pinocchio supplies FK and SciPy bounded least-squares supplies IK. Joint
limits, joint speed and Cartesian increment speeds are enforced. Each new
target starts at the achieved simulated pose, so unreachable targets do not
accumulate a backlog. Orientation uses all three axes.

Both the startup posture and Right OK's return target are loaded from
`ros2_ws/src/ctl_msgs_hw/config/zeropos.yaml`, the same joint-zero configuration
used by the hardware. The seven named joints of each arm map in controller order
to `arm_l_joint_1..7` and `arm_r_joint_1..7`. Restart `make gesture-rviz` after
updating this file. Missing, non-finite or out-of-limit values abort startup.
Grippers start open and retain their commands during homing. The simulation
reads the configuration locally without connecting to the hardware.

RViz is a kinematic visualization: it has no dynamics, collisions, towel
contacts, force feedback or rendered training cameras. Recordings are
simulation trajectory checks, not real-robot LeRobot demonstrations.

Left OK creates `data/gesture_rviz/episode_*.jsonl.partial`. Stop OK or normal
shutdown finalizes it to `.jsonl`. A forcibly interrupted file remains partial.
Metadata labels the backend as simulation; samples contain timestamps, raw
input packets, joints, both TCPs in `world` (quaternion xyzw), gripper commands,
gesture/tracking states and events. Stale input is stored as null while the
held simulated state continues to be sampled. No camera images are recorded.

The launcher uses local ROS domain **84**, local UDP **9872** for primary input
and **9873** for the optional PnP stream, and publishes only
`/model_viewer/joint_states`, `/gesture_sim/status`, `/gesture_sim/status_marker`
and the viewer's own description/TF. It never starts the hardware servo/arm/robot
controllers or publishes their command topics. Existing VR and recording
commands are independent.

## Options and checks

```bash
make gesture-rviz ARGS="-- --delegate cpu"
make gesture-rviz ARGS="-- --mode rgbd"  # Explicit depth-only input
make gesture-rviz ARGS="-- --ok-distance 0.50 --pinch-distance 1.25"
make gesture-rviz ARGS="-- --mode rgb --calibration /absolute/path/hand_geometry.json"
make gesture-rviz ARGS="--no-rviz --duration 10 -- --no-display"
make gesture-rviz ARGS="--no-camera --no-rviz --duration 3"
make gesture-rviz ARGS="--port 9880 --domain-id 86 --recording-root /tmp/gesture-recordings"
```

Detector arguments follow `--`. RGB and compare modes require the existing
personal and camera calibration; absent a calibration file, the default is
RGB-D only. `--rgb-port` overrides the second port (otherwise primary port + 1).
`--gesture-root` and
`--perception-python` override the bundled detector directory and Conda Python paths.
Use `--no-camera` to feed externally generated packets directly to the selected
UDP port; do not start the old ROS bridge on the same port.

For ROS inspection, source the ROS environment and set `ROS_DOMAIN_ID=84`:

```bash
source /opt/ros/jazzy/setup.bash
ROS_DOMAIN_ID=84 ros2 topic echo /gesture_sim/status
```

Run hardware-free unit and ROS integration checks:

```bash
source .venv-robot/bin/activate
export TOWEL_OPENROBOTS_PREFIX=/opt/openrobots
source scripts/env.sh
python -m pytest -q tests/test_gesture_teleop.py
python tests/gesture_rviz_runtime.py          # isolated domain 85, synthetic UDP
python tests/gesture_rviz_runtime.py --hybrid # PnP fallback and pose recovery
python tests/gesture_rviz_runtime.py --rviz   # also show the robot during check
```

These checks validate coordinate math, transitions and the software path.
Actual left/right identity, camera mounting and open-palm pose stability
must be checked with the operator in front of the camera.
