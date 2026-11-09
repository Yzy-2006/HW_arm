# RGB-D Hand Tracking

Live hand landmarks and dual palm 6-DoF tracking with MediaPipe, an Intel
RealSense D435i, OpenCV, and ROS 2 Jazzy / RViz.

## Included in towel_task

This directory is a local copy of the hand-perception project, including its
scripts, tests, RViz configurations, Conda environment specification, model and
current calibration. The parent repository's `make gesture-rviz`,
`make gesture-check` and `make gesture` use this copy by default; no sibling
`gesture_teleoperation` checkout is required. The original external project
is independent and is not updated when this copy changes.

From the `towel_task` root, recreate and check the perception environment with:

```bash
conda env create -f gesture_teleoperation/environment.yml
make -C gesture_teleoperation test
make -C gesture_teleoperation check
```

The checks do not open a camera unless `--camera` is supplied. On installations
outside `~/anaconda3`, override the Python path:

```bash
make -C gesture_teleoperation test PYTHON=/path/to/mediapipe/bin/python
make gesture-rviz ARGS="--perception-python /path/to/mediapipe/bin/python"
```

`models/hand_landmarker.task` and `calibration/hand_geometry.json` are included
in Git's upload scope. The calibration belongs to the current operator and
D435i serial `349522071600`; a different operator/camera should run calibration
again. Its presence selects RGB-D plus RGB/PnP comparison mode in the parent
launcher. Run `make -C gesture_teleoperation calibrate ARGS="--serial CAMERA_SERIAL"`
with the intended camera and operator. Additional calibration files and
temporary files remain ignored.

The commands below are standalone perception commands, run inside this
directory (or using `make -C gesture_teleoperation ...` from the parent).
Robot actions such as fist-to-gripper mapping live in the parent `teleop/`
modules; see [hardware controls](../docs/GESTURE_ROBOT.md) and
[RViz simulation](../docs/GESTURE_RVIZ.md).

## Run Tracking

```bash
make run
```

The Makefile uses `~/anaconda3/envs/mediapipe/bin/python` directly; activating
Conda is optional. Run `make help` (or just `make`) for all commands.

| Command | Action |
| --- | --- |
| `make run` | Full tracking with both OpenCV windows, ROS bridge, and RViz |
| `make debug` | Full tracking with normalized fingertip distance logs |
| `make preview` | Both OpenCV windows only; no ROS bridge, RViz, or UDP output |
| `make headless` | Tracking and ROS bridge without windows, for 10 seconds |
| `make capture-check` | Tracking without windows or ROS, for 90 frames |
| `make calibrate` | Collect RGB-D palm geometry for RGB/PnP tracking |
| `make rgb` | Color-only calibrated PnP with OpenCV, ROS bridge, and RViz |
| `make rgb-preview` | Color-only calibrated PnP with OpenCV only |
| `make compare` | Same-frame RGB-D and RGB/PnP poses overlaid in one RViz |
| `make check` | Dependency imports and CPU inference check |
| `make check-gpu` | GPU inference check |
| `make check-camera` | CPU inference and D435i RGB-D capture check |
| `make check-deps` | Installed package compatibility check |
| `make test` | Geometry, facing, gesture, tracking, and protocol tests |
| `make test-ros` | ROS bridge tests in ROS domain 83 |

```bash
make run PINCH_DISTANCE=1.25 OK_DISTANCE=0.50
make run DELEGATE=cpu
make run ARGS="--no-mirror --no-show-indices"
make run ARGS="--no-rviz"
make headless DURATION=20
make capture-check FRAMES=150
```

Defaults remain GPU, mirrored display, landmark IDs, `PINCH_DISTANCE=1.0`,
and `OK_DISTANCE=0.42`. Set `PORT=9871` for a different ROS bridge port.
Extra script options go in `ARGS`; those options follow the preset arguments.
For another installation, override `PYTHON=/path/to/env/bin/python` and/or
`ROS_SETUP=/path/to/setup.bash`. Run camera targets one at a time.
The direct Python commands documented below remain available.

`make run` starts RGB-D perception, a ROS 2 bridge, and one configured RViz window.
OpenCV defaults to GPU inference, mirrored display, and landmark IDs 0-20.
Closing any window or pressing Ctrl+C in the launcher stops all three
processes and releases the camera. Q or Esc in either OpenCV window also stops it.

The launcher automatically sources `/opt/ros/jazzy/setup.bash` for the ROS
processes. Perception runs in Conda Python 3.11; the ROS bridge uses system
Python 3.12. They communicate through JSON UDP packets on loopback port 9870,
so ROS Python extension modules are not imported into Conda. No camera images
are saved or transmitted by this program.

RViz shows left hands in cyan and right hands in orange, with measured 3D
landmarks, palm origins, position labels, and RGB axes for X/Y/Z. The OpenCV
window shows 2D keypoints, projected palm axes, valid pose count, and palm XYZ
in meters. If keypoints are detected but no reliable depth pose is available,
the keypoints remain visible while the valid pose count drops.

A separate OpenCV window, `D435i Palm-Back Flags`, displays a flag for each hand:

| Flag | Surface Facing the Camera |
| --- | --- |
| `1` / `PALM` | Palm |
| `0` / `BACK` | Back of hand |
| `-1` / `UNKNOWN` | Missing/invalid pose, ambiguous handedness, edge-on, or transition |

The palmar normal is +Z of the existing Right hand frame and -Z of the Left
hand frame. Its dot product with the hand-to-camera direction determines the
visible side; a cosine magnitude of at most 0.3 is treated as edge-on. A new
classification must persist for 120 ms. Missing or invalid data clears the flag
immediately, and a pending flip displays UNKNOWN. Pose smoothing can add further
delay during a turn. Input geometry is unmirrored, so display mirroring does
not invert these flags. This geometric estimate depends on correct handedness
and palm landmarks; severe occlusion or a closed fist can prevent classification.
Each valid hand's local UDP record also carries `palm_facing`; ROS pose topics
are unchanged. `--no-display` suppresses both OpenCV windows.

## Hand Gestures

The same status window displays an independent gesture for each hand:

| ID | Label | Gesture |
| --- | --- | --- |
| `1` | `FIST` | Four fingers curled with the thumb held near the palm |
| `2` | `OK` | Thumb and index tips approach while at least two other tips remain away from the contact and their own knuckles |
| `3` | `FIVE_TIP_PINCH` | All five fingertips cluster near the thumb tip, after ruling out a fist |
| `4` | `OPEN_PALM` | All four fingers extended, with an extended thumb spread outward |
| `0` | `OTHER` | A valid tracked skeleton that does not match these gestures |
| `-1` | `UNKNOWN` | Missing/invalid skeleton, ambiguous identity, or a pending transition |

Recognition uses MediaPipe's relative 3D world landmarks, with distances
normalized by palm size (the average of palm width and wrist-to-middle-knuckle
length). Fist recognition retains its PIP-angle and finger-extension rules.
OK and five-tip pinch primarily use fingertip distances, allowing bent fingers:
the default thumb/index limit is `0.42` palm sizes; for five-tip pinch, every
thumb-to-fingertip distance must be below `1.0`, with a mean below `0.75`.
Pixel-space contact can assist when the model's relative 3D depth is moderately
inaccurate, subject to a 3D separation limit and a non-edge-on palm projection.
Image overlap alone cannot establish physical contact.

Open-palm recognition requires all four fingers' PIP and DIP angles above
145 degrees and extension ratios above 0.85. The thumb must have an IP angle
above 145 degrees, an extension ratio above 0.8, and outward reach beyond the
index MCP above 0.25 palm sizes. Parallel or spread fingers are accepted;
a folded thumb or curled finger is not a fully open palm. Fist, five-tip pinch,
and OK retain their existing recognition priority. A non-edge-on image can
corroborate an open palm when the world PIP-angle estimate is inaccurate;
all image-space extension/abduction checks must pass, and the world skeleton
must still show extended fingers, DIP angles above 130 degrees, and an extended
thumb. The same gesture classifier
runs in RGB-D, RGB/PnP, and comparison modes and does not require depth.

Recognition runs independently of measured camera depth, so a gesture can remain valid
while the palm pose or palm/back flag is UNKNOWN. A candidate must persist for
150 ms; missing data clears the result immediately. Display mirroring does not
change classification. Where available, the 6-DoF track's detection association
is reused for identity; duplicate or low-confidence handedness yields UNKNOWN.

To inspect normalized distances once per second or loosen the 3D limits:

```bash
python scripts/run_hand_tracking.py --gesture-debug
python scripts/run_hand_tracking.py --ok-distance 0.50 --pinch-distance 1.25
```

Higher distance limits accept looser contact but can increase false positives.
The debug log reports the raw candidate before the 150 ms confirmation period.

The classifier uses geometric thresholds, not a separately trained gesture
model. Occluded fingertips, loose variants, and different finger proportions
can affect the result. Synthetic geometry tests cover the four target shapes
and rigid-transform/scale/reflection invariance; model-derived open-hand and
thumb-up examples provide additional regression checks. These checks do not
establish gesture recognition accuracy on a labeled real-world dataset.

The local UDP packet includes a top-level `gestures` dictionary with `Left` and
`Right` IDs even when no depth pose is valid. Valid pose records also include
`gesture_id`. These are local metadata; no new ROS gesture topic is published.

```bash
python scripts/run_hand_tracking.py --delegate cpu
python scripts/run_hand_tracking.py --no-mirror --no-show-indices
python scripts/run_hand_tracking.py --no-rviz --no-display --duration 10
```

Use `--port 9871` to select another loopback port. `--python /path/to/python`
selects the perception Python and `--ros-setup /path/to/setup.bash` selects the
ROS installation. Only one process should open the D435i at a time.

## Calibrated Color-Only Tracking

RGB-D calibration provides metric palm geometry so a later RGB-only run can
recover translation in meters and orientation using OpenCV planar PnP. This
mode uses the D435i's **color stream only**: it does not enable depth, align
depth, read the depth scale, or substitute depth when PnP fails. It targets the
same person and the same D435i color camera at 640x480; other cameras require
their own intrinsics and calibration support.

First collect a calibration:

```bash
make calibrate
```

Hold each open palm toward the camera at approximately 0.3-0.7 m, with fingers
naturally extended. Keep a consistent palm shape and gently vary the tilt.
The hands can appear together or one at a time. The console and preview show
`L:n/60` and `R:n/60`; only high-confidence, front-facing open hands with valid
depth are collected, at most ten samples per second per hand. Poor depth and
unstable geometry are rejected. Once both hands finish, the program saves
`calibration/hand_geometry.json`, prints palm dimensions and sample variation,
and exits. Closing early leaves an existing calibration untouched.
The console also reports why a visible hand is not being collected (for
example, curled fingers, uncertain handedness, or low depth support).

The file stores color-camera factory intrinsics/distortion, camera serial and
resolution, and independent metric palm models for Left and Right. Palm models
use anchors **0, 5, 9, 13, 17** in the same local frame as RGB-D tracking.
Samples are transformed into this frame, robustly combined, and flattened to
an approximately rigid palm plane. Sample variation describes consistency,
not absolute dimensional accuracy. Depth measures skin surfaces and the 2D
landmarks are model estimates; this does not calibrate anatomical bone lengths.

Then run color-only tracking:

```bash
make rgb                 # OpenCV + ROS bridge + RViz
make rgb-preview         # OpenCV only
make rgb ARGS="--gesture-debug"
make rgb ARGS="--no-rviz --no-display --max-frames 150"
```

GPU, mirroring, landmark indices, gestures, and palm/back flags remain available.
ROS topics and coordinate frames are shared with RGB-D mode. RGB RViz labels
include `[RGB PnP]`; only the five fitted palm anchors have 3D positions in this
mode, so the RViz skeleton shows the palm, while OpenCV still shows all 21
2D landmarks. A low reprojection error does not guarantee accurate metric pose.

PnP generates SQPnP and IPPE candidates, keeps distinct branches, and refines
with LM. SQPnP covers IPPE initializations that can become unstable near
180-degree rotations. By default, MediaPipe's relative world landmarks from
the **same RGB image** also provide a palm-orientation cue. That cue seeds an
additional PnP optimization and can choose among similarly fitting branches;
the returned pose remains a PnP candidate, rather than a manually flipped axis
or a direct copy of the model's orientation. No live depth enters this process.

Only candidates within `min(best RMS + 0.75, max(2 * best RMS, best RMS + 0.15))`
pixels are eligible for orientation-based selection. A competing branch must
differ by over 15 degrees, and the RGB cue must favor it by over 10 degrees;
if all eligible candidates disagree with a valid cue by over 45 degrees, the
pose is rejected. Invalid/degenerate RGB cues are ignored. The cue is learned
from RGB and can itself be inaccurate; planar tilt is not guaranteed observable
from these landmarks in every view. Disable the cue for a geometric-only PnP
comparison using `make compare ARGS="--no-pnp-orientation-hint"`.

The estimator rejects poor
reprojection fits (default RMS >4 pixels or any point >8 pixels), offscreen or
edge-on palms, out-of-range depths, ambiguous initial solutions, and large
orientation jumps. A recent pose helps select among similar planar solutions;
its prior expires after 400 ms. Invalid frames produce no pose and existing
ROS expiration behavior applies. The RGB quality field describes reprojection
fit, whereas RGB-D quality describes depth support and planarity; neither is
a calibrated probability, and the two scores are not directly comparable.
Packets carry `pose_source` (`rgbd` or `rgb_pnp`); RGB records also include
`reprojection_rms_px` and set `plane_rms_m` to null. Additional RGB diagnostics
are `pnp_candidate_count`, `pnp_orientation_hint_used` (branch selection), and
`pnp_hint_angle_deg` (rotation difference to a valid RGB cue, otherwise null).

```bash
make calibrate CALIBRATION=calibration/operator_a.json CALIBRATION_SAMPLES=100
make rgb CALIBRATION=calibration/operator_a.json
make calibrate ARGS="--calibration-hands Right"  # Save a Right-only model
make rgb ARGS="--pnp-max-error 3"
```

Each calibration run writes only the requested hands; a Right-only run replaces
the file with a Right-only model rather than merging into a prior session.
An uncalibrated hand retains 2D keypoints/gestures but has no PnP pose. Camera
identity, resolution, and intrinsics must match the file. Recalibrate for a
different operator or camera. Calibration JSON files are excluded from Git.

Direct equivalents are `python scripts/track_hands.py --calibrate` and
`python scripts/run_hand_tracking.py --mode rgb --calibration calibration/hand_geometry.json`.

A palm is only approximately rigid. Cupping, finger splay, occlusion, a fist,
and near-frontal planar ambiguity can degrade RGB pose, particularly depth and
tilt. This mode cannot provide the same independent metric depth measurement
as RGB-D; calibrated RGB-D remains useful as a reference for later accuracy
evaluation. See [OpenCV's PnP documentation](https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html)
for the projection convention and planar solver.

## Compare RGB-D and RGB/PnP

After personal calibration, start one comparison window:

```bash
make calibrate  # Skip when the correct operator/camera calibration already exists
make compare
```

Comparison uses a single D435i RGB-D pipeline and one MediaPipe inference per
frame. Both pose methods receive the same unmirrored color landmarks, sequence
number, and capture timestamp. RGB/PnP uses the saved geometry, color
intrinsics, and an RGB-only MediaPipe orientation cue; it never uses the current depth or the RGB-D pose as an initial
guess or fallback. Two independent trackers use the same smoothing settings.
The comparison reports differences between these displayed, filtered poses.

The single RViz window overlays both methods at their actual positions in
`camera_link`, with no display offset:

- **Cyan** palm points/skeleton and larger axes: RGB-D.
- **Magenta** palm points/skeleton and smaller axes: RGB/PnP.
- **Yellow** links and labels: position difference in millimeters and relative
  rotation angle in degrees, computed separately for Left and Right.

Toggle the two MarkerArray displays to inspect one method. RGB-D may show all
depth-valid joints; RGB/PnP shows its five fitted palm anchors. The console
prints differences once per second. There is no comparison for a hand unless
both methods have valid poses with the same hand label and detection index,
and all five reference palm anchors have depth (so their origins agree).
When a result disappears, the corresponding pose/markers and comparison
validity clear independently. Differences describe agreement between methods;
RGB-D is a reference, not calibrated ground truth.

| Data | Comparison-mode Topic |
| --- | --- |
| RGB-D poses/validity/quality | `/hands/rgbd/{left,right}/{pose,valid,quality}` |
| RGB/PnP poses/validity/quality | `/hands/rgb/{left,right}/{pose,valid,quality}` |
| RGB-D markers + difference labels | `/hands/rgbd/markers` |
| RGB/PnP markers | `/hands/rgb/markers` |
| Difference validity | `/hands/comparison/{left,right}/valid` |
| Position difference in meters | `/hands/comparison/{left,right}/position_difference_m` |
| Orientation difference in degrees | `/hands/comparison/{left,right}/orientation_difference_deg` |

Check comparison `valid` and message freshness before using the numeric
difference topics; invalid frames stop publishing numeric differences. TF
children are `rgbd_left_hand_palm`, `rgbd_right_hand_palm`, `rgb_left_hand_palm`,
and `rgb_right_hand_palm`. Single-mode `/hands/...` topics remain unchanged.

```bash
make compare CALIBRATION=calibration/operator_a.json
make compare PORT=9900  # Uses UDP 9900 for RGB-D, 9901 for RGB/PnP
make compare ARGS="--no-rviz --no-display --max-frames 300"
```

`python scripts/run_hand_tracking.py --mode compare` is the direct equivalent.
`--comparison-port` overrides the second UDP port. Comparison mode enables
depth for the RGB-D reference; use `make rgb` for an actual color-only session.

## Pose Definition

Positions and quaternions use `camera_color_optical_frame`, with X right,
Y down, Z forward, and distances in meters. RViz uses fixed frame `camera_link`,
with X forward, Y left, Z up. A static rotation connects the frames; neither
frame is calibrated to a robot base.

Each hand frame is defined from palm anchors 0, 5, 9, 13, and 17:

- Origin: centroid of the valid palm anchors (at least 0, 5, 9, and 17).
- +Y: wrist toward the middle-finger MCP, projected onto the fitted palm plane.
- +X: little-finger MCP toward index MCP, orthogonalized within that plane.
- +Z: X cross Y. This is a right-handed geometric convention for each hand;
  it does not designate the outward palm normal for both anatomical hands.
- Orientation: quaternion in `[x, y, z, w]` order, mapping hand axes into the
  camera optical frame.

Depth is aligned to color using the RealSense SDK. Depth scale and aligned
intrinsics come from the device. A palm-interior depth estimate gates 7x7
neighborhood samples around landmarks; invalid, out-of-range, and local depth
outliers are rejected. Valid palm anchors are deprojected with the SDK and an
SVD plane fit supplies the normal. Degenerate geometry, missing anchors,
implausible dimensions, and poor plane consistency invalidate the pose.
Default depth range is 0.2-1.5 m, configurable with `--min-depth` and
`--max-depth`. A practical starting distance is 0.3-0.8 m.

Temporal association combines predicted handedness and 3D proximity to maintain
left/right identities across detection reordering. A duplicate handedness label
cannot initialize a second identity. Position and quaternion filters have time
constants of 60 ms and 100 ms, reset after a 400 ms absence, and reject position
jumps above 20 cm. Invalid frames emit no pose for the affected hand. Missing
finger depths remain missing rather than being filled with invented joints.

Depth measures visible skin surfaces. These poses approximate palm frames;
they are not anatomical wrist joint centers or validated metrology. Occlusion,
hands crossing, a nearly edge-on palm, or a closed fist can interrupt tracking.
Filtering adds latency. Handedness remains model-estimated. Mirroring affects
only the OpenCV display, never geometry or published coordinates.

## ROS Topics

| Topic | Type / Meaning |
| --- | --- |
| `/hands/left/pose`, `/hands/right/pose` | `geometry_msgs/PoseStamped`, valid palm poses |
| `/hands/left/valid`, `/hands/right/valid` | `std_msgs/Bool`, current validity |
| `/hands/left/quality`, `/hands/right/quality` | `std_msgs/Float32`, depth support / planarity score, not a calibrated probability |
| `/hands/markers` | `visualization_msgs/MarkerArray`, axes and measured skeletons |
| `/tf` | Dynamic `left_hand_palm` and `right_hand_palm` transforms |
| `/tf_static` | `camera_link` to `camera_color_optical_frame` |

The bridge keeps the newest pending packet, rejects packets older than 350 ms,
marks missing hands invalid, and deletes expired markers. Markers also have a
350 ms lifetime. Pose and TF publication stop on invalid input; TF consumers
must check validity and timestamp age because TF buffers retain old transforms.
Pose timestamps represent host reception of the camera frameset before alignment
and inference, not a calibrated exposure timestamp. The UDP freshness clock is
the shared host monotonic clock. Simulated ROS time is not supported here.

To inspect messages from another terminal:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic echo /hands/left/pose
ros2 topic echo /hands/right/valid
```

RViz configuration is in `config/hands.rviz`.

## Activate

The environment is installed at `/home/yanziyu/anaconda3/envs/mediapipe`.

```bash
conda activate mediapipe
```

If Conda activation is unavailable in a new Bash terminal:

```bash
source /home/yanziyu/anaconda3/etc/profile.d/conda.sh
conda activate mediapipe
```

Installed direct dependencies:

| Package | Version |
| --- | --- |
| Python | 3.11.18 |
| MediaPipe | 1.2.2 |
| pyrealsense2 | 2.59.1.11053 |
| opencv-contrib-python | 5.0.0.93 |
| NumPy | 2.4.6 |
| SciPy | 1.17.1 |

Use the MediaPipe Tasks API (`mediapipe.tasks.python.vision.HandLandmarker`).
Only one OpenCV distribution is installed, since the different OpenCV wheels
share the same `cv2` module.

## Live Hand Landmarks

Connect the D435i and run from the project directory:

```bash
conda activate mediapipe
python scripts/track_hands.py
```

An OpenCV window shows the live color stream, 21 landmarks per hand, a colored
finger skeleton, model-predicted handedness and its score, the hand count, and
the processing frame rate. Up to two hands are detected with the GPU delegate.
Press Q or Esc, close the window, or press Ctrl+C in the terminal to stop.

Optional display settings and CPU execution:

```bash
python scripts/track_hands.py --no-mirror --no-show-indices
python scripts/track_hands.py --delegate cpu
```

Mirroring affects the displayed image and overlays only; the model always sees
the original camera image. The camera color stream is configured to 640x480 at
30 FPS. VIDEO mode keeps inference and overlays on the same frame and enables
temporal tracking. FPS includes capture and processing and is not a measurement
of sensor-to-display latency. Aligned depth supplies metric palm poses and 3D
landmarks, which are sent to the local ROS bridge if it is running. Use the
combined launcher above to also open RViz.

For a bounded camera/inference check without a window:

```bash
python scripts/track_hands.py --no-display --max-frames 90
```

Close other applications using the D435i before starting the viewer.

GPU is the requested default. The installed MediaPipe version still reports a
tensor multiple-write/synchronization diagnostic during GPU VIDEO inference.
Live inference continues; the diagnostic does not prove that outputs are
corrupted. CPU remains available via `--delegate cpu`. GPU/CPU numerical
equivalence and long-duration tracking reliability have not been established.

## Tests

The geometry/tracking/protocol/facing/gesture/calibration/PnP/comparison suite
has 52 tests; the ROS suite has three tests covering dual poses, optical-to-camera
TF conversion, separate comparison topics/TF, marker differences, and expiration.
A live D435i GPU run processed 792 frames at an average 28.6 FPS: 695 frames
contained a valid palm pose and 497 contained both palms. The bridge received
the matching counts. This is a functional observation, not a calibrated pose
accuracy or latency benchmark.

```bash
conda activate mediapipe
python -m unittest discover -s tests -v
```

ROS transport, TF, and stale-data tests use system Python in a separate ROS
domain:

```bash
source /opt/ros/jazzy/setup.bash
ROS_DOMAIN_ID=83 /usr/bin/python3 -m unittest discover -s tests/ros -v
```

## Verify

Run these commands from the project directory after activating the environment:

```bash
python -m pip check
python scripts/check_environment.py
python scripts/check_environment.py --delegate gpu
python scripts/check_environment.py --camera
```

The default check imports dependencies and runs Hand Landmarker on a blank
image. A result of zero detected hands is expected. This checks inference, not
tracking accuracy or performance.

The optional camera check opens the D435i at 640x480 and 30 FPS for both depth
and color, aligns depth to color, and checks ten frames after five warmup frames.
It does not display, store, or upload camera images. Close other applications
using the D435i before running it.

The GPU check completed successfully on this machine using the EGL/OpenGL ES
delegate, with renderer `NVIDIA GeForce RTX 5080 Laptop GPU/PCIe/SSE2` and driver
595.99.02. This environment does not require a separate CUDA toolkit, PyTorch,
or TensorFlow installation for that delegate. GPU initialization can depend on
the graphics drivers and display/session configuration.

The CPU inference and camera checks also passed. The camera reported a depth
scale of approximately 0.001 meters/unit. Read this value from the SDK rather
than hardcoding it. The configured 30 FPS is not a measured end-to-end tracking
rate.

## Recreate

```bash
conda env create -f environment.yml
conda activate mediapipe
```

Direct package versions are pinned in `environment.yml`; transitive dependency
versions are resolved during installation.

The official Hand Landmarker model is included at `models/hand_landmarker.task`.
To download it again if missing:

```bash
curl --fail --location --create-dirs \
  --output models/hand_landmarker.task \
  https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task
sha256sum models/hand_landmarker.task
```

Expected SHA-256:

```text
fbc2a30080c3c557093b5ddfc334698132eb341044ccee322ccf8bcf3607cde1
```

References: [MediaPipe Python setup](https://developers.google.com/edge/mediapipe/solutions/setup_python),
[Hand Landmarker](https://developers.google.com/edge/mediapipe/solutions/vision/hand_landmarker?tab=Python),
and [RealSense Python wrapper](https://github.com/realsenseai/librealsense/blob/master/wrappers/python/readme.md).
