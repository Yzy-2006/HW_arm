# Left Arm Zero Calibration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (- [ ]) syntax for tracking.

**Goal:** Build a passive left-arm calibration mode that holds the right arm at its initial feedback pose and saves a stable left-arm feedback pose as source zero positions.

**Architecture:** A ROS parameter selects a narrowly scoped ServoNode branch before automatic homing and normal control. That branch publishes feedback continuously, sends zero current to left joints, and holds right joints at their startup feedback positions. A standalone Python subscriber validates a stable feedback window and atomically changes only the relevant YAML entries; Make targets compose the tools.

**Tech Stack:** ROS 2 Jazzy (rclcpp, rclpy), C++17, Python 3, PyYAML, GNU Make, pytest.

**Spec:** docs/superpowers/specs/2026-09-19-left-arm-zero-calibration.md

## Global Constraints

- Do not start EtherCAT or command real robot hardware during automated verification.
- Add no third-party dependencies; import ROS Python modules only from the script CLI path.
- SIDE accepts exactly left or right, while the servo node accepts only the left calibration parameter and rejects every other nonempty value.
- Save only the seven existing left_arm keys in ros2_ws/src/ctl_msgs_hw/config/zeropos.yaml, creating a timestamped backup before an os.replace atomic update.
- Preserve all existing, uncommitted user modifications.

## Review Focus

- Missing, NaN, incomplete, all-zero, moving, or discontinuous feedback must never change the YAML; Task 2 tests these inputs.
- Normal make servo must have no calibration parameter and retain normal homing; Task 1 tests its Make contract and code branch.
- Invalid SIDE must exit before Make starts ROS or Python; Task 3 tests each target's guard.
- Right-arm calibration commands must use startup feedback positions and existing low-stiffness gains; Task 1 pins the helper contract.
- Documentation must state gravity-drop risk, physical support, power/green-light checks, exclusive-process rule, rebuild, verification, and backup recovery; Task 3 tests required text.

---

### Task 1: Servo-node passive calibration branch

**Files:**
- Modify: ros2_ws/src/servo_control_hw/include/ros_controller/rosLoop.h
- Modify: ros2_ws/src/servo_control_hw/src/ros_controller/rosLoop.cpp
- Modify: Makefile
- Create: tests/test_arm_zero_calibration.py

**Interfaces:**
- Consumes: ROS parameter arm_zero_calibration_side: std::string = "".
- Produces: void ServoNode::apply_left_arm_zero_calibration_commands(const std::vector<double>& right_hold_positions).
- Produces: make calibrate-arm-zero SIDE=left, invoking servo with ARGS="--ros-args -p arm_zero_calibration_side:=left".

- [ ] **Step 1: Write failing source-contract tests**

~~~
def test_calibration_branch_isolated_from_normal_homing():
    source = (ROOT / 'ros2_ws/src/servo_control_hw/src/ros_controller/rosLoop.cpp').read_text()
    assert 'arm_zero_calibration_side' in source
    assert 'if (!left_arm_zero_calibration_)' in source
    assert 'transMod = 5' in source
    assert 'left arm calibration: zero-current' in source

def test_make_calibration_target_and_normal_servo_arguments():
    makefile = (ROOT / 'Makefile').read_text()
    assert 'calibrate-arm-zero:' in makefile
    assert 'arm_zero_calibration_side:=$(SIDE)' in makefile
    assert '@bash "$(ROOT)/scripts/run.sh" servo $(ARGS)' in makefile
~~~

- [ ] **Step 2: Verify the tests fail**

Run: python3 -m pytest -q tests/test_arm_zero_calibration.py -k 'calibration_branch or make_calibration_target'

Expected: FAIL because neither parameter nor target exists.

- [ ] **Step 3: Add the minimum safe branch**

~~~
const auto side = this->declare_parameter<std::string>("arm_zero_calibration_side", "");
if (!side.empty() && side != "left") {
    throw std::runtime_error("arm_zero_calibration_side must be left or empty");
}
left_arm_zero_calibration_ = side == "left";
~~~

Wrap the current return-to-zero code in if (!left_arm_zero_calibration_). In calibration startup, snapshot seven right feedback positions. In its loop, publish feedback, skip ctlmsg_get() and ctlMsg_runin(), set every left command to transMod=5 and joint_tor=0.0, set every right command to mode 1 using the snapshot plus rArm_defaultKp/Kd, and then call EtherCATrun(). Make servo forward $(ARGS); make calibrate-arm-zero rejects an invalid SIDE before launching it.

- [ ] **Step 4: Verify the tests pass**

Run: python3 -m pytest -q tests/test_arm_zero_calibration.py -k 'calibration_branch or make_calibration_target'

Expected: PASS.

- [ ] **Step 5: Commit**

~~~
git add Makefile ros2_ws/src/servo_control_hw/include/ros_controller/rosLoop.h ros2_ws/src/servo_control_hw/src/ros_controller/rosLoop.cpp tests/test_arm_zero_calibration.py
git commit -m "feat: add passive left-arm zero calibration mode"
~~~

### Task 2: Stable feedback capture and atomic YAML writer

**Files:**
- Create: scripts/calibrate_arm_zero.py
- Modify: tests/test_arm_zero_calibration.py

**Interfaces:**
- Consumes: /dexbotCtl/dualArmCtl_fdback (ctl_msgs_hw.msg.RetInformation) and --side left|right --config Path.
- Produces: validate_pose_samples(samples: list[tuple[float, ...]]) -> list[float] and save_zero_pose(path: Path, side: str, samples: list[tuple[float, ...]]) -> dict[str, float].

- [ ] **Step 1: Write failing writer tests**

~~~
def test_save_left_zero_pose_uses_medians_and_preserves_right_arm(tmp_path):
    path = tmp_path / 'zeropos.yaml'; path.write_text(YAML_WITH_LEFT_AND_RIGHT)
    pose = save_zero_pose(path, 'left', [(0.1, .2, .3, .4, .5, .6, .7)] * 20)
    saved = yaml.safe_load(path.read_text())
    assert pose['left_elbow_joint'] == pytest.approx(.4)
    assert saved['right_arm'] == yaml.safe_load(YAML_WITH_LEFT_AND_RIGHT)['right_arm']
    assert len(list(tmp_path.glob('zeropos.yaml.*.backup'))) == 1

@pytest.mark.parametrize('samples', [
    [(0.0,) * 7] * 20, [(0.1,) * 6] * 20, [(0.1,) * 7] * 19,
    [(0.1,) * 6 + (float('nan'),)] * 20,
    [(0.1,) * 7] * 19 + [(0.2,) * 7],
])
def test_bad_capture_leaves_config_unchanged(tmp_path, samples):
    path = tmp_path / 'zeropos.yaml'; path.write_text(YAML_WITH_LEFT_AND_RIGHT); before = path.read_text()
    with pytest.raises(ValueError): save_zero_pose(path, 'left', samples)
    assert path.read_text() == before
~~~

- [ ] **Step 2: Verify the tests fail**

Run: python3 -m pytest -q tests/test_arm_zero_calibration.py -k 'save_left_zero_pose or bad_capture'

Expected: FAIL with ModuleNotFoundError for calibrate_arm_zero.

- [ ] **Step 3: Implement validation, persistence, and the capture CLI**

~~~
def save_zero_pose(path: Path, side: str, samples: list[tuple[float, ...]]) -> dict[str, float]:
    pose = validate_pose_samples(samples)
    original = path.read_text()
    config = yaml.safe_load(original)
    keys = ARM_JOINT_KEYS[side]
    if set(config[side + '_arm']) != set(keys):
        raise ValueError('Zero-position configuration keys do not match the arm schema')
    # validate, make backup, fsync same-directory temporary file, os.replace it
~~~

Require at least 20 finite seven-element samples in a 1.0-second window, no inter-message gap over 0.2 seconds, per-joint spread at most 0.01 rad, and at least one nonzero position. Store per-joint medians. Delay rclpy and message imports until main(). On every error the source file remains untouched.

- [ ] **Step 4: Verify the tests pass**

Run: python3 -m pytest -q tests/test_arm_zero_calibration.py -k 'save_left_zero_pose or bad_capture'

Expected: PASS.

- [ ] **Step 5: Commit**

~~~
git add scripts/calibrate_arm_zero.py tests/test_arm_zero_calibration.py
git commit -m "feat: save stable arm zero feedback atomically"
~~~

### Task 3: Save command and operator documentation

**Files:**
- Modify: Makefile
- Modify: README.md
- Create: docs/ARM_ZERO_CALIBRATION.md
- Modify: tests/test_arm_zero_calibration.py

**Interfaces:**
- Consumes: Task 1 make calibrate-arm-zero and Task 2 scripts/calibrate_arm_zero.py.
- Produces: make save-arm-zero SIDE=left and a linked Chinese calibration procedure.

- [ ] **Step 1: Write failing command and document tests**

~~~
def test_save_target_and_docs_cover_safe_calibration_sequence():
    makefile = (ROOT / 'Makefile').read_text()
    docs = (ROOT / 'docs/ARM_ZERO_CALIBRATION.md').read_text()
    assert 'save-arm-zero:' in makefile
    assert 'Usage: make save-arm-zero SIDE={left|right}' in makefile
    for phrase in ('急停', '绿灯', 'make calibrate-arm-zero SIDE=left',
                   'make save-arm-zero SIDE=left', 'make build', '备份'):
        assert phrase in docs
~~~

- [ ] **Step 2: Verify the tests fail**

Run: python3 -m pytest -q tests/test_arm_zero_calibration.py -k save_target_and_docs

Expected: FAIL because neither target nor document exists.

- [ ] **Step 3: Add command and exact safety sequence**

~~~
save-arm-zero:
	@if [[ "$(SIDE)" != "left" && "$(SIDE)" != "right" ]]; then echo "Usage: make save-arm-zero SIDE={left|right}"; exit 2; fi
	@$(ROBOT_ENV)
	@python3 "$(ROOT)/scripts/calibrate_arm_zero.py" --side "$(SIDE)"
~~~

Link the document from README. Its ordered procedure must cover: emergency-stop and physical support; EtherCAT link and left green LEDs; only calibration running; terminal 1 calibration command; terminal 2 feedback watch; manually support and pose the zero-current left arm; terminal 3 save while still; inspect captured values and backup; stop calibration; make build; normal make servo verification; and backup restoration. Prohibit controller, VR, arms, robot, teleop, and experiment processes concurrently.

- [ ] **Step 4: Verify all non-hardware tests**

Run: make test

Expected: PASS without starting ROS or EtherCAT hardware.

- [ ] **Step 5: Commit**

~~~
git add Makefile README.md docs/ARM_ZERO_CALIBRATION.md tests/test_arm_zero_calibration.py
git commit -m "docs: add left arm zero calibration procedure"
~~~

## Self-Review

1. **Spec coverage:** Task 1 owns passive left commands, held right feedback positions, feedback publication, parameter rejection, and unchanged normal servo. Task 2 owns stable/full/all-zero feedback validation, exact source update, backup, and atomic replacement. Task 3 owns commands, electrical/physical safety, recovery, rebuild, and verification.
2. **Placeholder scan:** No TBD/TODO or generic test instruction remains; every task provides files, interfaces, a failing test, exact test command, expected result, and implementation shape.
3. **Type consistency:** Task 1 defines arm_zero_calibration_side and Task 3 passes it through Make. Task 2 defines save_zero_pose(Path, str, list[tuple[float, ...]]) -> dict[str, float], while Task 3 only calls its CLI.
4. **Review focus:** Bad feedback is tested in Task 2, normal servo and calibration branch contracts in Task 1, and Make guards plus safety text in Task 3.
