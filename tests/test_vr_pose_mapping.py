from types import SimpleNamespace

import numpy as np
import pytest

from test_vr_debug import load_vr_publisher


def rotation(axis, angle):
    c, s = np.cos(angle), np.sin(angle)
    if axis == 0:
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    if axis == 1:
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


@pytest.fixture
def make_stream(monkeypatch):
    module = load_vr_publisher(monkeypatch)

    def create(head_rotation=None):
        # Exercise the real publishing path without creating ROS nodes or hardware connections.
        now = [0.0]
        monkeypatch.setattr(module.time, "monotonic", lambda: now[0])
        node = SimpleNamespace(
            first_frame=True, _last_sample_time=None, target_period_s=0.02,
            position_deadband_m=0.0004, orientation_deadband_rad=0.002,
            max_position_delta_m=0.05, max_orientation_delta_rad=0.20,
            max_position_speed_mps=0.75, max_orientation_speed_radps=4.0,
            reference_frame="head", position_scale=1.0, orientation_scale=1.0,
            position_axis_scale=np.ones(3), position_axis_sign=np.ones(3),
            position_filter_alpha=1.0, orientation_filter_alpha=1.0,
            translation_yaw_offset_rad=0.0, prev_a_button=False, prev_b_button=False,
            precision_mode=False,
            fast_position_scale=1.0, fast_orientation_scale=1.0,
            fast_position_speed_mps=0.75, fast_orientation_speed_radps=4.0,
            fine_position_scale=0.45, fine_orientation_scale=0.60,
            fine_position_speed_mps=0.30, fine_orientation_speed_radps=1.5,
            get_logger=lambda: SimpleNamespace(info=lambda text: None),
            _to_vec3=module.VRPoseIncrementPublisher._to_vec3,
        )
        output = {}
        for side in ("left", "right"):
            for kind in ("pos", "rot"):
                for state in ("residual", "filter"):
                    setattr(node, f"_{side}_{kind}_{state}", np.zeros(3))
            for kind in ("pos", "euler"):
                topic = f"{side}_{kind}"
                output[topic] = []

                def publish(msg, topic=topic):
                    output[topic].append(np.array([msg.x, msg.y, msg.z]))

                setattr(node, f"{topic}_delta_pub", SimpleNamespace(publish=publish))
        head_pose = np.eye(4)
        if head_rotation is not None:
            head_pose[:3, :3] = head_rotation
        arm_pose = head_pose.copy()
        arm_pose[:3, 3] = [0.15, 0.0, 0.45]
        data = SimpleNamespace(
            head_pose=head_pose, left_arm_pose=arm_pose.copy(), right_arm_pose=arm_pose.copy(),
            tele_state=SimpleNamespace(
                left_trigger_state=True, right_trigger_state=True, right_aButton=False,
            ),
        )

        def step(dt=0.02):
            now[0] += dt
            module.VRPoseIncrementPublisher.publish_increment(node, data)

        step()
        return SimpleNamespace(node=node, data=data, step=step, output=output)

    return create


@pytest.mark.parametrize("dt", [0.01, 0.02, 0.04, 0.06])
def test_displacement_is_independent_of_frame_interval(make_stream, dt):
    stream = make_stream()
    stream.data.left_arm_pose[0, 3] += 0.004
    stream.data.right_arm_pose[1, 3] -= 0.004
    stream.step(dt)
    np.testing.assert_allclose(stream.output["left_pos"][-1], [0.004, 0, 0], atol=1e-12)
    np.testing.assert_allclose(stream.output["right_pos"][-1], [0, -0.004, 0], atol=1e-12)


@pytest.mark.parametrize("side", ["left", "right"])
def test_slow_translation_and_rotation_accumulate(make_stream, side):
    stream = make_stream()
    pose = getattr(stream.data, f"{side}_arm_pose")
    for i in range(1, 11):
        pose[0, 3] += 0.0001
        pose[:3, :3] = rotation(2, i * 0.0005)
        stream.step()
    position = np.sum(stream.output[f"{side}_pos"], axis=0)
    orientation = np.sum(stream.output[f"{side}_euler"], axis=0)
    assert position[0] > 0
    assert orientation[0] > 0
    np.testing.assert_allclose(
        position + getattr(stream.node, f"_{side}_pos_residual"), [0.001, 0, 0], atol=1e-12,
    )
    np.testing.assert_allclose(
        orientation + getattr(stream.node, f"_{side}_rot_residual"), [0.005, 0, 0], atol=1e-12,
    )
    other = "right" if side == "left" else "left"
    np.testing.assert_allclose(stream.output[f"{other}_pos"], 0, atol=1e-12)
    np.testing.assert_allclose(stream.output[f"{other}_euler"], 0, atol=1e-12)


def test_subthreshold_back_and_forth_motion_cancels(make_stream):
    stream = make_stream()
    for _ in range(10):
        stream.data.left_arm_pose[0, 3] += 0.0001
        stream.step()
        stream.data.left_arm_pose[0, 3] -= 0.0001
        stream.step()
    np.testing.assert_allclose(stream.output["left_pos"], 0, atol=1e-12)
    np.testing.assert_allclose(stream.node._left_pos_residual, 0, atol=1e-12)


@pytest.mark.parametrize("axis", [0, 1, 2])
@pytest.mark.parametrize("angle", [-0.01, 0.01])
def test_startup_head_tilt_does_not_mix_robot_rotation_axes(make_stream, axis, angle):
    yaw = rotation(2, 0.6)
    head = yaw @ rotation(1, 0.35) @ rotation(0, -0.25)
    stream = make_stream(head)
    np.testing.assert_allclose(stream.node.head_orientation_reference, yaw, atol=1e-12)
    local_increment = yaw.T @ rotation(axis, angle) @ yaw
    for side in ("left", "right"):
        getattr(stream.data, f"{side}_arm_pose")[:3, :3] = head @ local_increment
    stream.step()
    expected = np.zeros(3)
    expected[2 - axis] = angle  # Wire format is yaw, pitch, roll.
    for side in ("left", "right"):
        np.testing.assert_allclose(stream.output[f"{side}_euler"][-1], expected, atol=1e-12)


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("reset", ["release", "stale"])
def test_release_or_stall_clears_pending_motion_and_reanchors(make_stream, side, reset):
    stream = make_stream()
    pose = getattr(stream.data, f"{side}_arm_pose")
    pose[0, 3] += 0.0001
    stream.step()
    assert getattr(stream.node, f"_{side}_pos_residual")[0] > 0
    stream.node.position_filter_alpha = 0.5
    stream.node.orientation_filter_alpha = 0.5
    getattr(stream.node, f"_{side}_pos_filter")[:] = 0.001
    getattr(stream.node, f"_{side}_rot_filter")[:] = 0.001
    if reset == "release":
        setattr(stream.data.tele_state, f"{side}_trigger_state", False)
    pose[0, 3] += 0.02
    stream.step(0.1 if reset == "stale" else 0.02)
    for kind in ("pos", "rot"):
        for state in ("residual", "filter"):
            np.testing.assert_allclose(getattr(stream.node, f"_{side}_{kind}_{state}"), 0)
    np.testing.assert_allclose(stream.output[f"{side}_pos"][-1], 0)
    np.testing.assert_allclose(stream.output[f"{side}_euler"][-1], 0)
    setattr(stream.data.tele_state, f"{side}_trigger_state", True)
    stream.step()
    np.testing.assert_allclose(stream.output[f"{side}_pos"][-1], 0)
    stream.node.position_filter_alpha = 1.0
    pose[0, 3] += 0.004
    stream.step()
    np.testing.assert_allclose(stream.output[f"{side}_pos"][-1], [0.004, 0, 0], atol=1e-12)


@pytest.mark.parametrize("dt", [0.01, 0.02, 0.04])
def test_speed_limit_uses_elapsed_time(make_stream, dt):
    stream = make_stream()
    stream.data.left_arm_pose[:3, 3] += [0.024, 0.032, 0]
    stream.step(dt)
    expected = np.array([0.6, 0.8, 0]) * 0.75 * dt
    np.testing.assert_allclose(stream.output["left_pos"][-1], expected, atol=1e-12)


def test_tracking_jump_is_rejected_before_speed_limit(make_stream):
    stream = make_stream()
    stream.data.left_arm_pose[0, 3] += 0.1
    stream.step()
    np.testing.assert_allclose(stream.output["left_pos"][-1], 0)
    stream.data.left_arm_pose[0, 3] += 0.004
    stream.step()
    np.testing.assert_allclose(stream.output["left_pos"][-1], [0.004, 0, 0], atol=1e-12)


def test_right_b_toggles_precision_mode_on_press_edges(make_stream):
    stream = make_stream()
    stream.data.tele_state.right_bButton = True
    stream.step()
    assert stream.node.precision_mode is True
    assert stream.node._left_pos_residual.tolist() == [0, 0, 0]
    stream.step()
    assert stream.node.precision_mode is True
    stream.data.tele_state.right_bButton = False
    stream.step()
    stream.data.tele_state.right_bButton = True
    stream.step()
    assert stream.node.precision_mode is False


def test_right_a_and_b_simultaneous_only_trigger_a(make_stream):
    stream = make_stream()
    commands = []
    stream.node.zero_return_pub = SimpleNamespace(publish=lambda msg: commands.append("zero"))
    stream.data.tele_state.right_aButton = True
    stream.data.tele_state.right_bButton = True
    stream.step()
    stream.step()
    assert commands == ["zero"]
