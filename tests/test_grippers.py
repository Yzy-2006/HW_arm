import copy
import math
import sys
from pathlib import Path

import pytest
import yaml

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'teleop/VR_controller'))
sys.path.insert(0,str(ROOT/'scripts'))
sys.path.insert(0,str(ROOT/'collection'))
from grippers import DualGrippers, Gripper, load_config
from calibrate_grippers import save_endpoint
from core import binary_states, StreamCache


def config():
    side={'calibrated':True,'open_deg':-30.,'closed_deg':45.,'motion_time_s':.6}
    return {'left':dict(side,open_deg=30.,closed_deg=-45.),'right':side,'feedback_timeout_s':.2}


def test_both_directions_independent_targets_and_feedback():
    ctl=DualGrippers(config())
    ctl.feedback([30,0,0,0,-30,0,0,0],0)
    assert ctl.step(0,{})==([30.,-30.],[0,0,0,0])
    ctl.step(.01,{'left':True})
    ctl.feedback([30,0,0,0,-30,0,0,0],.1)
    positions,states=ctl.step(.15,{})
    assert -45 < positions[0] < 30 and positions[1] == -30
    assert states==[0,0,1,0] # Target closed but still physically open.
    ctl.step(.15,{'right':True})
    ctl.feedback([-45,0,0,0,45,0,0,0],.2)
    assert ctl.step(.2,{})[1]==[1,1,1,1]


def test_timeout_invalid_feedback_and_recovery_reanchor():
    ctl=DualGrippers(config())
    ctl.feedback([30,0,0,0,-30,0,0,0],0)
    ctl.step(.01,{'left':True})
    positions, states = ctl.step(.3,{})
    assert all(map(math.isnan, positions)) and states == [-1]*4
    ctl.feedback([0,0,0,0,0,0,0,0],.31)
    assert ctl.step(.31,{})[0]==[0,0]
    ctl.feedback([float('nan')]*8,.32)
    assert ctl.step(.32,{})[1]==[-1]*4


def test_uncalibrated_side_does_not_move_or_claim_state():
    cfg=config();cfg['left'].update(calibrated=False,open_deg=None,closed_deg=None)
    ctl=DualGrippers(cfg)
    ctl.feedback([0]*8,0)
    positions,states=ctl.step(.01,{'left':True})
    assert math.isnan(positions[0]) and states[0]==states[2]==-1
    with pytest.raises(ValueError):binary_states(states)


def test_hysteresis_and_position_trajectory():
    side=Gripper(config()['right'])
    side.feedback(-30,0)
    for fraction in [.45,.55,.59]:
        side.feedback(-30+75*fraction,0);assert side.measured==0
    side.feedback(-30+75*.61,0);assert side.measured==1
    side.feedback(-30+75*.45,0);assert side.measured==1
    side.feedback(-30+75*.39,0);assert side.measured==0
    side.feedback(-30,0);side.toggle(0)
    assert side.position(0) == -30
    assert side.position(.3) == pytest.approx(7.5)
    assert side.position(.6) == 45
    assert side.position(1) == 45


def test_position_offset_and_mid_motion_reversal():
    side = Gripper(dict(config()['right'], zero_pos_deg=-8.0))
    side.feedback(-30, 0)
    assert side.position(0) == -22
    side.toggle(0)
    assert side.position(.6) == 53
    side.feedback(10, .2)
    side.toggle(.3)
    assert side.position(.3) == 18
    assert side.position(1) == -22


def test_invalid_side_does_not_block_the_other_side():
    ctl = DualGrippers(config())
    ctl.feedback([math.nan,0,0,0,-30,0,0,0], 0)
    positions, states = ctl.step(.1, {'right': True})
    assert math.isnan(positions[0]) and positions[1] == -30
    assert states == [-1,0,-1,1]


def test_load_uses_servo_zero_and_checks_endpoint_limits(tmp_path):
    endpoint_path = tmp_path/'grippers.yaml'
    endpoint_path.write_text(yaml.safe_dump(config()))
    control = {'mode':'position', 'zero_pos_deg':-8., 'pos_min_deg':-50., 'pos_max_deg':50.}
    controller = {'gripperControl': {'l_gripper_ctl':dict(control), 'r_gripper_ctl':dict(control)}}
    controller_path = tmp_path/'controller.yaml'
    controller_path.write_text(yaml.safe_dump(controller))
    loaded = load_config(endpoint_path, controller_path)
    assert loaded['left']['zero_pos_deg'] == loaded['right']['zero_pos_deg'] == -8
    controller['gripperControl']['r_gripper_ctl']['pos_max_deg'] = 40
    controller_path.write_text(yaml.safe_dump(controller))
    with pytest.raises(ValueError, match='outside servo position limits'):
        load_config(endpoint_path, controller_path)
    controller['gripperControl']['r_gripper_ctl']['mode'] = 'torque'
    controller_path.write_text(yaml.safe_dump(controller))
    with pytest.raises(ValueError, match='must use position mode'):
        load_config(endpoint_path, controller_path)


def test_checked_in_position_config_matches_calibrated_endpoints():
    cfg = load_config(controller_path=ROOT/'ros2_ws/src/ctl_msgs_hw/config/controllerConfig.yaml')
    DualGrippers(cfg)


def test_endpoint_capture_both_sides_and_degenerate_rejection(tmp_path):
    cfg=config()
    for side in ('left','right'):cfg[side]['calibrated']=False
    path=tmp_path/'grippers.yaml';path.write_text(yaml.safe_dump(cfg))
    result=save_endpoint(path,'left','open',[(30,.0)]*30)
    assert not result['calibrated']
    result=save_endpoint(path,'left','closed',[(-45,.0)]*30)
    assert result['calibrated'] and result['closed_deg']==-45
    original=path.read_text()
    with pytest.raises(ValueError,match='5 degrees'):
        save_endpoint(path,'left','closed',[(31,0)]*30)
    assert path.read_text()==original
    with pytest.raises(ValueError,match='moving'):
        save_endpoint(path,'right','closed',[(45,.3)]*30)
    assert path.read_text()==original
    assert len(list(tmp_path.glob('*.backup')))==2


def test_nonbinary_recording_status_rejected_and_invalidates_cache():
    cache=StreamCache()
    cache.update('gripper_state',[0,1,1,0],1)
    with pytest.raises(ValueError):cache.update('gripper_state',[-1,1,1,0],1.1)
    assert 'gripper_state' not in cache.items
    with pytest.raises(ValueError):binary_states([0,.5,0,1])


def test_vr_left_buttons_record_discard_and_squeezes_toggle_matching_grippers(monkeypatch):
    import importlib.util
    import types
    import time
    pytest.importorskip('rclpy')
    pytest.importorskip('ctl_msgs_hw.msg')
    televuer=types.ModuleType('televuer');televuer.TeleVuerWrapper=object
    monkeypatch.setitem(sys.modules,'televuer',televuer)
    spec=importlib.util.spec_from_file_location('towel_vr_test',ROOT/'teleop/VR_controller/vr_pose_increment_publisher.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    published=[];states=[];calls=[]
    ctl=DualGrippers(config());now=time.monotonic()
    ctl.feedback([30,0,0,0,-30,0,0,0],now)
    fake=types.SimpleNamespace(
        dual_grippers=ctl,prev_record_button=False,prev_discard_button=False,
        prev_left_gripper_button=False,prev_right_gripper_button=False,
        gripper_ctl_pub=types.SimpleNamespace(publish=published.append),
        gripper_state_pub=types.SimpleNamespace(publish=states.append),
        toggle_recording=lambda:calls.append('record'),discard_recording=lambda:calls.append('discard'))
    buttons=types.SimpleNamespace(
        left_squeeze_ctrl_state=True, right_squeeze_ctrl_state=True,
        left_aButton=False, left_bButton=False, right_bButton=False,
    )
    data=types.SimpleNamespace(tele_state=buttons)
    for _ in range(3):module.VRPoseIncrementPublisher.update_gripper(fake,data)
    assert list(states[-1].data)==[0,0,1,1] # No repeat toggle while held.
    assert -45 <= published[-1].l_arm_gripper_pos <= 30
    assert -30 <= published[-1].r_arm_gripper_pos <= 45
    assert published[-1].l_arm_gripper_tor == published[-1].r_arm_gripper_tor == 0
    buttons.left_bButton=True
    module.VRPoseIncrementPublisher.update_gripper(fake,data)
    assert calls==['record']
    buttons.left_bButton=False
    module.VRPoseIncrementPublisher.update_gripper(fake,data)
    buttons.left_aButton=True
    module.VRPoseIncrementPublisher.update_gripper(fake,data)
    assert calls==['record', 'discard']
    buttons.left_squeeze_ctrl_state=False
    module.VRPoseIncrementPublisher.update_gripper(fake,data)
    buttons.left_squeeze_ctrl_state=True;buttons.right_bButton=True
    module.VRPoseIncrementPublisher.update_gripper(fake,data)
    assert list(states[-1].data)==[0,0,0,1]
