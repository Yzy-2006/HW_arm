import copy
import sys
from pathlib import Path

import pytest
import yaml

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'teleop/VR_controller'))
sys.path.insert(0,str(ROOT/'scripts'))
sys.path.insert(0,str(ROOT/'collection'))
from grippers import DualGrippers, Gripper
from calibrate_grippers import save_endpoint
from core import binary_states, StreamCache


def config():
    side={'calibrated':True,'open_deg':-30.,'closed_deg':45.,'kp':1.1,'kd':.12,
          'motion_time_s':.6,'torque_limit_nm':3.}
    return {'left':dict(side,open_deg=30.,closed_deg=-45.),'right':side,'feedback_timeout_s':.2}


def test_both_directions_independent_targets_and_feedback():
    ctl=DualGrippers(config())
    ctl.feedback([30,0,0,0,-30,0,0,0],0)
    assert ctl.step(0,{})==([0.,0.],[0,0,0,0])
    ctl.step(.01,{'left':True})
    ctl.feedback([30,0,0,0,-30,0,0,0],.1)
    torques,states=ctl.step(.15,{})
    assert torques[0]<0 and torques[1]==0
    assert states==[0,0,1,0] # Target closed but still physically open.
    ctl.step(.15,{'right':True})
    ctl.feedback([-45,0,0,0,45,0,0,0],.2)
    assert ctl.step(.2,{})[1]==[1,1,1,1]


def test_timeout_invalid_feedback_and_recovery_reanchor():
    ctl=DualGrippers(config())
    ctl.feedback([30,0,0,0,-30,0,0,0],0)
    ctl.step(.01,{'left':True})
    assert ctl.step(.3,{})==([0.,0.],[-1,-1,-1,-1])
    ctl.feedback([0,0,0,0,0,0,0,0],.31)
    assert ctl.step(.31,{})[0]==[0,0]
    ctl.feedback([float('nan')]*8,.32)
    assert ctl.step(.32,{})[1]==[-1]*4


def test_uncalibrated_side_does_not_move_or_claim_state():
    cfg=config();cfg['left'].update(calibrated=False,open_deg=None,closed_deg=None)
    ctl=DualGrippers(cfg)
    ctl.feedback([0]*8,0)
    torques,states=ctl.step(.01,{'left':True})
    assert torques[0]==0 and states[0]==states[2]==-1
    with pytest.raises(ValueError):binary_states(states)


def test_hysteresis_and_torque_limit():
    side=Gripper(config()['right'])
    side.feedback(-30,0)
    for fraction in [.45,.55,.59]:
        side.feedback(-30+75*fraction,0);assert side.measured==0
    side.feedback(-30+75*.61,0);assert side.measured==1
    side.feedback(-30+75*.45,0);assert side.measured==1
    side.feedback(-30+75*.39,0);assert side.measured==0
    side.feedback(-1000,100);side.toggle(0)
    assert abs(side.torque(1))<=3


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
    assert published[-1].l_arm_gripper_tor < 0
    assert published[-1].r_arm_gripper_tor > 0
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


def test_vr_right_b_holds_right_arm_feedforward(monkeypatch):
    import importlib.util
    import types
    pytest.importorskip('rclpy')
    pytest.importorskip('ctl_msgs_hw.msg')
    televuer=types.ModuleType('televuer');televuer.TeleVuerWrapper=object
    monkeypatch.setitem(sys.modules,'televuer',televuer)
    spec=importlib.util.spec_from_file_location('towel_vr_feedforward_test',ROOT/'teleop/VR_controller/vr_pose_increment_publisher.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    published=[]
    fake=types.SimpleNamespace(
        right_arm_ff_pub=types.SimpleNamespace(publish=published.append),
        right_arm_ff_force=[1., 2., 3., 4., 5., 6.],
    )
    buttons=types.SimpleNamespace(right_bButton=True, right_squeeze_ctrl_state=False)

    module.VRPoseIncrementPublisher.publish_arm_feedforward(
        fake, types.SimpleNamespace(tele_state=buttons)
    )

    msg=published[-1]
    assert (msg.force.x, msg.force.y, msg.force.z)==(1., 2., 3.)
    assert (msg.torque.x, msg.torque.y, msg.torque.z)==(4., 5., 6.)
