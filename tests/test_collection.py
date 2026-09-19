import json
import math
import sys
from pathlib import Path

import cv2
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'collection'))
from core import EpisodeWriter, REQUIRED, StreamCache, vectors
from export_lerobot import export, load_episode, resample


def sample():
    data = {key: list(range(size)) for key, size in REQUIRED.items()}
    data['joint_target'] = list(range(20,34))
    data['gripper_state'] = [0,1,1,0]
    return data


def test_state_action_use_distinct_measured_and_commanded_gripper_bits():
    state, action = vectors(sample())
    assert len(state) == len(action) == 16
    assert state[:7] == list(range(7))
    assert state[7] == 0
    assert state[8:15] == list(range(7,14))
    assert state[15] == 1
    assert action[:7] == list(range(20,27))
    assert action[7] == 1
    assert action[8:15] == list(range(27,34))
    assert action[15] == 0


def test_cache_requires_all_streams_and_checks_age_and_skew():
    cache = StreamCache()
    with pytest.raises(ValueError, match='Missing'): cache.snapshot(1,.2,.1)
    for key,value in sample().items(): cache.update(key,value,1)
    assert cache.snapshot(1.1,.2,.1)['joint_target'][0] == 20
    with pytest.raises(ValueError, match='stale'): cache.snapshot(1.3,.2,.1)
    cache.update('q',sample()['q'],1.15)
    with pytest.raises(ValueError, match='skew'): cache.snapshot(1.16,.2,.1)
    with pytest.raises(ValueError): cache.update('q',[float('nan')]*14,1.16)
    with pytest.raises(ValueError, match='Missing'): cache.snapshot(1.16,.2,.1)


def metadata():
    return {'config': {'task':'wipe with towel', 'cameras':{
            'head':'h', 'left_wrist':'lw', 'right_wrist':'rw'},
            'width':4,'height':3,'fps':30,'max_frame_gap_s':.1}}


def write_episode(tmp_path):
    writer = EpisodeWriter(tmp_path, metadata(), cv2.imwrite)
    images = {key:np.zeros((3,4,3),np.uint8) for key in metadata()['config']['cameras']}
    for t in [0,.033,.080,.100]:
        writer.add(dict(sample(),timestamp_s=t),images)
    return writer.finish(3)


def test_save_validate_resample_and_no_accidental_overwrite(tmp_path):
    path=write_episode(tmp_path)
    meta,rows=load_episode(path)
    assert meta['frames']==4
    sampled=list(resample(rows,30))
    assert len(sampled)==4
    assert [r['timestamp_s'] for _,r in sampled]==[0,.033,.033,.100]
    export(tmp_path,tmp_path/'unused','local/test',validate_only=True)
    with pytest.raises(ValueError,match='already exists'):
        export(tmp_path,path,'local/test')


def test_abort_short_and_discard_never_become_exportable(tmp_path):
    writer=EpisodeWriter(tmp_path,metadata(),cv2.imwrite)
    writer.abort('camera disconnected')
    assert writer.path.name.endswith('.partial')
    assert json.loads((writer.path/'episode.json').read_text())['status']=='aborted'
    writer2=EpisodeWriter(tmp_path,metadata(),cv2.imwrite)
    writer2.discard()
    assert not writer2.path.exists()
    writer3=EpisodeWriter(tmp_path,metadata(),cv2.imwrite)
    assert writer3.finish(2) is None
    with pytest.raises(ValueError,match='No complete'):
        export(tmp_path,tmp_path/'out','local/test',True)


def test_corrupt_image_and_gap_are_rejected(tmp_path):
    path=write_episode(tmp_path)
    image=path/'images/head/000000.png'
    original=image.read_bytes()
    image.write_bytes(b'broken')
    with pytest.raises(ValueError,match='corrupt image'): load_episode(path)
    image.write_bytes(original)
    rows=[json.loads(line) for line in (path/'frames.jsonl').read_text().splitlines()]
    rows[-1]['timestamp_s']=.5
    (path/'frames.jsonl').write_text('\n'.join(json.dumps(row) for row in rows))
    with pytest.raises(ValueError,match='gap'): load_episode(path)


def test_write_failure_does_not_increment_count(tmp_path):
    writer=EpisodeWriter(tmp_path,metadata(),lambda *args:False)
    with pytest.raises(IOError): writer.add(sample(),{'head':np.zeros((3,4,3),np.uint8)})
    assert writer.frames==0
    writer.abort('disk failure')


def test_export_frame_layout_through_dataset_api(tmp_path, monkeypatch):
    import types
    path=write_episode(tmp_path/'raw')
    captured=[]
    class Dataset:
        @classmethod
        def create(cls, **kwargs):
            cls.schema=kwargs['features']
            (kwargs['root']/'meta').mkdir(parents=True)
            return cls()
        def add_frame(self, frame):
            assert set(frame)==set(self.schema)|{'task'}
            for key,feature in self.schema.items():
                assert frame[key].shape==feature['shape']
            captured.append(frame)
        def save_episode(self): pass
        def finalize(self): pass
    package=types.ModuleType('lerobot.datasets.lerobot_dataset')
    package.LeRobotDataset=Dataset
    monkeypatch.setitem(sys.modules,'lerobot',types.ModuleType('lerobot'))
    monkeypatch.setitem(sys.modules,'lerobot.datasets',types.ModuleType('lerobot.datasets'))
    monkeypatch.setitem(sys.modules,'lerobot.datasets.lerobot_dataset',package)
    export(tmp_path/'raw',tmp_path/'out','local/test')
    assert len(captured)==4
    assert captured[0]['action'][0]==20
    assert captured[0]['observation.state'][0]==0
    assert captured[0]['action'][15]==0
    assert (tmp_path/'out/meta/towel_schema.json').is_file()


def test_recorder_callbacks_and_episode_boundaries(tmp_path,monkeypatch):
    # Use the freshly generated real ROS message definitions, but a fake Node:
    # this exercises callback fields and service state without DDS or hardware.
    node_module=pytest.importorskip('rclpy.node')
    pytest.importorskip('ctl_msgs_hw.msg')
    from types import SimpleNamespace as NS
    class FakeNode:
        def __init__(self,name): self.callbacks={}
        def create_publisher(self,*args): return NS(publish=lambda msg:None)
        def create_timer(self,*args): pass
        def create_service(self,*args): pass
        def create_subscription(self,typ,topic,callback,qos): self.callbacks[topic]=(typ,callback)
        def get_clock(self):return NS(now=lambda:NS(nanoseconds=1000000000))
        def get_logger(self):return NS(info=lambda *a:None,warning=lambda *a:None,error=lambda *a:None)
        def destroy_node(self):pass
    monkeypatch.setattr(node_module,'Node',FakeNode)
    import record
    import yaml
    cfg=yaml.safe_load((Path(record.ROOT)/'config/recording.yaml').read_text())
    cfg.update(dataset_root=str(tmp_path),min_episode_frames=1,width=4,height=3)
    class Camera:
        def __init__(self): self.index=0;self.broken=False
        def snapshot(self,now):
            if self.broken:raise ValueError('disconnected')
            self.index+=1
            return ({key:np.zeros((3,4,3),np.uint8) for key in cfg['cameras']},
                    {key:{'frame_number':self.index} for key in cfg['cameras']})
    camera=Camera(); node=record.make_node(cfg,camera)
    reply=node.record(NS(data=True),NS())
    assert not reply.success
    values=sample()
    def deliver(topic,**fields):
        typ,callback=node.callbacks[topic]
        msg=typ()
        for key,value in fields.items():
            setattr(msg,key,([int(x) for x in value] if topic=='/towel/gripper_state' else [float(x) for x in value]) if isinstance(value,list) else float(value))
        callback(msg)
    def feed():
        deliver('/dexbotCtl/midMonitor_msg_send',l_arm_pos_fdback=values['q'][:7],
                r_arm_pos_fdback=values['q'][7:],l_arm_spd_fdback=values['dq'][:7],r_arm_spd_fdback=values['dq'][7:])
        deliver('/dexbotCtl/ctl_data_pub',l_position=values['joint_target'][:7],r_position=values['joint_target'][7:])
        deliver('/dexbotCtl/DescPub/Fk_ret_data_pub',**{a+'_'+s:i for i,(s,a) in enumerate(
            (s,a) for s in ('l','r') for a in ('x','y','z','roll','pitch','yaw'))})
        deliver('/dexbotCtl/upperMonitor_msg_send',l_arm_des_pos_descartes=values['tcp_target'][:6],r_arm_des_pos_descartes=values['tcp_target'][6:])
        deliver('/towel/gripper_state',data=values['gripper_state'])
    feed()
    assert node.record(NS(data=True),NS()).success
    node.sample()
    assert node.episode.frames==1
    assert node.record(NS(data=False),NS()).success
    assert node.episode is None
    saved=[p for p in tmp_path.glob('episode_*') if not p.name.endswith('.partial')]
    assert len(saved)==1
    _,rows=load_episode(saved[0])
    assert rows[0]['joint_target']==values['joint_target']
    feed(); assert node.record(NS(data=True),NS()).success
    node.sample(); assert node.discard(None,NS()).success
    feed(); assert node.record(NS(data=True),NS()).success
    camera.broken=True;node.sample()
    assert node.episode is None
    assert len(list(tmp_path.glob('*.partial')))==1
    node.close()
