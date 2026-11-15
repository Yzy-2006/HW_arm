"""Real ROS bridge/VR/recorder workflow, with synthetic cameras, policy and hardware."""

import json
import os
from pathlib import Path
from queue import Queue, Empty
import sys
import threading
import time
from types import SimpleNamespace as NS

import numpy as np
import yaml
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String, Float64MultiArray
from std_srvs.srv import Trigger, SetBool
from ctl_msgs_hw.msg import MidNodeCtlMsgRet, MidNodeCtlMsgSend, CtlInformation, FkDataPub, BridgeCtlMsg

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / name) for name in ("deployment", "collection", "teleop/VR_controller")]
import record
import pi05_bridge
from ros_shutdown import drain_executor
from worker_lifecycle import WorkerManagementServer, WorkerProcess
from vr_recording import switch_to_recording_only

sys.modules["televuer"] = NS(TeleVuerWrapper=object)
from vr_pose_increment_publisher import VRPoseIncrementPublisher, run_vr_tick


class Cameras:
    frames_immutable = True

    def __init__(self, config):
        self.names = config["cameras"]
        self.frame = 0
        self.closed = False

    def snapshot(self, now):
        self.frame += 1
        images = {name: np.full((32, 32, 3), 100, np.uint8) for name in self.names}
        for image in images.values():
            image.setflags(write=False)
        return (images,
                {name: dict(frame_number=self.frame, receive_monotonic_s=now) for name in self.names})

    def close(self):
        self.closed = True


class Policy:
    def __init__(self, *args):
        self.resets = []

    def infer(self, state, images, task, request_id, *, reset):
        self.resets.append(reset)
        return {"actions": np.repeat(np.asarray(state)[None, :], 50, axis=0).tolist()}

    def close(self):
        pass

    def health(self):
        pass


class Keyboard:
    enabled = True

    def __init__(self):
        self.keys = Queue()

    def poll(self):
        try:
            return self.keys.get_nowait()
        except Empty:
            return None

    def close(self):
        self.enabled = False


class Hardware(Node):
    def __init__(self):
        super().__init__("mock_intervention_hardware")
        self.home_requests = 0
        self.resume_at = 0.
        self.commands = []
        self.create_service(Trigger, "/dexbotCtl/policy_intervene", self.takeover)
        self.create_service(Trigger, "/dexbotCtl/policy_home", self.home)
        self.create_service(SetBool, "/dexbotCtl/vr_override", self.resume)
        self.create_subscription(BridgeCtlMsg, "/dexbotCtl/bridge_ctl_msg", self.commands.append, 1)
        self.streams = [
            (self.create_publisher(String, "/towel/safety/state",
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)), String(data="ready")),
            (self.create_publisher(Float64MultiArray, "/gripper/fdb", 1), Float64MultiArray(data=[0.] * 8)),
        ]
        for typ, topic in ((MidNodeCtlMsgRet, "/dexbotCtl/midMonitor_msg_send"),
                           (MidNodeCtlMsgSend, "/dexbotCtl/upperMonitor_msg_send"),
                           (CtlInformation, "/dexbotCtl/ctl_data_pub"),
                           (FkDataPub, "/dexbotCtl/DescPub/Fk_ret_data_pub")):
            msg = typ()
            for field, kind in msg.get_fields_and_field_types().items():
                if kind == "sequence<double>":
                    setattr(msg, field, [0.] * (6 if "descartes" in field else 7))
                elif kind == "sequence<boolean>":
                    setattr(msg, field, [True] * 7)
            self.streams.append((self.create_publisher(typ, topic, 1), msg))
        self.create_timer(.02, self.feedback)

    def feedback(self):
        for pub, msg in self.streams:
            pub.publish(msg)

    def takeover(self, request, response):
        response.success, response.message = True, "mock VR hold accepted"
        return response

    def home(self, request, response):
        self.home_requests += 1
        self.resume_at = time.monotonic() + .2 + 3.
        response.success, response.message = True, "mock home trajectory accepted"
        return response

    def resume(self, request, response):
        response.success = time.monotonic() >= self.resume_at
        response.message = "ready" if response.success else "home timing pending"
        return response


def main():
    assert os.environ["ROS_DOMAIN_ID"] == "220"
    destination = Path(sys.argv[1])
    config = yaml.safe_load((ROOT / "config/pi05_deployment.yaml").read_text())
    camera_cfg = yaml.safe_load((ROOT / "config/recording.yaml").read_text())
    camera_cfg.update(min_episode_frames=2, min_joint_delta_rad=0., max_frame_gap_s=.5,
                      max_stream_age_s=.5, max_stream_skew_s=.5)
    camera_path = destination / "recording.yaml"
    camera_path.write_text(yaml.safe_dump(camera_cfg))
    config["camera_config"] = str(camera_path)
    worker = WorkerProcess([sys.executable, "-c", "import time; time.sleep(1000)"], shutdown_timeout=.2)
    manager_stopped = threading.Event()
    manager = WorkerManagementServer(("127.0.0.1", 0), worker, on_shutdown=manager_stopped.set)
    config["worker"]["control_port"] = manager.server_address[1]
    config["bridge"]["intervention"] = dict(enabled=True, stop_worker=True, dataset_root=str(destination / "data"))
    config_path = destination / "deployment.yaml"
    config_path.write_text(yaml.safe_dump(config))
    record.Cameras, pi05_bridge.PolicyClient, pi05_bridge.TerminalKeys = Cameras, Policy, Keyboard
    rclpy.init(args=[])
    executor = MultiThreadedExecutor(num_threads=3)
    bridge = vr = hardware = session = None
    manager_thread = threading.Thread(target=manager.serve_forever, kwargs={"poll_interval": .05}, daemon=True)
    try:
        original_pid = worker.start()["pid"]
        manager_thread.start()
        hardware = Hardware()
        bridge = pi05_bridge.create_bridge_node(NS(config=config_path, worker_host=None,
            worker_port=None, token=None, slow_factor=None, motion=True))
        vr = VRPoseIncrementPublisher(policy_intervention=True)
        for node in (hardware, bridge, vr, *bridge.companion_nodes):
            executor.add_node(node)
        buttons = NS(left_aButton=False, left_bButton=False, right_aButton=False,
                     left_trigger_state=False, right_trigger_state=False,
                     left_squeeze_ctrl_state=False, right_squeeze_ctrl_state=False)
        data = NS(tele_state=buttons, head_pose=np.eye(4), left_arm_pose=np.eye(4), right_arm_pose=np.eye(4))
        wrapper = NS(get_motion_state_data=lambda: data)

        def spin(duration):
            nonlocal executor, session
            end = time.monotonic() + duration
            while time.monotonic() < end:
                executor.spin_once(timeout_sec=.01)
                if session is None and bridge.recording_only_requested:
                    executor, session = switch_to_recording_only(executor, bridge)
                run_vr_tick(vr, wrapper)
                time.sleep(.005)

        def until(predicate, timeout=4.):
            end = time.monotonic() + timeout
            while not predicate() and time.monotonic() < end:
                spin(.05)
            assert predicate(), (bridge.control_mode, bridge.safety.fault)

        def press(button):
            setattr(buttons, button, True)
            spin(.12)
            setattr(buttons, button, False)
            spin(.12)

        until(lambda: bool(hardware.commands))
        assert bridge.control_mode == "policy" and not vr._vr_enabled
        press("left_aButton")
        until(lambda: bridge.control_mode == "vr" and vr._vr_enabled)
        until(lambda: bridge.worker_suspended)
        until(lambda: session is not None)
        until(manager_stopped.is_set)
        assert worker.closed
        assert type(executor).__name__ == "SingleThreadedExecutor"
        assert bridge not in executor.get_nodes() and session.recorder in executor.get_nodes()
        assert all(not thread.is_alive() for thread in bridge.inference_pool._threads)
        assert bridge.lifecycle_pool._shutdown and not bridge.keyboard.enabled
        assert not worker.status()["running"]
        try:
            os.kill(original_pid, 0)
        except ProcessLookupError:
            pass
        else:
            raise AssertionError("VR recording enabled before worker process was reaped")
        count, inference_count = len(hardware.commands), len(bridge.client.resets)
        spin(.2)
        assert len(hardware.commands) == count and len(bridge.client.resets) == inference_count
        press("left_bButton")
        until(lambda: session.recorder.start_time is not None)
        spin(.2)
        assert hardware.home_requests == 0 and bridge.control_mode == "vr"
        press("left_bButton")
        until(lambda: not session.recorder.busy())
        episodes = list((destination / "data").glob("*/episode.json"))
        assert len(episodes) == 1
        metadata = json.loads(episodes[0].read_text())
        assert metadata["status"] == "complete" and metadata["frames"] >= 2
        assert metadata["source"] == "vr_policy_intervention"
        spin(.4)
        assert vr._vr_enabled and not session.recorder.busy()
        assert not worker.status()["running"] and len(bridge.client.resets) == inference_count
        assert len(hardware.commands) == count and hardware.home_requests == 0
        press("left_bButton")
        until(lambda: session.recorder.start_time is not None)
        press("left_aButton")
        until(lambda: not session.recorder.busy())
        assert len(list((destination / "data").glob("*/episode.json"))) == 1

        session.safety_received_at = time.monotonic() - 1.
        assert session.recording_guard() == "Waiting for fresh servo safety state"
        until(lambda: session.recording_guard() is None)
        press("left_bButton")
        until(lambda: session.recorder.start_time is not None)
        hardware.streams[0][1].data = "fault:synthetic_recording_fault"
        until(lambda: not session.recorder.busy())
        assert "synthetic_recording_fault" in session.recorder.last_recording_error
        assert session.recording_guard() is not None
        saved = [path for path in (destination / "data").glob("*/episode.json")
                 if not path.parent.name.endswith(".partial")]
        assert saved == episodes
        partial = list((destination / "data").glob("*.partial/episode.json"))
        assert len(partial) == 1 and json.loads(partial[0].read_text())["status"] == "aborted"
        assert not worker.status()["running"] and len(hardware.commands) == count
        print("workflow_passed: policy -> worker shutdown -> bridge destroyed -> single-thread VR record/save/discard")
    finally:
        drain_executor(executor)
        if session is not None:
            session.close()
            assert session.cameras.closed
            session.destroy_node()
        elif bridge is not None:
            bridge.close()
            assert bridge.cameras.closed
            bridge.destroy_node()
        for node in (vr, hardware):
            if node is not None:
                node.destroy_node()
        if manager_thread.is_alive():
            manager.shutdown()
            manager_thread.join()
        manager.server_close()
        worker.close()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
