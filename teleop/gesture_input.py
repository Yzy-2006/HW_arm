"""Shared local UDP transport for gesture simulation and hardware adapters."""

import socket

from gesture_control import decode_packet
from gesture_pose_fusion import PosePacketFusion


class HandPacketReceiver:
    def __init__(self, port, rgb_port, config):
        self.sockets = {}
        self.timeout = config["timeout_s"]
        self.fusion = PosePacketFusion(rgb_port is not None, config["min_quality"], self.timeout)
        try:
            for source, selected in (("rgbd", port), ("rgb", rgb_port)):
                if selected is None:
                    continue
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.sockets[source] = sock
                sock.bind(("127.0.0.1", selected))
                sock.setblocking(False)
        except Exception:
            self.close()
            raise

    def poll(self, now):
        for source, sock in self.sockets.items():
            for _ in range(256):
                try:
                    raw, _ = sock.recvfrom(65535)
                except BlockingIOError:
                    break
                packet = decode_packet(raw, now, self.timeout)
                if packet is not None:
                    self.fusion.feed(packet, source, now)
        return self.fusion.take(now)

    def close(self):
        for sock in self.sockets.values():
            sock.close()
