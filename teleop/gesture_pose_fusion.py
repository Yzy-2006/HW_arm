"""Select measured RGB-D or calibrated PnP poses from matching capture frames."""

from gesture_control import OPEN_PALM


class PosePacketFusion:
    def __init__(self, hybrid=False, min_quality=0.0, timeout=0.35):
        self.hybrid, self.min_quality, self.timeout = hybrid, min_quality, timeout
        self.frames = {}
        self.last_stamp = -1
        self.preferred = {}

    def feed(self, packet, source, now):
        stamp = packet["capture_monotonic_ns"]
        if stamp <= self.last_stamp:
            return
        frame = self.frames.setdefault(stamp, {"received": now})
        frame[source] = packet
        # Bound memory even if one source stops or the sender floods packets.
        for old in sorted(self.frames)[:-16]:
            del self.frames[old]

    def take(self, now):
        self.frames = {stamp: frame for stamp, frame in self.frames.items()
                       if now - stamp / 1e9 <= self.timeout}
        ready = [stamp for stamp, frame in self.frames.items()
                 if not self.hybrid or ("rgbd" in frame and "rgb" in frame)
                 or now - frame["received"] >= 0.05]
        if not ready:
            return None
        stamp = max(ready)
        frame = self.frames[stamp]
        if self.last_stamp >= 0 and (stamp - self.last_stamp) / 1e9 > self.timeout:
            self.preferred.clear()
        self.last_stamp = stamp
        self.frames = {key: value for key, value in self.frames.items() if key > stamp}
        primary, rgb = frame.get("rgbd"), frame.get("rgb")
        base = primary if primary is not None else rgb
        result = dict(base, hands=[], gestures=base["gestures"].copy())
        sources = {
            name: {hand["label"]: hand for hand in packet["hands"] if hand["quality"] >= self.min_quality}
            for name, packet in (("rgbd", primary), ("rgb", rgb)) if packet is not None
        }
        for label, gesture in result["gestures"].items():
            if primary is not None and rgb is not None and (
                    primary["sequence"] != rgb["sequence"] or primary["gestures"][label] != rgb["gestures"][label]):
                result["gestures"][label] = -1
                continue
            if gesture >= 0 and gesture != OPEN_PALM:
                self.preferred.pop(label, None)
            # Stay on PnP while holding an open palm, avoiding repeated reference
            # resets when marginal depth alternates valid/invalid every frame.
            order = ("rgb", "rgbd") if self.preferred.get(label) == "rgb" else ("rgbd", "rgb")
            for source in order:
                hand = sources.get(source, {}).get(label)
                if hand is not None:
                    self.preferred[label] = source
                    result["hands"].append(dict(hand, pose_source=hand.get(
                        "pose_source", "rgb_pnp" if source == "rgb" else "rgbd")))
                    break
        return result
