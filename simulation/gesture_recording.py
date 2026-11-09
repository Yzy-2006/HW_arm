"""Small simulation-only JSONL episodes; never mixed with real robot datasets."""

from datetime import datetime, timezone
import json
from pathlib import Path
import uuid


class SimulationRecorder:
    def __init__(self, root, config):
        self.root, self.config = Path(root), config
        self.file = None
        self.path = None

    @property
    def active(self):
        return self.file is not None

    def write(self, value):
        if self.active:
            self.file.write(json.dumps(value, allow_nan=False) + "\n")
            self.file.flush()

    def start(self):
        self.root.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
        self.path = self.root / f"episode_{stamp}_{uuid.uuid4().hex[:8]}.jsonl.partial"
        self.file = self.path.open("x")
        self.write({"type": "metadata", "schema_version": 1, "simulation": True,
                    "backend": "rviz_kinematics", "tcp_frame": "world", "config": self.config})

    def stop(self, reason):
        if not self.active:
            return None
        self.write({"type": "end", "reason": reason})
        self.file.close()
        self.file = None
        complete = self.path.with_suffix("")
        self.path.rename(complete)
        return complete
