#!/usr/bin/env python3
"""Run a local PI05 policy server for the ENCOS ROS2 bridge.

The worker intentionally has no ROS dependency. It receives JPEG observations over
a small length-prefixed JSON protocol and returns post-processed action chunks. It
supports both LeRobot ``config.json`` checkpoints and the published OpenPI/JAX
``inference.py`` plus Orbax checkpoint layout.
"""

from __future__ import annotations

import argparse
import hmac
import importlib.util
import io
import json
import logging
import os
import signal
import socket
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np

from pi05_common import decode_bytes, recv_packet, send_packet


LOG = logging.getLogger("pi05_worker")
OPENPI_INFERENCE_ENTRIES = ("inference.py", "inference_flat_towel.py", "inference_mix159.py")


def _request_shutdown(_signum, _frame):
    raise KeyboardInterrupt


def _load_pi05_policy_class():
    try:
        from lerobot.policies.pi05 import PI05Policy
    except ImportError:
        from lerobot.policies.pi05.modeling_pi05 import PI05Policy
    return PI05Policy


def _is_openpi_checkpoint(model_path: Path) -> bool:
    """OpenPI publishes inference.py/Orbax files instead of config.json."""
    return (model_path / "inference.py").is_file() and not (model_path / "config.json").is_file()


def _load_openpi_inference_module(model_path: Path, entry: str = "inference.py"):
    if entry not in OPENPI_INFERENCE_ENTRIES:
        raise ValueError(f"Unsupported OpenPI inference entry: {entry}")
    inference_path = model_path / entry
    if not inference_path.is_file():
        raise FileNotFoundError(f"OpenPI inference entry not found: {inference_path}")
    # Release-specific entries import helpers from their sibling inference.py.
    sys.path.insert(0, str(model_path.resolve()))
    spec = importlib.util.spec_from_file_location("downloaded_pi05_inference", inference_path)
    if spec is None or spec.loader is None:
        raise ImportError(f"Could not load OpenPI inference module: {inference_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class PolicyRunner:
    def __init__(
        self,
        model_path: str,
        device: str,
        allow_cpu: bool,
        camera_map: dict[str, str],
        checkpoint: str,
        num_steps: int,
        action_horizon: int = 50,
        inference_entry: str = "inference.py",
    ):
        model_path = str(Path(model_path).expanduser())
        model_dir = Path(model_path)
        if not model_dir.exists():
            raise FileNotFoundError(f"Policy path does not exist: {model_path}")
        self.model_path = model_path
        self.camera_map = camera_map
        self.action_horizon = action_horizon
        self.backend = "openpi" if _is_openpi_checkpoint(model_dir) else "lerobot"

        if self.backend == "openpi":
            self._init_openpi(model_dir, device, allow_cpu, checkpoint, num_steps, inference_entry)
            return

        import torch
        from lerobot.policies import make_pre_post_processors

        self.torch = torch
        if device.startswith("cuda") and not torch.cuda.is_available():
            if not allow_cpu:
                raise RuntimeError("CUDA is unavailable; use a working GPU or pass --allow-cpu")
            device = "cpu"
        self.device = torch.device(device)

        from lerobot.configs.policies import PreTrainedConfig
        from lerobot.policies import get_policy_class

        policy_config = PreTrainedConfig.from_pretrained(model_path, device=str(self.device))
        if policy_config.type in {"pi05", "pi0.5"}:
            policy_cls = _load_pi05_policy_class()
        else:
            policy_cls = get_policy_class(policy_config.type)
        LOG.info("Loading LeRobot %s policy from %s", policy_config.type, model_path)
        self.policy = policy_cls.from_pretrained(model_path, config=policy_config).to(self.device).eval()
        self.preprocessor, self.postprocessor = make_pre_post_processors(
            policy_cfg=self.policy.config,
            pretrained_path=model_path,
            preprocessor_overrides={"device_processor": {"device": str(self.device)}},
            postprocessor_overrides={"device_processor": {"device": str(self.device)}},
        )
        self.input_camera_keys = [
            key
            for key in getattr(self.policy.config, "input_features", {})
            if key.startswith("observation.images.")
        ]
        if not self.input_camera_keys:
            raise ValueError("PI05 checkpoint has no observation.images.* inputs")
        LOG.info("Policy cameras: %s", self.input_camera_keys)
        LOG.info("Policy state: %s", getattr(self.policy.config, "input_features", {}).get("observation.state"))
        LOG.info("Policy action: %s", getattr(self.policy.config, "output_features", {}).get("action"))
        self.policy.reset()

    def _init_openpi(
        self,
        model_dir: Path,
        device: str,
        allow_cpu: bool,
        checkpoint: str,
        num_steps: int,
        inference_entry: str,
    ) -> None:
        try:
            import jax
        except ImportError as exc:
            raise RuntimeError(
                "This checkpoint is OpenPI/JAX format. Install its requirements-inference.txt "
                "in a dedicated inference environment before starting the worker."
            ) from exc
        devices = jax.devices()
        has_gpu = any(getattr(item, "platform", "") == "gpu" for item in devices)
        if device.startswith("cuda") and not has_gpu and not allow_cpu:
            raise RuntimeError(
                f"JAX cannot see a GPU (devices={devices}); install the JAX CUDA runtime "
                "or pass --allow-cpu for a slow smoke test"
            )
        checkpoint_path = Path(checkpoint).expanduser()
        if not checkpoint_path.is_absolute():
            checkpoint_path = model_dir / checkpoint_path
        if not (checkpoint_path / "params").is_dir():
            raise FileNotFoundError(f"OpenPI checkpoint has no params directory: {checkpoint_path}")
        LOG.info("Loading OpenPI/JAX PI05 policy from %s", checkpoint_path)
        LOG.info("OpenPI inference entry: %s", inference_entry)
        module = _load_openpi_inference_module(model_dir, inference_entry)
        self.policy = module.load_policy(checkpoint=str(checkpoint_path), num_steps=num_steps)
        native_horizon = getattr(getattr(self.policy, "_model", None), "action_horizon", None)
        if native_horizon is not None and native_horizon != self.action_horizon:
            raise ValueError(
                f"Configured action_horizon={self.action_horizon}, but the model produces {native_horizon} steps. "
                "Match worker.action_horizon to the model's inference.py; this setting does not resize the model."
            )
        self.device = ", ".join(str(item) for item in devices)
        self.input_camera_keys = ["head", "left_wrist", "right_wrist"]
        LOG.info("OpenPI devices: %s", devices)
        LOG.info("OpenPI action horizon: %d; num_steps=%d; worker returns absolute target chunks",
                 self.action_horizon, num_steps)

    def _decode_image(self, encoded: str) -> np.ndarray:
        from PIL import Image

        image = Image.open(io.BytesIO(decode_bytes(encoded))).convert("RGB")
        return np.asarray(image, dtype=np.uint8)

    def _camera_input(self, payload_images: dict[str, str]) -> dict[str, Any]:
        """Decode LeRobot images in the format expected by its policy pipeline.

        LeRobot checkpoints store visual normalization statistics as floating-point
        tensors.  The generic policy pipeline expects channel-first float images in
        the [0, 1] range; passing decoded uint8 HWC images makes it try to cast those
        statistics to uint8 and raises an overflow error.  OpenPI has a separate
        path above and continues to receive its native HWC uint8 images.
        """
        result = {}
        for feature_key in self.input_camera_keys:
            feature_name = feature_key.removeprefix("observation.images.")
            source_name = self.camera_map.get(feature_name, feature_name)
            if source_name not in payload_images:
                raise ValueError(
                    f"Missing camera '{source_name}' for policy feature '{feature_key}'. "
                    f"Received: {sorted(payload_images)}"
                )
            image = self._decode_image(payload_images[source_name])
            image_chw = image.transpose(2, 0, 1).astype(np.float32, copy=True) / 255.0
            # The LeRobot batch processor adds the batch dimension only to tensors.
            result[feature_key] = self.torch.from_numpy(image_chw)
        return result

    def infer(self, payload: dict[str, Any]) -> list:
        state = np.asarray(payload.get("state"), dtype=np.float32)
        if state.shape != (16,) or not np.isfinite(state).all():
            raise ValueError(f"Expected finite state with shape (16,), got {state.shape}")
        task = payload.get("task")
        if not isinstance(task, str) or not task.strip():
            raise ValueError("Task must be a non-empty string")
        images = payload.get("images")
        if not isinstance(images, dict):
            raise ValueError("Images must be a JSON object")

        if self.backend == "openpi":
            def image_for(name: str) -> np.ndarray:
                source_name = self.camera_map.get(name, name)
                if source_name not in images:
                    raise ValueError(
                        f"Missing camera '{source_name}' for OpenPI input '{name}'. "
                        f"Received: {sorted(images)}"
                    )
                return self._decode_image(images[source_name])

            result = self.policy.infer(
                {
                    "observation.state": state,
                    "observation.images.head": image_for("head"),
                    "observation.images.left_wrist": image_for("left_wrist"),
                    "observation.images.right_wrist": image_for("right_wrist"),
                    "prompt": task,
                }
            )
            actions = np.asarray(result.get("actions"), dtype=np.float32)
            if actions.ndim == 3 and actions.shape[0] == 1:
                actions = actions[0]
            if actions.ndim != 2 or actions.shape[1] != 16 or actions.shape[0] < 1:
                raise ValueError(f"Expected OpenPI actions with shape (horizon, 16), got {actions.shape}")
            expected_horizon = getattr(self, "action_horizon", 50)
            if len(actions) != expected_horizon:
                raise ValueError(f"Expected action_horizon={expected_horizon}, got {len(actions)} steps")
            if not np.isfinite(actions).all():
                raise ValueError("OpenPI action contains non-finite values")
            return actions.tolist()

        frame: dict[str, Any] = {
            # Pass tensors so to_batch_processor adds the batch dimension expected by ACT.
            "observation.state": self.torch.from_numpy(state),
            "task": task,
            **self._camera_input(images),
        }
        with self.torch.inference_mode():
            batch = self.preprocessor(frame)
            action = self.policy.select_action(batch)
            action = self.postprocessor(action)
        if hasattr(action, "detach"):
            action = action.detach().cpu().numpy()
        action = np.asarray(action, dtype=np.float32).reshape(-1)
        if action.shape != (16,) or not np.isfinite(action).all():
            raise ValueError(f"Expected finite action with shape (16,), got {action.shape}")
        return [float(value) for value in action]


def _handle_client(
    conn: socket.socket,
    runner: PolicyRunner,
    token: str,
) -> None:
    reset = getattr(runner.policy, "reset", None)
    if reset is not None:
        reset()
    with conn:
        conn.settimeout(10.0)
        while True:
            request = recv_packet(conn)
            if token and not hmac.compare_digest(str(request.get("token", "")), token):
                send_packet(conn, {"ok": False, "error": "invalid token"})
                continue
            kind = request.get("type")
            if kind == "health":
                send_packet(
                    conn,
                    {
                        "ok": True,
                        "backend": runner.backend,
                        "device": str(runner.device),
                        "model": runner.model_path,
                    },
                )
                continue
            if kind != "infer":
                send_packet(conn, {"ok": False, "error": f"unknown request type: {kind!r}"})
                continue
            started = time.perf_counter()
            try:
                if request.get("reset") and reset is not None:
                    reset()
                actions = np.asarray(runner.infer(request), dtype=np.float32).reshape(-1, 16)
                send_packet(
                    conn,
                    {
                        "ok": True,
                        "request_id": request.get("request_id"),
                        "action": actions[0].tolist(),
                        "actions": actions.tolist(),
                        "latency_ms": (time.perf_counter() - started) * 1000.0,
                    },
                )
            except Exception as exc:  # keep the server alive for the next observation
                LOG.exception("Inference failed")
                send_packet(conn, {"ok": False, "request_id": request.get("request_id"), "error": str(exc)})


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", help="Deployment YAML containing worker.num_steps and worker.action_horizon")
    parser.add_argument("--model", required=True, help="Local PI05 checkpoint directory")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--token", default="")
    parser.add_argument("--allow-cpu", action="store_true")
    parser.add_argument(
        "--checkpoint",
        default="step-19999",
        help="OpenPI checkpoint directory inside --model (ignored for LeRobot checkpoints)",
    )
    parser.add_argument("--num-steps", type=int, help="Override worker.num_steps (default: 10)")
    parser.add_argument("--action-horizon", type=int, help="Override expected OpenPI action count (default: 50)")
    parser.add_argument("--inference-entry", choices=OPENPI_INFERENCE_ENTRIES,
                        help="OpenPI entry filename inside --model; overrides worker.inference_entry")
    parser.add_argument(
        "--camera-map",
        default="{}",
        help="JSON mapping from policy camera suffix to bridge camera name",
    )
    parser.add_argument("--log-level", default="INFO")
    args = parser.parse_args(argv)
    worker_cfg = {}
    if args.config:
        import yaml

        try:
            config = yaml.safe_load(Path(args.config).expanduser().read_text())
        except (OSError, yaml.YAMLError) as exc:
            parser.error(f"Cannot read deployment config: {exc}")
        if not isinstance(config, dict) or not isinstance(config.get("worker", {}), dict):
            parser.error("Deployment config and its worker section must be mappings")
        worker_cfg = config.get("worker", {})
    if args.inference_entry is None:
        args.inference_entry = worker_cfg.get("inference_entry", "inference.py")
    if args.inference_entry not in OPENPI_INFERENCE_ENTRIES:
        parser.error("worker.inference_entry must be one of: " + ", ".join(OPENPI_INFERENCE_ENTRIES))
    for name, default in (("num_steps", 10), ("action_horizon", 50)):
        value = getattr(args, name)
        if value is None:
            value = worker_cfg.get(name, default)
        if type(value) is not int or value <= 0 or (name == "action_horizon" and value > 1000):
            parser.error(f"worker.{name} must be a positive integer" + (" <= 1000" if name == "action_horizon" else ""))
        setattr(args, name, value)
    return args


def main() -> int:
    args = parse_args()
    logging.basicConfig(level=getattr(logging, args.log_level.upper()), format="%(asctime)s %(levelname)s %(message)s")
    signal.signal(signal.SIGINT, _request_shutdown)
    signal.signal(signal.SIGTERM, _request_shutdown)
    LOG.info("Policy worker pid=%d interpreter=%s", os.getpid(), sys.executable)
    try:
        camera_map = json.loads(args.camera_map)
        if not isinstance(camera_map, dict):
            raise ValueError("--camera-map must be a JSON object")
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            # Reserve the port before allocating GPU memory for another model.
            server.bind((args.host, args.port))
            try:
                runner = PolicyRunner(
                    args.model,
                    args.device,
                    args.allow_cpu,
                    camera_map,
                    args.checkpoint,
                    args.num_steps,
                    args.action_horizon,
                    args.inference_entry,
                )
            except Exception:
                LOG.exception("Could not load policy")
                return 2
            server.listen(1)
            LOG.info("PI05 worker listening on %s:%d", args.host, args.port)
            while True:
                conn, address = server.accept()
                LOG.info("Bridge connected from %s", address)
                try:
                    _handle_client(conn, runner, args.token)
                except (ConnectionError, OSError, ValueError) as exc:
                    LOG.warning("Bridge disconnected: %s", exc)
    except KeyboardInterrupt:
        LOG.info("Stopping policy worker pid=%d; GPU memory is released when this process exits", os.getpid())
    except (OSError, ValueError):
        LOG.exception("Could not start policy worker")
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
