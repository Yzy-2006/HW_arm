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
import socket
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np

from pi05_common import decode_bytes, recv_packet, send_packet


LOG = logging.getLogger("pi05_worker")


def _load_pi05_policy_class():
    try:
        from lerobot.policies.pi05 import PI05Policy
    except ImportError:
        from lerobot.policies.pi05.modeling_pi05 import PI05Policy
    return PI05Policy


def _is_openpi_checkpoint(model_path: Path) -> bool:
    """OpenPI publishes inference.py/Orbax files instead of config.json."""
    return (model_path / "inference.py").is_file() and not (model_path / "config.json").is_file()


def _load_openpi_inference_module(model_path: Path):
    inference_path = model_path / "inference.py"
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
    ):
        model_path = str(Path(model_path).expanduser())
        model_dir = Path(model_path)
        if not model_dir.exists():
            raise FileNotFoundError(f"Policy path does not exist: {model_path}")
        self.model_path = model_path
        self.camera_map = camera_map
        self.backend = "openpi" if _is_openpi_checkpoint(model_dir) else "lerobot"

        if self.backend == "openpi":
            self._init_openpi(model_dir, device, allow_cpu, checkpoint, num_steps)
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
        module = _load_openpi_inference_module(model_dir)
        self.policy = module.load_policy(checkpoint=str(checkpoint_path), num_steps=num_steps)
        self.device = ", ".join(str(item) for item in devices)
        self.input_camera_keys = ["head", "left_wrist", "right_wrist"]
        LOG.info("OpenPI devices: %s", devices)
        LOG.info("OpenPI action horizon: 50; worker returns absolute target chunks")

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


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
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
    parser.add_argument("--num-steps", type=int, default=10, help="OpenPI flow integration steps")
    parser.add_argument(
        "--camera-map",
        default="{}",
        help="JSON mapping from policy camera suffix to bridge camera name",
    )
    parser.add_argument("--log-level", default="INFO")
    args = parser.parse_args()
    logging.basicConfig(level=getattr(logging, args.log_level.upper()), format="%(asctime)s %(levelname)s %(message)s")
    try:
        camera_map = json.loads(args.camera_map)
        if not isinstance(camera_map, dict):
            raise ValueError("--camera-map must be a JSON object")
        runner = PolicyRunner(
            args.model,
            args.device,
            args.allow_cpu,
            camera_map,
            args.checkpoint,
            args.num_steps,
        )
    except Exception:
        LOG.exception("Could not load PI05 policy")
        return 2

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((args.host, args.port))
    server.listen(1)
    LOG.info("PI05 worker listening on %s:%d", args.host, args.port)
    try:
        while True:
            conn, address = server.accept()
            LOG.info("Bridge connected from %s", address)
            try:
                _handle_client(conn, runner, args.token)
            except (ConnectionError, OSError, ValueError) as exc:
                LOG.warning("Bridge disconnected: %s", exc)
    except KeyboardInterrupt:
        LOG.info("Stopping PI05 worker")
    finally:
        server.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
