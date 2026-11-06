#!/usr/bin/env python3
"""Benchmark serial PI05 inference without cameras, ROS, or robot commands."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import importlib.metadata
import io
import json
import logging
import os
from pathlib import Path
import platform
import time

import numpy as np
from PIL import Image
import yaml

from pi05_common import encode_bytes
from pi05_worker import PolicyRunner, _is_openpi_checkpoint, parse_args as worker_args


ROOT = Path(__file__).resolve().parents[1]
CAMERAS = ("head", "left_wrist", "right_wrist")
LOG = logging.getLogger("pi05_benchmark")


def read_observation(config, input_path=None):
    if input_path:
        with np.load(input_path, allow_pickle=False) as data:
            observation = {key: data[key].copy() for key in data.files}
        observation["prompt"] = str(observation.get("prompt", config["task"]))
    else:
        camera_path = Path(os.path.expandvars(config.get("camera_config", "config/recording.yaml"))).expanduser()
        if not camera_path.is_absolute():
            camera_path = ROOT / camera_path
        camera = yaml.safe_load(camera_path.read_text())
        rng = np.random.default_rng(42)
        observation = {
            "observation.state": np.zeros(16, dtype=np.float32),
            "prompt": config["task"],
            **{f"observation.images.{name}": rng.integers(
                0, 256, (camera["height"], camera["width"], 3), dtype=np.uint8,
            ) for name in CAMERAS},
        }
    state = np.asarray(observation["observation.state"], dtype=np.float32)
    if state.shape != (16,) or not np.isfinite(state).all():
        raise ValueError("Observation must contain finite observation.state[16]")
    observation["observation.state"] = state
    if not observation["prompt"].strip():
        raise ValueError("Observation prompt must not be empty")
    for name in CAMERAS:
        image = observation[f"observation.images.{name}"]
        if image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3:
            raise ValueError(f"Camera {name} must be RGB uint8 HWC")
    return observation


def make_payload(observation, jpeg_quality):
    images = {}
    for name in CAMERAS:
        with io.BytesIO() as buffer:
            Image.fromarray(observation[f"observation.images.{name}"]).save(
                buffer, format="JPEG", quality=jpeg_quality,
            )
            images[name] = encode_bytes(buffer.getvalue())
    return {"state": observation["observation.state"].tolist(),
            "task": observation["prompt"], "images": images}


def measure(infer, *, horizon, warmup, iterations, min_seconds):
    def timed_call():
        started = time.perf_counter()
        # Host conversion waits for JAX device results, so asynchronous dispatch
        # cannot make an unfinished GPU computation appear to be fast.
        actions = np.asarray(infer(), dtype=np.float32)
        elapsed_ms = (time.perf_counter() - started) * 1000
        if actions.shape != (horizon, 16) or not np.isfinite(actions).all():
            raise ValueError(f"Expected finite actions[{horizon},16], got {actions.shape}")
        return elapsed_ms

    first_call_ms = timed_call()
    LOG.info("First call %.1f ms (may include compilation); warming up %d calls", first_call_ms, warmup)
    for _ in range(warmup):
        timed_call()
    latencies = []
    started = time.perf_counter()
    while len(latencies) < iterations or time.perf_counter() - started < min_seconds:
        latencies.append(timed_call())
        if len(latencies) % 100 == 0:
            LOG.info("Measured %d calls", len(latencies))
    wall_s = time.perf_counter() - started
    values = np.asarray(latencies)
    return {
        "first_call_ms": first_call_ms,
        "warmup_calls_after_first": warmup,
        "iterations": len(values), "wall_s": wall_s,
        "mean_ms": float(values.mean()),
        **{f"p{q}_ms": float(np.percentile(values, q)) for q in (50, 95, 99)},
        "min_ms": float(values.min()), "max_ms": float(values.max()),
        "serial_inferences_per_s": len(values) / wall_s,
        "latencies_ms": latencies,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(ROOT / "config/pi05_deployment.yaml"))
    parser.add_argument("--model", required=True)
    parser.add_argument("--checkpoint", default="step-19999")
    parser.add_argument("--num-steps", type=int)
    parser.add_argument("--inference-entry", choices=("inference.py", "inference_flat_towel.py"))
    parser.add_argument("--mode", choices=("policy", "worker", "both"), default="both")
    parser.add_argument("--input", type=Path, help="Optional NPZ: flat observation.* keys and prompt")
    parser.add_argument("--warmup", type=int, default=5)
    parser.add_argument("--iterations", type=int, default=100)
    parser.add_argument("--min-seconds", type=float, default=0, help="Minimum measured duration per mode")
    parser.add_argument("--output", type=Path, default=Path("outputs/pi05_benchmark.json"))
    args = parser.parse_args()
    if args.warmup < 1 or args.iterations < 1 or not np.isfinite(args.min_seconds) or args.min_seconds < 0:
        parser.error("warmup and iterations must be positive; min-seconds must be finite and nonnegative")
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    model_path = Path(os.path.expandvars(args.model)).expanduser().resolve()
    if not _is_openpi_checkpoint(model_path):
        parser.error("Benchmark requires the published OpenPI PI05 model directory containing inference.py")
    settings_argv = ["--config", args.config, "--model", str(model_path), "--checkpoint", args.checkpoint]
    if args.num_steps is not None:
        settings_argv += ["--num-steps", str(args.num_steps)]
    if args.inference_entry is not None:
        settings_argv += ["--inference-entry", args.inference_entry]
    settings = worker_args(settings_argv)
    config = yaml.safe_load(Path(args.config).expanduser().read_text())
    observation = read_observation(config, args.input)
    payload = make_payload(observation, int(config.get("bridge", {}).get("jpeg_quality", 80)))

    load_started = time.perf_counter()
    runner = PolicyRunner(settings.model, "cuda", False, {}, settings.checkpoint,
                          settings.num_steps, settings.action_horizon, settings.inference_entry)
    import jax
    from flax import nnx

    # Finish checkpoint transfers before measuring the first inference call.
    jax.block_until_ready(nnx.state(runner.policy._model))
    load_s = time.perf_counter() - load_started
    versions = {}
    for package in ("jax", "jaxlib", "flax", "orbax-checkpoint", "numpy", "torch"):
        try:
            versions[package] = importlib.metadata.version(package)
        except importlib.metadata.PackageNotFoundError:
            versions[package] = None
    report = {
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "model": str(model_path), "checkpoint": settings.checkpoint,
        "inference_entry": settings.inference_entry,
        "num_steps": settings.num_steps, "action_horizon": settings.action_horizon,
        "device": runner.device, "platform": platform.platform(), "machine": platform.machine(),
        "devices": [{"id": device.id, "kind": device.device_kind, "platform": device.platform}
                    for device in jax.local_devices()],
        "versions": versions, "model_load_s": load_s,
        "input": str(args.input) if args.input else "synthetic fixed RGB images, zero state, seed 42",
        "image_shapes": {name: list(observation[f"observation.images.{name}"].shape) for name in CAMERAS},
        "allocation_env": {name: os.environ.get(name) for name in (
            "XLA_PYTHON_CLIENT_PREALLOCATE", "XLA_PYTHON_CLIENT_MEM_FRACTION", "XLA_FLAGS")},
        "scope": "batch=1 serial; preprocessing, GPU inference, postprocessing and host result; no TCP/ROS/camera capture",
        "results": {},
    }
    modes = ("policy", "worker") if args.mode == "both" else (args.mode,)
    for mode in modes:
        LOG.info("Benchmarking %s mode, num_steps=%d horizon=%d", mode, settings.num_steps, settings.action_horizon)
        infer = (lambda: runner.policy.infer(observation)["actions"]) if mode == "policy" else lambda: runner.infer(payload)
        result = measure(infer, horizon=settings.action_horizon, warmup=args.warmup,
                         iterations=args.iterations, min_seconds=args.min_seconds)
        report["results"][mode] = result
        print(f"{mode}: mean={result['mean_ms']:.2f} ms p50={result['p50_ms']:.2f} ms "
              f"p95={result['p95_ms']:.2f} ms p99={result['p99_ms']:.2f} ms "
              f"serial_inferences_per_s={result['serial_inferences_per_s']:.2f}", flush=True)
        # Preserve completed modes even if a later benchmark is interrupted.
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    LOG.info("Saved benchmark report: %s", args.output.resolve())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
