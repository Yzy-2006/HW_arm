import io
from pathlib import Path
import sys

import numpy as np
from PIL import Image
import pytest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "deployment"))

from benchmark_pi05 import make_payload, measure, read_observation
from pi05_common import decode_bytes


def test_benchmark_excludes_first_call_and_warmup_from_statistics(monkeypatch):
    import benchmark_pi05

    now = [0.0]
    calls = []
    monkeypatch.setattr(benchmark_pi05.time, "perf_counter", lambda: now[0])

    def infer():
        calls.append(None)
        now[0] += 1.0 if len(calls) == 1 else 0.01
        return np.zeros((50, 16))

    result = measure(infer, horizon=50, warmup=2, iterations=4, min_seconds=0)
    assert len(calls) == 7
    assert result["iterations"] == 4
    assert result["first_call_ms"] == pytest.approx(1000)
    assert result["mean_ms"] == pytest.approx(10)
    assert result["serial_inferences_per_s"] == pytest.approx(100)


def test_benchmark_waits_for_minimum_duration(monkeypatch):
    import benchmark_pi05

    now = [0.0]
    monkeypatch.setattr(benchmark_pi05.time, "perf_counter", lambda: now[0])

    def infer():
        now[0] += 0.1
        return np.zeros((50, 16))

    result = measure(infer, horizon=50, warmup=1, iterations=2, min_seconds=0.5)
    assert result["iterations"] >= 5
    assert result["wall_s"] >= 0.5


@pytest.mark.parametrize("actions", [np.zeros((1, 16)), np.full((50, 16), np.nan)])
def test_benchmark_rejects_invalid_model_outputs(actions):
    with pytest.raises(ValueError, match="Expected finite actions"):
        measure(lambda: actions, horizon=50, warmup=1, iterations=1, min_seconds=0)


def test_synthetic_input_matches_camera_settings_and_worker_payload(tmp_path):
    camera = tmp_path / "cameras.yaml"
    camera.write_text("height: 24\nwidth: 32\n")
    config = {"task": "fold", "camera_config": str(camera)}
    observation = read_observation(config)
    repeated = read_observation(config)
    payload = make_payload(observation, 80)
    assert payload["task"] == "fold"
    assert len(payload["state"]) == 16
    for name, encoded in payload["images"].items():
        rgb = observation[f"observation.images.{name}"]
        np.testing.assert_array_equal(rgb, repeated[f"observation.images.{name}"])
        assert rgb.shape == (24, 32, 3)
        with Image.open(io.BytesIO(decode_bytes(encoded))) as image:
            assert image.mode == "RGB"
            assert image.size == (32, 24)


def test_real_npz_input_uses_its_own_task_and_state(tmp_path):
    source = tmp_path / "observation.npz"
    fields = {f"observation.images.{name}": np.zeros((24, 32, 3), dtype=np.uint8)
              for name in ("head", "left_wrist", "right_wrist")}
    np.savez(source, **fields, **{"observation.state": np.ones(16), "prompt": "real task"})
    observation = read_observation({"task": "fallback"}, source)
    assert observation["prompt"] == "real task"
    np.testing.assert_array_equal(observation["observation.state"], np.ones(16))
