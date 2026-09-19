import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts/watch_arm_feedback.py"


def load_module():
    assert SCRIPT.is_file(), "Dedicated arm-feedback watcher has not been created"
    spec = importlib.util.spec_from_file_location("watch_arm_feedback_test", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_watcher_formats_only_seven_joint_angles():
    module = load_module()

    rendered = module.format_joint_feedback(
        "left", (0.0, 0.1, -0.2, 0.3, -0.4, 0.5, -0.6)
    )

    assert rendered == "left: +0.0000 +0.1000 -0.2000 +0.3000 -0.4000 +0.5000 -0.6000 rad"


def test_watcher_throttles_to_requested_rate():
    module = load_module()
    throttle = module.FeedbackThrottle(rate_hz=3.0)

    assert throttle.ready(1.0)
    throttle.record(1.0)
    assert not throttle.ready(1.2)
    assert throttle.ready(1.334)


def test_watcher_formats_both_arms_on_one_feedback_line():
    module = load_module()

    rendered = module.format_both_arm_feedback(
        (0.0, 0.1, -0.2, 0.3, -0.4, 0.5, -0.6),
        (0.6, -0.5, 0.4, -0.3, 0.2, -0.1, 0.0),
    )

    assert rendered == (
        "left: +0.0000 +0.1000 -0.2000 +0.3000 -0.4000 +0.5000 -0.6000 rad"
        " | right: +0.6000 -0.5000 +0.4000 -0.3000 +0.2000 -0.1000 +0.0000 rad"
    )


def test_make_target_passes_side_and_rate_to_watcher():
    makefile = (ROOT / "Makefile").read_text()

    assert "watch-arm:" in makefile
    assert "Usage: make watch-arm SIDE={left|right|both} [RATE=3]" in makefile
    assert 'watch_arm_feedback.py" --side "$(SIDE)" --rate "$(RATE)"' in makefile
