from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_make_can_scan_starts_dedicated_scan_mode():
    makefile = (ROOT / "Makefile").read_text()

    assert "scan-arm-can:" in makefile
    assert 'can_channel_scan:=true' in makefile
    assert '$(MAKE) --no-print-directory servo ARGS=' in makefile


def test_can_scan_covers_every_pdo_channel_with_zero_current_only():
    source = (
        ROOT / "ros2_ws/src/servo_control_hw/src/servo_control/transmit.cpp"
    ).read_text()
    header = ROOT / "ros2_ws/src/servo_control_hw/include/servo_control/transmit.h"

    assert "CanChannelScanBegin" in header.read_text()
    assert "CanChannelScanStep" in header.read_text()
    assert "CanChannelScanPrintResult" in header.read_text()
    assert "for (int channel = 0; channel < CAN_CHANNELS; ++channel)" in source
    assert "set_motor_cur_tor(&scan_tx[channel][slot], motor_id, 0, 0, 1)" in source
    assert "[CAN scan] CAN%d:" in source
    assert "ID%d=%u" in source


def test_can_scan_mode_cannot_fall_through_to_normal_control_or_homing():
    source = (
        ROOT / "ros2_ws/src/servo_control_hw/src/ros_controller/rosLoop.cpp"
    ).read_text()

    assert '"can_channel_scan"' in source
    assert "CAN scan: zero-current" in source
    assert "CanChannelScanBegin();" in source
    assert "CanChannelScanStep();" in source
    assert "CanChannelScanPrintResult();" in source


def test_calibration_document_explains_the_passive_can_scan():
    document = (ROOT / "docs/ARM_ZERO_CALIBRATION.md").read_text()

    assert "make scan-arm-can" in document
    assert "CAN0–CAN2" in document
    assert "零电流" in document
    assert "不保存零位" in document
