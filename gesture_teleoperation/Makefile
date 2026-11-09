SHELL := /bin/bash
.DEFAULT_GOAL := help

PYTHON ?= $(HOME)/anaconda3/envs/mediapipe/bin/python
ROS_PYTHON ?= /usr/bin/python3
ROS_SETUP ?= /opt/ros/jazzy/setup.bash
DELEGATE ?= gpu
OK_DISTANCE ?= 0.42
PINCH_DISTANCE ?= 1.0
PORT ?= 9870
DURATION ?= 10
FRAMES ?= 90
CALIBRATION ?= calibration/hand_geometry.json
CALIBRATION_SAMPLES ?= 60
ROS_TEST_DOMAIN ?= 83
ARGS ?=

TRACKING_ARGS = --delegate $(DELEGATE) --ok-distance $(OK_DISTANCE) --pinch-distance $(PINCH_DISTANCE)
LAUNCH = "$(PYTHON)" scripts/run_hand_tracking.py --python "$(PYTHON)" --ros-setup "$(ROS_SETUP)" --port $(PORT)

.PHONY: help run debug preview headless capture-check calibrate rgb rgb-preview compare check check-gpu check-camera check-deps test test-ros

help:
	@printf '%s\n' \
	  'make run             OpenCV tracking/status windows + ROS bridge + RViz' \
	  'make debug           Full tracking with fingertip distance logs' \
	  'make preview         OpenCV windows only; no ROS bridge or RViz' \
	  'make headless        ROS bridge + tracking, no windows (DURATION=10)' \
	  'make capture-check   Bounded tracking without windows or ROS (FRAMES=90)' \
	  'make calibrate       Collect RGB-D geometry for both palms and save calibration' \
	  'make rgb             Color-only calibrated PnP + OpenCV + ROS + RViz' \
	  'make rgb-preview     Color-only calibrated PnP with OpenCV windows only' \
	  'make compare         Same-frame RGB-D vs RGB/PnP in one RViz window' \
	  'make check           Check dependencies and CPU inference' \
	  'make check-gpu       Check GPU inference' \
	  'make check-camera    Check CPU inference and D435i RGB-D capture' \
	  'make check-deps      Check installed package compatibility' \
	  'make test            Run geometry, facing, gesture and protocol tests' \
	  'make test-ros        Run ROS bridge tests in a separate ROS domain' \
	  '' \
	  'Defaults: DELEGATE=gpu OK_DISTANCE=0.42 PINCH_DISTANCE=1.0 PORT=9870' \
	  'Examples: make run PINCH_DISTANCE=1.25' \
	  '          make run DELEGATE=cpu' \
	  '          make run ARGS="--no-mirror --no-show-indices"' \
	  '          make run ARGS="--no-rviz"' \
	  '          make calibrate ARGS="--calibration-hands Right"' \
	  'RGB targets use CALIBRATION=calibration/hand_geometry.json (run make calibrate first).' \
	  'Override PYTHON=/path/to/conda/env/bin/python or ROS_SETUP=/path/to/setup.bash as needed.' \
	  'Run only one camera target at a time. Close a window or press Ctrl+C to stop.'

run:
	$(LAUNCH) $(TRACKING_ARGS) $(ARGS)

debug:
	$(LAUNCH) $(TRACKING_ARGS) --gesture-debug $(ARGS)

preview:
	"$(PYTHON)" scripts/track_hands.py --udp-port 0 $(TRACKING_ARGS) $(ARGS)

headless:
	$(LAUNCH) $(TRACKING_ARGS) --no-rviz --no-display --duration $(DURATION) $(ARGS)

capture-check:
	"$(PYTHON)" scripts/track_hands.py --udp-port 0 $(TRACKING_ARGS) --no-display --max-frames $(FRAMES) $(ARGS)

calibrate:
	"$(PYTHON)" scripts/track_hands.py --udp-port 0 $(TRACKING_ARGS) --mode rgbd --calibrate --calibration "$(CALIBRATION)" --calibration-samples $(CALIBRATION_SAMPLES) $(ARGS)

rgb:
	$(LAUNCH) $(TRACKING_ARGS) --mode rgb --calibration "$(CALIBRATION)" $(ARGS)

rgb-preview:
	"$(PYTHON)" scripts/track_hands.py --udp-port 0 $(TRACKING_ARGS) --mode rgb --calibration "$(CALIBRATION)" $(ARGS)

compare:
	$(LAUNCH) $(TRACKING_ARGS) --mode compare --calibration "$(CALIBRATION)" $(ARGS)

check:
	"$(PYTHON)" scripts/check_environment.py $(ARGS)

check-gpu:
	"$(PYTHON)" scripts/check_environment.py --delegate gpu $(ARGS)

check-camera:
	"$(PYTHON)" scripts/check_environment.py --camera $(ARGS)

check-deps:
	"$(PYTHON)" -m pip check

test:
	"$(PYTHON)" -m unittest discover -s tests -v

test-ros:
	source "$(ROS_SETUP)" && ROS_DOMAIN_ID=$(ROS_TEST_DOMAIN) "$(ROS_PYTHON)" -m unittest discover -s tests/ros -v
