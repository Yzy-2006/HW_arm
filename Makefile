SHELL := /bin/bash
.ONESHELL:
.DEFAULT_GOAL := help

ROOT := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
LAUNCH_FILE := towel_experiment.launch.py
ARGS ?=
VENV ?= $(ROOT)/.venv-robot
OPENROBOTS_PREFIX ?= /opt/openrobots
WATCH_TOPIC ?= /towel/gripper_state
WATCH_ARGS ?=
RATE ?= 3
TARGET ?= both
DATASET_ROOT ?= $(ROOT)/data/raw/fold_towel
EXPORT_CONFIG ?= $(ROOT)/config/export_lerobot.yaml
PI05_CONFIG ?= $(ROOT)/config/pi05_deployment.yaml
PI05_MODEL ?= $(HOME)/models/pi05-encos-fold-towel-93
PI05_ENV ?= pi05-openpi
PI05_CHECKPOINT ?= step-19999
PI05_HOST ?= 127.0.0.1
PI05_PORT ?= 8765
PI05_ARGS ?=
ACT_CHECKPOINT ?= $(ROOT)/outputs/act_fold_towel_b12/checkpoints/100000/pretrained_model
ACT_ENV ?= lerobot
ACT_HOST ?= 127.0.0.1
ACT_PORT ?= 8765
ACT_ARGS ?=
PI05_SLOW_FACTOR ?=
SERVO_NODE := $(ROOT)/ros2_ws/install/servo_control_hw/lib/servo_control_hw/servo_node

# Every robot-side recipe gets the same Python/ROS/OpenRobots environment.
# Override, for example: make check OPENROBOTS_PREFIX=/custom/openrobots
define ROBOT_ENV
if [[ ! -f "$(VENV)/bin/activate" ]]; then
  echo "Robot venv not found: $(VENV). Create it as documented in environments/README.md." >&2
  exit 2
fi
source "$(VENV)/bin/activate"
export TOWEL_OPENROBOTS_PREFIX="$(OPENROBOTS_PREFIX)"
source "$(ROOT)/scripts/env.sh"
endef

.PHONY: help env check check-control check-record check-visualize build test launch teleop experiment teleop-watch preview camera-list servo arms robot vr vr-debug record record-opencv record-rviz watch watch-arm calibrate calibrate-arm-zero scan-arm-can save-arm-zero validate-data export-data pi05 act bridge motion replay clean-build

help:
	@echo "Towel task commands"
	@echo "  make check                  Read-only dependency check"
	@echo "  make build                  Build the ROS workspace"
	@echo "  make vr-debug               Diagnose XR controller events without publishing control commands"
	@echo "  make watch                  Observe /towel/gripper_state (override WATCH_TOPIC=... )"
	@echo "  make watch-arm SIDE=both RATE=3  Print one or both arms' joint angles at a controlled rate"
	@echo "  make servo                  Start only the servo node"
	@echo "  make camera-list            List connected RealSense serial numbers"
	@echo "  make preview                Show three OpenCV camera windows without recording"
	@echo "  make record                 Start only the passive recorder"
	@echo "  make record-opencv          Recorder plus three OpenCV camera windows"
	@echo "  make record-rviz            Recorder plus camera topics for RViz"
	@echo "  make launch ARGS=\"visualize:=true\"  Start optional RViz/TF visualization"
	@echo "  make calibrate SIDE=left ENDPOINT=open   Capture one gripper endpoint"
	@echo "  make calibrate-arm-zero SIDE=left|right|both  Start passive arm zero calibration"
	@echo "  make scan-arm-can                         Passively scan CAN0-CAN2 for motor replies"
	@echo "  make save-arm-zero SIDE=left|right|both [TARGET=joint|cartesian|both|preset]  Save zero poses or right-B preset (default: both zero files)"
	@echo "  make validate-data          Validate completed raw episodes"
	@echo "  make export-data OUTPUT=/absolute/output [EXPORT_CONFIG=...]  Export with the training environment"
	@echo "  make pi05                         Start the OpenPI PI05 worker"
	@echo "  make act                          Start the LeRobot ACT worker"
	@echo "  make bridge                       Start the policy bridge in dry-run mode"
	@echo "  make motion                       Start the policy bridge with motion enabled"
	@echo "  make motion PI05_SLOW_FACTOR=0.1  Start motion at 10% joint step size"
	@echo "  make replay ARGS=\"--episode 0\"  Replay recorded actions (dry-run by default)"
	@echo "  Defaults: VENV=$(VENV), OPENROBOTS_PREFIX=$(OPENROBOTS_PREFIX), DATASET_ROOT=$(DATASET_ROOT), EXPORT_CONFIG=$(EXPORT_CONFIG)"

env:
	@$(ROBOT_ENV)
	@printf 'VIRTUAL_ENV=%s\nTOWEL_ROOT=%s\nTOWEL_OPENROBOTS_PREFIX=%s\n' "$$VIRTUAL_ENV" "$$TOWEL_ROOT" "$$TOWEL_OPENROBOTS_PREFIX"

check:
	@$(ROBOT_ENV)
	@python3 "$(ROOT)/scripts/preflight.py"

check-control:
	@$(ROBOT_ENV)
	@python3 "$(ROOT)/scripts/preflight.py" --scope control

check-record:
	@$(ROBOT_ENV)
	@python3 "$(ROOT)/scripts/preflight.py" --scope record

check-visualize:
	@$(ROBOT_ENV)
	@python3 "$(ROOT)/scripts/preflight.py" --scope visualize

build:
	@$(ROBOT_ENV)
	@bash "$(ROOT)/scripts/build.sh" || exit $$?
	@echo "Granting EtherCAT network capabilities to $(SERVO_NODE) ..."
	@sudo setcap cap_net_raw,cap_net_admin+ep "$(SERVO_NODE)" || exit $$?
	@getcap "$(SERVO_NODE)"

test:
	@$(ROBOT_ENV)
	@python3 -m pytest -q "$(ROOT)/tests"

launch:
	@$(ROBOT_ENV)
	@ros2 launch towel_bringup "$(LAUNCH_FILE)" $(ARGS)

# Keep the removed one-command names as explicit failures: `teleop/` is a
# source directory, so omitting this recipe would otherwise make `make teleop`
# exit successfully without starting anything.
teleop experiment:
	@echo "make $@ has been removed. Start each process separately: servo -> arms -> robot -> record -> vr." >&2
	@exit 2

watch:
	@$(ROBOT_ENV)
	@ros2 topic echo "$(WATCH_TOPIC)" $(WATCH_ARGS)

watch-arm:
	@if [[ "$(SIDE)" != "left" && "$(SIDE)" != "right" && "$(SIDE)" != "both" ]]; then echo "Usage: make watch-arm SIDE={left|right|both} [RATE=3]"; exit 2; fi
	@$(ROBOT_ENV)
	@python3 "$(ROOT)/scripts/watch_arm_feedback.py" --side "$(SIDE)" --rate "$(RATE)"

# Backward-compatible alias for the old VR-specific watcher name.
teleop-watch: watch

servo:
	@$(ROBOT_ENV)
	@bash "$(ROOT)/scripts/run.sh" servo $(ARGS)

calibrate-arm-zero:
	@if [[ "$(SIDE)" != "left" && "$(SIDE)" != "right" && "$(SIDE)" != "both" ]]; then echo "Usage: make calibrate-arm-zero SIDE={left|right|both}"; exit 2; fi
	@$(MAKE) --no-print-directory servo ARGS="--ros-args -p arm_zero_calibration_side:=$(SIDE)"

scan-arm-can:
	@$(MAKE) --no-print-directory servo ARGS="--ros-args -p can_channel_scan:=true"

save-arm-zero:
	@if [[ "$(SIDE)" != "left" && "$(SIDE)" != "right" && "$(SIDE)" != "both" ]]; then echo "Usage: make save-arm-zero SIDE={left|right|both}"; exit 2; fi
	@if [[ "$(TARGET)" != "joint" && "$(TARGET)" != "cartesian" && "$(TARGET)" != "both" && "$(TARGET)" != "preset" ]]; then echo "TARGET must be joint, cartesian, both, or preset"; exit 2; fi
	@$(ROBOT_ENV)
	@python3 "$(ROOT)/scripts/calibrate_arm_zero.py" --side "$(SIDE)" --target "$(TARGET)"

arms:
	@$(ROBOT_ENV)
	@bash "$(ROOT)/scripts/run.sh" arms

robot:
	@$(ROBOT_ENV)
	@bash "$(ROOT)/scripts/run.sh" robot

preview:
	@$(ROBOT_ENV)
	@python3 "$(ROOT)/collection/record.py" --preview-opencv

camera-list:
	@$(ROBOT_ENV)
	@python3 "$(ROOT)/scripts/list_realsense_devices.py"

record:
	@$(ROBOT_ENV)
	@python3 "$(ROOT)/collection/record.py"

record-opencv:
	@$(ROBOT_ENV)
	@python3 "$(ROOT)/collection/record.py" --show-opencv

record-rviz:
	@$(ROBOT_ENV)
	@python3 "$(ROOT)/collection/record.py" --publish-rviz

vr:
	@$(ROBOT_ENV)
	@python3 "$(ROOT)/teleop/VR_controller/vr_pose_increment_publisher.py"

vr-debug:
	@$(ROBOT_ENV)
	@python3 "$(ROOT)/teleop/VR_controller/vr_pose_increment_publisher.py" --debug-controller-input

calibrate:
	@if [[ -z "$(SIDE)" || -z "$(ENDPOINT)" ]]; then echo "Usage: make calibrate SIDE={left|right} ENDPOINT={open|closed}"; exit 2; fi
	@$(ROBOT_ENV)
	@python3 "$(ROOT)/scripts/calibrate_grippers.py" --side "$(SIDE)" --endpoint "$(ENDPOINT)"

validate-data:
	@python3 "$(ROOT)/collection/export_lerobot.py" --root "$(DATASET_ROOT)" --config "$(EXPORT_CONFIG)" --validate-only

export-data:
	@if [[ -z "$(OUTPUT)" ]]; then echo "Usage: make export-data OUTPUT=/absolute/path (run in towel-pi05 environment)"; exit 2; fi
	@python3 "$(ROOT)/collection/export_lerobot.py" --root "$(DATASET_ROOT)" --output "$(OUTPUT)" --repo-id local/fold_towel --config "$(EXPORT_CONFIG)"

pi05:
	set -e
	@if ! command -v conda >/dev/null 2>&1; then echo "conda command not found; initialize Conda before running make pi05" >&2; exit 2; fi
	@if [[ ! -d "$(PI05_MODEL)/$(PI05_CHECKPOINT)/params" ]]; then echo "OpenPI checkpoint not found: $(PI05_MODEL)/$(PI05_CHECKPOINT)/params" >&2; exit 2; fi
	cd "$(ROOT)"
	exec python3 scripts/run_policy_worker.py --env "$(PI05_ENV)" -- --config "$(PI05_CONFIG)" --model "$(PI05_MODEL)" --checkpoint "$(PI05_CHECKPOINT)" --host "$(PI05_HOST)" --port "$(PI05_PORT)" $(PI05_ARGS)

bridge:
	@$(ROBOT_ENV)
	@python3 "$(ROOT)/deployment/pi05_bridge.py" --config "$(PI05_CONFIG)" $(ARGS)

motion:
	@if [[ -n "$(PI05_SLOW_FACTOR)" && ( "$(PI05_SLOW_FACTOR)" == "0" || "$(PI05_SLOW_FACTOR)" == -* ) ]]; then echo "PI05_SLOW_FACTOR must be greater than 0" >&2; exit 2; fi
	@$(ROBOT_ENV)
	@python3 "$(ROOT)/deployment/pi05_bridge.py" --config "$(PI05_CONFIG)" --motion $(if $(PI05_SLOW_FACTOR),--slow-factor $(PI05_SLOW_FACTOR),) $(ARGS)

replay:
	@$(ROBOT_ENV)
	@python3 "$(ROOT)/deployment/replay_dataset.py" $(ARGS)

act:
	set -e
	@if ! command -v conda >/dev/null 2>&1; then echo "conda command not found; initialize Conda before running make act" >&2; exit 2; fi
	@if [[ ! -f "$(ACT_CHECKPOINT)/config.json" ]]; then echo "ACT checkpoint not found: $(ACT_CHECKPOINT)" >&2; exit 2; fi
	cd "$(ROOT)"
	exec python3 scripts/run_policy_worker.py --env "$(ACT_ENV)" -- --model "$(ACT_CHECKPOINT)" --host "$(ACT_HOST)" --port "$(ACT_PORT)" $(ACT_ARGS)

clean-build:
	@echo "Build artifacts are not deleted automatically. Remove only the specific ros2_ws/build, install and log directories when intentional."
