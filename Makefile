SHELL := /bin/bash
.ONESHELL:
.DEFAULT_GOAL := help

ROOT := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
LAUNCH_FILE := towel_experiment.launch.py
VISUALIZE ?= false
ARGS ?=
VENV ?= $(ROOT)/.venv-robot
OPENROBOTS_PREFIX ?= /opt/openrobots
WATCH_TOPIC ?= /towel/gripper_state
WATCH_ARGS ?=
RATE ?= 3
DATASET_ROOT ?= $(ROOT)/data/raw/fold_towel
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

.PHONY: help env check check-control check-record check-visualize build test launch experiment teleop teleop-watch preview camera-list servo arms robot force vr record record-rviz watch watch-arm calibrate calibrate-arm-zero save-arm-zero tare-force validate-data export-data clean-build

help:
	@echo "Towel task commands"
	@echo "  make check                  Read-only dependency check"
	@echo "  make build                  Build the ROS workspace"
	@echo "  make teleop                 Check and start servo, force, arms, robot and VR"
	@echo "  make teleop VISUALIZE=true  Start the same VR teleoperation with RViz"
	@echo "  make watch                  Observe /towel/gripper_state (override WATCH_TOPIC=... )"
	@echo "  make watch-arm SIDE=both RATE=3  Print one or both arms' joint angles at a controlled rate"
	@echo "  make servo|force            Start only the servo or force-sensor node"
	@echo "  make tare-force SIDE=left   Tare one running force sensor (left or right)"
	@echo "  make experiment             Start servo, force, arms, robot, recorder and VR"
	@echo "  make experiment VISUALIZE=true  Start the same experiment with RViz"
	@echo "  make camera-list            List connected RealSense serial numbers"
	@echo "  make preview                Start recorder and RViz without robot control"
	@echo "  make record                 Start only the passive recorder"
	@echo "  make record-rviz            Recorder plus camera topics for RViz"
	@echo "  make launch ARGS='...'      Start selected processes through launch arguments"
	@echo "  make calibrate SIDE=left ENDPOINT=open   Capture one gripper endpoint"
	@echo "  make calibrate-arm-zero SIDE=both        Start passive dual-arm zero calibration"
	@echo "  make save-arm-zero SIDE=both             Save stable dual-arm zero poses"
	@echo "  make validate-data          Validate completed raw episodes"
	@echo "  make export-data OUTPUT=/absolute/output  Export with the training environment"
	@echo "  Defaults: VENV=$(VENV), OPENROBOTS_PREFIX=$(OPENROBOTS_PREFIX), DATASET_ROOT=$(DATASET_ROOT)"

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

experiment:
	@$(MAKE) --no-print-directory launch ARGS="start_servo:=true start_force:=true start_arms:=true start_robot:=true start_record:=true start_vr:=true visualize:=$(VISUALIZE)"

# Single-command empty-workspace VR check: omit recorder and cameras, but
# retain the same safe launch ordering as the full experiment.
teleop: check-control
	@$(MAKE) --no-print-directory launch ARGS="start_servo:=true start_force:=true start_arms:=true start_robot:=true start_vr:=true visualize:=$(VISUALIZE)"

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
	@if [[ "$(SIDE)" != "left" && "$(SIDE)" != "both" ]]; then echo "Usage: make calibrate-arm-zero SIDE={left|both}"; exit 2; fi
	@$(MAKE) --no-print-directory servo ARGS="--ros-args -p arm_zero_calibration_side:=$(SIDE)"

save-arm-zero:
	@if [[ "$(SIDE)" != "left" && "$(SIDE)" != "both" ]]; then echo "Usage: make save-arm-zero SIDE={left|both}"; exit 2; fi
	@$(ROBOT_ENV)
	@python3 "$(ROOT)/scripts/calibrate_arm_zero.py" --side "$(SIDE)"

arms:
	@$(ROBOT_ENV)
	@bash "$(ROOT)/scripts/run.sh" arms

robot:
	@$(ROBOT_ENV)
	@bash "$(ROOT)/scripts/run.sh" robot

force:
	@$(ROBOT_ENV)
	@bash "$(ROOT)/scripts/run.sh" force

preview:
	@$(MAKE) --no-print-directory launch ARGS="start_record:=true visualize:=true"

camera-list:
	@$(ROBOT_ENV)
	@python3 "$(ROOT)/scripts/list_realsense_devices.py"

record:
	@$(ROBOT_ENV)
	@python3 "$(ROOT)/collection/record.py"

record-rviz:
	@$(ROBOT_ENV)
	@python3 "$(ROOT)/collection/record.py" --publish-rviz

vr:
	@$(ROBOT_ENV)
	@python3 "$(ROOT)/teleop/VR_controller/vr_pose_increment_publisher.py"

calibrate:
	@if [[ -z "$(SIDE)" || -z "$(ENDPOINT)" ]]; then echo "Usage: make calibrate SIDE={left|right} ENDPOINT={open|closed}"; exit 2; fi
	@$(ROBOT_ENV)
	@python3 "$(ROOT)/scripts/calibrate_grippers.py" --side "$(SIDE)" --endpoint "$(ENDPOINT)"

tare-force:
	@if [[ "$(SIDE)" != "left" && "$(SIDE)" != "right" ]]; then echo "Usage: make tare-force SIDE={left|right}"; exit 2; fi
	@$(ROBOT_ENV)
	@ros2 service call "/force_sensor/$(SIDE)/tare" std_srvs/srv/Trigger '{}'

validate-data:
	@python3 "$(ROOT)/collection/export_lerobot.py" --root "$(DATASET_ROOT)" --validate-only

export-data:
	@if [[ -z "$(OUTPUT)" ]]; then echo "Usage: make export-data OUTPUT=/absolute/path (run in towel-pi05 environment)"; exit 2; fi
	@python3 "$(ROOT)/collection/export_lerobot.py" --root "$(DATASET_ROOT)" --output "$(OUTPUT)" --repo-id local/fold_towel

clean-build:
	@echo "Build artifacts are not deleted automatically. Remove only the specific ros2_ws/build, install and log directories when intentional."
