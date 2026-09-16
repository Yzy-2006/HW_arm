SHELL := /bin/bash
.ONESHELL:
.DEFAULT_GOAL := help

ROOT := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
LAUNCH_FILE := towel_experiment.launch.py
VISUALIZE ?= false
ARGS ?=
VENV ?= $(ROOT)/.venv-robot
OPENROBOTS_PREFIX ?= /opt/openrobots

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

.PHONY: help env check check-control check-record check-visualize build test launch experiment preview record record-rviz vr calibrate validate-data export-data clean-build

help:
	@echo "Towel task commands"
	@echo "  make check                  Read-only dependency check"
	@echo "  make build                  Build the ROS workspace"
	@echo "  make experiment             Start servo, force, arms, robot, recorder and VR"
	@echo "  make experiment VISUALIZE=true  Start the same experiment with RViz"
	@echo "  make preview                Start recorder and RViz without robot control"
	@echo "  make record                 Start only the passive recorder"
	@echo "  make record-rviz            Recorder plus camera topics for RViz"
	@echo "  make launch ARGS='...'      Start selected processes through launch arguments"
	@echo "  make calibrate SIDE=left ENDPOINT=open   Capture one gripper endpoint"
	@echo "  make validate-data          Validate completed raw episodes"
	@echo "  make export-data OUTPUT=/absolute/output  Export with the training environment"
	@echo "  Defaults: VENV=$(VENV), OPENROBOTS_PREFIX=$(OPENROBOTS_PREFIX)"

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
	@bash "$(ROOT)/scripts/build.sh"

test:
	@$(ROBOT_ENV)
	@python3 -m pytest -q "$(ROOT)/tests"

launch:
	@$(ROBOT_ENV)
	@ros2 launch towel_bringup "$(LAUNCH_FILE)" $(ARGS)

experiment:
	@$(MAKE) --no-print-directory launch ARGS="start_servo:=true start_force:=true start_arms:=true start_robot:=true start_record:=true start_vr:=true visualize:=$(VISUALIZE)"

preview:
	@$(MAKE) --no-print-directory launch ARGS="start_record:=true visualize:=true"

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

validate-data:
	@python3 "$(ROOT)/collection/export_lerobot.py" --root "$(ROOT)/data/raw/towel_wipe" --validate-only

export-data:
	@if [[ -z "$(OUTPUT)" ]]; then echo "Usage: make export-data OUTPUT=/absolute/path (run in towel-pi05 environment)"; exit 2; fi
	@python3 "$(ROOT)/collection/export_lerobot.py" --root "$(ROOT)/data/raw/towel_wipe" --output "$(OUTPUT)" --repo-id local/towel_wipe

clean-build:
	@echo "Build artifacts are not deleted automatically. Remove only the specific ros2_ws/build, install and log directories when intentional."
