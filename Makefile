SHELL := /bin/bash
.ONESHELL:
.DEFAULT_GOAL := help

ROOT := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
LAUNCH_FILE := towel_experiment.launch.py
ARGS ?=
VENV ?= $(ROOT)/.venv-robot
SIM_ENV ?= towel-newton
OPENROBOTS_PREFIX ?= /opt/openrobots
WATCH_TOPIC ?= /towel/gripper_state
WATCH_ARGS ?=
RATE ?= 3
TARGET ?= both
DATASET_ROOT ?= $(ROOT)/data/raw/fold_towel
EXPORT_CONFIG ?= $(ROOT)/config/export_lerobot.yaml
MIN_JOINT_DELTA_RAD ?= 0.02
PI05_CONFIG ?= $(ROOT)/config/pi05_deployment.yaml
PI05_MODEL ?= $(HOME)/models/pi05-encos-fold-towel-93
PI05_ENV ?= pi05-openpi
PI05_CHECKPOINT ?= step-19999
PI05_HOST ?= 127.0.0.1
PI05_PORT ?= 8765
PI05_ARGS ?=
BENCH_ARGS ?=
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

.PHONY: help env check check-control check-record check-visualize build test launch visualize sim-check sim-demo sim teleop experiment teleop-watch preview camera-list servo arms robot vr vr-opencv vr-debug record record-opencv record-rviz watch watch-arm calibrate calibrate-arm-zero scan-arm-can save-arm-zero validate-data export-data filter-data pi05 pi05-bench act bridge motion replay clean-build

help:
	@echo "Towel task commands"
	@echo "  make check                  Read-only dependency check"
	@echo "  make build                  Build the ROS workspace"
	@echo "  make vr-debug               Diagnose XR controller events without publishing control commands"
	@echo "  make vr-opencv              Compatibility alias for make vr; cameras come from make record"
	@echo "  make watch                  Observe /towel/gripper_state (override WATCH_TOPIC=... )"
	@echo "  make watch-arm SIDE=both RATE=3  Print one or both arms' joint angles at a controlled rate"
	@echo "  make servo                  Start only the servo node"
	@echo "  make read-motors            Query all configured motors; no homing or motion commands"
	@echo "  make zero-motor SIDE=left JOINT=2  Set ONE arm motor's internal zero at its current pose"
	@echo "  make camera-list            List connected RealSense serial numbers"
	@echo "  make preview                Show three OpenCV camera windows without recording"
	@echo "  make record                 Recorder plus three OpenCV camera windows"
	@echo "  make record-opencv          Compatibility alias for make record"
	@echo "  make record-rviz            Recorder plus camera topics for RViz"
	@echo "  make visualize              Show the robot model in RViz with joint sliders (no hardware)"
	@echo "  make sim-check              Check Newton and CUDA in Conda environment $(SIM_ENV)"
	@echo "  make sim-demo               Run the official Newton robot-cloth example"
	@echo "  make sim                    Run the ENCOS dual-arm, gripper and towel scene"
	@echo "  make sim-fold               Run automatic corner grasp and one dual-arm half-fold"
	@echo "  make sim-vr                 Control Newton arms and grippers with existing VR input (no video)"
	@echo "  make launch ARGS=\"visualize:=true\"  Start optional RViz/TF visualization"
	@echo "  make calibrate SIDE=left ENDPOINT=open   Capture one gripper endpoint"
	@echo "  make calibrate-arm-zero SIDE=left|right|both  Start passive arm zero calibration"
	@echo "  make scan-arm-can                         Passively scan CAN0-CAN2 for motor replies"
	@echo "  make save-arm-zero SIDE=left|right|both [TARGET=joint|cartesian|both|preset]  Save zero poses or right-B preset (default: both zero files)"
	@echo "  make validate-data          Validate completed raw episodes"
	@echo "  make export-data OUTPUT=/absolute/output [EXPORT_CONFIG=...]  Export with the training environment"
	@echo "  make filter-data DATASET=/path/to/lerobot OUTPUT=/new/path MIN_JOINT_DELTA_RAD=0.02  Offline motion filtering (training environment; ARGS=--dry-run for counts only)"
	@echo "  make pi05                         Start the OpenPI PI05 worker"
	@echo "  make pi05-bench                   Benchmark PI05 inference without robot hardware"
	@echo "  make act                          Start the LeRobot ACT worker"
	@echo "  make bridge                       Start the policy bridge in dry-run mode"
	@echo "  make motion                       Start the policy bridge with motion enabled"
	@echo "  make motion PI05_SLOW_FACTOR=0.1  Start motion at 10% joint step size"
	@echo "  make motion ARGS=\"--intervention\"  Enable VR correction capture (starts paused)"
	@echo "  make vr ARGS=\"--intervention\"      VR standby; left B takes over/saves and resumes"
	@echo "  make replay ARGS=\"--episode 0\"  Replay recorded actions (dry-run by default)"
	@echo "  make replay ARGS=\"--episode 0 --max-frames 90 --rviz\"  Replay safely in RViz"
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
	@servo_binary="$$(readlink -f -- "$(SERVO_NODE)")" || exit $$?
	@if [[ ! -f "$$servo_binary" || ! -x "$$servo_binary" ]]; then echo "Servo executable not found: $(SERVO_NODE)" >&2; exit 2; fi
	@echo "Granting EtherCAT network capabilities to $$servo_binary ..."
	@sudo setcap cap_net_raw,cap_net_admin+ep "$$servo_binary" || exit $$?
	@getcap "$$servo_binary"

test:
	@$(ROBOT_ENV)
	@python3 -m pytest -q "$(ROOT)/tests"

launch:
	@$(ROBOT_ENV)
	@ros2 launch towel_bringup "$(LAUNCH_FILE)" $(ARGS)

visualize:
	@$(ROBOT_ENV)
	@ros2 launch "$(ROOT)/ros2_ws/src/towel_bringup/launch/model_viewer.launch.py" $(ARGS)

sim-check:
	@conda run --no-capture-output -n "$(SIM_ENV)" python -I -u "$(ROOT)/scripts/check_newton.py" $(ARGS)

sim-demo:
	@conda run --no-capture-output -n "$(SIM_ENV)" python -I -u -m newton.examples cloth_franka --device cuda:0 $(ARGS)

sim:
	@conda run --no-capture-output -n "$(SIM_ENV)" python -I -u "$(ROOT)/simulation/towel_scene.py" $(ARGS)

.PHONY: sim-fold
sim-fold:
	@conda run --no-capture-output -n "$(SIM_ENV)" python -I -u "$(ROOT)/simulation/towel_scene.py" --auto-fold --num-frames 2400 $(ARGS)

.PHONY: sim-vr
sim-vr:
	@conda run --no-capture-output -n "$(SIM_ENV)" python -I -u "$(ROOT)/simulation/towel_scene.py" --vr --vr-python "$(VENV)/bin/python" $(ARGS)

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

.PHONY: read-motors
read-motors:
	@$(MAKE) --no-print-directory servo ARGS="--ros-args -p feedback_only:=true"

.PHONY: zero-motor
zero-motor:
	@if [[ "$(SIDE)" != "left" && "$(SIDE)" != "right" ]]; then echo "Usage: make zero-motor SIDE={left|right} JOINT={1..7}" >&2; exit 2; fi
	@case "$(JOINT)" in
	  1) joint_name=shoulder_pitch;;
	  2) joint_name=shoulder_roll;;
	  3) joint_name=shoulder_yaw;;
	  4) joint_name=elbow;;
	  5) joint_name=wrist_roll;;
	  6) joint_name=wrist_pitch;;
	  7) joint_name=wrist_yaw;;
	  *) echo "JOINT must be exactly one integer from 1 to 7" >&2; exit 2;;
	 esac
	@$(MAKE) --no-print-directory servo ARGS="--ros-args -p motor_zero_joint:=$(SIDE)_$${joint_name}_joint"

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
	@python3 "$(ROOT)/collection/record.py" --show-opencv

record-opencv:
	@$(MAKE) --no-print-directory record

record-rviz:
	@$(ROBOT_ENV)
	@python3 "$(ROOT)/collection/record.py" --publish-rviz

vr:
	@$(ROBOT_ENV)
	@python3 "$(ROOT)/teleop/VR_controller/vr_pose_increment_publisher.py" $(ARGS)

vr-opencv:
	@echo "相机窗口由 make record 启动；vr-opencv 现在只兼容转发到 make vr" >&2
	@$(MAKE) --no-print-directory vr

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

filter-data:
	@if [[ -z "$(DATASET)" ]]; then echo "Usage: make filter-data DATASET=/path/to/lerobot OUTPUT=/new/path MIN_JOINT_DELTA_RAD=0.02 [ARGS=--dry-run] (run in the LeRobot environment)"; exit 2; fi
	@python3 "$(ROOT)/collection/filter_lerobot.py" --dataset "$(DATASET)" --min-joint-delta-rad "$(MIN_JOINT_DELTA_RAD)" $(if $(OUTPUT),--output "$(OUTPUT)") $(ARGS)

pi05:
	set -e
	@if ! command -v conda >/dev/null 2>&1; then echo "conda command not found; initialize Conda before running make pi05" >&2; exit 2; fi
	@if [[ ! -d "$(PI05_MODEL)/$(PI05_CHECKPOINT)/params" ]]; then echo "OpenPI checkpoint not found: $(PI05_MODEL)/$(PI05_CHECKPOINT)/params" >&2; exit 2; fi
	cd "$(ROOT)"
	exec python3 scripts/run_policy_worker.py --env "$(PI05_ENV)" -- --config "$(PI05_CONFIG)" --model "$(PI05_MODEL)" --checkpoint "$(PI05_CHECKPOINT)" --host "$(PI05_HOST)" --port "$(PI05_PORT)" $(PI05_ARGS)

pi05-bench:
	set -e
	cd "$(ROOT)"
	exec python3 scripts/run_policy_worker.py --env "$(PI05_ENV)" --benchmark -- --config "$(PI05_CONFIG)" --model "$(PI05_MODEL)" --checkpoint "$(PI05_CHECKPOINT)" $(BENCH_ARGS)

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
