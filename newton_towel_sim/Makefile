SHELL := /bin/bash
.DEFAULT_GOAL := help
ROOT := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
SIM_ENV ?= towel-newton
VR_ENV ?= towel-vr
ARGS ?=
VR_PYTHON ?=
HOST_IP ?=
SIM = conda run --no-capture-output -n "$(SIM_ENV)" python -I -u

.PHONY: help check sim sim-fold sim-vr test test-vr-input certificate verify package
help:
	@echo "make check          Check Newton and CUDA"
	@echo "make sim            Interactive robot, towel and cameras"
	@echo "make sim-fold       Automatic single half-fold"
	@echo "make sim-vr         VR motion input, no headset video"
	@echo "make test           Scene and synthetic VR regressions (GPU required)"
	@echo "make certificate HOST_IP=192.168.1.10   Create local VR TLS files"
	@echo "make test-vr-input  Live local WebXR transport check (port 8012 must be free)"
	@echo "make verify         Verify delivery file hashes and assets"
	@echo "make package        Refresh manifest and create ../newton_towel_sim.tar.gz"
check:
	@$(SIM) "$(ROOT)/scripts/check_newton.py" $(ARGS)
sim:
	@cd "$(ROOT)" && $(SIM) "$(ROOT)/simulation/towel_scene.py" $(ARGS)
sim-fold:
	@cd "$(ROOT)" && $(SIM) "$(ROOT)/simulation/towel_scene.py" --auto-fold --num-frames 2400 $(ARGS)
sim-vr:
	@set -e; vr_python='$(VR_PYTHON)'; \
	if [[ -z "$$vr_python" ]]; then vr_python="$$(conda run -n "$(VR_ENV)" python -I -c 'import sys; print(sys.executable)')"; fi; \
	cd "$(ROOT)"; $(SIM) "$(ROOT)/simulation/towel_scene.py" --vr --vr-python "$$vr_python" $(ARGS)
test:
	@$(SIM) "$(ROOT)/scripts/check_newton.py"
	@$(SIM) "$(ROOT)/tests/test_newton_scene.py"
	@$(SIM) "$(ROOT)/tests/test_newton_vr.py"
test-vr-input:
	@conda run --no-capture-output -n "$(VR_ENV)" python -I "$(ROOT)/tests/check_sim_vr_input.py"
certificate:
	@bash "$(ROOT)/scripts/create_certificate.sh" "$(HOST_IP)"
verify:
	@python3 -I "$(ROOT)/scripts/package.py" --verify
package:
	@python3 -I "$(ROOT)/scripts/package.py"
