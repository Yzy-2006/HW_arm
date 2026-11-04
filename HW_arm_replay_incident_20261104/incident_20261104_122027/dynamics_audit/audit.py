"""Isolated numerical audit: native filters + Pinocchio, no ROS or hardware IO.

The open-loop cases use actual recorded targets, ideal 15 Hz arrival, a steady
warm-up, production middle-layer formulas, and no physical plant/feedback loop.
They demonstrate failure mechanisms; they are not an exact accident replay.
"""
import csv
import ctypes as C
import hashlib
import json
import math
import os
from pathlib import Path

import numpy as np
import pinocchio as pin
import yaml

os.environ.setdefault("MPLCONFIGDIR", "/tmp/hw-arm-incident-matplotlib")
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
INCIDENT = HERE.parent
PROJECT = next(p for p in HERE.parents if (p / "deployment/replay_dataset.py").is_file())
CONFIG = yaml.safe_load((INCIDENT / "evidence/ros2_ws/install/ctl_msgs_hw/share/ctl_msgs_hw/config/controllerConfig.yaml").read_text())
GRAV = CONFIG["gravComp"]
FILTERS = CONFIG["gravCompFtr"]
with (INCIDENT / "evidence/outputs/replay_logs/episode_3_start_0_20261104_122027.csv").open() as f:
    ROWS = list(csv.DictReader(f))
LABELS = [f"{side}_joint_{i}" for side in ["left", "right"] for i in range(1, 8)]
TARGETS = np.array([[float(r["dataset_" + label]) for label in LABELS] for r in ROWS])

LIB = C.CDLL(str(HERE / "filter_native.so"))
LIB.filter_create.argtypes = [C.c_double] * 3
LIB.filter_create.restype = C.c_void_p
LIB.filter_step.argtypes = [C.c_void_p, C.c_double, C.c_int]
LIB.filter_step.restype = C.c_double
LIB.filter_destroy.argtypes = [C.c_void_p]


class NativeFilters:
    def __init__(self, older=False):
        self.older = older
        self.handles = [LIB.filter_create(0., q, r) for q, r in zip(GRAV["accFtr"]["Qgain"], GRAV["accFtr"]["Rgain"])]

    def step(self, values):
        return np.array([LIB.filter_step(p, float(x), self.older) for p, x in zip(self.handles, values)])

    def close(self):
        for p in self.handles:
            LIB.filter_destroy(p)


def coefficients(section, actual_rate, corrected=False):
    # Current code assigns alpha=1-exp(-2*pi*fc/Fs) to the OLD output.
    result = []
    for side in ["lArm", "rArm"]:
        rates, cutoffs = section[side]["sampRate"], section[side]["cutoffRate"]
        result.extend(math.exp(-2 * math.pi * fc / actual_rate) if corrected
                      else 1 - math.exp(-2 * math.pi * fc / fs)
                      for fs, fc in zip(rates, cutoffs))
    return np.array(result)


MODEL = pin.buildModelFromUrdf(str(PROJECT / "ros2_ws/install/dual_arm_hw/share/dual_arm_hw/urdf/dual_arm.urdf"), pin.JointModelFreeFlyer())
assert MODEL.nq == 21 and MODEL.nv == 20
assert list(MODEL.names)[2:] == [f"arm_{s}_joint_{i}" for s in ["l", "r"] for i in range(1, 8)]


def simulate(label, rate, older=False, corrected_lowpass=False, warmup_s=65.2):
    dt = 1.0 / rate
    acc_filters = NativeFilters(older)
    data = MODEL.createData()
    q = pin.neutral(MODEL)
    q[7:] = TARGETS[0]
    v = np.zeros(MODEL.nv)
    q_prev = TARGETS[0].copy()
    q_out, q_in = q_prev.copy(), q_prev.copy()
    v_out, v_in, v_previous = np.zeros((3, 14))
    grav_out, inertia_out = np.zeros((2, 14))
    qp = coefficients(FILTERS["posOutpFtr"], rate, corrected_lowpass)
    vp = coefficients(FILTERS["velOutpFtr"], rate, corrected_lowpass)
    qi = coefficients(FILTERS["posInpFtr"], rate, corrected_lowpass)
    vi = coefficients(FILTERS["velInpFtr"], rate, corrected_lowpass)
    tp = coefficients(FILTERS["torOutpFtr"], rate, corrected_lowpass)
    warm_ticks = round(warmup_s * rate)
    # Stationary warm-up updates the real filter covariance, with zero input.
    for _ in range(warm_ticks):
        acc_filters.step(np.zeros(14))
    records = []
    for tick in range(math.ceil(20 * rate)):
        t = tick / rate
        target = TARGETS[min(int(t * 15 + 1e-9), len(TARGETS) - 1)]
        # runinMod=1, upper_monitor_time=middle dt: one-tick linear target.
        raw_velocity = (target - q_prev) / dt
        q_prev = target.copy()
        q_out = qp * q_out + (1 - qp) * target
        v_out = vp * v_out + (1 - vp) * raw_velocity
        q_in = qi * q_in + (1 - qi) * q_out
        v_in = vi * v_in + (1 - vi) * v_out
        raw_acceleration = (v_in - v_previous) / dt
        v_previous = v_in.copy()
        acceleration = acc_filters.step(raw_acceleration)
        q[7:] = q_in
        v[6:] = v_in
        gravity = pin.computeGeneralizedGravity(MODEL, data, q)[6:] / GRAV["GravCompInte"]
        coriolis = (pin.computeCoriolisMatrix(MODEL, data, q, v) @ v)[6:] / GRAV["CoriousCompInte"]
        mass = pin.crba(MODEL, data, q)
        # Pinocchio Python returns a symmetric CRBA matrix; verify on first use.
        if tick == 0:
            assert np.max(np.abs(mass - mass.T)) < 1e-10
        inertia = (mass[6:, 6:] @ acceleration) / GRAV["InteriaCompInte"] + coriolis
        grav_out = tp * grav_out + (1 - tp) * gravity
        inertia_out = tp * inertia_out + (1 - tp) * inertia
        records.append([t, warmup_s + t, *(
            float(np.max(np.abs(x))) for x in [raw_velocity, v_in, raw_acceleration, acceleration, grav_out, inertia_out, grav_out + inertia_out]
        ), float(acceleration[0]), float(inertia_out[0]), float(grav_out[0])])
    acc_filters.close()
    values = np.array(records)
    fields = ["replay_s", "filter_runtime_s", "max_raw_velocity", "max_dyn_velocity", "max_raw_acceleration",
              "max_filtered_acceleration", "max_gravity", "max_inertia", "max_total_feedforward",
              "left_j1_acceleration", "left_j1_inertia", "left_j1_gravity"]
    with (HERE / f"open_loop_{label}.csv").open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(fields)
        writer.writerows(values)
    summary = {"rate_hz": rate, "warmup_s": warmup_s, "older_filter_method": older,
               "corrected_lowpass_in_experiment_only": corrected_lowpass}
    summary.update({key: float(np.max(values[:, i])) for i, key in enumerate(fields) if key.startswith("max_")})
    summary["first_total_over_90_replay_s"] = next((float(r[0]) for r in values if r[8] > 90), None)
    return summary, values


results = {}
curves = {}
for label, rate, older, corrected in [
    ("current_300", 300, False, False),
    ("same_code_50", 50, False, False),
    ("older_filter_300", 300, True, False),
    ("correct_lowpass_only_300", 300, False, True),
]:
    results[label], curves[label] = simulate(label, rate, older, corrected)
    print(label, json.dumps(results[label]), flush=True)

# Native covariance probe uses precisely the original C++ methods.
native = np.genfromtxt(HERE / "native_filter.csv", delimiter=",", names=True)
summary = {"native_probe": {
    "bounded_input_amplitude": .01,
    "maximum_absolute_output": float(np.max(np.abs(native["output"]))),
    "maximum_absolute_gain": float(np.max(np.abs(native["gain"]))),
    "first_negative_variance_iteration": int(native["iteration"][np.flatnonzero(native["p00"] < 0)[0]]),
    "older_version_maximum_absolute_output": float(np.max(np.abs(native["old_version_output"]))),
}, "open_loop": results, "model": {"nq": MODEL.nq, "nv": MODEL.nv}, "source_hashes": {}}
for rel in ["ros2_ws/src/dual_arm_control_hw/src/controller/JntController.cpp",
            "ros2_ws/src/dual_arm_control_hw/include/controller/JntController.h",
            "ros2_ws/src/dual_arm_control_hw/src/controller/ArmController.cpp",
            "ros2_ws/src/dual_arm_control_hw/src/controller/ArmInterface.cpp",
            "ros2_ws/install/dual_arm_hw/share/dual_arm_hw/urdf/dual_arm.urdf"]:
    source = PROJECT / rel
    summary["source_hashes"][rel] = hashlib.sha256(source.read_bytes()).hexdigest()
(HERE / "audit_summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2))

fig, axes = plt.subplots(3, 1, figsize=(11, 10), sharex=True)
axes[0].plot(native["time_at_300hz"], native["input"], label="Bounded input (+/-0.01)")
axes[0].plot(native["time_at_300hz"], native["output"], label="Production FilteredResCal", alpha=.8)
axes[0].plot(native["time_at_300hz"], native["old_version_output"], label="Older method, same input", alpha=.8)
axes[0].set_yscale("symlog", linthresh=.02)
axes[0].set_ylabel("Input / output")
axes[0].legend()
axes[1].plot(native["time_at_300hz"], native["gain"], linewidth=.5)
axes[1].set_yscale("symlog", linthresh=1)
axes[1].set_ylabel("Internal scalar gain")
axes[2].plot(native["time_at_300hz"], native["p00"], linewidth=.5)
axes[2].set_yscale("symlog", linthresh=.0001)
axes[2].axhline(0, color="black", linestyle="--", linewidth=.8)
axes[2].set_ylabel("P[0,0] (variance)")
axes[2].set_xlabel("Seconds at 300 filter calls per second")
for ax in axes:
    ax.set_xlim(40, 80)
    ax.grid(alpha=.25)
fig.suptitle("Unmodified native C++ filter: isolated numerical failure")
fig.tight_layout()
fig.savefig(HERE / "native_filter_failure.png", dpi=150)
plt.close(fig)

fig, ax = plt.subplots(figsize=(11, 5))
for label, values in curves.items():
    ax.plot(values[:, 0], values[:, 8], label=label, linewidth=1)
ax.axhline(90, color="black", linestyle="--", label="Protocol bound (not a safety threshold)")
ax.set_yscale("log")
ax.set(xlabel="Seconds into target sequence", ylabel="Max absolute internal feedforward",
       title="Actual recorded targets, synthetic open-loop timing: not a physical accident replay")
ax.grid(alpha=.25)
ax.legend()
fig.tight_layout()
fig.savefig(HERE / "open_loop_comparison.png", dpi=150)
plt.close(fig)
