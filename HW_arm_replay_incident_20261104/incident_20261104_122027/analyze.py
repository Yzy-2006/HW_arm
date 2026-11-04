"""Offline incident analysis. Reads saved CSV evidence; never imports ROS."""
from pathlib import Path
import csv
import datetime as dt
import json
import os
import re

import numpy as np

os.environ.setdefault("MPLCONFIGDIR", "/tmp/hw-arm-incident-matplotlib")
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parent
START = 1793766027.188
ZONE = dt.timezone(dt.timedelta(hours=8))


def time_label(timestamp):
    return dt.datetime.fromtimestamp(timestamp, ZONE).strftime("%H:%M:%S.%f")[:-3]


with (ROOT / "evidence/outputs/arm_monitor/monitor.csv").open() as stream:
    metadata = stream.readline()
    origin = int(re.search(r"unix_origin_ns=(\d+)", metadata).group(1)) / 1e9
    monitor = [r for r in csv.DictReader(stream) if r.get("r6_peak_step_rad") is not None]
monitor = [r for r in monitor if START - 1 <= origin + float(r["elapsed_s"]) <= START + 22]
with (ROOT / "evidence/outputs/replay_logs/episode_3_start_0_20261104_122027.csv").open() as stream:
    replay = list(csv.DictReader(stream))

with (ROOT / "event_monitor.csv").open("w", newline="") as stream:
    writer = csv.DictWriter(stream, fieldnames=["local_time", *monitor[0]])
    writer.writeheader()
    for row in monitor:
        writer.writerow({"local_time": time_label(origin + float(row["elapsed_s"])), **row})

event = [r for r in monitor if origin + float(r["elapsed_s"]) >= START]
conditions = {
    "left_feedforward_abs_over_10": lambda r: max(abs(float(r[f"l{i}_feedforward"])) for i in range(7)) > 10,
    "left_feedforward_abs_over_90": lambda r: max(abs(float(r[f"l{i}_feedforward"])) for i in range(7)) > 90,
    "left_speed_peak_over_2": lambda r: max(float(r[f"l{i}_peak_speed_rad_s"]) for i in range(7)) > 2,
    "left_decode_age_over_100ms": lambda r: max(float(r[f"l{i}_decode_age_ms"]) for i in range(7)) > 100,
    "bad_wkc_window": lambda r: bool(int(r["window_flags"]) & 2),
    "command_timeout_window": lambda r: bool(int(r["window_flags"]) & 1),
}
summary = {"replay_rows": len(replay), "monitor_rows": len(monitor), "first_observed": {}}
for label, condition in conditions.items():
    row = next((r for r in event if condition(r)), None)
    summary["first_observed"][label] = None if row is None else {
        "local_time": time_label(origin + float(row["elapsed_s"])),
        "seconds_after_launch": origin + float(row["elapsed_s"]) - START,
        "window_flags": int(row["window_flags"]),
    }
for field in ["l0_feedforward", "l0_peak_speed_rad_s"]:
    row = max(event, key=lambda r: abs(float(r[field])))
    summary[field + "_largest_absolute"] = {
        "value": float(row[field]), "local_time": time_label(origin + float(row["elapsed_s"]))
    }
summary["left_shoulder_dataset_max_frame_step_rad"] = float(np.max(np.abs(np.diff([
    float(r["dataset_left_joint_1"]) for r in replay
]))))
(ROOT / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2))

tm = np.array([origin + float(r["elapsed_s"]) - START for r in monitor])
tr = np.array([float(r["elapsed_s"]) for r in replay])
fig, axes = plt.subplots(4, 1, figsize=(11, 12), sharex=True)
for group, label, color in [("dataset", "Recorded target", "tab:blue"),
                            ("command", "Replay bridge target", "tab:orange")]:
    axes[0].plot(tr, [float(r[f"{group}_left_joint_1"]) for r in replay], label=label, color=color)
axes[0].plot(tm, [float(r["l0_q_cmd"]) for r in monitor], label="Servo target", color="tab:purple")
axes[0].plot(tm, [float(r["l0_q_fb"]) for r in monitor], label="Decoded motor position", color="tab:green")
axes[0].set_ylabel("Left J1 angle (rad)")
axes[0].legend(loc="lower left", ncol=2)
axes[1].plot(tm, [float(r["l0_feedforward"]) for r in monitor], label="Before protocol clipping", color="tab:red")
axes[1].set_yscale("symlog", linthresh=10)
axes[1].axhline(90, color="black", linestyle="--", linewidth=.8, label="Protocol bound +/-90")
axes[1].axhline(-90, color="black", linestyle="--", linewidth=.8)
axes[1].set_ylabel("Feedforward field\n(symlog, not measured torque)")
axes[1].legend(loc="upper right")
axes[2].plot(tm, [float(r["l0_peak_speed_rad_s"]) if float(r["l0_decode_age_ms"]) <= 100 else np.nan
                 for r in monitor], color="tab:red", label="Left J1 window peak speed (fresh decode)")
axes[2].axhline(2, color="black", linestyle="--", label="Monitor marking threshold")
axes[2].set_ylabel("Decoded speed (rad/s)")
axes[2].legend(loc="upper right")
for axis, label in [("l0", "Left J1"), ("l1", "Left J2")]:
    axes[3].plot(tm, [float(r[axis + "_decode_age_ms"]) for r in monitor], label=label)
axes[3].axhline(100, color="black", linestyle="--", linewidth=.8)
axes[3].set_ylabel("ACK1 decode age (ms)")
axes[3].set_ylim(-50, 4000)
axes[3].set_xlabel("Seconds after 12:20:27.188 (Asia/Shanghai)")
axes[3].legend(loc="upper left")
for ax in axes:
    ax.axvline(1793766032.839 - START, color="gray", linestyle=":", linewidth=1)
    ax.grid(alpha=.25)
    ax.set_xlim(4.5, 10)
fig.suptitle("2026-11-04 replay incident: saved evidence, 20 Hz monitor")
fig.tight_layout()
fig.savefig(ROOT / "incident_timeline.png", dpi=150)
plt.close(fig)
print(json.dumps(summary, ensure_ascii=False, indent=2))
