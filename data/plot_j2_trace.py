"""Plot one J2 log: the goal the roboRIO holds vs. the measured joint angle.

The CSV was recorded with roborio/tools/nt/nt4_watch_arm.py while ROS (MoveIt)
streamed joint targets to the roboRIO over NetworkTables.

    uv run --with matplotlib python data/plot_j2_trace.py
"""

import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = Path(__file__).parent
CSV = HERE / "j2_trace_jerk.csv"
OUT = HERE.parent / "media" / "j2_goal_vs_angle.png"

rows = list(csv.DictReader(CSV.open()))
t0 = float(rows[0]["t"])
t = [float(r["t"]) - t0 for r in rows]
goal = [float(r["goal_deg"]) for r in rows]
angle = [float(r["angle_deg"]) for r in rows]
volts = [float(r["out_volts_cmd"]) for r in rows]

fig, (a1, a2) = plt.subplots(2, 1, figsize=(8, 5), sharex=True, height_ratios=[3, 1])
a1.step(t, goal, where="post", color="tab:blue", lw=1.2, label="goal held by the roboRIO")
a1.plot(t, angle, ".", color="tab:orange", ms=2.5, label="measured J2 angle")
a1.set_ylabel("J2 angle (deg)")
a1.legend(loc="best", frameon=False)
a1.grid(alpha=0.3)
a2.plot(t, volts, color="0.3", lw=0.8)
a2.set_ylabel("motor volts")
a2.set_xlabel("time (s)")
a2.grid(alpha=0.3)
fig.tight_layout()
fig.savefig(OUT, dpi=120)
print(f"{len(rows)} samples over {t[-1]:.1f} s -> {OUT}")
