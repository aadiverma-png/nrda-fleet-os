#!/usr/bin/env python3
"""Aggregate the scenario runs into a KPI table and figures.

    python3 benchmarks/report.py            # reads benchmarks/results/raw/*.json

Writes ``benchmarks/results/summary.csv``, one figure per headline metric, and
``benchmarks/results/summary.md``. Everything lands under ``results/``, which is
gitignored: generated output does not belong in ``docs/``.
"""

from __future__ import annotations

import csv
import glob
import json
import math
import os
import statistics
from collections import defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))
RAW = os.path.join(HERE, "results", "raw")
OUT = os.path.join(HERE, "results")
REPORT = os.path.join(OUT, "summary.md")
BAR = "#4F7BA3"

METRICS = [  # key, label, which direction is good
    ("tasks_completed", "Tasks delivered", "higher"),
    ("throughput_per_hour", "Throughput (tasks/h)", "higher"),
    ("latency_mean_s", "Mean task latency (s)", "lower"),
    ("latency_p95_s", "p95 task latency (s)", "lower"),
    ("utilisation_pct", "Fleet utilisation (%)", "higher"),
    ("collisions", "Collisions", "lower"),
    ("collisions_robot_robot", "Robot-robot collisions", "lower"),
    ("near_misses", "Near misses", "lower"),
    ("safety_brakes", "Lidar safety-stop interventions", "lower"),
    ("risk_brakes", "Predictive risk brakes", "lower"),
    ("deadlocks_resolved", "Deadlocks resolved (DRP)", "info"),
    ("standoffs", "In-aisle chain reverses asked (L2b)", "info"),
    ("standoff_yields", "Robots that backed off for one (L2b)", "info"),
    ("wait_time_s", "Total waiting (robot-s)", "lower"),
    ("distance_m", "Distance driven (m)", "info"),
    ("bypasses", "Local bypass manoeuvres", "info"),
    ("reroutes", "Re-routes", "info"),
    ("plan_ms_p95", "Planning time p95 (ms, per robot)", "lower"),
    ("bytes_per_robot_s", "Peer traffic (bytes/robot/s)", "lower"),
]
FIGURES = ("throughput_per_hour", "latency_mean_s", "collisions", "near_misses",
           "wait_time_s", "plan_ms_p95")


def t95(n: int) -> float:
    """Two-sided 95 % t multiplier for n degrees of freedom."""
    table = {1: 12.706, 2: 4.303, 3: 3.182, 4: 2.776, 5: 2.571, 6: 2.447, 7: 2.365, 8: 2.306, 9: 2.262, 10: 2.228}
    return table.get(n, 1.96)


def summarise(vals: list[float]) -> tuple[float, float, int]:
    """Mean, 95 % confidence half-width and sample count, skipping missing values."""
    vals = [v for v in vals if v is not None and not (isinstance(v, float) and math.isnan(v))]
    if not vals:
        return math.nan, math.nan, 0
    m = statistics.mean(vals)
    ci = t95(len(vals) - 1) * statistics.stdev(vals) / math.sqrt(len(vals)) if len(vals) > 1 else math.nan
    return m, ci, len(vals)


def main() -> None:
    runs = [json.load(open(f)) for f in sorted(glob.glob(os.path.join(RAW, "*.json")))]
    if not runs:
        print("no runs in", RAW)
        return
    by = defaultdict(list)
    for r in runs:
        by[r["scenario"]].append(r)
    scenarios = sorted(by)

    # ---------------------------------------------------------------- CSV
    os.makedirs(OUT, exist_ok=True)
    with open(os.path.join(OUT, "summary.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["scenario", "metric", "mean", "ci95", "n"])
        for sc in scenarios:
            for key, _label, _dir in METRICS:
                m, ci, n = summarise([r.get(key) for r in by[sc]])
                w.writerow([sc, key, round(m, 3) if n else "", round(ci, 3) if n > 1 else "", n])

    # ---------------------------------------------------------------- figures
    figs = []
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        for key, label, direction in [m for m in METRICS if m[0] in FIGURES]:
            fig, ax = plt.subplots(figsize=(7.2, 3.4), dpi=130)
            ms, cis = [], []
            for sc in scenarios:
                m, ci, _n = summarise([r.get(key) for r in by[sc]])
                ms.append(0 if math.isnan(m) else m)
                cis.append(0 if math.isnan(ci) else ci)
            xs = range(len(scenarios))
            ax.bar(xs, ms, 0.6, yerr=cis, capsize=3, color=BAR)
            for x_, m_ in zip(xs, ms):
                ax.text(x_, m_, f"{m_:.0f}" if m_ >= 10 else f"{m_:.1f}", ha="center", va="bottom", fontsize=7)
            ax.set_xticks(list(xs))
            ax.set_xticklabels([s.replace("_", " ") for s in scenarios], fontsize=8)
            ax.set_title(f"{label} ({'higher' if direction == 'higher' else 'lower'} is better)",
                         fontsize=10, loc="left")
            ax.spines[["top", "right"]].set_visible(False)
            ax.grid(axis="y", alpha=0.25)
            fig.tight_layout()
            path = os.path.join(OUT, f"fig_{key}.png")
            fig.savefig(path)
            plt.close(fig)
            figs.append((label, os.path.basename(path)))
    except Exception as e:  # noqa: BLE001  (matplotlib missing or headless trouble)
        print("figures skipped:", e)

    # ---------------------------------------------------------------- report
    lines = ["# NRDA-FA scenario benchmarks", "",
             "Generated by `benchmarks/report.py` from `benchmarks/results/raw/`. Twenty AMR-01 robots in the "
             "NRDS warehouse, driven by the same edge agents that run in Gazebo (`core/nrda_fleet/sim2d.py`): "
             "same lane graph, same CBBA allocation, same station docking, same peer bus, same MotionArbiter "
             "and lidar safety stop.", "",
             "Values are the mean over seeds, with the 95 % confidence half-width where more than one seed "
             "was run.", ""]
    for sc in scenarios:
        spec = by[sc][0]
        lines += [f"## {sc.replace('_', ' ')}", "",
                  f"{spec['robots']} robots, {spec['duration']} s simulated, "
                  f"{spec['tasks_released']} tasks released, {len(by[sc])} seed(s).", "",
                  "| Metric | Value | Good |", "|---|---|---|"]
        for key, label, direction in METRICS:
            m, ci, n = summarise([r.get(key) for r in by[sc]])
            if n == 0:
                cell = "–"
            else:
                cell = f"{m:.1f}" + (f" ± {ci:.1f}" if n > 1 and not math.isnan(ci) else "")
            good = {"higher": "higher", "lower": "lower", "info": ""}[direction]
            lines.append(f"| {label} | {cell} | {good} |")
        lines.append("")
    if figs:
        lines += ["## Figures", ""] + [f"![{label}]({path})" for label, path in figs] + [""]
    lines += ["## How to reproduce", "", "```bash",
              "./nrda.sh bench --seeds 5          # all scenarios",
              "./nrda.sh bench --quick            # S1 + S2, 2 seeds", "```", ""]
    with open(REPORT, "w") as f:
        f.write("\n".join(lines))
    print(f"{len(runs)} runs -> {REPORT}, {OUT}/summary.csv, {len(figs)} figures")


if __name__ == "__main__":
    main()
