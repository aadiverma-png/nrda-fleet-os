#!/usr/bin/env python3
"""Run NRDA-FA over the benchmark scenarios in the 2-D simulator.

    python3 benchmarks/run_scenarios.py --quick                      # S1 + S2, 2 seeds
    python3 benchmarks/run_scenarios.py --scenarios S1_nominal S3_busy --seeds 5

Each run is one process (runs in parallel on all cores) and writes
``benchmarks/results/raw/<scenario>__s<seed>.json``. Then run
``benchmarks/report.py`` for the KPI tables and figures.
"""

from __future__ import annotations

import argparse
import json
import os
import statistics
import sys
import time
from concurrent.futures import ProcessPoolExecutor, as_completed

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "core"))

import yaml  # noqa: E402

def run_one(scenario: str, spec: dict, seed: int, out_dir: str, lidar: bool) -> dict:
    from nrda_fleet.agent import AgentConfig
    from nrda_fleet.bus import CommProfile
    from nrda_fleet.lane_graph import LaneGraph
    from nrda_fleet.metrics import FleetLedger
    from nrda_fleet.sim2d import DynObstacle, Sim2D
    from nrda_fleet.site import Site
    from nrda_fleet.tasks import generate_workload

    t_wall = time.time()
    site = Site.load()
    graph = LaneGraph.load_or_build(site)
    comm = CommProfile.lossy(spec["comm_loss"]) if spec.get("comm_loss") else CommProfile()
    sim = Sim2D(site, graph, AgentConfig(), robots=spec.get("robots", 20), seed=seed,
                comm=comm, lidar=lidar, beams=91)
    ledger = FleetLedger()
    sim.bus.observers.append(lambda m: ledger.on_message(m, sim.t))
    stations = {n: node.station for n, node in graph.stations.items()}
    tasks = generate_workload(stations, spec["tasks"], spec["rate"], seed, t0=5.0)
    sim.add_tasks(tasks)
    for i, (x, y, t_on, t_off) in enumerate(spec.get("blockages", [])):
        sim.obstacles.append(DynObstacle(f"obs_{i + 1}", x, y, 1.1, 0.9, t_on, t_off))
    for rid, t_fail in spec.get("failures", []):
        if rid in sim.robots:
            sim.fail_robot(rid, t_fail)
    sim.run(spec["duration"])
    for ev in sim.events:
        ledger.add_safety(ev)
    k = ledger.kpis()
    agents = [r.agent for r in sim.robots.values()]
    plan = sorted(x for a in agents for x in a.stats.plan_ms)
    tick = sorted(x for a in agents for x in a.stats.tick_ms)
    res = {
        "scenario": scenario, "seed": seed, "robots": len(agents),
        "duration": spec["duration"], "wall_s": round(time.time() - t_wall, 1),
        "tasks_released": sum(1 for t in tasks if t.release <= spec["duration"]),
        "tasks_completed": k["tasks_completed"],
        "throughput_per_hour": k["throughput_per_hour"],
        "latency_mean_s": k["latency_mean_s"], "latency_p95_s": k["latency_p95_s"],
        "assign_wait_mean_s": k["assign_wait_mean_s"],
        "utilisation_pct": k["utilisation_pct"],
        "collisions": sum(1 for e in sim.events if e.level == "COLLISION"),
        "collisions_robot_robot": sum(1 for e in sim.events if e.level == "COLLISION" and e.other.startswith("AMR_")),
        "near_misses": sum(1 for e in sim.events if e.level == "NEAR_MISS"),
        "avoided": sum(1 for e in sim.events if e.level == "AVOIDED"),
        "min_gap_m": round(sim.min_gap, 3) if sim.min_gap != float("inf") else None,
        "safety_brakes": sum(a.stats.safety_brakes for a in agents),
        "risk_brakes": sum(a.stats.risk_brakes for a in agents),
        "waits": sum(a.stats.waits for a in agents),
        "wait_time_s": round(sum(a.stats.wait_time for a in agents), 1),
        "distance_m": round(sum(a.stats.distance for a in agents), 1),
        "energy": round(sum(a.stats.energy for a in agents), 4),
        "replans": sum(a.stats.replans for a in agents),
        "reroutes": sum(a.stats.reroutes for a in agents),
        "bypasses": sum(a.stats.bypasses for a in agents),
        "deadlocks_resolved": sum(a.stats.deadlocks for a in agents),
        "standoffs": sum(a.stats.standoffs for a in agents),
        "standoff_yields": sum(a.stats.standoff_yields for a in agents),
        "plan_ms_p50": round(statistics.median(plan), 2) if plan else None,
        "plan_ms_p95": round(plan[int(0.95 * (len(plan) - 1))], 2) if plan else None,
        "tick_ms_p95": round(tick[int(0.95 * (len(tick) - 1))], 2) if tick else None,
        "messages_sent": sim.bus.sent, "messages_dropped": sim.bus.dropped,
        "bytes_per_robot_s": round(sim.bus.bytes / max(len(agents), 1) / spec["duration"], 1),
        "safety_events": [e.__dict__ for e in sim.events if e.level != "AVOIDED"][:50],
    }
    path = os.path.join(out_dir, f"{scenario}__s{seed}.json")
    with open(path, "w") as f:
        json.dump(res, f, indent=1)
    return res


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenarios", nargs="*", default=None)
    ap.add_argument("--seeds", type=int, default=3)
    ap.add_argument("--quick", action="store_true", help="S1 + S2, 2 seeds")
    ap.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) - 2))
    ap.add_argument("--no-lidar", action="store_true", help="skip lidar ray casting (faster, no safety stop)")
    ap.add_argument("--out", default=os.path.join(HERE, "results", "raw"))
    args = ap.parse_args()
    with open(os.path.join(HERE, "scenarios", "scenarios.yaml")) as f:
        scen = yaml.safe_load(f)
    names = args.scenarios or (["S1_nominal", "S2_blockages"] if args.quick else list(scen))
    seeds = 2 if args.quick else args.seeds
    os.makedirs(args.out, exist_ok=True)
    # build the lane graph once (cached on disk) before forking
    sys.path.insert(0, os.path.join(HERE, "..", "core"))
    from nrda_fleet.lane_graph import LaneGraph
    from nrda_fleet.site import Site
    LaneGraph.load_or_build(Site.load())
    jobs = [(n, scen[n], s) for n in names for s in range(1, seeds + 1)]
    print(f"{len(jobs)} runs on {args.workers} workers")
    t0 = time.time()
    with ProcessPoolExecutor(args.workers) as ex:
        futs = {ex.submit(run_one, n, sp, s, args.out, not args.no_lidar): (n, s) for n, sp, s in jobs}
        for fut in as_completed(futs):
            n, s = futs[fut]
            try:
                r = fut.result()
                print(f"[{time.time() - t0:6.0f}s] {n:18s} seed {s}: {r['tasks_completed']:3d} done, "
                      f"{r['throughput_per_hour']:6.1f}/h, collisions {r['collisions']}, near {r['near_misses']}, "
                      f"wall {r['wall_s']} s", flush=True)
            except Exception as e:  # noqa: BLE001
                print(f"{n} seed {s} FAILED: {e!r}", flush=True)


if __name__ == "__main__":
    main()
