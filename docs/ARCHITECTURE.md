# NRDA Fleet OS: architecture

## Processes (Gazebo run, 20 robots)

| Process | Count | Role |
|---|---|---|
| `gz sim` server (+ optional GUI) | 1 | Physics at 250 Hz, 20 × AMR-01 DiffDrive, contact sensors, IMU (GPU lidars with `sensor_profile:=fleet`) |
| `parameter_bridge` | 1 | Gazebo ↔ ROS: `/clock`, `/AMR_i/{cmd_vel, odom, contacts, imu}`, TF, ground truth |
| `edge_agent` (`/AMR_i/edge_agent`) | 20 | **The robot's brain.** NRDA-FA layers L1–L5; the only writer of `/AMR_i/cmd_vel` |
| `lidar_emulator` | 1 | `light` profile: two 270°, 271-beam, 10 Hz scans per robot, ray-cast on the CPU from the map, the true robot poses and injected obstacles |
| `safety_monitor` | 1 | Independent collision / near-miss detection from ground truth and contacts; stores events in SQLite; applies the `on_collision` policy |
| `fleet_viz` | 1 | RViz markers: fleet, routes, claims, lane network, stations, safety events |
| `fleet_gateway` | 1 | Web console + REST/SSE API on :8080; releases the scenario workload |
| `static_frames`, `map_server` | 2 | TF `map → AMR_i/odom`, lidar frames, warehouse mesh marker, occupancy map |
| `rviz2` | 1 | Operator 3-D view (fleet layout `nrda_fleet_ros/config/fleet.rviz`) |

## Why these design choices

- **Lane graph instead of a free grid.**
  - The warehouse has 2.4 m aisles and AMR-01 is 1.20 × 0.78 m. Two directed lanes per aisle (right-hand traffic) give every robot a 21 cm clearance to the racks.
  - Head-on meetings become impossible outside deliberate bypasses. Each aisle is a `visison.geometry.Aisle`, so the bypass planner works on whole aisles across their intersections.
  - The graph (23 lines, 112 intersections, 45 stations) is generated from the SDF collision boxes, validated with the real footprint and cached (`~/.cache/nrda`).
- **Owner-authoritative claims instead of a shared table or a server.**
  - Each robot publishes only its own space-time claims. Conflicts resolve by a precedence order every robot computes identically: physical occupancy, then claim age, then task priority, then AMR number. No consensus rounds, no leader, no single point of failure.
  - A silent robot keeps only its body blocked.
- **Rolling window with movement authority.**
  - Robots claim about 12 s ahead and never stop inside a crossing. This keeps claims short-lived, re-plans at 4 Hz and tolerates robots that run late (ORCA speed scaling, braking).
- **visison for local decisions.**
  - Blocked lanes (loading robots, pallets, failed robots) are the case where warehouse fleets lose most time. The visison planner computes the exact earliest safe window in the oncoming lane and claims it atomically through the same claim book.
- **Narrow corridor traffic engineering and directional enforcement.**
  - Narrow corridors where lane width $w < 1.05\text{ m}$ (total aisle width $< 2.0\text{ m}$) cannot support two AMRs passing simultaneously (clearance requires $\ge 1.5\text{ m}$).
  - Corridor `VE` connects decant pickup and charging stations to the facility. Bidirectional traffic caused head-on standoffs where rolling reservations expired mid-transit ($48\text{ s}$ transit vs. $10\text{ s}$ claim window).
  - Enforced one-way Northbound traffic on `VE/A` via `is_lane_enabled(line_name, lane)`. Southbound trips route via parallel wide aisle `V14` ($w = 1.2\text{ m}$), guaranteeing 100% station reachability (1,892 pairs) with zero opposing conflicts by design.
- **Distributed Deadlock Resolution Protocol (DRP).**
  - Stalls and cycle deadlocks are resolved autonomously without a central coordinator.
  - Robots form local groups via union-find, elect a deterministic coordinator, and classify the contention into types (D1 queue, D2 opposing standoff / head-on, D3 crossing, D4 doorway, D5 obstacle, D6 general cycle).
  - In single-file corridors where reverse back-off is geometrically impossible, DRP assigns opposing losers to dynamic rerouting (`reroute`), breaking standoffs cleanly.
- **CPU lidar model by default.**
  - Forty GPU lidars over the 24 MB warehouse mesh ran at a real-time factor of 0.26 on the RTX 3050. Hiding the mesh behind a lidar proxy (`models/nrds_lidar_proxy`) removed the GPU load, but per-sensor render overhead still capped it at about 0.5.
  - The CPU model produces the same scans at real time. The agents cannot tell the difference: same topics, frames and noise.

## Data flow of one robot tick (20 Hz)

1. Odometry to pose, both scans to world points.
2. Perception:
   - static-map filter;
   - clustering;
   - alpha-beta tracks;
   - peer identification (from `/fleet/state`);
   - collision risk (BRAKE means an EMERGENCY intent);
   - unmapped static clusters in a lane become fleet-wide obstacles.
3. At 4 Hz (1 Hz when parked):
   - L5 state machine;
   - L3 schedule (or the L2 bypass tick);
   - DRP deadlock monitor and no-progress watchdog;
   - claim snapshot on `/fleet/claims` if changed.
4. At 2 Hz: L4 CBBA round, broadcast on `/fleet/bids`.
5. At 10 Hz: state and heartbeat on `/fleet/state`.
6. `MotionArbiter.step`: intent by priority, then the Kanayama tracker, then ORCA-DD (speed-only), then the limit gate, then the odometry watchdog.
7. Lidar safety stop (hard-stop zone, protective field, warning field, rotation guard). Output: `/AMR_i/cmd_vel`.

## Code map

| Module | Layer | Purpose |
|---|---|---|
| `nrda_fleet/site.py` | Warehouse model, stations, validation | Site geometry and stations |
| `nrda_fleet/lane_graph.py` | Lanes, intersections, turns, routing | Topology, `is_lane_enabled`, directional constraints |
| `nrda_fleet/claims.py` | L3 owner-authoritative claim book | Space-time claims, precedence, gossip |
| `nrda_fleet/scheduler.py` | L3 schedule, holds, movement authority | Rolling window, `_is_legal_hold`, hold points |
| `nrda_fleet/deadlock.py` | L3 Distributed Deadlock Resolution (DRP) | Cycle detection, D1–D6 classification, standoff rerouting, chain reverse |
| `nrda_fleet/allocation.py` | L4 CBBA | Peer-to-peer auction with lane-graph distances |
| `nrda_fleet/cbba.py` | L4 CBBA core | Bundle building, marginal utility, consensus table |
| `nrda_fleet/waitgraph.py` | L3 wait-for graph | Cyclic-wait detection feeding the DRP |
| `nrda_fleet/health.py` | L5 health | Fault states and the predicates that gate behaviour |
| `nrda_fleet/bypass.py` | L2 visison bypass / reroute / hold | Blocked lane bypass decisions |
| `nrda_fleet/safety.py` | L1 lidar safety stop | Hard stop, protective/warning fields, rotation guard |
| `nrda_fleet/perception.py` | L1 lidar points, static map, clusters | Obstacle extraction and tracking |
| `nrda_fleet/agent.py` | Edge agent (L1–L5) | Onboard agent loop, DRP execution, motor control |
| `nrda_fleet/sim2d.py`, `raycast.py`, `safety_monitor.py`, `metrics.py` | Simulator, lidar model, safety, KPIs | Verification tools and telemetry |
