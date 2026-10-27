# NRDA-FA: the NRDA fleet-coordination algorithm

NRDA-FA is the one algorithm that runs on every AMR in the NRDA fleet. It is a five-layer onboard
stack: task and health, allocation, routing with space-time reservation, local coordination, and
motion with safety. There is no central planner and no fleet server; robots coordinate only by
peer-to-peer messages.

Two invariants hold everywhere:

- **one owner per decision** -- every decision below has exactly one layer that makes it;
- **one writer of motor commands per robot** -- the `MotionArbiter`. Every layer submits an
  *intent*; nothing else touches `cmd_vel`. The receiving robot's arbiter is also the only thing
  that can move it, so a peer or an operator sends directives, never actuation.

Implementation: `core/nrda_fleet/` (integration) and `core/visison/` (planner, ORCA, risk,
arbiter). There is no test suite: `core/tests/` was deleted. Every algorithm with its published source and
mathematics: [`FINAL_AUDIT.md`](FINAL_AUDIT.md).

---

## 1. The NRDA-FA pipeline (per robot, all onboard)

```
L5 Task + health     state machine IDLE → TO_PICKUP → LOADING → TO_DROP → UNLOADING → (CHARGING | PARKING)
                     FaultState, battery model, heartbeat                                   [event]
L4 Allocation        CBBA over the peer bus → bundle; execution order = bundle order       [2 Hz]
L3 Route + reserve   lane-graph A* (learned congestion cost) → rolling-window space-time
                     schedule (earliest_free_point per cell) → owner-authoritative claims;
                     movement authority = confirmed prefix; WFG on wait edges              [4 Hz]
L2 Local coord.      lane blocked by an obstacle or a long-stopped robot → BypassPlanner
                     (squeeze | bypass | wait | reroute | hold)                          [4 Hz]
L1 Motion + safety   ObjectTracker + EmergencyMonitor (risk) · protective field (raw lidar)
                     MotionArbiter: tracker → ORCA-DD (speed-only) → limit gate → watchdog
                     → safety speed cap → **cmd_vel**                                        [20 Hz]
HAL                  RobotInterface: Gazebo (ROS topics) | 2-D simulator | real robot
```

### 1.1 Lane graph (the space the robots share)
- Every rack aisle, cross-aisle and corridor is a **two-lane directed road** with right-hand traffic. Each straight piece between two intersections is one `visison.geometry.Aisle` with lanes `<aisle>/A` and `<aisle>/B`, of width 1.2 m and with 0.5 m cells.
- **Narrow corridor directional constraints:** In narrow corridors where lane width $w < 1.05\text{ m}$ (total aisle width $< 2.0\text{ m}$, where passing requires $\ge 1.5\text{ m}$), bidirectional operation causes geometric standoffs. The function `is_lane_enabled(line_name, lane)` in `lane_graph.py` restricts Corridor `VE` ($w = 0.975\text{ m}$) to one-way traffic (Northbound lane `A` only). All 8 stations along `VE` face North; Southbound traffic routes through parallel wide aisle `V14` ($w = 1.2\text{ m}$), preserving 100% reachability across all 1,892 station pairs.
- An **intersection** is a single reservation resource (`X<i>`). A robot may only enter it together with the lane cells beyond it ("don't block the box").
- All turns happen at the four lane-crossing points inside an intersection, as a rotation in place. The rotation circle (r = 0.72 m for AMR-01) stays inside the 2.4 m square.

### 1.2 Space-time claims (L3), owner-authoritative gossip
- **Scheduling.** A robot schedules its route by sweeping its padded body along the timed route (0.35 m ahead and behind while moving). For every covered resource (lane cell, station footprint, 0.25 m free-space cell) it checks the interval against every peer claim with precedence, widened by τ = 1.0 s.
- **Waiting & legal holds.** On a conflict it inserts a wait at the **last legal hold point** before that resource. The wait is sized by `visison.intervals.earliest_free_point`. A legal hold point is:
  - a lane position whose body is clear of every crossing core (lane centre ± (half width + 5 cm) of the crossing lanes);
  - outside single-file corridors (`_is_legal_hold(..., is_peer=bool(auth_blockers))` forbids yielding robots from holding inside narrow bottlenecks);
  - reachable with the robot's real braking capability.
- **Claiming.** It claims its **rolling window**: 12 s, extended to the next legal hold point so authority never ends inside a crossing. It publishes its whole claim set as an owner-authoritative snapshot, on change plus a 1 s keep-alive. A replica is the union of the latest snapshots.
- **Precedence.** Physical occupancy wins first, then the older claim stamp, then the higher task priority, then the lower AMR number. Every replica computes the same winner without messages.
- **Movement authority.** Authority is the prefix of claims that are *usable*. A claim is usable once it is 0.3 s old (so a racing peer claim would have arrived), or immediately if no live peer is within 4 m (nobody can race us). The executed trajectory brakes to a stop at the authority limit unless the next replan (4 Hz while moving, 1 Hz while parked) extends it.
- **Forced stops.** If a conflict can no longer be avoided by stopping before it, the robot brakes as early as possible anyway, and the blocker is handed to L2.
- **Silent peers.** A silent peer (no heartbeat for 3 s) loses every claim except the cells its body covers, which stay blocked.
- **Unmapped obstacles.** Obstacles seen by any robot's lidar are turned into permanently blocked resources in every replica until the obstacle is seen cleared.

### 1.3 Local coordination (L2)
- **Triggers.** L2 starts when the robot is held at a hold point:
  - by a blocked lane cell of an unmapped obstacle, found by its own lidar or reported by a peer;
  - by a **stopped** peer (loading at a picker stop, faulted, silent) for more than 4 s.
- **Decision.** The blocker is converted into a `visison.geometry.Obstacle` in that aisle's frame, and `BypassController.on_obstacle` makes the single decision: squeeze, bypass now, wait-then-bypass, re-route or hold.
- **Oncoming lane.** Bypass claims on the oncoming lane are made atomically through the same claim book. Oncoming robots therefore schedule around them, and the window is re-checked every tick until the lane change starts.
- **Re-routing.** `REROUTE` is costed on the lane graph with the blocked stretch removed. A hold that lasts 30 s escalates to a re-route.
- **Hand-back.** After the manoeuvre the robot drops the route legs it drove past and L3 re-plans from its new pose.

### 1.4 Emergency braking and deadlocks (L1/L3)
- **Risk brake.** Every robot rates its lidar tracks with `collision_risk`. Peers are identified by their broadcast pose, so they are cooperative with a known deceleration. A **BRAKE** risk produces an EMERGENCY brake intent on the robot's own arbiter and an `Alert` on the bus; the console counts it as an avoided collision.
- **Why head-on standoffs are rare.** With directed lanes, one-way corridor enforcement, and claimed bypass windows, two robots meeting head-on in one lane can only happen after a claim race lost under communication loss.
- **Distributed Deadlock Resolution Protocol (DRP).** Cyclic waits and progress stalls are detected autonomously:
  - Wait-for graph: elementary cycles by depth-first search on back edges
    (`nrda_fleet/waitgraph.py`) detect cyclic wait dependencies;
  - No-progress watchdog catches stalled clusters (< 0.3 m moved in 20 s);
  - Deadlock members group via union-find and elect a deterministic coordinator (`min(-priority, rid)`);
  - The coordinator classifies the contention into **D1** (queue), **D2** (opposing standoff / head-on), **D3** (crossing contention), **D4** (station doorway), **D5** (obstacle), or **D6** (general cycle).
- **Single-file corridor standoff resolution:**
  - In normal aisles, D2 resolves by chain reverse back-offs (0.3–1.4 m) for non-priority robots;
  - In narrow single-file corridors where passing clearance ($\ge 1.5\text{ m}$) does not exist (`single_file_standoff`), chain reverse cannot clear the long corridor;
  - The DRP recognizes `single_file_standoff` and assigns the opposing loser to **`reroute`** (`Move(loser.rid, "reroute", 0.0, 0)`), cleanly vacating the corridor;
  - During reroute, contested lane cells are temporarily forbidden to prevent immediate re-entry.

### 1.5 Lidar safety stop (L1, independent of every coordination layer)
- Uses both 270° scans, **raw**, with no map filtering.
- Hard-stop zone (unconditional, the ESPE pattern of IEC 61496-1): any return within 0.28 m of the bumper inside ±12° of the direction of travel stops the robot at once.
- Protective field: a rectangle ahead of or behind the robot, of length `d_react + v²/(2·a_e) + 0.10 m` and width = body + 2 × 0.05 m. A hit means an EMERGENCY brake.
- Warning field: 2× the length, which caps speed proportionally.
- While rotating in place the field is a circle of radius 0.77 m.
- It can only lower the speed, never raise it. `/fleet/estop` (latched) produces an OPERATOR brake.

---

## 2. Command authority and decision ownership

1. **One motor writer per robot: `MotionArbiter`.** Every layer submits a `MotionIntent`. Priority, highest first:
   - `OPERATOR` (e-stop, pause);
   - `EMERGENCY` (lidar safety stop, BRAKE risk);
   - `FLEET` (WFG yield, cooperative delay);
   - `PLANNER` (route and bypass trajectories).

   The safety speed cap is applied to the arbiter's output inside the same component. It can only lower the command.
2. **One owner per decision:**

   | Decision | Owner |
   |---|---|
   | which task | CBBA (L4) |
   | which path and when | L3 schedule |
   | bypass / wait / reroute / hold at a blocked lane | `BypassPlanner` (L2) |
   | in-aisle deadlock | WFG yield (L3); head-on standoffs prevented by claims |
   | cross-aisle deadlock | WFG (L3) |
   | stop | safety (L1) |
3. **Safe-point rule.** A new CBBA bundle, a reroute or a WFG yield takes effect only when the robot is in its own lane. It must not be inside a bypass manoeuvre, an intersection, or a station dwell. Until then the change is queued.
4. **Deadlock-scope rule.** A robot running an L2 manoeuvre (bypass) is never chosen to yield by the WFG: the manoeuvre finishes first (safe-point rule).
5. **No remote actuation.** Leaders, peers and the web console send *directives*. Only the receiving robot's arbiter moves its motors.
6. **Launch-level guarantee.** Nav2 and RMF are not in the workspace. `nrda_fleet_ros` checks at start-up that no other publisher exists on `/AMR_<i>/cmd_vel`.

These rules were covered by `core/tests/test_nrda_fleet.py` and `core/tests/test_deadlock.py`
(single-writer e-stop precedence 2.1, supervisor-lowers-only and lidar hard stop §5.1/§4.5,
safe-point rule 2.3, directives never actuate 2.5, DRP head-on and single-file classification
1.4). Those tests have been deleted; only `edge_agent_node._check_single_writer` (2.6) is still
checked, at run time. The rules remain implemented and unverified.

---

## 3. Parameters (AMR-01)

| Parameter | Value | Source |
|---|---|---|
| footprint | 1.20 x 0.78 m | `models/amr_01/model.sdf`, `nrda_fleet/amr.py` |
| v_hw, v_cruise, accel, decel, e_decel | 2.2, 1.8, 1.2, 1.2, 1.5 m/s, m/s^2 | `nrda_fleet/amr.py`; SDF DiffDrive limits |
| a_hw, ω_max, α_max | 1.6 m/s^2, 1.5 rad/s, 2.0 rad/s^2 | SDF |
| wheel separation, wheel radius | 0.62 m, 0.075 m | SDF |
| w_turn, α_turn (rotation in place) | 1.0 rad/s, 1.2 rad/s^2 | `nrda_fleet/amr.py` |
| lane width, cell | 1.2 m, 0.5 m | 2.4 m aisles |
| τ (claim time margin) | 1.0 s | `AgentConfig.tau` = `Scheduler.tau`; `bypass.py` sets the same total for L2 |
| claim window, confirm, replan | 12 s, 0.3 s, 4 Hz | `AgentConfig.window`, `claims.confirmed` |
| bypass trigger | 4 s blocked | `AgentConfig.bypass_trigger` |
| peer timeout | 3 s | `AgentConfig.peer_timeout` |

---

## 4. Verification

```bash
./nrda.sh test                   # the ROS node tests; the ROS-free core has no suite
./nrda.sh build                  # colcon: core, warehouse, messages, fleet runtime
./nrda.sh bench --quick          # scenario benchmark in the 2-D simulator
```

The evidence table, including the peak-load stress results, is
[`FINAL_AUDIT.md`](FINAL_AUDIT.md) section 12. The hazard-to-mitigation mapping is
[`SAFETY_CASE.md`](SAFETY_CASE.md).
