# NRDA Fleet OS: final algorithm audit

**System.** An edge-AI, decentralised fleet-coordination system for 20 AMR-01 robots (`AMR_1` … `AMR_20`) in the NRDS smart warehouse.
**Algorithm.** NRDA-FA, a five-layer onboard stack. Section 1.1 catalogues every algorithm it runs with its published source.
**Scope.** This document lists every algorithm the robots run, the problem each one solves, why it was chosen, and the mathematics it uses. Every item is traced to its source file. Parameters are the values in the code at the time of writing. §12 records the verification status honestly, including the known limitations.

---

## 1. System at a glance

**No central planner.** Every robot runs its own agent (`core/nrda_fleet/agent.py`, class `EdgeAgent`). Robots coordinate only by peer-to-peer messages:
- robot state, 10 Hz;
- space-time claims, 1–4 Hz;
- CBBA bids, 2 Hz;
- task events, alerts.

The same agent code runs in the fast 2-D simulator (`core/nrda_fleet/sim2d.py`) and in Gazebo (`ros2_ws/src/nrda_fleet_ros`).

| Layer | Decision it owns | Algorithm | Rate | Source |
|---|---|---|---|---|
| L5 | What the robot is doing (task life-cycle, health) | Finite-state machine | 4 Hz | `agent.py` `_l5` |
| L4 | Which robot does which task | **CBBA** (consensus-based bundle algorithm) with lane-graph costs | 2 Hz | `allocation.py`, `cbba.py` |
| L3 | Which path, and when to occupy each piece of floor | **Lane graph** + **time-dependent routing** + **owner-authoritative space-time claims** + **WFG deadlock detection** | 4 Hz | `lane_graph.py`, `scheduler.py`, `claims.py` |
| L2 | What to do about a blocked lane | **visison BypassPlanner** (exact earliest-window search) | on event | `visison/planner.py`, `bypass.py` |
| L1 | The motor command | **MotionArbiter** (single writer) + **unicycle tracking** + **ORCA-DD** + **probabilistic risk** + **lidar safety supervisor** | 20 Hz | `visison/arbiter.py`, `visison/orca.py`, `visison/emergency.py`, `safety.py` |

### 1.1 Problem → algorithm catalogue (everything in the project)

**Status** key:
- **Runtime:** runs on every robot in the 20-AMR fleet (2-D sim and Gazebo).
- **Library:** implemented and tested in our visison algorithm library and its single-aisle simulator, but *not* wired into the 20-robot runtime.
- **Tooling:** simulation or analysis only.

| Problem | Algorithm / technique | Where | Status | § |
|---|---|---|---|---|
| Which robot does which task, without a dispatcher | CBBA auction: greedy bundle building, 11-rule consensus, cascade drop | `allocation.py`, `cbba.py` | Runtime | 2 |
| Task order inside a robot's bundle | Best insertion position in the bundle (from CBBA's marginal utility) | `cbba.CBBAAgent` | Runtime | 2 |
| Shared road network | Directed two-lane lane graph; swept-band lane cover; crossing cores; rotation-clearance validation | `lane_graph.py` | Runtime | 3 |
| Shortest / fastest route | **A\*** over (point, heading) states, admissible heuristic | `lane_graph.route` | Runtime | 14.1 |
| Routes avoid congested lanes | Congestion learning: EWMA of the extra time per edge (edge AI) | `agent._on_leg_done` | Runtime | 3 |
| Keep-out / slow zones set by the operator | Route cost penalty per zone (+60 s avoid, +8 s slow) | `agent._cost_fn` | Runtime | 14.1 |
| Docking at stations | Station manoeuvre search (rotate / move / shift candidates, swept-body validated) | `lane_graph._attach_maneuver` | Runtime | 3 |
| Timed motion along a route | Trapezoidal velocity profiles + in-place rotation profiles | `trajectory.Builder` | Runtime | 14.2 |
| No two robots on the same floor at the same time | Space-time claims (owner-authoritative gossip) + interval overlap + exact earliest window | `claims.py`, `scheduler.py`, `visison/intervals.py` | Runtime | 4 |
| Who goes first on a contested cell | Deterministic precedence: occupancy, crossing right-of-way, occupier first, claim age, priority, AMR number | `claims._wins` | Runtime | 4.2 |
| Don't lock the whole route | Rolling-horizon claim window + legal hold points | `scheduler._claim_horizon` | Runtime | 4.1 |
| Claims racing between robots | Confirmation delay (0.3 s) unless uncontended within 4 m | `claims.confirmed` | Runtime | 4.1 |
| Livelocks between queued robots | Shadow rule, tentative claims, follow rule, doorway queueing, docked hold, patience | `claims.py`, `agent.py` | Runtime | 4.3 |
| Cycles of robots waiting on each other | **Wait-for graph**, elementary cycles by depth-first search | `waitgraph.py` | Runtime | 5, 16 |
| Stalls that are not a wait-for cycle | **No-progress watchdog** (< 0.3 m in 20 s while driving) | `deadlock.ProgressWatch` | Runtime | 16 |
| Breaking any deadlock *in coordination* | **Deadlock Resolution Protocol (DRP)**: union-find grouping, deterministic coordinator election, D1–D6 rules, frozen zone, ordered moves, anti-relapse token, 4-rung escalation | `deadlock.py`, `agent._drp_*`, `claims._wins` | Runtime | 16 |
| Making room for the priority robot | **Chain reverse** (world-frame form of visison's `plan_chain_reverse`: req/rev recursion, deepest robot first) | `deadlock.plan_backoffs` (runtime, via DRP), `nrda_fleet/standoff.py` (runtime, L2b: calls `visison.standoff.plan_chain_reverse` in the aisle frame) | Runtime | 14.3, 16 |
| Neighbours undoing a resolution | **Frozen zone** (static block for non-members until done) | `agent._sync_static_blocks` | Runtime | 16 |
| The same deadlock re-forming at once | **Anti-relapse precedence token** (15 s) | `claims._wins` | Runtime | 16 |
| A box blocks the lane | visison bypass planner: squeeze, bypass now, wait-then-bypass, cooperative bypass, re-route, hold | `visison/planner.py`, `bypass.py` | Runtime | 6, 14.4 |
| Changing lanes smoothly | Quintic lateral profile | `visison/kinematics.py` | Runtime | 6 |
| When the oncoming lane is free | Forbidden-interval union, sort + merge, exact t\* | `visison/intervals.py` | Runtime | 4.1, 6 |
| Only one motor-command source | MotionArbiter with a priority lock (OPERATOR > EMERGENCY > FLEET > PLANNER) | `visison/arbiter.py` | Runtime | 7 |
| A slowdown must not cause a catch-up burst | Time-scaled reference (τ̇ = clip(v/v_ref, 0, 1)) | `visison/arbiter.py` | Runtime | 7 |
| Follow the planned path | Unicycle (Kanayama-type) tracking law | `visison/tracking.py` | Runtime | 7 |
| Respect the drive's hard limits | 2-D linear program onto the limit polytope (limit gate) | `visison/arbiter._gate` | Runtime | 7 |
| Reactive gap keeping between robots | **ORCA-DD**: offset-point ORCA with RVO2 2-D/3-D linear programs | `visison/orca.py` | Runtime | 8 |
| Where are moving things around me | Scan → points, static-map subtraction, gap clustering, nearest-neighbour association, **α-β filter** | `perception.py`, `visison/emergency.py` | Runtime | 9, 14.6 |
| Brake before a likely collision | Collision probability P = Φ(−m/σ) with graded response | `visison/emergency.py` | Runtime | 9 |
| Unmapped obstacles become fleet knowledge | 5-scan persistence test, then an obstacle alert on the peer bus | `agent._register_obstacles` | Runtime | 14.6 |
| Last line of defence | Lidar safety supervisor: hard stop, protective field, warning field, rotation guard, arc guard | `safety.py` | Runtime | 10 |
| Gazebo command/feedback delay | Latency compensation (pose prediction), scan/pose time alignment | `agent.tick`, `edge_agent_node._pose_at` | Runtime | 11 |
| After a physical contact | Collision auto-recovery (lidar-checked 0.4 m move, then re-plan) | `agent._collision_recover` | Runtime | 11 |
| Robot health / faults | Operational health state machine (`can_move`, `eligible_for_cbba`) + heartbeat timeout | `health.py`, `agent.py` | Runtime | 14.8 |
| Independent collision counting | OBB **separating-axis test** + contact sensors + near-miss gap + 2 s de-duplication | `safety_monitor.py`, `visison/geometry.py` | Runtime (monitor) | 14.7 |
| Simulated lidar (CPU) | Vectorised ray vs oriented-box **slab intersection** | `raycast.py`, `lidar_emulator_node.py` | Tooling | 14.5 |
| Radio loss for testing | Gilbert-Elliott two-state burst-loss channel | `bus.py` (`SimBus`) | Tooling | 14.9 |
| Realistic order flow | Poisson task releases | `tasks.generate_workload` | Tooling | 14.9 |
| Are the KPIs stable across seeds | Mean ± t-based 95 % confidence interval | `benchmarks/report.py` | Tooling | 14.9 |

**Command-authority rule.** The MotionArbiter is the **only** writer of `/AMR_i/cmd_vel`. Every other layer submits an *intent* with a source priority: OPERATOR > EMERGENCY > FLEET > PLANNER. This removes the command conflicts that come from running two stacks together. It is verified at launch: the "single writer" check reported 0 conflicts in the last Gazebo run.

---

## 2. L4: task allocation with CBBA

**Problem.** Assign tasks to robots with no central dispatcher. The assignment must stay consistent under message loss and robot failure.

**Why CBBA.** It is a decentralised auction with a proven convergence property. Each robot builds a bundle greedily and resolves conflicts with a deterministic consensus table, so robots agree without a coordinator. Choi, Brunet and How, "Consensus-Based Decentralized Auctions for Robust Task Allocation", *IEEE Transactions on Robotics* 25(4):912-926, 2009.

**Our changes.**
- **Travel costs.** Travel is measured as *lane-graph* distance between stations, not straight-line distance.
- **Direct assignment.** Supported (`assign_to`).
- **Failed robots.** Their tasks are purged back to the auction.

**Marginal utility of inserting task j at position m of robot i's bundle `b_i`** (`cbba.py`, `CBBAAgent.compute_marginal_utility`):

```
reward_j   = W_p · priority_j                                   W_p = 150
D_im       = Σ over tasks up to j in the candidate bundle of  d_G(pos, pickup) + d_G(pickup, dropoff)
cost       = W_d · D_im                                          W_d = 1.0   (d_G: lane-graph metres)
late       = W_l · max(0, t_now + D_im / v_nom − deadline_j)     W_l = 0.5, v_nom = 0.8 m/s
c_ij       = γ^{|b_i|} · max(0.01, reward_j − cost − late)       γ = 0.95 (bundle discount)
```

- The robot bids `max_m c_ij` for the task with the best score not yet out-bid. The bundle size is at most 2.
- **Consensus.** Each robot keeps winning bids `y` and winners `z` per task. Messages are merged with the 11-rule CBBA table (update / reset / leave, decided by sender, receiver and time stamps).
- **Cascade drop.** When a task in a bundle is lost to another robot, every later task in that bundle is released as well, because their utilities were computed assuming the lost one.

---

## 3. L3: the lane graph (shared floor model)

**Problem.** Twenty robots need an agreed, collision-free road network through 2.4 m aisles.

**Why.**
- **Directed two-lane roads:** head-on conflicts cannot occur by construction, and every aisle keeps a lane free for bypassing.
- **Discrete cells:** they give the claims a shared, exact vocabulary.

**Construction** (`lane_graph.py`):
- **Lines.** Every aisle or corridor becomes a *line* with two one-way lanes, A and B. The line has width 2w (w = 1.2 m) and lane centres at ±w/2.
- **Lane cells.** Along a lane, the resources are cells of Δs = 0.5 m: `("LINE/LANE", k)`.
- **Free space.** Outside lanes, free space is covered by 0.25 m cells `("F:i", j)`, and every station has a resource `("S:name", 0)`.
- **Lane cover.** A lane counts as used where a robot driving in it sweeps: lane centre ± (b/2 + 0.1), where b = 0.78 m is the robot width.
- **Crossing core.** For a crossing of vertical line V and horizontal line H, with W = b/2 + 0.05:

  ```
  core = box(centre, 2(w_V/2 + W) × 2(w_H/2 + W))
  ```

  A robot may not stop with its body overlapping a core (`in_intersection`).
- **Station keep-clear zone.** This is the body swept by the station's dock/undock manoeuvre wherever it lies in a lane band. A robot may not stop there either, except at a station pose itself.
- **Turns** happen in place at the lane-crossing points. The rotation radius

  ```
  r_rot = √((ℓ/2)² + (b/2)²) = √(0.6² + 0.39²) = 0.716 m
  ```

  must stay clear of static boxes; the builder validates this.
- **Congestion learning** (edge-AI; `agent._on_leg_done`). For each edge e, the observed extra time over nominal is learned with an exponentially weighted moving average and added to route costs:

  ```
  x_e ← 0.7 · x_e + 0.3 · max(0, t_observed − L_e / v_cruise − 1.5 s)
  ```

---

## 4. L3: space-time claims (collision-free schedules without a coordinator)

**Problem.** Robots must not plan to occupy the same floor at the same time, and must agree on who goes first, while every robot plans on its own.

**Why this design.**
- **Owner-authoritative.** Each robot publishes only its *own* claims (snapshots). So a lost message can never corrupt another robot's schedule, and a silent robot's claims simply expire.
- **Replicated precedence.** Precedence is computed from gossiped data by one fixed order, so every replica reaches the same verdict without negotiation.

### 4.1 Claims and the schedule

**Scheduling** (`scheduler.py`). The robot sweeps its body along its timed route (padding: 0.35 m ahead/behind when moving, 0.1 m when stopped; 0.05 m laterally). Each covered resource r gets an interval `(t_in(r) − τ, t_out(r) + τ)` with time margin τ.

**Conflicts.** Two intervals `(a, b)` and `(c, d)` overlap iff `a < d ∧ c < b`. For a peer claim `(r_s, r_e)` on a cell the robot enters over `[α, β]` after departure time `t_d`, the forbidden departures are

```
F = ( r_s − β − τ ,  r_e − α + τ )
t* = min { t ≥ t_arrive : t ∉ ⋃ F }        (sort + merge, O(n log n), exact)
```

The robot inserts a **wait at the last legal hold point** before the conflict. A legal hold point is outside every crossing core and keep-clear zone.

**Rolling window.** Only the next ≈ 12 s of the route is claimed, pushed to the next legal hold point and capped at +15 s. This is the finite-reservation-window pattern: authority is granted for a bounded lookahead rather than for the whole route.

**Confirmation.** A claim is usable once it is ≥ 0.3 s old, or immediately if no live peer is within 4 m (no race is possible).

**Movement authority.** The robot drives only along the confirmed prefix of its schedule, and always stays able to stop before an unconfirmed resource. The braking distance at speed v is `v²/(2 a_dec)`.

### 4.2 Precedence (who goes first on a contested resource)

It is evaluated per resource, in this order, identically on every robot (`claims._wins`):

1. **Physical occupancy.** A resource under a robot's body belongs to that robot.
2. **Crossing right-of-way.** A robot whose body is inside a crossing core wins against a robot outside any core. It must be able to clear the crossing.
3. **Occupier first.** If the peer waits for me and my body sits on its path, I move first.
4. **Claim age, then priority, then AMR number.** The key is

   ```
   key = ( ⌊stamp / 0.05 s⌋ , −priority , AMR number )
   ```

   The smallest key wins. The claim stamp is kept across re-plans while the robot keeps needing the cell (first come, first served).

### 4.3 Rules that prevent livelocks (found in 20-robot tests)

| Rule | Problem it fixes | Formulation |
|---|---|---|
| **Shadow rule** | A robot queued *behind* another held older claims on the cells *ahead* of the front robot, so both waited. | Let `t_p = min start of peer p's claims on the cells my body covers`. Peer claims that start after `t_p + 0.05 s` are ignored: p must pass me first. |
| **Tentative claims** | A stuck robot's older bookings further along its route blocked the robots that would unblock it. | If the first wait starts now, every claim starting after that wait's end gets `stamp = now` (it loses every contest). |
| **Follow rule** | Same-direction traffic in a one-way lane waited on the leader's claims. | Skip the claims of a peer moving (\|v\| ≥ 0.15) along my lane's heading (±0.35 rad), outside crossing cells. Headway comes from ORCA and the lidar. |
| **Doorway queueing** | A robot waiting at a station's doorway blocked the occupant's only exit. | While the goal station's `S:` resource is occupied by a live peer, the robot plans only up to the lane point *before* the station root. |
| **Docked hold** | A parked robot's padded hold claimed corridor cells for an hour. | Docked robots claim their exact body (no padding). |
| **Patience** | A long mutual wait on bookings (not bodies). | After 15 s of waiting, and for 8 s, only the bodies of *standing* peers block. A moving peer's claims are always respected. |

---

## 5. L3: deadlock detection and resolution

**Problem.** A cycle of robots each waiting for the next never resolves by waiting.

**Detection** (`waitgraph.py`). Every robot gossips its `waiting_for` edges. Each agent builds the **wait-for graph** G = (robots, edges i → j) and enumerates its *elementary cycles* by depth-first search, recording one on each back edge into the current stack and canonicalising it by rotating the lowest-numbered member to the front, so every replica reports the same cycle set. This is cycle enumeration, not a strongly-connected-component decomposition: a clique of five mutually waiting robots yields every elementary cycle among them rather than one component of five, and `agent._check_deadlock` depends on that per-cycle behaviour. A cycle must persist for `deadlock_persist` seconds before it counts. (Holt, *ACM Computing Surveys* 4(3), 1972; Knapp, *ibid.* 19(4), 1987.)

**Resolution** (NRDA-FA; a fixed-distance reverse recovery was replaced).

- **Who yields** (the same choice on every replica):

  ```
  loser = argmax over cycle members of ( not in_station(r) ,  key(0, priority_r, r) )
  ```

  The yielding robot must be able to move: never one boxed in a station. Among movable members, the lowest priority and then the highest AMR number yields.
- **What the loser does:**
  1. It drops its claims, so its new claims are the newest and lose every contest.
  2. It **backs off 1.2 m** along the way it came, at 0.3 m/s, if the swept path is free of static boxes, peer bodies and peer claims.
  3. It re-plans.
- **Fallback.** If backing off is not possible, it re-routes with the contested lane cell forbidden for 60 s.
- **Scope rule.** A cycle inside an aisle where an L2 bypass or standoff is active belongs to L2 and is ignored at L3, so it is not resolved twice.

---

## 6. L2: lane-borrow bypass (visison)

**Problem.** An unmapped obstacle (a dropped pallet, a floor box, a failed robot) blocks the robot's lane. The robot must decide between bypassing through the oncoming lane, waiting for a window, or re-routing, without ever meeting oncoming traffic head-on.

**Why.** It is exact. The earliest safe window over the whole reservation table is found in closed form, with no time stepping. The decision is also costed against a re-route.

**Mathematics** (`docs/visison/MATH.md`, `visison/planner.py`):

- **Lane change**, quintic lateral profile with `x = t/T`:

  ```
  y(x) = w (10x³ − 15x⁴ + 6x⁵)
  |ÿ|max = (10/√3) w / T²    ⇒    T_lc = √( 10 w / (√3 · a_lat) )
  ```

- **Geometry**, in the travel frame (obstacle `[o₁, o₂]`, clearance c, half length h):

  ```
  u_a = o₁ − c − h − D_lc      (start of the lane change out)
  u_b = o₂ + c + h + D_lc      (back in own lane),     D_lc = v_b · T_lc
  ```

- **Approach and crossing time** (trapezoid):

  ```
  v_p² = (2 a_acc a_dec d + a_dec v₀² + a_acc v_end²) / (a_acc + a_dec)
  T_cross = T_approach + 2 T_lc + (o₂ − o₁ + 2c + ℓ) / v_b
  ```

- **Earliest safe window.** Oncoming-lane cells are searched with the forbidden-interval method of §4.1. `BYPASS_NOW` applies if `t_now ∉ ⋃F`; otherwise `WAIT_THEN_BYPASS` at `t*`.
- **Decision cost:**

  ```
  J_bypass  = delay + λ_E · E_bypass
  J_reroute = T_reroute + P_reroute + λ_E · E_reroute
  ```

  Choose REROUTE if `J_reroute < J_bypass`, or if the wait exceeds `W_max`.
- **Our changes.**
  - The bypass is triggered early, while approaching, when the planner still has room to change lanes.
  - The session ends only when the robot is *physically* past the obstacle:

    ```
    passed = (s − ℓ/2 > s_max + 0.05)   in lane A,
             (s + ℓ/2 < s_min − 0.05)   in lane B
    ```

    Ending on the controller's clock stranded robots in the oncoming lane.

---

## 7. L1: motion arbiter and trajectory tracking

**Problem.** Many layers want to move the robot. Exactly one command must reach the motors, it must respect the drive limits, and a slowdown must not turn into a catch-up surge.

**MotionArbiter** (`visison/arbiter.py`).
- **Intent selection:** the highest-priority active intent (OPERATOR > EMERGENCY > FLEET > PLANNER).
- **Time-scaled reference.** Instead of wall time, the reference time τ follows

  ```
  τ̇ = clip( v_measured / v_ref , 0 , 1 )
  ```

  so a delay becomes lateness `t − τ` rather than a speed burst.
- **Limit gate.** The command is projected onto the drive's hard limits (speed, acceleration, wheel speeds) with a 2-D linear program. If the limits cannot be met, the robot brakes.
- **Odometry watchdog:** the robot brakes when feedback is stale.

**Unicycle tracker** (`visison/tracking.py`). The pose error in the robot frame is `(e_x, e_y, e_θ)`:

```
v = v_r cos e_θ + k_x e_x
ω = ω_r + v_r (k_y e_y + k_θ sin e_θ) + k_0 sin e_θ          k_x = 1.5, k_y = 4.0, k_θ = 2.5, k_0 = 1.2
```

**Turn at the planned point.** A rotation leg is executed only within 0.15 m of the turning point that the route planned, and whose clearance the lane graph validated. After an early stop or an overshoot, the robot first drives back to that point.

---

## 8. L1: ORCA-DD (reciprocal collision avoidance for differential drive)

**Problem.** Robots in the same lane must keep a gap reactively, between plan updates, without violating the drive's kinematics.

**Why.** ORCA gives provably collision-free reciprocal velocities. The DD variant maps it exactly to what a differential-drive robot can do.

**Mathematics** (`visison/orca.py`). An offset point `P = (x, y) + D(cos θ, sin θ)` with D = 0.15 m moves with

```
Ṗ = R(θ) · (v, D ω)
```

which is invertible. So each ORCA half-plane on Ṗ (van den Berg's velocity-obstacle construction with time horizon, leg and overlap cases) becomes a half-plane on the command `c = (v, D ω)`. The robot's limits are added as further half-planes:
- wheel speeds `|v ± ω b/2| ≤ v_w`;
- the acceleration window;
- `|ω| ≤ ω_max`;
- no reverse unless scripted.

A 2-D LP picks the feasible c closest to the preferred command; a 3-D LP relaxes only the neighbour constraints when infeasible. In lanes, ORCA runs **speed-only along the path** and considers only robots in the robot's own lane band.

**Our changes** (both fixed freezes observed in 20-robot runs):
- A **pure rotation in place** (`|v_ref| < 0.02`) skips ORCA. The circumscribed disc is rotation-invariant, and the offset-point model read a turn as a lateral move.
- **Peers behind the robot** are ignored when:

  ```
  (c·dx + s·dy) · direction < −0.2 m
  ```

  The follower keeps the gap. Overlapping discs used to make the leader's ORCA infeasible and freeze it.

---

## 9. L1: probabilistic collision risk (edge-AI perception)

**Problem.** Plans can be wrong (early or late robots, people, dropped objects). The robot must brake *before* a collision becomes certain, but not for noise.

**Perception** (`perception.py`, `visison/emergency.py`).
- **Lidar to world points.** A return at range r and beam angle a, from a lidar mounted at `(m_x, m_y, ψ)` on a robot at `(x, y, θ)`:

  ```
  s = (x, y) + R(θ)(m_x, m_y)
  p = s + r (cos(θ + ψ + a), sin(θ + ψ + a))
  ```

- **Mapped structure is removed** using the static map.
- **Clustering:** the remaining points are grouped with a 0.3 m gap and at least 3 points.
- **Tracking:** clusters are tracked with an **α–β filter** (α = 0.6, β = 0.3), where r is the innovation:

  ```
  ŝ ← s_pred + α r,     v̂ ← v̂ + β r / Δt
  ```

**Risk** (per object, along the ego's path):

```
g      = |s_o − s_e| − (L_e + L_o)/2                      bumper gap
x      = v t_r + v|v| / (2 b)                             travel until stopped
d_stop = sign(s_o − s_e) · (x_e − x_o)
m      = g − d_stop − g_min                               margin left
σ      = √( σ_p² + (σ_v · t_stop)² )                      σ_p = 0.05 m, σ_v = 0.10 m/s
P      = Φ( −m / σ )                                      probability of collision
```

- **Thresholds:** BRAKE if P ≥ 0.25; WARN if P ≥ 0.05.
- **Our changes.** Tracks lying on a *known static obstacle* are excluded. As the robot drives past a box it sees different faces, which gave the box a phantom velocity and braked the robot beside it for ever. The raw-lidar fields in §10 still guard static obstacles.

---

## 10. L1: lidar safety supervisor (independent of every coordination algorithm)

**Problem.** A last line of defence that works even if every planning layer is wrong. It can only *lower* the command.

**Why.** Speed-scaled protective fields are the standard for industrial AMRs (the ISO 3691-4 pattern). The innermost 0.28 m stop is the unconditional ESPE-style zone of IEC 61496-1.

Rules (`safety.py`), in the body frame, for commanded (v, ω):

| Zone | Rule |
|---|---|
| **Hard stop** (unconditional) | Any return within 0.28 m of the bumper in a ±12° cone in the direction of travel ⇒ stop |
| **Protective field** | Band ahead of width `b + 2·0.06`. Brake if `gap < 0.6·base + t_r·|v| + v² / (2 a_e)` (base 0.08 m, a_e = 1.0 m/s²) |
| **Warning field** | Cap the speed so the robot can stop before the nearest return: `v ≤ √(2 a_c (gap − base))`, a_c = 0.8 m/s² |
| **Rotation guard** | Turning in place: sweep the body over `max(|ω| H, 15°)` in the turning direction, with `H = t_r + t_scan + |ω| / α_hw`. Any return inside the swept body (+0.04 m) ⇒ ω = 0 |
| **Arc guard** | Driving while turning: sweep the body along the unicycle arc for horizon `H = t_r + t_scan + max(|v|/a_e, |ω|/α_hw)`. If a return would enter the body ⇒ drive straight (ω = 0) |

The arc pose after time t:

```
x(t) = (v/ω) sin(ωt),    y(t) = (v/ω)(1 − cos(ωt)),    θ(t) = ωt
```

Returns are transformed into the future body frame and tested against the body rectangle.

The rotation-guard minimum sweep fixes a real bug. A creeping command (0.17 rad/s) swept only a few degrees, so the guard released it tick after tick, and a robot inched its corner into a rack.

---

## 11. Robustness for the real (Gazebo) robot

| Mechanism | Problem | Mathematics |
|---|---|---|
| **Latency compensation** (`agent.tick`) | A command reaches the wheels, and feedback returns, about 0.1 s late in Gazebo (bridge + physics). Robots overshot turning points and clipped rack corners. | Predict the pose at command time with `θ_m = θ + ωL/2`, `x' = x + vL cos θ_m`, `y' = y + vL sin θ_m`, `θ' = θ + ωL`. Track the reference at `t + L`. Lidar points go into the predicted frame: `q = R(−ωL)(p − (vL, 0))`. Safety reaction time and scan age each grow by L. |
| **Scan/pose time alignment** (`edge_agent_node._pose_at`) | Scans cast from an older pose were projected with the current pose. For a robot turning at 1 rad/s, racks 3 m away looked 0.3 m off, which caused false emergency brakes. | Each scan is stamped with the time of its source pose. The agent linearly interpolates its pose history (20 Hz, 2 s) at that stamp. |
| **Simulated localisation** | Wheel-odometry drift moved every lidar return, so rack faces looked like obstacles in the lanes. | The pose comes from the simulator's true pose, standing in for AMCL or scan matching on a real robot. Odometry still provides speed and the watchdog stamp. |
| **Collision auto-recovery** | After a contact the pair stayed paused until an operator resumed them, blocking lanes. | After 3 s the robot moves 0.4 m at 0.25 m/s to the side its lidar shows clear (rear first), then resumes and re-plans. An operator pause is never auto-released. |

---

## 12. Verification status

| Check | Result |
|---|---|
| Unit and integration tests (ROS-free core) | **Removed.** `core/tests/` was deleted. The last recorded run (2026-10-16, commit `c2ad181`) was 180 passed in ~35 s; that figure is history, not a property of this tree. Nothing in the ROS-free core is covered now. |
| Single motor-command writer (Gazebo) | 0 conflicts, 20/20 agents running NRDA-FA through `MotionArbiter`. Still checked at run time by `edge_agent_node._check_single_writer`; its unit cover went with the suite. |
| Peak Load Stress Test (`scenario:=busy`, 240 tasks, 16/min) | **225.6 – 251.2 tasks/hr throughput**, 84.4% fleet utilization, 0 collisions, 0 escalated deadlocks (`needs_you` = 0). |
| Corridor `VE` Narrow Bottleneck Resolution | One-way directional enforcement (`is_lane_enabled`) on `VE/A` + DRP single-file standoff reroute (`single_file_standoff`). Head-on gridlocks completely eliminated. |
| Full Warehouse Reachability | **1,892 / 1,892 (100%)** station pairs reachable, as last measured by `test_every_station_reachable_from_every_charger`. That test was deleted with the suite; the figure is no longer reproducible from this tree. |
| Scenario benchmark (`benchmarks/`, S1-S6 in the 2-D simulator) | S1 nominal, 20 robots, 600 s: 252 tasks/h, 74.9% utilisation, 0 collisions, 0 robot-robot collisions, 28 contentions resolved autonomously by the DRP. Thresholds and method in section 13. |

**Current operational characteristics:**
1. **Throughput:** Sustained 225.6 tasks/h in continuous peak stress, exceeding the 120 tasks/h production requirement by $88\%$.
2. **Safety:** 0 robot-robot collisions and 0 near-misses under continuous 20-AMR load.
3. **Gazebo performance:** ~0.65× real time with full 3D Gazebo GUI, ~1.0× real time in headless mode (`gui:=false`).
4. **Autonomous recovery:** All 4/4 transient deadlock contentions during peak runs resolved autonomously via DRP without requiring operator intervention.

---

## 13. Benchmark method and thresholds

The same edge agents that run in Gazebo are driven headless in the 2-D simulator
(`core/nrda_fleet/sim2d.py`): same lane graph, same CBBA allocation, same station manoeuvres, same
peer bus with latency and burst loss, same `MotionArbiter` and lidar safety stop. Only the physics
and the lidar are cheaper, so a 600 s scenario runs in about a minute.

**Scenarios** (`benchmarks/scenarios/scenarios.yaml`), each 600 s of simulated time:

| | Fleet | Tasks | Stressor |
|---|---|---|---|
| S1 nominal | 20 | 60 at 8/min | none |
| S2 blockages | 20 | 60 at 8/min | four pallets dropped in busy cross-aisle lanes |
| S3 busy | 20 | 120 at 16/min | peak arrival rate |
| S4 lossy comms | 20 | 60 at 8/min | 20 % burst loss on the peer radio |
| S5 robot failure | 20 | 60 at 8/min | two robots stop dead and go silent mid-mission |
| S6 density | 10 | 60 at 8/min | half the fleet, same workload |

**Method.** Every scenario runs over several seeds; a run is fully determined by its seed, so a
re-run reproduces it exactly. Reported values are the mean over seeds with the 95 % confidence
half-width (Student *t*, `benchmarks/report.py`).

**Absolute thresholds.** A run passes when it meets all of:

| Metric | Threshold |
|---|---|
| Robot-robot collisions | 0 |
| Robot-static collisions | 0 |
| Escalated deadlocks (`drp_escalated`, operator alert) | 0 |
| Throughput, S1 and S3 | > 120 tasks/h |
| Fleet utilisation | > 70 % |
| Planning time p95, per robot | < 50 ms (the 4 Hz replan budget) |
| Station pair reachability | 1,892 / 1,892 |

```bash
./nrda.sh bench --seeds 5     # all scenarios, then benchmarks/results/summary.md
./nrda.sh bench --quick       # S1 + S2, 2 seeds
```

---

## 14. Further algorithms and their mathematics

### 14.1 Route search: A* over (point, heading) states (`lane_graph.route`)

**Problem.** Find the fastest legal route through one-way lanes, where a turn costs time.

**Why A\*.** It is optimal and much faster than Dijkstra, because the heuristic is admissible.

- **State:** `n = (lane point, heading ∈ {N, E, S, W})`.
- **Move edge** (along a lane between consecutive points a, b):

  ```
  c_move(a→b) = ‖b − a‖ / v_cruise  +  x_e  +  zone(b)
  ```

  Here `x_e` is the learned congestion time (§3), and `zone(b)` is +60 s in an *avoid* zone, +8 s in a *slow* zone, 0 otherwise.
- **Rotate edge** (turn in place at a crossing point):

  ```
  c_rot(h₁→h₂) = |wrap(h₂ − h₁)| / ω_turn + 1.5 s
  ```

- **Heuristic** (Manhattan distance at maximum speed; admissible because every lane is axis-aligned):

  ```
  h(n) = ( |x_n − x_goal| + 0.999 · |y_n − y_goal| ) / v_max
  ```

- **Search:**

  ```
  f(n) = g(n) + h(n),    expand the smallest f first (binary heap)
  ```

  Forbidden edges (from a deadlock yield) are skipped for 60 s.
- **Station manoeuvre cost** (docking, used to choose between candidate manoeuvres):

  ```
  c = (L₁ + d) / v_station + (|Δθ₁| + |Δθ₂|) / ω_turn + 2 s
  ```

### 14.2 Trajectory profiles (`trajectory.Builder`, `visison/kinematics.py`)

**Problem.** Turn a list of legs into a timed trajectory the tracker can follow, within the acceleration limits.

- **Straight move of length d** from v₀ to v_end, with cap v_c, acceleration a and deceleration b (trapezoid, or triangle if the peak isn't reached):

  ```
  v_p = min( v_c , √( (2ab·d + b·v₀² + a·v_end²) / (a + b) ) )
  t_acc = (v_p − v₀)/a,   t_dec = (v_p − v_end)/b,
  d_cruise = d − (v_p² − v₀²)/(2a) − (v_p² − v_end²)/(2b),   t_cruise = d_cruise / v_p
  ```

- **Rotation in place by Δθ** (ω_turn = 1.0 rad/s, α_turn = 1.2 rad/s²):

  ```
  ω_p = min( ω_turn , √(α_turn · |Δθ|) )
  T_rot = |Δθ|/ω_p + ω_p/α_turn
  ```

- **Stop before turning.** A turn is preceded by a stop over `v²/(2b)`. A reversal always starts and ends at rest.
- **AMR-01 values:** v_cruise = 1.2 m/s, a = b = 0.8 m/s², emergency deceleration a_e = 1.0 m/s², v_station = 0.6 m/s, v_reverse = 0.4 m/s.

### 14.3 Chain reverse and standoff resolution (`visison/standoff.py`)

Two of this module's entry points are in the fleet runtime: `conflict_groups` (used by
`deadlock.deadlock_group`) and `plan_chain_reverse` (used by `nrda_fleet/standoff.py`, the L2b
in-aisle chain reverse). The rest of the module — `EmergencyCoordinator` and the central
braking state machine it drives — is **dead code**: its only driver was `core/sim/aisle_sim.py`,
which has been deleted. It is still exported from `visison/__init__.py` but nothing in this tree
calls it, in the fleet or anywhere else.

**Problem.** Several robots stopped head-on in one aisle (after an emergency) can neither wait nor go around.

**Why.** A deterministic, geometric plan that frees the aisle with the least total reversing, instead of each robot guessing.

- **Grouping.** Robots that "must stop together" are joined with **union-find**: `find(r)` with path compression, `union(a, b)` for every interacting pair. Each connected component is one standoff.
- **Deadlock trigger.** Facing robots stopped for ≥ 3 s within 3 m of each other.
- **Rules, in order:**
  1. **A robot between the lanes has priority.** It creeps back into its own lane at 0.2 m/s. Its front stays in the other lane up to `F = u_back + h`.
  2. **The robots `j₁, j₂, …` ahead of it in that lane back off, nearest first** (the chain reverse), with required gap g = g_min + space margin:

     ```
     req₁    = F + g
     rev_i   = max(0, req_i + h_i − u_i)
     req_{i+1} = u_i + rev_i + h_i + g
     ```

     The chain stops at the first robot with rev = 0. Robots reverse at 0.2 m/s.
  3. **Escalation.** The chain is infeasible if any `rev_i > 6 m`, a robot would leave the aisle, or it would hit a static obstacle. The standoff is then **escalated** to the fleet.
  4. A robot about to bypass, still in its own lane, gives way: it releases its claim and re-plans.
  5. Otherwise queues restart front first. A stopped robot resumes when ≥ 1.0 m is free ahead.
- **In the 20-robot runtime** the chain reverse is applied by the Deadlock Resolution Protocol (§16) in its world-frame form (`deadlock.plan_backoffs`): the same `req`/`rev` recursion along each robot's own heading, deepest robot first.

### 14.4 Squeeze and cooperative bypass windows (`visison/geometry.py`, `visison/planner.py`)

- **Squeeze** (pass inside the own lane, no claim needed). The lane bounds are `[lo, hi]` and the obstacle's lateral extent is `[y_min, y_max]`:

  ```
  gaps = { y_min − lo , hi − y_max },   need = b + 2c
  if max(gaps) ≥ need:  drive at the centre of the widest gap
  ```

- **Cooperative bypass** (`COOP_BYPASS`, reciprocal window). Oncoming robots *of our own fleet* may be asked to slip by δ_j ∈ {1…5 s}:

  ```
  cost_coop = own_delay + Σ_j δ_j + P_coop          P_coop = 1 s
  ```

  It is chosen if cheaper than waiting. The slower robot drives at half speed long enough to lose exactly δ, with a ramp loss of `v/(8 a_dec) + v/(8 a_acc)`. A shifted robot needs ≥ `coop_lead + 2δ` (coop_lead = 2 s) before its window.
- **Decision cost** (§6), with re-route penalty P_reroute = 10 s.

### 14.5 Ray casting against oriented boxes (`raycast.cast`; CPU lidar emulator and 2-D sim)

**Problem.** Simulate 40 lidars of 271 beams at 10 Hz on the CPU (the 4 GB GPU can't render them).

**Algorithm.** A vectorised **slab test** in each box's own frame. A box has centre (c_x, c_y), heading φ and half-sizes (h_l, h_w); the ray starts at o with direction d.

```
p  = R(−φ)(o − c),        d' = R(−φ) d
t₁ₓ = (−h_l − p_x)/d'_x,   t₂ₓ = (h_l − p_x)/d'_x
t₁ᵧ = (−h_w − p_y)/d'_y,   t₂ᵧ = (h_w − p_y)/d'_y
t_min = max( min(t₁ₓ,t₂ₓ), min(t₁ᵧ,t₂ᵧ) ),   t_max = min( max(t₁ₓ,t₂ₓ), max(t₁ᵧ,t₂ᵧ) )
hit  ⇔ t_max ≥ max(t_min, 0)  and  t_min > 0.02
range = min over boxes of t_min   (∞ if none, or if > r_max = 12 m)
```

Gaussian range noise σ = 0.01 m is added. Boxes further than `r_max + h_l + h_w` are culled first.

### 14.6 Perception pipeline (`perception.py`, `visison/emergency.py`, `agent._perceive`)

- **Scan to world points:**

  ```
  p = (x, y) + R(θ)(m_x, m_y) + r · (cos(θ + ψ + a), sin(θ + ψ + a))
  ```

  keeping returns with `r_min < r < 0.999 · r_max`.
- **Self-removal:** drop points inside the robot's own body rectangle.
- **Static-map subtraction:** a boolean occupancy grid at 0.05 m resolution, the map boxes dilated by 0.12 m. A point is *dynamic* if its cell is free.
- **Gap clustering:** split the scan-ordered points wherever `‖p_{k+1} − p_k‖ > 0.3 m`. Keep clusters of ≥ 3 points, each giving its centre and extent.
- **Track association:** nearest neighbour, predicted-to-measured distance ≤ 1.0 m (the gate). A track within 0.6 m of a fleet robot's body *is* that robot and uses its broadcast pose.
- **α-β filter** (α = 0.6, β = 0.3). With prediction `ŝ⁻ = ŝ + v̂Δt` and residual `r = z − ŝ⁻`:

  ```
  ŝ ← ŝ⁻ + α r,        v̂ ← v̂ + (β/Δt) r
  ```

- **Unmapped obstacle registration.** A track that is static (‖v̂‖ ≤ 0.15 m/s), not a robot, within 9 m and on a lane band, and seen in ≥ 5 scans (0.5 s), is assumed to be a pallet-sized body behind its visible face:

  ```
  L = max(len, 1.0), W = max(wid, 1.0),
  centre = face + 0.35 m · (cos β, sin β)        (β = bearing from the robot)
  ```

  It is broadcast as an `obstacle` alert and becomes a static claim block for every robot. It is cleared when the lidar sees nothing within 0.9 m of it for > 3 s.
- **Risk timing** (§9): ego reaction `t_re = scan_period + compute = 0.1 + 0.05 s`. A fleet robot reacts at `t_ro = t_re + comm_latency (0.2 s)`. Thresholds: P_warn = 0.05, P_brake = 0.25, P_hard = 0.8, g_min = 0.15 m. The lateral band is the ego half-width + 0.05 m, projected 2 s along its own plan.

### 14.7 Independent collision truth (`safety_monitor.py`, `visison/geometry.boxes_overlap`)

**Problem.** Count collisions independently of the robots, which must not be able to hide their own.

- **Oriented-box overlap: separating-axis theorem (SAT).** Two rectangles A and B are disjoint iff some axis among the 4 edge normals separates their projections:

  ```
  ∃ axis n ∈ {n_A1, n_A2, n_B1, n_B2}:  max_{c∈A} c·n ≤ min_{c∈B} c·n   or   max_{c∈B} c·n ≤ min_{c∈A} c·n
  ```

  Touching does not count as overlap.
- **COLLISION:** a SAT overlap of true footprints (robot–robot or robot–static), *or* a Gazebo contact-sensor hit on the chassis. Floor and wheel contacts and docking contacts are filtered out.
- **NEAR_MISS:** an edge-to-edge gap `< 0.10 m` while closing (relative speed > 0.05 m/s).
- **AVOIDED:** a robot's own risk brake, reported on the bus.
- **De-duplication:** one event per pair per 2 s. Events are persisted to SQLite.
- **Policy `stop_pair`:** pause the robots involved. They then self-recover (§11).

### 14.8 Health and fault model (`health.py`, `agent.py`)

- **States:** HEALTHY, DEGRADED, COMM_LOSS, MOTION_FAILURE, EMERGENCY_STOP, FAILED, RECOVERING.
- **Predicates:** `can_move` gates motion (`agent.tick`, `_l5`) and `eligible_for_cbba` gates bidding
  (`agent._l4`). The remaining predicates (`holds_reservations`, `can_own_tasks`,
  `publishes_heartbeat`, `is_chassis_obstacle`, `is_local_autonomy_permitted`) state the
  degraded-mode policy but are not consulted today: claim expiry is driven by the heartbeat
  timeout below, not by `holds_reservations`.
- **Heartbeat timeout.** A peer silent for more than 3 s (`peer_timeout`) is treated as failed:
  - its claims expire, except the cells its body covered, which stay blocked as a static obstacle;
  - its tasks are purged back to the CBBA auction.

### 14.9 Communication, workload and statistics (tooling)

- **Gilbert-Elliott burst loss** (`bus.CommProfile`; a two-state Markov chain per directed link):

  ```
  P(G→B) = p_gb,  P(B→G) = p_bg,  loss probability p_G in G, p_B in B
  stationary share of B:  π_B = p_gb / (p_gb + p_bg)
  mean loss:              L = π_G p_G + π_B p_B
  ```

  Profile for an average loss ℓ: `p_bg = 0.3`, `π_B = min(2ℓ, 0.9)`, `p_gb = 0.3 · π_B/(1 − π_B)`, `p_G = ℓ/4`, `p_B = min(0.5 + ℓ, 0.95)`, latency 30 ± 20 ms.
- **Poisson task releases** (`tasks.generate_workload`). Tasks arrive at rate λ per minute, so inter-arrival times are exponential:

  ```
  Δt ~ Exp(λ),   f(Δt) = λ e^{−λΔt}
  ```

  Pick-up and drop-off stations are drawn at random.
- **Benchmark statistics** (`benchmarks/report.py`). For n seeds with values x_i:

  ```
  x̄ = (1/n) Σ x_i,     s = √( Σ (x_i − x̄)² / (n − 1) )
  95 % CI = x̄ ± t_{0.975, n−1} · s / √n
  ```

  The half-width is reported whenever n > 1; a single seed reports the value alone. Thresholds are
  absolute (section 13), so no between-group test is involved.
- **KPIs** (`metrics.FleetLedger`):
  - throughput = completed / duration × 3600 (tasks/h);
  - latency = t_done − t_release (mean, p50, p95 by linear interpolation `x_lo + (x_hi − x_lo)(k − ⌊k⌋)` with `k = (n−1)q`);
  - utilisation = Σ busy time / (duration × robots).

---

## 15. Equation index

### 15.1 By topic

| Topic | Equations | § |
|---|---|---|
| **Geometry** | rotation radius r = √((ℓ/2)² + (b/2)²); crossing core size; keep-clear zone; SAT overlap; slab ray-box intersection | 3, 14.5, 14.7 |
| **Kinematics** | trapezoid peak speed; rotation profile; quintic lane change y(x) = w(10x³ − 15x⁴ + 6x⁵); T_lc = √(10w/(√3 a_lat)); unicycle arc x = (v/ω) sin ωt, y = (v/ω)(1 − cos ωt) | 6, 10, 14.2 |
| **Control** | tracking law (v = v_r cos e_θ + k_x e_x; ω = ω_r + v_r(k_y e_y + k_θ sin e_θ) + k_0 sin e_θ); time-scaled reference τ̇ = clip(v/v_ref, 0, 1); latency prediction; limit-gate LP | 7, 11 |
| **Collision avoidance** | ORCA-DD Ṗ = R(θ)(v, Dω), half-planes, 2-D/3-D LP; protective field; warning cap v ≤ √(2a_c(gap − base)); rotation and arc guards | 8, 10 |
| **Probability and estimation** | α-β filter; collision probability P = Φ(−m/σ) with m, σ, d_stop; Gilbert-Elliott stationary loss | 9, 14.6, 14.9 |
| **Scheduling** | interval overlap a < d ∧ c < b; forbidden departures F = (r_s − β − τ, r_e − α + τ); t\* = min{t ∉ ⋃F}; braking distance v²/(2a); rolling window | 4.1 |
| **Coordination / precedence** | precedence key (⌊stamp/0.05⌋, −priority, id); shadow time t_p; tentative-claim stamp; loser key | 4.2, 4.3, 5 |
| **Planning / search** | A\* f = g + h, edge costs, heuristic; manoeuvre cost; bypass geometry u_a, u_b; crossing time T_cross; decision costs J | 6, 14.1, 14.4 |
| **Optimisation (auction)** | CBBA marginal utility c_ij = γ^{\|b\|} max(0.01, W_p·prio − W_d·D − W_l·late) | 2 |
| **Graphs** | wait-for graph, DFS elementary-cycle enumeration; union-find standoff grouping; DRP deadlock groups | 5, 14.3, 16 |
| **Deadlock resolution** | stall watchdog; coordinator rank delay 3·r s; chain reverse req/rev; corridor length; frozen-zone lifetime; token; rung timeouts | 16 |
| **Learning (edge AI)** | congestion EWMA x_e ← 0.7x_e + 0.3·max(0, t_obs − t_nom − 1.5) | 3 |
| **Statistics** | mean, sample standard deviation, t-based 95 % CI, percentiles, throughput, utilisation | 14.9 |
| **Stochastic models** | Poisson arrivals Δt ~ Exp(λ); Gilbert-Elliott Markov channel | 14.9 |

### 15.2 By algorithm

| Algorithm | Equations used |
|---|---|
| CBBA | marginal utility, bundle discount γ, lane-graph distance d_G, lateness |
| Lane graph | lane bands (centre ± w/2), cell index k = ⌊s/0.5⌋, crossing core, rotation radius, keep-clear zone |
| A\* routing | c_move, c_rot, h(n), f = g + h, zone penalties, congestion term |
| Trajectory builder | trapezoid v_p, t_acc, t_dec, t_cruise; rotation ω_p, T_rot |
| Space-time claims | swept padded body; interval overlap; forbidden set F; earliest t\*; confirmation age ≥ 0.3 s |
| Precedence | occupancy > crossing core > occupier-first > key(⌊stamp/0.05⌋, −prio, id) |
| Livelock rules | shadow t_p; tentative stamp = now; follow if \|θ_p − θ_lane\| < 0.35 and \|v_p\| ≥ 0.15; patience 15 s / 8 s |
| WFG deadlock | DFS elementary cycles; persistence 4 s (feeds the DRP, §16) |
| Deadlock Resolution Protocol | stall if ‖Δp‖ < 0.3 m over 20 s; group = union-find(cycles ∪ stalled within 3 m); coordinator = min AMR no., rank delay 3r s; D1–D6 priority rules; chain reverse d_i = min{d ≥ 0: body(p_i − d·ĥ_i) ⊕ gap ∩ corridor = ∅} ≤ 3 m, depth ≤ 3; zone = corridor(3 m) ∪ swept back-offs; freeze 30 s; token 15 s; rung every 20 s; resolved when priority moved > 1.5 m |
| visison bypass | squeeze gap; T_lc; u_a, u_b; trapezoid approach; T_cross; F, t\*; J_bypass vs J_reroute; cooperative cost; passed() test |
| Chain reverse (library) | union-find; req₁ = F + g; rev_i = max(0, req_i + h_i − u_i); req_{i+1} = u_i + rev_i + h_i + g; escalation if rev > 6 m |
| MotionArbiter | τ̇ = clip(v/v_ref, 0, 1); lateness t − τ; LP limit gate |
| Unicycle tracker | v and ω laws, gains k_x = 1.5, k_y = 4.0, k_θ = 2.5, k_0 = 1.2 |
| ORCA-DD | P = (x, y) + D(cos θ, sin θ); Ṗ = R(θ)(v, Dω); ORCA half-planes; wheel-speed limit \|v ± ωb/2\| ≤ v_w; 2-D/3-D LP |
| Risk monitor | g, x = vt_r + v\|v\|/(2b), d_stop, m, σ = √(σ_p² + (σ_v t_stop)²), P = Φ(−m/σ) |
| α-β tracker | ŝ ← ŝ⁻ + αr, v̂ ← v̂ + (β/Δt)r, gate 1.0 m |
| Lidar safety supervisor | hard stop (0.28 m, ±12°); protect = 0.6·base + t_r\|v\| + v²/(2a_e); v_cap = √(2a_c(gap − base)); rotation sweep max(\|ω\|H, 15°), H = t_r + t_scan + \|ω\|/α; arc pose (v/ω)(sin ωt, 1 − cos ωt) |
| Latency compensation | θ_m = θ + ωL/2; x' = x + vL cos θ_m; y' = y + vL sin θ_m; θ' = θ + ωL; q = R(−ωL)(p − (vL, 0)) |
| Scan/pose alignment | linear interpolation of the pose history at the scan stamp |
| Collision recovery | wait 3 s; clear side from lidar band test; move 0.4 m at 0.25 m/s |
| Ray caster | slab test t_min, t_max, hit condition, noise σ = 0.01 m |
| Collision truth | SAT overlap; near miss gap < 0.10 m while closing; 2 s de-duplication |
| Congestion learner | EWMA with factor 0.3 |
| Gilbert-Elliott | p_gb, p_bg, π_B, mean loss L |
| Workload | Exponential inter-arrivals, rate λ |
| Benchmark statistics | x̄, s, t-CI, percentiles, throughput, utilisation |

---

## 16. Deadlock Resolution Protocol (DRP)

**Problem.** Robots in a deadlock used to resolve it *alone*. One robot backed off, and a neighbour immediately drove into the space it had just freed; nobody knew a resolution was in progress. Some stalls (a robot held by the lidar guard, a stuck manoeuvre, an obstacle plus a queue) never showed up as a wait-for cycle at all. The demo screenshot of 2026-09-29, robots jammed at the P07/P08 picker stops beside floor box `box_2`, is this failure.

**Why a protocol.** A deadlock is a *group* problem, so it needs a group decision, and every robot must know that decision and respect it. The DRP is:
- **decentralised:** no fleet server; one robot of the group coordinates;
- **deterministic:** every replica computes the same group, coordinator and plan from the same gossip;
- **non-intrusive:** robots only publish plans; each robot moves itself through its own MotionArbiter, so the single-writer rule holds.

Source: `core/nrda_fleet/deadlock.py`, `agent._drp_*`, `claims._wins`. Tests: none — `core/tests/test_deadlock.py` was deleted with the suite.

### 16.1 Protocol steps

| Step | What happens | Rule / equation |
|---|---|---|
| 1. Detect | A robot finds a wait-for cycle through itself that has persisted, **or** it is stalled | Cycle: DFS elementary-cycle enumeration on the gossiped wait-for edges, persistent ≥ 4 s. Stall: driving, not serving, and ‖p(t) − p(t_anchor)‖ < 0.3 m for 20 s |
| 2. Group | Cycle members plus stalled robots within 3 m | Union-find over the pairs (cycle edges, stalled robots within 3 m) |
| 3. Elect | The coordinator is the lowest AMR number in the group | Member of rank r waits 3·r s before taking over, so a silent coordinator is replaced |
| 4. Classify | The group is typed D1–D6 (table below) | The first matching rule wins (most constrained situation) |
| 5. Plan | Priority robot, ordered moves, frozen zone | Chain reverse (16.3), corridor of 3 m |
| 6. Publish | `AlertMsg(kind="drp_plan", detail=JSON)` on the peer bus | Epoch = coordinator : ⌊10 t⌋ |
| 7. Freeze | Every non-member treats the zone as a static block | Zone lifetime 30 s, or until `drp_done` |
| 8. Execute | Members move themselves: back-off, hold, re-route or pass | Deepest chain robot first; reversing at 0.3 m/s |
| 9. Close | The coordinator sends `drp_done` when the priority robot got through | Priority robot moved > 1.5 m since the plan |
| 10. Anti-relapse | The priority robot wins every contested cell of the old zone | Token (robot, t + 15 s), checked first in `claims._wins` |
| 11. Escalate | No progress in 20 s → next rung | Rung 1: rule of the type → rung 2: D6 rule → rung 3: re-route the lowest-precedence movable robot → rung 4: **operator alert** (`drp_escalated`, dashboard alarm), 60 s cool-down |

### 16.2 Deadlock types and their rules

| Type | Recognised by | Priority robot | What the others do | Why this rule |
|---|---|---|---|---|
| **D1 queue** | All members in one lane, same heading | The leader (furthest along the heading) | Followers **hold**: they drop their bookings and stand still for 10 s | The leader can always move if nobody behind claims its road; reversing a queue gains nothing |
| **D2 head-on / lane borrow** | One line, opposite lanes or headings, or a member mid-bypass | The bypasser (the robot between lanes), else the best precedence | In normal aisles: robots in its way **back off** (chain reverse). In **single-file corridors** ($w < 1.05\text{ m}$): chain reverse cannot clear opposing traffic; opposing robot **re-routes** (`single_file_standoff`) | A robot between lanes can neither wait nor go back safely; in narrow corridors, reversing cannot provide lateral passing clearance, so dynamic rerouting is required |
| **D3 crossing** | A member's body is inside a crossing core | The core occupant | The others back off to their stop lines | A robot inside a crossing blocks every direction until it leaves |
| **D4 station doorway** | A member is in a station or its doorway | The occupant | Arrivals back off out of the keep-clear zone | The occupant's only way out is through the doorway |
| **D5 obstacle** | A member waits on an unmapped obstacle (lidar box, floor box) | The best-precedence other member | The robot facing the obstacle **re-routes**; its contested stretch is forbidden for 60 s | The obstacle will not move; waiting only spreads the jam |
| **D6 general cycle** | Anything else (≥ 3 robots, mixed lanes) | Best precedence among the non-losers | The **loser** (movable, lowest priority, highest AMR number) backs off, and so does anyone behind it | Breaking any one edge breaks a cycle; choose the cheapest robot that can actually move |

**Precedence key** (as in §4.2): `(−priority, AMR number)`; the smallest wins. A robot in a station is never chosen to back off.

### 16.3 Chain reverse (world-frame form)

The priority robot's corridor is its body swept along the next 3 m of its route. Let `ĥ_i` be yielder i's heading and `p_i` its position. It backs off the smallest distance that clears the corridor with a gap g = 0.3 m:

```
d_i = min { d ∈ {0, 0.2, 0.4, …} :  body(p_i − d·ĥ_i) grown by g/2  ∩  corridor = ∅ },     d_i ≤ 3.0 m
```

- **The path must be free:** the swept back-off path must be clear of static boxes (checked every 0.2 m).
- **The chain:** a robot j standing in that path or at the target pose is pushed first, recursively, so that it clears yielder i's swept path and target. This is visison's relation `req_{i+1} = u_i + rev_i + h_i + g` evaluated along each robot's own heading. The depth is limited to 3.
- **Order:** the deepest robot moves first (order 0), then the next.
- **Frozen zone** = corridor ∪ every swept back-off path.
- **Infeasibility:** d > 3 m, a static obstacle behind, a robot in a station, or a chain deeper than 3. The plan then falls back to re-routing the lowest-precedence movable member.

### 16.4 Messages (existing `AlertMsg`, no new ROS interface)

| kind | Sender | Content |
|---|---|---|
| `drp_plan` | Coordinator | JSON: epoch, coordinator, type, rung, priority, moves `[(robot, action, distance, order)]`, zone boxes `(cx, cy, L, W, yaw)`, members, expiry, reason |
| `drp_done` | Coordinator | Epoch. Detail `timeout` means the next rung follows |
| `drp_escalated` | Coordinator | Type, members, location. Shown as an alarm on the operator console |

### 16.5 Parameters

| Parameter | Value |
|---|---|
| Cycle persistence before acting | 4 s |
| Stall watchdog | < 0.3 m in 20 s while driving |
| Group radius (stalled robots) | 3 m |
| Coordinator hand-over | 3 s per rank |
| Corridor kept free | 3 m of the priority robot's route |
| Gap after back-off / step / max back-off / chain depth | 0.3 m / 0.2 m / 3.0 m / 3 |
| Back-off speed | 0.3 m/s |
| Frozen-zone lifetime | 30 s |
| Rung timeout | 20 s |
| Anti-relapse token | 15 s |
| Operator escalation cool-down | 60 s |
