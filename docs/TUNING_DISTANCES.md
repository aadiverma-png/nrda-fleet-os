# Tuning reference: distances, times and thresholds

Current values after the 2026-10-18 tuning pass and the same-day revert of seven of them
(see **Reverted** at the end for what was rolled back and why). Every row is the value **in force now** —
look here first when you need a number instead of grepping the source.

Robot: AMR-01, length **1.20 m**, width **0.78 m**, `v_cruise` **1.8 m/s**, `e_decel` **1.6 m/s²**.

---

## Distances (m)

| Parameter | Value | File | What it governs |
|---|---|---|---|
| `pad_long` | **0.30 m** | `scheduler.py:122` | Road reserved ahead of the body. **Floor** — below 0.30 a robot at 1 m/s stops reaching into the 0.5 m cell it is entering, so it no longer reserves the road ahead |
| `pad_lat` | 0.05 m | `scheduler.py` | Lateral claim margin |
| `pad_rear` | 0.05 m | `scheduler.py` | Road kept behind a moving robot |
| `min_gap` | **0.15 m** | `agent.py:143` | Bumper gap required after both robots stop. 1.9σ of lidar noise (σ = 0.08 m) |
| `CORRIDOR` | **2.6 m** | `deadlock.py:38` | Priority robot's path kept free during a deadlock. Stays above the 2.4 m two-lane crossing band so it cannot stall inside an intersection |
| `arrive_tol` | **0.15 m** | `agent.py:87` | How close counts as arrived at a leg end. 0.20 left too much body in the lane at a narrow REST bay |
| `GAP` | 0.3 m | `deadlock.py:34` | Free gap left between robots after a back-off |
| `MAX_REVERSE` | 3.0 m | `deadlock.py:35` | Most any robot is asked to back off (DRP) |
| `MAX_REVERSE` (L2b) | 6.0 m | `standoff.py` | In-aisle chain reverse. Larger than the DRP's: the distance is set by the rejoin creep's ~4.8 m reach |
| `STALL_M` | 0.3 m | `deadlock.py:43` | Movement below this counts as no progress |
| `CELL` | 0.5 m | `lane_graph.py:47` | Lane cell granularity. Not reduced — compute scales inversely |
| `CORE_PAD` | 0.10 m | `lane_graph.py:66` | Crossing no-stop zone beyond the lane band |

### Safety layer — unchanged, do not tune

| Parameter | Value | File | Why it stays |
|---|---|---|---|
| `hard_stop` | 0.28 m | `safety.py:52` | Unconditional bumper stop. Fires only below 0.65 m/s; at 0.60 m/s the robot needs 0.20 m to stop, leaving 0.08 m residual |
| `hard_arc` | 12° | `safety.py:53` | Half of the 24° hard-stop cone. Width, not a distance — narrowing buys no speed |
| `base` | 0.08 m | `safety.py:48` | Always free in front of the bumper (0.02 m while docking) |
| `side` | 0.06 m | `safety.py:49` | Lateral margin of the protective fields |

Protective field = `0.6·base + reaction·v + v²/(2·e_decel)` → **1.33 m at 1.8 m/s**,
4.8× further out than `hard_stop`. This is what actually prevents collisions at speed.

---

## Times (s)

| Parameter | Value | File | What it governs |
|---|---|---|---|
| `tau` (scheduler) | **0.6 s** | `agent.py:62`, `scheduler.py:121` | Time margin around every claim. Applied **both sides**, so it costs `2·tau·v` = 2.16 m of lane at cruise |
| `window` | **8.0 s** | `agent.py:61` | Planning / reservation horizon. **Floor** — shorter means myopic routing and *more* deadlocks |
| `plan_period` | **0.33 s** | `agent.py:58` | Full replan cadence (3 Hz) |
| `state_period` | **0.15 s** | `agent.py:60` | Peer gossip broadcast (6.7 Hz) |
| `control_dt` | 0.05 s | `agent.py:57` | Control tick (20 Hz) — arbiter and safety every tick |
| `bid_period` | 0.5 s | `agent.py:59` | CBBA auction round |
| `bypass_trigger` | **2.5 s** | `agent.py:72` | Waiting on a stopped robot before L2 bypass is asked. Deliberately below `deadlock_persist` so L2 beats the DRP to it |
| `deadlock_persist` | 4.0 s | `agent.py:86` | A wait cycle must persist this long before the DRP acts |
| `reroute_wait` | **12.0 s** | `agent.py:71` | Scheduled waiting on a stopped peer before re-routing |
| `patience` | **15.0 s** | `agent.py:75` | Waiting on bookings before driving on bodies + lidar alone. The impatient window that follows ignores the bookings of *standing* peers, so a short value wedges robots into occupied bays |
| `peer_timeout` | 3.0 s | `agent.py:77` | No gossip for this long → peer counts as silent |
| `RUNG_S` | **20.0 s** | `deadlock.py:41` | No progress before the DRP escalates a rung. Mechanism needs ~6.5 s, but `_back_off` retries when blocked — at 12.0 s, 16 of 19 plans timed out |
| `STALL_S` | **20.0 s** | `deadlock.py:46` | No-progress watchdog. At 12.0 s legitimate waits were declared stalled, opening DRP episodes that were not needed |
| `FREEZE_S` | **20.0 s** | `deadlock.py:39` | DRP frozen-zone lifetime (blocks non-members) |
| `TOKEN_S` | 15.0 s | `deadlock.py:42` | Anti-relapse precedence for the priority robot |
| ORCA `tau` | **2.0 s** | `agent.py:135` | Avoidance horizon = 3.60 m of closing at cruise |
| `reaction` | 0.15 s | `safety.py:50` | Safety-layer reaction time — do not tune |
| `scan_age` | 0.1 s | `safety.py:54` | Worst-case age of the newest lidar scan |
| `RETRY_S` (L2b) | 6.0 s | `standoff.py` | Between chain-reverse attempts |
| `PLAN_TTL` (L2b) | 20.0 s | `standoff.py` | How long a broadcast chain-reverse plan stays valid |
| Station `dwell` | 5–10 s | `site.py:44` | Nominal service time. Does **not** trip `STALL_S` (watchdog runs only while driving) |

---

## Thresholds and counts

| Parameter | Value | File | What it governs |
|---|---|---|---|
| `p_brake` | **0.25** | `emergency.py` (default) | Collision probability that triggers a risk brake |
| `p_warn` | 0.05 | `emergency.py` | Warning level |
| `p_hard` | 0.8 | `emergency.py` | Always brake at or above this, whatever ORCA says |
| `station_wip` | **4** | `agent.py:87` | Robots working towards one station: one served + three queued |
| `sigma_pos` | 0.08 m | `agent.py:141` | Lidar position noise — sets the `min_gap` floor |
| `sigma_vel` | 0.15 m/s | `agent.py:141` | Lidar velocity noise |
| `CHAIN_DEPTH` | 3 | `deadlock.py:37` | Robots pushed behind a yielder |
| `MAX_ATTEMPTS` (L2b) | 3 | `standoff.py` | Chain-reverse tries before L2 holds and L3 re-routes |
| `decel_cap` | 0.8 m/s² | `safety.py:51` | Deceleration used for the warning-field speed cap |

---

## Inert — changing these does nothing

| Parameter | Where | Why |
|---|---|---|
| `headway` 0.6 | `agent.py:133` OrcaConfig | `solve()` reads only `cfg.tau`; read elsewhere only by the unwired `EmergencyCoordinator` |
| `neighbour_range` 6.0 | `agent.py:133` OrcaConfig | The agent hardcodes its own cutoff (`36.0` m² = 6 m) |
| `radius_margin` 0.1 | `OrcaConfig` default | The agent passes its own `0.02` to `effective_radius` |
| `cascade_range` 12.0 m | `EmergencyConfig` | Only `EmergencyCoordinator` reads it, and that is not wired into the runtime |

---

## Derived figures

| Quantity | Value | How it is derived |
|---|---|---|
| Lane reserved per robot | **3.66 m** | `body 1.20 + pad_long 0.30 + 2·tau·v (2·0.6·1.8 = 2.16)` |
| Throughput after tuning | **230 tasks/h** | measured 2026-10-18, up from 113/h (×2.03) |
| Protective field at cruise | **1.33 m** | `0.6·0.08 + 0.15·1.8 + 1.8²/(2·1.6)` |
| Stopping distance at cruise | **1.28 m** | `0.15·1.8 + 1.8²/(2·1.6)` |
| `hard_stop` crossover speed | **0.65 m/s** | Below this `hard_stop` fires first; above it the protective field does |
| Two-lane crossing band | **2.4 m** | `2 × lane width 1.2` — the floor for `CORRIDOR` |
| DRP escalation cycle | **60 s** | `3 rungs × RUNG_S 20 s` |

---

## Reverted on 2026-10-18 (kept here so nobody re-applies them)

Seven values were cut in the tuning pass and rolled back the same day after a Gazebo run.
Fleet-side deadlock resolution fell from **54 % to 11 %** (16 of 19 plans timed out) and robots
wedged themselves in the REST bays. Throughput (×2.03) and the compute win came from the
*other* changes, which were kept.

| Parameter | Tried | Reverted to | What went wrong |
|---|---|---|---|
| `RUNG_S` | 12.0 s | **20.0 s** | 16 of 19 DRP plans timed out |
| `patience` | 8.0 s | **15.0 s** | Impatient mode (ignores standing peers' bookings) ran ~50 % of waiting time; robots drove into occupied REST bays |
| `arrive_tol` | 0.20 m | **0.15 m** | Body protruded into the lane at narrow bays |
| `STALL_S` | 12.0 s | **20.0 s** | False stalls inflated DRP episodes (19 plans for 6 deadlocks) |
| `p_brake` | 0.35 | **0.25** | Reverted with the braking group; no isolated evidence either way |
| ORCA `tau` | 1.2 s | **2.0 s** | Reverted with the braking group |
| `min_gap` | 0.12 m | **0.15 m** | Reverted with the braking group |
