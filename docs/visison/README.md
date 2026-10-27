# Visison — reservation-aware obstacle bypass for AMRs

An autonomous mobile robot (R1) is driving along lane A of a two-lane
warehouse aisle when a box falls off a shelf and blocks its lane. Usually it has
two options: trace back and reroute (for example with PIBT), or turn around. Both
cost time and battery and break the timestamps other robots rely on.

Visison adds a third option. **Borrow lane B, pass the box, and return to lane A
before anyone who has reserved lane B needs that space.** It combines
closed-form kinematics with an exact search over the space-time reservation
table, and it picks one of these actions:

| action | when |
|---|---|
| `SQUEEZE` | the gap next to the box is wide enough to pass inside lane A |
| `BYPASS_NOW` | lane B is free long enough right now (e.g. R2 is 20 s away) |
| `WAIT_THEN_BYPASS` | lane B is busy now (e.g. R2 is 5 s away). R1 stops, waits for the earliest safe window `t*`, then bypasses |
| `REROUTE` | a reroute is cheaper, the wait is too long, or both lanes are blocked |
| `HOLD` | nothing is possible and no reroute exists. R1 waits and re-evaluates |

In every case R1 **reports the obstacle** to the fleet so a person can remove it.

```
 lane B  ◄── R2 ─────────────────────────────  (reserved space-time cells)
         ─────────────╮▁▁▁▁▁▁▁▁╭───────────────
 lane A  R1 ──────────╯  [box] ╰──────────────►
              u_a  change out  pass  change in  u_b
```

## Quickstart

```bash
./nrda.sh bench --quick                   # the library at twenty robots, in the 2-D benchmark
```

**In the fleet:** this library is layer L1/L2 of NRDA-FA. To see it in Gazebo with
20 robots, use `./nrda.sh sim` from the repository root.

It uses only the Python ≥ 3.10 standard library.

The two-lane `sim/aisle_sim.py` harness that once drove this library standalone has been
removed, and so has the test suite. Visison is now exercised only through the fleet:
`nrda_fleet` imports it in fourteen modules, and the 2-D benchmark (`./nrda.sh bench`)
runs it at twenty robots. There is no unit-level cover for it in this tree.

## Using the planner directly

```python
from visison import *

aisle = Aisle(lane_width=1.2, length=40.0)
spec = RobotSpec(Footprint(0.9, 0.6), MotionLimits(v_max=1.5, accel=0.8, decel=1.2),
                 QuinticLaneChange(max_lateral_accel=0.5), bypass_speed=1.0)
planner = BypassPlanner(aisle, spec)
table = InMemoryReservationTable()                # or your fleet manager's table

decision = planner.evaluate(
    RobotState("R1", s=8.0, v=1.5, lane="A"),
    Obstacle(20.0, 20.6, 0.1, 1.1),
    t_now=0.0,
    reservations=table,                           # any ReservationProvider
    reroute=RerouteEstimate(extra_time=45, extra_distance=60),
)
decision.action, decision.reason
decision.plan.trajectory.sample(t)                # reference pose for your tracker
decision.plan.claims                              # lane-B reservations to claim
```

## Integrating with a robot and fleet

`BypassController` runs the whole cycle:

1. It reports the obstacle and evaluates the options.
2. It claims the window atomically, and plans again if the claim is refused.
3. It shifts the robot's later reservations by the delay.
4. Until the lane change starts, every tick re-checks the window and perception. If either fails, it aborts safely.
5. When the robot is back in lane A, it releases the lane-B claims.

You implement four small protocols:

| protocol | module | what you provide |
|---|---|---|
| `RobotInterface` | `visison.adapters` | `state()`, `follow(trajectory)`, `stop()` |
| `ReservationProvider` | `visison.reservation` | `query`, atomic `try_reserve`, `release`, `shift` |
| `ReroutePlanner` (optional) | `visison.adapters` | `estimate()` → `RerouteEstimate`, `execute()` |
| `FleetReporter` (optional) | `visison.adapters` | `report_obstacle()`, `report_schedule_conflicts()` |

`InMemoryReservationTable` and `LoggingReporter` are reference
implementations. `core/nrda_fleet/bypass.py` is the worked example of the
wiring: it is how the fleet runtime drives this library on ROS 2.

```python
ctl = BypassController(robot, planner, reservations, reporter, rerouter)
ctl.on_obstacle(obstacle, t_now, other_lane_obstacles)   # from perception
ctl.tick(t_now)                                          # 10–50 Hz
```

Differential-drive and car-like robots use `QuinticLaneChange`. Mecanum and
omni robots use `HolonomicLaneChange`. Both robot directions (`direction=±1`)
are supported.

## When the schedule is wrong: emergency layer

Reservations can be wrong: a robot is early or late, or a timing was
miscalculated. Every robot rates its lidar tracks with a collision
probability (`visison/emergency.py`, MATH.md section 8). A high risk brakes **every robot involved**
(`visison/standoff.py`, section 9), and then the standoff is resolved:

| situation | resolution |
|---|---|
| a robot is stuck **between the lanes** | **chain reverse**: the robots in front of it back off exactly as far as needed, one after another, and it rejoins its lane |
| a robot was **about to** bypass | it gives way and plans again |
| a queue caught up with a stopped robot | the queue restarts, front first |
| an unknown object (e.g. a person) | hold until it has left the path |
| facing robots stopped for 3 s | deadlock → the same resolver |
| nothing works / timeout | escalate to the fleet / an operator |

It works for any number of robots (`EmergencyCoordinator.register`).

## ORCA-DD and one command path

- **`MotionArbiter`** (`visison/arbiter.py`): planner, fleet and emergency layer only send *intents*. One arbiter per robot turns them into the single motor command. It uses priorities, a time-scaled reference, a limit gate and a watchdog.
- **ORCA-DD** (`visison/orca.py`, `visison/motion_models.py`): reciprocal collision avoidance solved in the robot's *command space*. Every command respects the wheel, acceleration, turn-rate, turning-radius and reverse limits, for diff-drive, Ackermann and omni robots. In aisles it runs lane-locked (speed only) with a time headway.
- **Graded response**: small timing errors are absorbed by ORCA without an emergency brake.
- **Reciprocal windows**: oncoming robots of our own fleet slip by 1–5 s when that saves more than the bypasser would wait.

See MATH.md section 10 and [`plans/PLAN_A_ORCA.md`](plans/PLAN_A_ORCA.md). In the sim: `--orca`, `--coop`. In Gazebo: `orca:=true coop:=true`.

## How it decides

See [`MATH.md`](MATH.md) for the derivations and worked examples. In short:

* Lane-change time: `T_lc = √(10w / (√3·a_lat))`.
* Borrowed stretch: `[o₁ − c − ℓ/2 − v_b·T_lc, o₂ + c + ℓ/2 + v_b·T_lc]`.
* Each reservation `(r_s, r_e)` on cell `k` forbids start times in
  `(r_s − β_k − τ, r_e − α_k + τ)`. The earliest safe start `t*` is the first
  point outside the union of these intervals, found in O(n log n).
* The cost is the delay plus weighted energy, compared with the reroute cost
  plus a penalty for disturbing other robots' schedules.

## Layout

```
visison/   geometry, kinematics, intervals, reservation, planner, controller, adapters, reporting,
           emergency (collision risk, tracker), standoff (coordinator, chain reverse),
           arbiter (one command path), orca + motion_models (ORCA-DD), tracking
docs/      MATH.md, plans/PLAN_A_ORCA.md
```
