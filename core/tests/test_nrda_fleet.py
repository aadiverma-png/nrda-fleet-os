"""NRDA-FA integration layer tests (site, lane graph, claims, scheduler, agents, safety)."""

import math

import numpy as np
import pytest

from nrda_fleet.agent import AgentConfig, EdgeAgent
from nrda_fleet.bus import ClaimMsg, ControlMsg
from nrda_fleet.claims import ClaimBook, precedence_key
from nrda_fleet.ids import id_key, parse_robot_id, robot_id
from nrda_fleet.lane_graph import LaneGraph
from nrda_fleet.safety import SafetySupervisor
from nrda_fleet.scheduler import Scheduler
from nrda_fleet.sim2d import DynObstacle, Sim2D
from nrda_fleet.site import Site
from nrda_fleet.tasks import Task
from visison.motion_models import Command


@pytest.fixture(scope="module")
def site():
    return Site.load()


@pytest.fixture(scope="module")
def graph(site):
    return LaneGraph.load_or_build(site)


# ------------------------------------------------------------------ ids / site


def test_robot_ids():
    assert robot_id(7) == "AMR_7" and parse_robot_id("AMR_20") == 20
    assert sorted(["AMR_10", "AMR_2", "AMR_1"], key=id_key) == ["AMR_1", "AMR_2", "AMR_10"]
    with pytest.raises(ValueError):
        parse_robot_id("amr_01")


def test_roster_is_twenty_amrs_with_ten_plus_chargers(site):
    assert list(site.robots) == [f"AMR_{i}" for i in range(1, 21)]
    # at least 10 chargers in the building: dockable ones plus showcase towers (batteries do not drain)
    showcase = sum("showcase" in a for a in site.adjustments)
    assert len(site.stations_of("charger")) + showcase >= 10


def test_spawn_and_station_poses_clear_of_obstacles(site):
    for name, st in site.stations.items():
        if st.approach == "lane":
            continue
        assert not site.body_hits(st.x, st.y, st.yaw, 0.0, st.ignore), name


# ------------------------------------------------------------------ lane graph


def test_every_station_reachable_from_every_charger(graph):
    chargers = [n for n, node in graph.stations.items() if node.station.kind == "charger"]
    for name, node in graph.stations.items():
        c = graph.stations[chargers[0]]
        start = (c.maneuver.root, c.maneuver.exit_heading)
        goal_h = node.maneuver.root_heading if node.maneuver else node.heading
        assert graph.route(start, node.root, goal_h) is not None, name


def test_lanes_are_clear_of_racks_with_margin(site, graph):
    amr = site.amr
    for es in graph.edges.values():
        for e in es:
            if e.kind != "move":
                continue
            a, b = graph.points[e.src[0]], graph.points[e.dst[0]]
            th = math.atan2(b.y - a.y, b.x - a.x)
            for f in (0.0, 0.5, 1.0):
                x, y = a.x + (b.x - a.x) * f, a.y + (b.y - a.y) * f
                assert not site.index.box_hits(site.body(x, y, th), 0.04), (e.src, e.dst)
    assert all(ln.w >= amr.width / 2 + 0.1 for ln in graph.lines.values())


def test_cover_uses_swept_lane_bands(graph):
    ln = graph.lines["H3"]
    x, y = ln.lane_point("A", -12.45)
    res = graph.cover(x, y, 0.0)
    assert any(k == "H3/A" for k, _ in res)
    assert not any(k == "H3/B" for k, _ in res)  # the oncoming lane is not blocked by a robot in its own lane


# ------------------------------------------------------------------ claims


def _snap(rid, stamp, claims, prio=2, occupied=()):
    return ClaimMsg(rid, stamp, int(stamp * 10), prio, claims, list(occupied))


def test_claim_precedence_older_wins_then_priority_then_id():
    assert precedence_key(1.0, 2, "AMR_9") < precedence_key(2.0, 4, "AMR_1")
    assert precedence_key(1.0, 4, "AMR_9") < precedence_key(1.01, 2, "AMR_1")
    assert precedence_key(1.0, 2, "AMR_2") < precedence_key(1.0, 2, "AMR_10")


def test_two_replicas_agree_on_the_winner():
    a, b = ClaimBook("AMR_1"), ClaimBook("AMR_2")
    a.now = b.now = 5.0
    a.set_route_claims([(("H3/A", 10), 6.0, 8.0)], 4.0)
    b.set_route_claims([(("H3/A", 10), 7.0, 9.0)], 4.5)
    a.on_msg(b.snapshot(5.0), 5.0)
    b.on_msg(a.snapshot(5.0), 5.0)
    assert a.blocking(("H3/A", 10)) == []  # AMR_1 wins: it ignores the losing claim
    assert b.blocking(("H3/A", 10))  # AMR_2 must respect AMR_1
    assert b.lost() and not a.lost()


def test_silent_peer_keeps_only_its_body_blocked():
    a = ClaimBook("AMR_1", peer_timeout=3.0)
    a.on_msg(_snap("AMR_5", 0.0, [("H3/A", 12, 0.0, 30.0, 0.0)], occupied=[("H3/A", 11)]), 0.0)
    a.now = 10.0
    assert a.blocking(("H3/A", 12), now=10.0) == []
    assert a.blocking(("H3/A", 11), now=10.0) == [("AMR_5", -math.inf, math.inf)]


# ------------------------------------------------------------------ scheduler


def test_scheduler_waits_at_a_legal_hold_point(graph):
    book = ClaimBook("AMR_1")
    ln = graph.lines["H3"]
    # a peer holds a stretch of H3/A for 20 s
    cells = [("H3/A", k, 0.0, 20.0, 0.0) for k in range(30, 36)]
    book.on_msg(_snap("AMR_2", 0.0, cells), 0.0)
    book.now = 0.0
    sch = Scheduler(graph, book)
    x0, y0 = ln.lane_point("A", -24.0)
    x1, y1 = ln.lane_point("A", -8.0)
    from nrda_fleet.trajectory import Leg
    plan = sch.plan(0.0, x0, y0, 0.0, 0.0, [Leg("move", x1, y1)])
    assert plan.waits and plan.waits[0].blockers == ("AMR_2",)
    hold_x = x0 + plan.waits[0].u
    assert not graph.in_intersection(hold_x, y0, 0.0)
    for res, s_, e_ in plan.claims:
        if res in {(k, i) for k, i, *_ in cells}:
            assert s_ >= 20.0 - 1e-6 or e_ <= 0.0


def test_scheduler_holds_outside_aisle_for_peer_conflict(graph):
    from nrda_fleet.scheduler import _geometry, _pose_at
    from nrda_fleet.trajectory import Leg
    book = ClaimBook("AMR_1")
    ln_v = graph.lines["V04"]
    book.on_msg(_snap("AMR_2", 0.0, [("V04/A", 10, 0.0, 30.0, 0.0)]), 0.0)
    book.now = 0.0
    sch = Scheduler(graph, book)
    ln_hb = graph.lines["HB"]
    pt_hb = ln_hb.lane_point("A", ln_v.center - 6.0)
    pt_entry = ln_hb.lane_point("A", ln_v.center)
    pt_goal = ln_v.lane_point("A", -10.0)
    legs = [Leg("move", pt_entry[0], pt_entry[1]), Leg("rotate", th1=ln_v.lane_heading("A")), Leg("move", pt_goal[0], pt_goal[1])]
    plan = sch.plan(0.0, pt_hb[0], pt_hb[1], 0.0, 0.0, legs)
    assert plan.waits and plan.waits[0].blockers == ("AMR_2",)
    geo = _geometry(pt_hb[0], pt_hb[1], 0.0, legs)
    hx, hy, _, _, _ = _pose_at(geo, plan.waits[0].u)
    assert not graph.in_aisle(hx, hy), f"Hold point ({hx:.2f}, {hy:.2f}) must not be inside aisle V04"


# ------------------------------------------------------------------ command authority (FINAL_ALGORITHM 5)


def test_safety_supervisor_only_lowers_commands(site):
    sup = SafetySupervisor(site.amr)
    rng = np.random.default_rng(3)
    for _ in range(300):
        pts = rng.uniform(-4, 4, size=(60, 2))
        cmd = Command(float(rng.uniform(-0.4, 1.2)), float(rng.uniform(-1, 1)))
        out = sup.filter(cmd, pts)
        assert abs(out.cmd.v) <= abs(cmd.v) + 1e-9 and abs(out.cmd.w) <= abs(cmd.w) + 1e-9


def test_lidar_hard_stop_kept(site):
    sup = SafetySupervisor(site.amr)
    pts = np.array([[site.amr.length / 2 + 0.2, 0.0]] * 5)  # 0.2 m ahead of the bumper
    out = sup.filter(Command(0.05, 0.0), pts)
    assert out.brake and "hard stop" in out.reason


def _agent(graph, rid="AMR_1"):
    sent = []
    a = EdgeAgent(rid, graph, __import__("nrda_fleet.allocation", fromlist=["x"]).StationDistances(graph),
                  sent.append, AgentConfig(), graph.site.robots[rid], 1.0)
    return a, sent


def test_single_cmd_writer_operator_estop_overrides_everything(graph):
    a, _ = _agent(graph)
    x, y, th = a.pose
    a.tick(0.1, (x, y, th), 0.5, 0.0)
    a.on_message(ControlMsg(0.2, "estop"))
    cmd = a.tick(0.3, (x, y, th), 0.5, 0.0)
    assert cmd.v < 0.5  # braking, whatever the planner wants
    assert a.arbiter.holder(0.3).name == "OPERATOR"


def test_directive_does_not_actuate_other_robots(graph):
    a, _ = _agent(graph, "AMR_1")
    before = a.arbiter.active(0.0)
    a.on_message(ControlMsg(0.1, "estop", "AMR_2"))  # addressed to someone else
    assert a.arbiter.active(0.1) is before and not a.estop


def test_safe_point_rule_blocks_reallocation_while_docking(graph):
    a, _ = _agent(graph, "AMR_12")  # parked: its exit starts with an undock leg
    a._go("P09", "to_pickup")
    assert a.legs[0].tag == "undock"
    assert not a._at_safe_point()


# ------------------------------------------------------------------ simulation


def test_three_robots_deliver_without_collisions(site, graph):
    sim = Sim2D(site, graph, AgentConfig(), robots=3, seed=2, lidar=True)
    sim.add_tasks([Task("T1", "P03", "packing_induct_2", 2, 1.0, 900), Task("T2", "P09", "drop_1", 2, 2.0, 900)])
    sim.run(400, until_done=True)
    assert all(t.status == "completed" for t in sim.tasks.values())
    assert not [e for e in sim.events if e.level == "COLLISION"]


def test_lidar_obstacle_triggers_bypass(site, graph):
    sim = Sim2D(site, graph, AgentConfig(), robots=1, seed=1, lidar=True)
    sim.add_tasks([Task("T1", "P09", "drop_1", 2, 1.0, 900)])
    sim.run(6.0)
    # drop an unmapped pallet in the middle of the longest straight lane stretch of the planned route,
    # inside the robot's own lane (the oncoming lane stays free for the bypass)
    a = sim.robots["AMR_1"].agent
    x0, y0 = a.pose[0], a.pose[1]
    best = None
    for lg in a.legs:
        if lg.kind != "move":
            continue
        n = math.hypot(lg.x1 - x0, lg.y1 - y0)
        if (best is None or n > best[0]) and lg.tag not in ("dock", "undock"):
            best = (n, (x0 + lg.x1) / 2, (y0 + lg.y1) / 2, abs(lg.x1 - x0) > abs(lg.y1 - y0))
        x0, y0 = lg.x1, lg.y1
    assert best is not None and best[0] > 5.0
    _, ox, oy, along_x = best
    sim.obstacles.append(DynObstacle("obs_1", ox, oy, 1.1 if along_x else 0.9, 0.9 if along_x else 1.1, 0, 1e9))
    sim.run(300, until_done=True)
    a = sim.robots["AMR_1"].agent
    assert sim.tasks["T1"].status == "completed"
    assert a.stats.bypasses >= 1
    assert not [e for e in sim.events if e.level == "COLLISION"]


def test_simulation_is_deterministic(site, graph):
    def run():
        sim = Sim2D(site, graph, AgentConfig(), robots=2, seed=5, lidar=False)
        sim.add_tasks([Task("T1", "P07", "drop_2", 2, 1.0, 900)])
        sim.run(60)
        return [(round(r.x, 6), round(r.y, 6)) for r in sim.robots.values()]
    assert run() == run()


def test_robot_recovers_by_itself_after_a_collision_pause(site, graph):
    sim = Sim2D(site, graph, AgentConfig(), robots=1, seed=1, lidar=True)
    sim.add_tasks([Task("T1", "P09", "drop_1", 2, 1.0, 900)])
    sim.run(25.0)
    a = sim.robots["AMR_1"].agent
    assert a.mode == "to_pickup"
    a.on_message(ControlMsg(sim.t, "pause", "AMR_1", "collision E00001"))  # what the safety monitor sends
    sim.run(1.0)
    assert a.paused
    sim.run(4.0)
    assert not a.paused and any("collision recovery" in e for _t, e in a.events)
    sim.run(300, until_done=True)
    assert sim.tasks["T1"].status == "completed"


def test_operator_pause_is_never_auto_released(site, graph):
    sim = Sim2D(site, graph, AgentConfig(), robots=1, seed=1, lidar=True)
    sim.add_tasks([Task("T1", "P09", "drop_1", 2, 1.0, 900)])
    sim.run(25.0)
    a = sim.robots["AMR_1"].agent
    a.on_message(ControlMsg(sim.t, "pause", "AMR_1", ""))
    sim.run(10.0)
    assert a.paused


def test_follower_queues_behind_a_robot_served_at_its_own_station(site, graph):
    from nrda_fleet.bypass import _stopped_blocker
    from nrda_fleet.scheduler import Wait
    sim = Sim2D(site, graph, AgentConfig(), robots=2, seed=1, lidar=False)
    sim.add_tasks([Task("T1", "P09", "drop_1", 2, 1.0, 900)])
    a = where = None
    for _ in range(120):  # until a robot drives in a lane towards its goal
        sim.run(1.0)
        a = next((r.agent for r in sim.robots.values() if r.agent.goal and r.agent.mode == "to_pickup"
                  and graph.line_of(*r.agent.pose) is not None), None)
        if a is not None:
            where = graph.line_of(*a.pose)
            break
    assert a is not None, "no robot reached a lane on its way to the pick station"
    line, lane = where
    other = next(r for r in sim.robots if r != a.rid)
    x, y, th = a.pose
    p = a.peers[other]
    p.x, p.y, p.th, p.v, p.mode = x + 2.0 * math.cos(th), y + 2.0 * math.sin(th), th, 0.0, "loading"
    a.peer_heard[other] = a.t
    a.plan.waits = [Wait(0.0, a.t + 5.0, (line.lane_key(lane), 0), (other,), a.t)]
    p.station = "P03" if a.goal != "P03" else "P04"  # served elsewhere: a stopped obstacle we may bypass
    assert _stopped_blocker(a, line, lane) is not None
    p.station = a.goal  # served at our own goal: queue, never overtake
    assert _stopped_blocker(a, line, lane) is None


def test_queue_for_an_occupied_bay_waits_outside_its_exit(site, graph):
    """A dock with a manoeuvre (sleeping bay, charger) is a dead end: a robot queued for it must wait clear of
    the turning circle and the exit path of the robot inside (AMR_18 could not leave in the 2026-09-30 demo)."""
    sim = Sim2D(site, graph, AgentConfig(), robots=1, seed=1, lidar=False)
    a = sim.robots["AMR_1"].agent
    for src, dst in (("P10", "REST_E1"), ("P14", "charger_A1"), ("P07", "REST_W8")):
        s0, node = graph.stations[src], graph.stations[dst]
        path = graph.route((s0.maneuver.root, s0.maneuver.exit_heading) if s0.maneuver else (s0.root, s0.heading),
                           node.root, node.maneuver.root_heading, lambda e: 0.0)
        a.legs = graph.legs_for(path) + list(node.maneuver.entry)
        a.goal = dst
        p0 = graph.points[path[0][0]]
        a._leg_start, a.pose = (p0.x, p0.y), (p0.x, p0.y, 0.0)
        k = next(i for i, lg in enumerate(a.legs) if lg.tag == node.maneuver.root)
        cut = a._queue_legs(k, node.maneuver.root, dst)
        moves = [lg for lg in cut if lg.kind == "move"]
        root = graph.points[node.maneuver.root]
        if not moves:
            continue  # no legal spot on this approach: the robot holds where it is
        last = moves[-1]
        assert math.hypot(last.x1 - root.x, last.y1 - root.y) > site.amr.length, (dst, last)
        for th in (0.0, math.pi / 2):
            if not graph.in_intersection(last.x1, last.y1, th):
                break
        else:
            raise AssertionError((dst, last))


def test_pack_and_drop_are_drive_through_lane_stops(site, graph):
    """Packing and the outbound drops are stops in a one-way lane (VP, VE2): no manoeuvre, never inside a
    crossing, and a robot can drive in, through and on to a sleeping bay."""
    for name in ("packing_induct_1", "packing_induct_2", "packing_induct_3", "drop_1", "drop_2", "drop_3"):
        node = graph.stations[name]
        assert node.maneuver is None and node.root == f"stop:{name}", name
        st = node.station
        assert not graph.in_intersection(st.x, st.y, st.yaw), name
        assert not site.body_hits(st.x, st.y, st.yaw, 0.05), name
        into = graph.route((graph.stations["P09"].root, graph.stations["P09"].heading), node.root, node.heading,
                           lambda e: 0.0)
        bay = graph.stations["REST_E4"]
        out = graph.route((node.root, node.heading), bay.root, bay.maneuver.root_heading, lambda e: 0.0)
        assert into is not None and out is not None, name
    assert {graph.stations[f"packing_induct_{i}"].station.x for i in (1, 2, 3)} == {26.7}
    assert AgentConfig().station_wip == 4  # one robot served + three queued per dock


def test_console_risk_and_incidents():
    """The console's Deadlocks and collisions section: live risk, incidents and their solved tick."""
    import json
    from nrda_fleet.bus import AlertMsg, RobotStateMsg, SafetyEventMsg
    from nrda_fleet.metrics import FleetLedger
    L = FleetLedger()

    def st(rid, t, x, y, th=0.0, v=0.0, wf=(), mode="to_drop"):
        m = RobotStateMsg(rid, t, x, y, th, v, 0.0, mode)
        m.waiting_for = list(wf)
        L.on_message(m, t)
    for t in (0.0, 10.0):  # AMR_1 <-> AMR_2 wait loop, AMR_3 and AMR_4 head-on at 1 m/s each, 3 m apart
        st("AMR_1", t, 0, 0, wf=["AMR_2"]); st("AMR_2", t, 2, 0, wf=["AMR_1"])
        st("AMR_3", t, 10, 0, 0.0, 1.0); st("AMR_4", t, 13, 0, math.pi, 1.0)
    risk = L.risk()
    kinds = {r["kind"]: r for r in risk}
    assert kinds["deadlock"]["p"] == 1.0 and kinds["deadlock"]["robots"] == ["AMR_1", "AMR_2"]
    assert kinds["collision_risk"]["p"] > 0.9 and kinds["collision_risk"]["robots"] == ["AMR_3", "AMR_4"]
    # a coordinated resolution opens a deadlock incident and ticks it when done
    plan = json.dumps({"epoch": "AMR_1:100", "kind": "D6", "rung": 1, "members": ["AMR_1", "AMR_2"]})
    L.on_message(AlertMsg("AMR_1", 10.0, "drp_plan", "AMR_2", 1, 0, 1.0, plan), 10.0)
    inc = L.incidents_view()[-1]
    assert inc["kind"] == "deadlock" and inc["status"] == "open"
    L.on_message(AlertMsg("AMR_1", 14.0, "drp_done", "AMR_1:100", 0, 0, 1.0, "D6"), 14.0)
    assert L.incidents_view()[-1]["status"] == "solved"
    # a collision stays open until the robots have separated
    L.on_message(SafetyEventMsg("E1", 15.0, "COLLISION", ["AMR_3", "AMR_4"], "AMR_4", 11.5, 0.0), 15.0)
    assert L.incidents_view()[-1]["status"] == "open"
    st("AMR_3", 20.0, 9.0, 0, math.pi, 0.3); st("AMR_4", 20.0, 14.0, 0, 0.0, 0.3)
    assert L.incidents_view()[-1]["status"] == "solved"


def test_a_stopped_leader_does_not_claim_the_cell_of_its_follower(site, graph):
    """A robot standing in a lane pads the road ahead of it, never behind: the cell behind it belongs to the
    robot queued there, and claiming it made the leader wait for its own follower for ever (2026-09-30)."""
    from nrda_fleet.claims import ClaimBook
    from nrda_fleet.scheduler import Scheduler
    ln = graph.lines["VE2"]
    sch = Scheduler(graph, ClaimBook("AMR_1"))
    x, y = ln.lane_point("A", 3.67)
    th = ln.lane_heading("A")
    body = graph.cover(x, y, th, 0.02, 0.02)
    stopped = sch._cover_moving(x, y, th, 0.0)
    ahead = {c for c in graph.cover(x + 0.4 * math.cos(th), y + 0.4 * math.sin(th), th, 0.02, 0.02) if c not in body}
    behind = {c for c in graph.cover(x - 0.4 * math.cos(th), y - 0.4 * math.sin(th), th, 0.02, 0.02) if c not in body}
    assert body <= stopped and not (stopped & behind)  # the whole body, nothing behind it
    moving = sch._cover_moving(x, y, th, 1.0)
    assert moving & ahead and not (moving & behind)  # driving: road ahead reserved, road behind released


def test_a_route_that_ends_short_of_the_goal_is_planned_again(site, graph):
    """If the legs run out before the goal the robot must re-plan. Nothing else catches it: the stall watchdog
    only watches robots that still have legs, so AMR_9 stood at the decant stop for 450 s (2026-09-30)."""
    sim = Sim2D(site, graph, AgentConfig(), robots=1, seed=1, lidar=False)
    a = sim.robots["AMR_1"].agent
    sim.add_tasks([Task("T1", "P09", "drop_1", 2, 1.0, 900)])
    sim.run(20.0)
    assert a.goal and a.mode in ("to_pickup", "to_drop")
    a.legs = []                      # lose the route half way, as a skipped leg or a manoeuvre can
    a._replan_empty = -1e9
    gap = a._goal_gap()
    assert gap > a.cfg.arrive_tol + 0.1, "the robot must still be short of its goal for this test to mean anything"
    sim.run(1.0)
    assert a.legs, "the robot re-plans instead of standing there for ever"
    sim.run(120.0)
    assert a._goal_gap() < gap        # and it is making its way there again
