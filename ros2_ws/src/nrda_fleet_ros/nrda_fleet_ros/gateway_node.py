"""NRDA operations gateway: REST API + live stream + the web console (http://localhost:8080).

The gateway is an *observer and injector*, never a coordinator:
* it releases tasks (scenario workload or operator orders) on ``/fleet/tasks``; the robots
  allocate them among themselves with CBBA (or the one robot named by a direct assignment);
* it relays operator commands (pause, resume, e-stop, charge, fault, zone rules) on ``/fleet/control``;
* it injects obstacles (``/fleet/obstacles`` + a static pallet model in Gazebo);
* it shows the fleet: robot states, tasks, bids, KPIs and the independent safety monitor's events.

Pure standard library (``http.server`` + Server-Sent Events) so it runs on any ROS 2 install.
API: see ``docs/INTERFACES.md`` and ``GET /api``.
"""

from __future__ import annotations

import json
import math
import queue
import mimetypes
import os
import random
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.parameter import Parameter

from nrda_fleet import bus as B
from nrda_fleet.metrics import FleetLedger
from nrda_fleet.tasks import DROP_KINDS, PICKUP_KINDS, generate_workload
from nrda_msgs.msg import Obstacle, ObstacleList

from .common import QOS_LATCHED, ROS_TYPE, TOPIC, from_ros, load_site, qos_for, sim_now, to_ros

SCENARIOS = {  # tasks, rate per minute
    "none": (0, 0.0),
    "demo": (60, 6.0),
    "nominal": (120, 8.0),
    "busy": (240, 16.0),
}

PALLET_SDF = """<?xml version='1.0'?><sdf version='1.9'><model name='{name}'><static>true</static><link name='l'>
<collision name='c'><pose>0 0 0.35 0 0 0</pose><geometry><box><size>{L} {W} 0.7</size></box></geometry></collision>
<visual name='v'><pose>0 0 0.35 0 0 0</pose><geometry><box><size>{L} {W} 0.7</size></box></geometry>
<material><ambient>0.65 0.45 0.2 1</ambient><diffuse>0.65 0.45 0.2 1</diffuse></material></visual>
<visual name='stripe'><pose>0 0 0.71 0 0 0</pose><geometry><box><size>{L} 0.12 0.01</size></box></geometry>
<material><ambient>1 0.1 0.1 1</ambient><diffuse>1 0.1 0.1 1</diffuse></material></visual></link></model></sdf>"""


class Gateway(Node):
    # s a lidar-reported obstacle stays on the console without being mentioned again. The robots
    # re-report a real one as they pass it, so this only drops beliefs nobody is maintaining.
    FOUND_TTL = 120.0

    def __init__(self):
        super().__init__("fleet_gateway", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.port = int(self.declare_parameter("port", 8080).value)
        self.token = self.declare_parameter("api_token", "").value
        scenario = self.declare_parameter("scenario", "none").value
        seed = int(self.declare_parameter("seed", 1).value)
        self.run_dir = os.path.expanduser(self.declare_parameter("run_dir", "~/.nrda/runs").value)
        self.run_id = self.declare_parameter("run_id", time.strftime("%Y%m%d-%H%M%S")).value
        self.site, self.graph = load_site()
        self.ledger = FleetLedger()
        self.lock = threading.Lock()
        self._pubs = {cls: self.create_publisher(ROS_TYPE[cls], TOPIC[cls], qos_for(cls))
                      for cls in (B.TaskMsg, B.ControlMsg)}
        self.obs_pub = self.create_publisher(ObstacleList, "/fleet/obstacles", QOS_LATCHED)
        for cls in (B.RobotStateMsg, B.TaskMsg, B.TaskEventMsg, B.BidMsg, B.SafetyEventMsg, B.AlertMsg):
            self.create_subscription(ROS_TYPE[cls], TOPIC[cls], self._on_msg, qos_for(cls))
        self.obstacles: dict[str, Obstacle] = {}
        self.found: dict[str, dict] = {}  # obstacles the robots reported from their own lidars
        self.zone_rules: dict[str, str] = {}
        self._op_seq = 0
        self._sse_clients: list[queue.Queue] = []  # one outbox per browser (Server-Sent Events)
        self._snap_cache: tuple[float, bytes] = (0.0, b"")
        n, rate = SCENARIOS.get(scenario, SCENARIOS["demo"])
        self.pending = generate_workload(self.graph_stations(), n, rate, seed, t0=15.0) if n else []
        self.get_logger().info(f"scenario '{scenario}': {len(self.pending)} tasks at {rate}/min; "
                               f"console on http://localhost:{self.port}")
        self.create_timer(0.25, self._release)
        self.create_timer(10.0, self._save_run)
        self.web_root = os.path.join(get_package_share_directory("nrda_fleet_ros"), "web")
        self.site_json = dumps(self._site_payload())
        threading.Thread(target=self._serve, daemon=True).start()

    def graph_stations(self):
        return {name: node.station for name, node in self.graph.stations.items()}

    # ------------------------------------------------------------------ ROS side

    def _on_msg(self, m) -> None:
        msg = from_ros(m)
        with self.lock:
            self.ledger.on_message(msg, sim_now(self))
            if isinstance(msg, B.AlertMsg) and msg.kind in ("obstacle", "obstacle_cleared"):
                # Obstacles the robots found with their own lidars. Only operator-injected ones used to
                # reach the console, so a lane blocked fleet-wide by a lidar report showed nothing at all:
                # the robots routed around something the operator could not see.
                self._on_found_obstacle(msg)
        if isinstance(msg, B.SafetyEventMsg):
            self._broadcast({"type": "safety", "event": dict(msg.__dict__)})

    def _on_found_obstacle(self, msg) -> None:
        """Track a lidar-reported obstacle for the console (caller holds the lock)."""
        if msg.kind == "obstacle_cleared":
            self.found.pop(msg.target, None)
            return
        L = W = 1.0
        if "x" in msg.detail:
            a, _, b = msg.detail.partition("x")
            try:
                L, W = float(a), float(b)
            except ValueError:
                pass
        prev = self.found.get(msg.target)
        self.found[msg.target] = dict(name=msg.target, x=round(msg.x, 2), y=round(msg.y, 2),
                                      length=L, width=W, found_by=msg.sender,
                                      t=round(prev["t"] if prev else msg.stamp, 1),
                                      seen=round(msg.stamp, 1))
        # Each new sighting gets a fresh name from the reporting robot, and a report whose clear never
        # arrives (the robot that could verify it left the area) would otherwise sit on the map for the
        # rest of the run. Drop the ones nobody has mentioned for a while: a real obstacle is re-reported.
        cut = msg.stamp - self.FOUND_TTL
        for name in [n for n, o in self.found.items() if o["seen"] < cut]:
            self.found.pop(name, None)

    def _release(self) -> None:
        t = sim_now(self)
        if t <= 0:
            return
        while self.pending and self.pending[0].release <= t:
            task = self.pending.pop(0)
            self._publish(task.msg(t))

    def _publish(self, msg) -> None:
        self._pubs[type(msg)].publish(to_ros(msg))
        with self.lock:
            self.ledger.on_message(msg, sim_now(self))

    def snapshot_bytes(self) -> bytes:
        """JSON snapshot, shared by every browser and cached for 0.2 s of wall time."""
        now = time.monotonic()
        t, data = self._snap_cache
        if now - t > 0.2 or not data:
            data = dumps({"type": "snapshot", **self.snapshot()}).encode()
            self._snap_cache = (now, data)
        return data

    def _save_run(self) -> None:
        os.makedirs(self.run_dir, exist_ok=True)
        with self.lock:
            data = {"run_id": self.run_id, "backend": "gazebo",
                    "kpis": self.ledger.kpis(), "tasks": [r.as_dict() for r in self.ledger.tasks.values()],
                    "safety_events": [e.__dict__ for e in self.ledger.safety]}
        tmp = os.path.join(self.run_dir, f".{self.run_id}.json")
        with open(tmp, "w") as f:
            json.dump(data, f, indent=1, default=str)
        os.replace(tmp, os.path.join(self.run_dir, f"{self.run_id}.json"))

    # ------------------------------------------------------------------ views

    def snapshot(self, light: bool = False) -> dict:
        with self.lock:
            L = self.ledger
            robots = [dict(id=s.robot_id, x=round(s.x, 2), y=round(s.y, 2), th=round(s.th, 3), v=round(s.v, 2),
                           mode=s.mode, battery=round(s.battery, 3), task=s.task_id, goal=s.goal, health=s.health,
                           station=s.station, waiting_for=list(s.waiting_for), bypass=s.bypass,
                           compute_ms=s.compute_ms, stale=(L.t - s.stamp) > 3.0,
                           age=round(max(L.t - s.stamp, 0.0), 2),  # s since this robot's last report
                           path=[[round(float(p[0]), 2), round(float(p[1]), 2)] for p in s.path][:30],
                           safety=dict(L.per_robot.get(s.robot_id, {})))
                      for s in sorted(L.robots.values(), key=lambda s: int(s.robot_id.split("_")[1]))]
            tasks = [r.as_dict() for r in list(L.tasks.values())[-150:]]
            out = {"t": round(L.t, 2), "robots": robots, "kpis": L.kpis(),
                   "tasks": tasks, "log": list(L.events_log)[-40:],
                   "safety_recent": [dict(e.__dict__, acked=e.event_id in L.acked) for e in L.safety[-60:]],
                   "obstacles": [dict(name=o.name, x=o.x, y=o.y, length=o.length, width=o.width,
                                      source="injected") for o in self.obstacles.values() if o.active]
                                + [dict(o, source="lidar") for o in self.found.values()],
                   "zone_rules": dict(self.zone_rules), "timeline": list(L.timeline)[-120:],
                   "risk": L.risk(), "incidents": L.incidents_view()}
        return out

    def _site_payload(self) -> dict:
        boxes = [dict(name=nb.name, kind=nb.kind, c=[[round(x, 3), round(y, 3)] for x, y in nb.box.corners()])
                 for nb in self.site.boxes]
        lanes = []
        for es in self.graph.edges.values():
            for e in es:
                if e.kind == "move":
                    a, b = self.graph.points[e.src[0]], self.graph.points[e.dst[0]]
                    lanes.append([round(a.x, 2), round(a.y, 2), round(b.x, 2), round(b.y, 2)])
        stations = [dict(name=n, kind=node.station.kind, x=node.station.x, y=node.station.y, yaw=node.station.yaw)
                    for n, node in self.graph.stations.items()]
        return {"boxes": boxes, "lanes": lanes, "stations": stations, "zones": self.site.zones,
                "pickups": sorted(n for n, node in self.graph.stations.items() if node.station.kind in PICKUP_KINDS),
                "dropoffs": sorted(n for n, node in self.graph.stations.items() if node.station.kind in DROP_KINDS),
                "robots": list(self.site.robots), "bounds": [-41, -26, 41, 26],
                "floor_boxes": [dict(name=n, x=x, y=y, yaw=yaw, length=L, width=W)
                                for n, (x, y, yaw, L, W) in self.site.floor_boxes.items()],
                "disabled": [a for a in self.site.adjustments if "disabled" in a]}

    # ------------------------------------------------------------------ actions

    def create_tasks(self, body: dict) -> list[str]:
        if body.get("random"):  # "Release 10 random tasks": random stations and priorities, auctioned by the fleet
            body = {"pickup": "random", "dropoff": "random", "priority": "random", "count": int(body["random"])}
        pickup, dropoff = body.get("pickup"), body.get("dropoff")
        count = max(1, min(int(body.get("count", 1)), 50))
        random_prio = str(body.get("priority", 2)) == "random"
        prio = 2 if random_prio else int(body.get("priority", 2))
        assign = body.get("assign_to", "") or ""
        if assign and assign not in self.site.robots:
            raise ValueError(f"unknown robot {assign}")
        if assign:
            s = self.ledger.robots.get(assign)
            if s is not None and (s.mode in ("fault", "estop", "paused") or s.health not in ("HEALTHY", "DEGRADED")):
                raise ValueError(f"{assign} cannot take tasks now ({s.mode}, {s.health})")
            if s is not None and s.battery < 0.35:
                raise ValueError(f"{assign} battery too low ({int(s.battery * 100)}%)")
        ids = []
        rng = random.Random()
        t = sim_now(self)
        for _ in range(count):
            p = pickup if pickup and pickup != "random" else rng.choice(self._site_payload_cache()["pickups"])
            d = dropoff if dropoff and dropoff != "random" else rng.choice(self._site_payload_cache()["dropoffs"])
            if p not in self.graph.stations or d not in self.graph.stations:
                raise ValueError("unknown station")
            self._op_seq += 1
            tid = f"OP{self._op_seq:04d}"
            pr = rng.choice((1, 2, 2, 2, 3, 4)) if random_prio else prio
            self._publish(B.TaskMsg(tid, t, p, d, pr, t + float(body.get("deadline_s", 600)), assign))
            ids.append(tid)
        return ids

    def _site_payload_cache(self):
        return json.loads(self.site_json)

    def cancel_task(self, tid: str) -> None:
        rec = self.ledger.tasks.get(tid)
        if rec is None:
            raise ValueError("unknown task")
        self._publish(B.TaskMsg(tid, sim_now(self), rec.pickup, rec.dropoff, rec.priority, rec.deadline,
                                rec.assign_to, cancelled=True))

    def control(self, command: str, target: str = "", value: str = "") -> None:
        if target and target not in self.site.robots:
            raise ValueError(f"unknown robot {target}")
        self._publish(B.ControlMsg(sim_now(self), command, target, value))

    def zone_rule(self, zone: str, rule: str) -> None:
        if zone not in self.site.zones:
            raise ValueError("unknown zone")
        if rule == "clear":
            self.zone_rules.pop(zone, None)
        else:
            self.zone_rules[zone] = rule
        self.control("zone_rule", "", f"{zone}={rule}")

    def add_obstacle(self, x: float, y: float, length: float = 1.2, width: float = 1.0) -> str:
        name = f"obs_{len(self.obstacles) + 1}"
        self.obstacles[name] = Obstacle(name=name, x=float(x), y=float(y), length=float(length), width=float(width),
                                        yaw=0.0, active=True)
        self._publish_obstacles()
        sdf = PALLET_SDF.format(name=name, L=length, W=width).replace('"', "'").replace("\n", " ")
        req = f'sdf: "{sdf}" name: "{name}" pose: {{position: {{x: {x}, y: {y}, z: 0.0}}}}'
        self._gz("create", "gz.msgs.EntityFactory", req)
        return name

    def remove_obstacle(self, name: str) -> None:
        ob = self.obstacles.get(name)
        if ob is None:
            raise ValueError("unknown obstacle")
        ob.active = False
        self._publish_obstacles()
        self._gz("remove", "gz.msgs.Entity", f'name: "{name}" type: MODEL')

    def _publish_obstacles(self) -> None:
        self.obs_pub.publish(ObstacleList(obstacles=[o for o in self.obstacles.values() if o.active]))

    def _gz(self, service: str, reqtype: str, req: str) -> None:
        cmd = ["gz", "service", "-s", f"/world/nrds_warehouse/{service}", "--reqtype", reqtype,
               "--reptype", "gz.msgs.Boolean", "--timeout", "3000", "--req", req]
        threading.Thread(target=lambda: subprocess.run(cmd, capture_output=True, timeout=10), daemon=True).start()

    # ------------------------------------------------------------------ HTTP

    def _broadcast(self, payload: dict) -> None:
        """Queue an event for every browser; the HTTP threads write it (never block the ROS executor)."""
        data = dumps(payload).encode()
        for q in list(self._sse_clients):
            try:
                q.put_nowait(data)
            except queue.Full:
                pass

    def _serve(self) -> None:
        gw = self

        class Handler(BaseHTTPRequestHandler):
            protocol_version = "HTTP/1.1"

            def log_message(self, *_a):
                pass

            def _json(self, obj, code=200):
                data = dumps(obj).encode()
                self.send_response(code)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(data)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(data)

            def _body(self) -> dict:
                n = int(self.headers.get("Content-Length") or 0)
                return json.loads(self.rfile.read(n) or b"{}") if n else {}

            def _authorised(self) -> bool:
                if not gw.token:
                    return True
                if self.headers.get("X-NRDA-Token") == gw.token:
                    return True
                self._json({"error": "missing or wrong X-NRDA-Token"}, 401)
                return False

            def do_GET(self):
                url = urlparse(self.path)
                p = url.path
                if p == "/api/stream":
                    # snapshots every 0.25 s of *wall* time (the console stays live while Gazebo starts or
                    # runs slower than real time), safety events the moment they arrive
                    self.send_response(200)
                    self.send_header("Content-Type", "text/event-stream")
                    self.send_header("Cache-Control", "no-store")
                    self.send_header("X-Accel-Buffering", "no")
                    self.end_headers()
                    self.close_connection = True
                    q: queue.Queue = queue.Queue(maxsize=200)
                    gw._sse_clients.append(q)
                    try:
                        self.wfile.write(b"retry: 1500\n\n")
                        next_snap = 0.0
                        while True:
                            now = time.monotonic()
                            if now >= next_snap:
                                self.wfile.write(b"data: " + gw.snapshot_bytes() + b"\n\n")
                                next_snap = now + 0.25
                            try:
                                ev = q.get(timeout=max(0.0, next_snap - time.monotonic()))
                                self.wfile.write(b"data: " + ev + b"\n\n")
                            except queue.Empty:
                                pass
                            self.wfile.flush()
                    except Exception:  # noqa: BLE001  (browser closed the page)
                        pass
                    finally:
                        if q in gw._sse_clients:
                            gw._sse_clients.remove(q)
                    return
                if p == "/api/snapshot":  # polling fallback for networks that buffer event streams
                    data = gw.snapshot_bytes()
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Content-Length", str(len(data)))
                    self.send_header("Cache-Control", "no-store")
                    self.end_headers()
                    self.wfile.write(data)
                    return
                if p == "/api/site":
                    data = gw.site_json.encode()
                    self.send_response(200)
                    self.send_header("Cache-Control", "no-store")
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Content-Length", str(len(data)))
                    self.end_headers()
                    self.wfile.write(data)
                    return
                if p in ("/api", "/api/"):
                    return self._json({"endpoints": API_DOC})
                if p == "/api/fleet":
                    return self._json(gw.snapshot()["robots"])
                if p.startswith("/api/robots/"):
                    rid = p.split("/")[3]
                    r = next((r for r in gw.snapshot()["robots"] if r["id"] == rid), None)
                    return self._json(r or {"error": "unknown robot"}, 200 if r else 404)
                if p == "/api/tasks":
                    return self._json(gw.snapshot()["tasks"])
                if p == "/api/kpi":
                    return self._json(gw.snapshot()["kpis"])
                if p == "/api/safety/events":
                    q = parse_qs(url.query)
                    evs = gw.snapshot()["safety_recent"]
                    if "level" in q:
                        evs = [e for e in evs if e["level"] == q["level"][0]]
                    if "robot" in q:
                        evs = [e for e in evs if q["robot"][0] in e["robots"]]
                    return self._json(evs)
                if p == "/api/safety/summary":
                    k = gw.snapshot()["kpis"]
                    return self._json({x: k[x] for x in ("collisions", "collisions_robot_robot",
                                                         "collisions_robot_static", "collisions_obstacle",
                                                         "near_misses", "avoided")})
                return self._static(p)

            def _static(self, p):
                if p == "/":
                    p = "/index.html"
                path = os.path.normpath(os.path.join(gw.web_root, p.lstrip("/")))
                if not path.startswith(gw.web_root) or not os.path.isfile(path):
                    return self._json({"error": "not found"}, 404)
                with open(path, "rb") as f:
                    data = f.read()
                self.send_response(200)
                self.send_header("Content-Type", mimetypes.guess_type(path)[0] or "application/octet-stream")
                self.send_header("Content-Length", str(len(data)))
                self.send_header("Cache-Control", "no-cache")  # always pick up a rebuilt console
                self.end_headers()
                self.wfile.write(data)

            def do_POST(self):
                if not self._authorised():
                    return
                p = urlparse(self.path).path
                try:
                    body = self._body()
                    if p == "/api/tasks":
                        return self._json({"created": gw.create_tasks(body)})
                    if p.startswith("/api/tasks/") and p.endswith("/cancel"):
                        gw.cancel_task(p.split("/")[3])
                        return self._json({"ok": True})
                    if p.startswith("/api/robots/") and p.endswith("/mode"):
                        rid = p.split("/")[3]
                        mode = body.get("mode", "")
                        cmd = {"pause": "pause", "resume": "resume", "estop": "estop", "release": "release_estop",
                               "charge": "charge", "fault": "fault", "clear_fault": "clear_fault"}.get(mode)
                        if cmd is None:
                            raise ValueError("mode must be pause|resume|estop|release|charge|fault|clear_fault")
                        gw.control(cmd, rid, body.get("value", ""))
                        return self._json({"ok": True})
                    if p in ("/api/fleet/estop", "/api/fleet/pause", "/api/fleet/resume", "/api/fleet/release"):
                        cmd = {"estop": "estop", "pause": "pause", "resume": "resume",
                               "release": "release_estop"}[p.rsplit("/", 1)[1]]
                        gw.control(cmd)
                        return self._json({"ok": True})
                    if p.startswith("/api/zones/") and p.endswith("/rule"):
                        gw.zone_rule(p.split("/")[3], body.get("rule", "clear"))
                        return self._json({"ok": True})
                    if p == "/api/inject/obstacle":
                        return self._json({"name": gw.add_obstacle(body["x"], body["y"], body.get("length", 1.2),
                                                                   body.get("width", 1.0))})
                    if p.startswith("/api/obstacles/") and p.endswith("/remove"):
                        gw.remove_obstacle(p.split("/")[3])
                        return self._json({"ok": True})
                    if p.startswith("/api/safety/events/") and p.endswith("/ack"):
                        with gw.lock:
                            gw.ledger.acked.add(p.split("/")[4])
                        return self._json({"ok": True})
                except (ValueError, KeyError) as e:
                    return self._json({"error": str(e)}, 400)
                return self._json({"error": "not found"}, 404)

        httpd = ThreadingHTTPServer(("0.0.0.0", self.port), Handler)
        httpd.daemon_threads = True
        httpd.serve_forever()


def _finite(o):
    """Replace NaN / inf (not valid JSON: the browser would reject the whole message) by null."""
    if isinstance(o, float):
        return o if math.isfinite(o) else None
    if isinstance(o, dict):
        return {str(k): _finite(v) for k, v in o.items()}
    if isinstance(o, (list, tuple, set, frozenset)):
        return [_finite(v) for v in o]
    return o


def dumps(obj) -> str:
    return json.dumps(_finite(obj), default=str, allow_nan=False, separators=(",", ":"))


API_DOC = {
    "GET /api/site": "warehouse geometry, lanes, stations, zones",
    "GET /api/stream": "Server-Sent Events: snapshots (4 Hz) and safety events",
    "GET /api/snapshot": "one snapshot (polling fallback)",
    "GET /api/fleet": "robot states", "GET /api/robots/{id}": "one robot",
    "GET /api/tasks": "tasks with timings", "GET /api/kpi": "fleet KPIs",
    "GET /api/safety/events?level=&robot=": "safety events", "GET /api/safety/summary": "safety counters",
    "POST /api/tasks": "{pickup, dropoff, priority, count, assign_to?} or {random: 10} -> created ids",
    "POST /api/tasks/{id}/cancel": "cancel a task",
    "POST /api/robots/{id}/mode": "{mode: pause|resume|estop|release|charge|fault|clear_fault}",
    "POST /api/fleet/estop|pause|resume|release": "fleet-wide control",
    "POST /api/zones/{zone}/rule": "{rule: avoid|slow|clear}",
    "POST /api/inject/obstacle": "{x, y, length?, width?}", "POST /api/obstacles/{name}/remove": "",
    "POST /api/safety/events/{id}/ack": "acknowledge",
}


def main():
    rclpy.init()
    node = Gateway()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
