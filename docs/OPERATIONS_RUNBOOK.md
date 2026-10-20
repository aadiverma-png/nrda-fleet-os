# Operations runbook

## Start and stop
- **Start (Standard):** `./nrda.sh sim`. Gazebo, RViz and the console come up in about 15 s, and the agents start in groups of four.
- **Start (Peak Stress):** `./nrda.sh sim scenario:=busy`. 20 robots, 240 tasks at 16/min (stress-tests throughput and deadlock elimination).
- **Stop:** Ctrl-C in the terminal, then `./nrda.sh stop` to clean up all simulation processes.
- **Headless:** `./nrda.sh sim gui:=false rviz:=false open_browser:=false`. The console is still at http://localhost:8080 (runs at ~1.0× real time).

## Console tasks
| Want to | Do |
|---|---|
| Run automated peak load | Start with `scenario:=busy` (16 tasks/min released automatically) |
| Release a task to the fleet | Tasks tab: choose pick-up and drop-off, "The fleet decides", then Release task. Watch the auction on the card |
| Give a task to one robot | Choose "Give it to" and a robot. Robots with a collision, fault, pause or low battery are greyed out |
| See collisions & near misses | Counter band at top (turns into hazard stripes on collision), Safety tab event list, and red markers on map and RViz |
| Monitor Corridor VE | Observe Northbound flow on Corridor `VE` and Southbound return via `V14` on the live dashboard |
| Keep robots out of a zone | Safety tab, Picker zones: Keep out (or Slow). Clear removes the rule |
| Stop one robot | Click it on the map, then Pause or Emergency stop |
| Stop everything | Emergency stop in the top bar (confirmation required). Release it with the same button |
| Test the lidar | Drop a pallet, then click an aisle. Robots detect it, bypass or re-route. Click the pallet to remove it |
| Simulate a failure | Click a robot, then Inject a motor fault. Its tasks are re-auctioned, and peers bypass it or route around it |

## After a collision
1. The robots involved are paused automatically (`on_collision:=stop_pair`).
2. Inspect the event: Safety tab, click the row (the map zooms there).
3. Mark it Reviewed, then Resume each robot from its detail card.

## Configuration
| File | Purpose |
|---|---|
| `ros2_ws/src/nrds_warehouse_sim/config/fleet.yaml` | Roster (AMR ids, spawn station, battery) and parking bays |
| `…/config/locations.yaml` | Picker zones, chargers (with `disabled:` reasons), drops, stations |
| `…/models/nrda_charger`, `nrda_parking_bay`, `nrds_lidar_proxy` | Added dock, floor marking, lidar-only geometry (`scripts/make_lidar_proxy.py`) |
| Launch arguments | `robots`, `scenario`, `seed`, `sensor_profile`, `gui`, `sim_speed`, `rviz`, `console`, `port`, `open_browser`, `on_collision`, `comm_loss`, `agents_per_process` |

After editing the warehouse or stations, rebuild with `./nrda.sh build`. The lane graph regenerates automatically: it is cached by a content hash.

## Logs and data
| Location | Contents |
|---|---|
| `~/.nrda/events.db` | Safety events (SQLite) |
| `~/.nrda/runs/<run>.json` | KPIs, tasks, events of each Gazebo run (written every 10 s) |
| `ros2 topic echo /diagnostics` | Per-robot battery, planning time, brakes, tasks |
