# Interfaces

## Robot hardware (per robot `AMR_i`)

| Topic | Type | Direction | Notes |
|---|---|---|---|
| `/AMR_i/cmd_vel` | `geometry_msgs/Twist` | agent → robot | **Single writer**: `/AMR_i/edge_agent`, which checks at start-up that no other publisher exists |
| `/AMR_i/odom` | `nav_msgs/Odometry` | robot → agent | DiffDrive odometry, frames `AMR_i/odom` → `AMR_i/base_link` |
| `/AMR_i/scan_front`, `/AMR_i/scan_rear` | `sensor_msgs/LaserScan` | robot → agent | 270°, 271 beams, 10 Hz; frames `AMR_i/base_link/lidar_front_right`, `…/lidar_rear_left` |
| `/AMR_i/contacts` | `ros_gz_interfaces/Contacts` | robot → safety monitor | Chassis contact sensor |
| `/AMR_i/imu` | `sensor_msgs/Imu` | robot → | 100 Hz |

## Peer bus (`nrda_msgs`)

| Topic | Message | QoS | Rate |
|---|---|---|---|
| `/fleet/state` | `RobotState` | best effort, depth 50 | 10 Hz per robot (also the heartbeat) |
| `/fleet/claims` | `ClaimSet` | reliable | On change, at least 1 Hz |
| `/fleet/bids` | `Bid` | reliable | 2 Hz (CBBA rounds) |
| `/fleet/tasks` | `Task` | reliable, transient local | Released by the gateway |
| `/fleet/task_events` | `TaskEvent` | reliable | in_progress / picked / completed |
| `/fleet/alerts` | `Alert` | reliable | risk brakes, obstacles, directives |
| `/fleet/control` | `Control` | reliable, transient local | Operator commands (e-stop is latched) |
| `/fleet/safety_events` | `SafetyEvent` | reliable | From the safety monitor |
| `/fleet/obstacles` | `ObstacleList` | transient local | Obstacles injected from the console |
| `/nrda/ground_truth` | `tf2_msgs/TFMessage` | – | Gazebo true poses (lidar emulator and safety monitor only) |

**Claim keys:**
- `<line>/<A|B>`, index = 0.5 m cell along the line (for example `V05/A`, 31);
- `S:<station>`, a station footprint;
- `F:<i>`, index `j`: 0.25 m free-space cell.

## Web API (`fleet_gateway`, http://localhost:8080)

| Method | Path | Body / query | Result |
|---|---|---|---|
| GET | `/api/site` | | Warehouse geometry, lanes, stations, zones |
| GET | `/api/stream` | | Server-Sent Events: fleet snapshots (4 Hz) and safety events |
| GET | `/api/fleet`, `/api/robots/{id}` | | Robot states |
| GET | `/api/tasks` | | Tasks with timings, bidders, winning bid |
| GET | `/api/kpi` | | Throughput, latency, utilisation, collisions … |
| GET | `/api/safety/events` | `?level=COLLISION&robot=AMR_3` | Safety events |
| GET | `/api/safety/summary` | | Collision / near-miss / avoided counters |
| POST | `/api/tasks` | `{pickup, dropoff, priority, count, assign_to?}` | `{created: [...]}`. Without `assign_to` the fleet auctions the task (CBBA); with it, only that robot may take it. Refused for faulted, paused, low-battery robots |
| POST | `/api/tasks/{id}/cancel` | | |
| POST | `/api/robots/{id}/mode` | `{mode: pause, resume, estop, release, charge, fault, clear_fault}` | |
| POST | `/api/fleet/estop`, `pause`, `resume`, `release` | | Fleet-wide |
| POST | `/api/zones/{zone}/rule` | `{rule: avoid, slow, clear}` | Every robot's router applies it |
| POST | `/api/inject/obstacle` | `{x, y, length?, width?}` | Static pallet in Gazebo and for the lidar model |
| POST | `/api/obstacles/{name}/remove` | | |
| POST | `/api/safety/events/{id}/ack` | | Mark reviewed |

With the parameter `api_token` set, write calls need the header `X-NRDA-Token`.
