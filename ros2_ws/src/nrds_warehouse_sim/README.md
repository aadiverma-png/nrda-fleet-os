# nrds_warehouse_sim

NRDS smart warehouse (80 x 50 m, 10 m clear) with 20 AMR-01 robots for **Gazebo Harmonic** and **ROS 2 Jazzy**.

## Build and run

Within the NRDA root workspace:

```bash
./nrda.sh build                           # colcon build across all 4 packages
./nrda.sh sim                             # Launches Gazebo 3D, RViz, 20 AMRs & Web Console
./nrda.sh sim scenario:=busy              # Peak load test (20 AMRs, 16 tasks/min)
./nrda.sh sim gui:=false                  # Headless Gazebo (RViz + Console only)
```

Direct ROS 2 launch:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch nrds_warehouse_sim nrds_warehouse.launch.py robots:=20
```

## Contents

- `worlds/nrds_warehouse.sdf`: World physics, illumination, contacts, warehouse model, chargers and parking bays
- `models/nrds_warehouse`: Static warehouse model (decluttered mesh `nrds_warehouse_no_roof.glb` + box collisions)
- `models/amr_01`: AMR-01 tote transporter (diff drive, dual 270° lidars, IMU, chassis contact sensors)
- `models/nrda_charger`, `nrda_parking_bay`: Charging stations and sleeping bays
- `models/nrds_lidar_proxy`: High-speed raycasting proxy geometry for CPU lidar emulation
- `config/bridge.yaml`: `ros_gz_bridge` mappings for `/AMR_i/{cmd_vel, odom, scan_front, scan_rear, imu, contacts, tf}`
- `config/fleet.yaml`, `config/locations.yaml`: 20-robot fleet roster, stations, chargers, parking bays
- `maps/nrds_warehouse.pgm/.yaml`: Warehouse floor occupancy map (0.1 m resolution)
- `launch/nrds_warehouse.launch.py`: Main warehouse Gazebo simulation launch
- `config/nrds_warehouse.rviz`: RViz visualization profile

The Nav2 map needs `sudo apt install ros-jazzy-nav2-map-server ros-jazzy-nav2-lifecycle-manager`; without it the launch still runs and logs a warning.

## Frames

X = east, Y = north, Z = up. TF: `map -> amr_XX/odom -> amr_XX/base_link` on `/tf` (also on `/amr_XX/tf`). Warehouse floor Z = 0, truck yard Z = -1.2. Robot heading 0 = +X.

## Notes

- Sensor topics use Gazebo's default scoped names; check with `gz topic -l` if a bridge entry stays silent.
- Humans, forklifts and parked trucks are static collision boxes.
- The roof (trusses, deck, skylights, solar array) is removed: the model uses `meshes/nrds_warehouse_no_roof.glb`. Point `model.sdf` back at `meshes/nrds_warehouse.glb` to restore it.

## Camera controls

The world opens on an isometric overview. Orbit: middle-drag or Shift + left-drag. Pan: left-drag. Zoom: scroll wheel.
The **View angle** panel (right dock) snaps to top/front/back/left/right views; its home button returns to the overview.
