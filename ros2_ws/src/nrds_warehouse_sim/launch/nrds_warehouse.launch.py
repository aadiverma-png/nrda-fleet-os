# Gazebo Harmonic + ROS 2 Jazzy: NRDS warehouse with the AMR_1 ... AMR_N fleet, RViz and the map.
#
#   ros2 launch nrds_warehouse_sim nrds_warehouse.launch.py robots:=20 gui:=false rviz:=true
#
# Robots are spawned from config/fleet.yaml (ids AMR_1 ... AMR_N) with a sensor profile:
#   fleet (default): 2 x 270-degree lidar at 10 Hz / 271 beams, IMU, chassis contact sensor
#   full           : every sensor of models/amr_01 (RGB-D + upward camera too), for single-robot work
# This launch file only starts the simulation. The NRDA edge agents are started by nrda_fleet_ros.
import os
import sys
import tempfile

import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, OpaqueFunction,
                            SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fleet_utils import box_sdf, bridge_config, floor_boxes, robot_sdf, roster  # noqa: E402

NVIDIA_EGL = '/usr/share/glvnd/egl_vendor.d/10_nvidia.json'


def has_package(name):
    try:
        get_package_share_directory(name)
        return True
    except PackageNotFoundError:
        return False


def _fleet(context):
    pkg = get_package_share_directory('nrds_warehouse_sim')
    count = int(LaunchConfiguration('robots').perform(context))
    profile = LaunchConfiguration('sensor_profile').perform(context)
    fleet = roster(pkg, count)
    tmp = tempfile.mkdtemp(prefix='nrda_fleet_')
    bridge_file = os.path.join(tmp, 'bridge.yaml')
    with open(bridge_file, 'w') as f:
        yaml.safe_dump(bridge_config([r[0] for r in fleet], profile), f, sort_keys=False)
    actions = [LogInfo(msg=f'[nrds_warehouse_sim] spawning {len(fleet)} AMR-01 ({profile} sensor profile): '
                           + ', '.join(r[0] for r in fleet))]
    spawns = []
    for i, (rid, x, y, yaw, station) in enumerate(fleet):
        sdf_file = os.path.join(tmp, f'{rid}.sdf')
        with open(sdf_file, 'w') as f:
            f.write(robot_sdf(pkg, rid, profile, float(LaunchConfiguration('lidar_hz').perform(context)),
                              int(LaunchConfiguration('lidar_beams').perform(context))))
        spawns.append(Node(
            package='ros_gz_sim', executable='create', output='log', name=f'spawn_{rid}',
            arguments=['-world', 'nrds_warehouse', '-file', sdf_file, '-name', rid,
                       '-x', str(x), '-y', str(y), '-z', '0.02', '-Y', str(yaw)]))
    if LaunchConfiguration('floor_boxes').perform(context).lower() == 'true':
        for name, b in floor_boxes(pkg).items():
            spawns.append(Node(
                package='ros_gz_sim', executable='create', output='log', name=f'spawn_{name}',
                arguments=['-world', 'nrds_warehouse', '-string', box_sdf(name, b['length'], b['width'], b['height']),
                           '-name', name, '-x', str(b['x']), '-y', str(b['y']), '-z', '0.0',
                           '-Y', str(b.get('yaw', 0.0))]))
    # stagger the spawns a little so the server is not flooded
    for k in range(0, len(spawns), 5):
        actions.append(TimerAction(period=2.0 + 0.8 * (k // 5), actions=spawns[k:k + 5]))
    actions.append(Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='log', name='fleet_bridge',
        parameters=[{'config_file': bridge_file, 'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('bridge'))))
    roster_file = os.path.join(tmp, 'roster.yaml')
    with open(roster_file, 'w') as f:
        yaml.safe_dump({'robots': {rid: {'x': x, 'y': y, 'yaw': yaw, 'spawn': st} for rid, x, y, yaw, st in fleet}}, f)
    actions.append(Node(
        package='nrds_warehouse_sim', executable='static_frames.py', output='log',
        arguments=[roster_file], parameters=[{'use_sim_time': True}]))
    speed = float(LaunchConfiguration('sim_speed').perform(context))
    if abs(speed - 1.0) > 1e-3:
        # target real-time factor of the physics (same 4 ms step): >1 plays the warehouse faster than real time.
        # Agents, safety and lidar all run on /clock, so every robot still sees the same physics per step.
        actions.append(TimerAction(period=3.0, actions=[ExecuteProcess(
            cmd=['gz', 'service', '-s', '/world/nrds_warehouse/set_physics', '--reqtype', 'gz.msgs.Physics',
                 '--reptype', 'gz.msgs.Boolean', '--timeout', '5000',
                 '--req', f'max_step_size: 0.004, real_time_factor: {speed:g}'],
            output='log')]))
        actions.append(LogInfo(msg=f'[nrds_warehouse_sim] simulation speed target: {speed:g}x real time'))
    return actions


def generate_launch_description():
    pkg = get_package_share_directory('nrds_warehouse_sim')
    models = os.path.join(pkg, 'models')
    world = os.path.join(pkg, 'worlds', 'nrds_warehouse.sdf')
    rviz_cfg = os.path.join(pkg, 'config', 'nrds_warehouse.rviz')
    map_yaml = os.path.join(pkg, 'maps', 'nrds_warehouse.yaml')

    # server always; GUI client only with gui:=true (the 20-robot fleet is viewed in RViz / the web console)
    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r -s ', world], 'on_exit_shutdown': 'true'}.items())
    # The GUI client is started *after* the fleet is in the world. It used to come up alongside the
    # server, which means it fetched the scene from an empty world and then had to pick the robots up
    # one by one from the scene broadcaster's incremental updates - while it was still blocked loading
    # the 24 MB warehouse mesh. Entities created in that window can be missed, and a missed entity is
    # never drawn again, although the server has it: the robot drives, publishes odometry and ground
    # truth, and answers `gz model --list`, but the viewport shows nothing. That is exactly the "a few
    # AMRs are not visible" symptom, and because it is a race the set differed from run to run.
    # Spawning finishes by ~5.2 s (batches of five from 2.0 s, 0.8 s apart), so the default start is
    # after that; `gui_delay` is there for a slower machine or a heavier mesh.
    gz_gui = TimerAction(period=LaunchConfiguration('gui_delay'), actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-g'}.items())],
        condition=IfCondition(LaunchConfiguration('gui')))
    rviz = Node(
        package='rviz2', executable='rviz2', output='log',
        arguments=['-d', LaunchConfiguration('rviz_config')], parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')))

    if has_package('nav2_map_server') and has_package('nav2_lifecycle_manager'):
        map_nodes = [
            Node(package='nav2_map_server', executable='map_server', name='map_server', output='log',
                 parameters=[{'yaml_filename': map_yaml, 'use_sim_time': True}],
                 condition=IfCondition(LaunchConfiguration('map'))),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_map',
                 output='log', parameters=[{'autostart': True, 'node_names': ['map_server'], 'use_sim_time': True}],
                 condition=IfCondition(LaunchConfiguration('map'))),
        ]
    else:
        map_nodes = [LogInfo(msg='[nrds_warehouse_sim] nav2_map_server not installed: /map stays empty',
                             condition=IfCondition(LaunchConfiguration('map')))]

    # PRIME laptops: render Gazebo sensors, GUI and RViz on the NVIDIA GPU
    use_nvidia = IfCondition(LaunchConfiguration('nvidia'))
    nvidia_env = [SetEnvironmentVariable(name, value, condition=use_nvidia)
                  for name, value in [('__NV_PRIME_RENDER_OFFLOAD', '1'), ('__GLX_VENDOR_LIBRARY_NAME', 'nvidia'),
                                      ('__EGL_VENDOR_LIBRARY_FILENAMES', NVIDIA_EGL)]]

    return LaunchDescription([
        DeclareLaunchArgument('robots', default_value='20', description='Number of AMRs (AMR_1 ... AMR_N) from config/fleet.yaml'),
        DeclareLaunchArgument('sensor_profile', default_value='light', description='light (CPU-emulated lidar, 20-robot default) | fleet (GPU lidars) | full'),
        DeclareLaunchArgument('gui', default_value='false', description='Open the Gazebo GUI client'),
        DeclareLaunchArgument('gui_delay', default_value='8.0',
                              description='s to wait before opening the GUI, so the fleet is already in '
                                          'the world when it fetches the scene (robots spawn by ~5.2 s)'),
        DeclareLaunchArgument('sim_speed', default_value='1.0',
                              description='Target real-time factor (1.0 = real time, 1.5 = 50% faster playback)'),
        DeclareLaunchArgument('lidar_hz', default_value='10', description='GPU lidar rate (fleet profile)'),
        DeclareLaunchArgument('lidar_beams', default_value='271', description='GPU lidar beams per 270-degree scan (fleet profile)'),
        DeclareLaunchArgument('bridge', default_value='true', description='Start ros_gz_bridge for the fleet'),
        DeclareLaunchArgument('floor_boxes', default_value='true',
                              description='Put the test boxes from config/floor_boxes.yaml on the floor (not in the robots map)'),
        DeclareLaunchArgument('map', default_value='true', description='Serve maps/nrds_warehouse.yaml'),
        DeclareLaunchArgument('rviz', default_value='true', description='Open RViz'),
        DeclareLaunchArgument('rviz_config', default_value=rviz_cfg),
        DeclareLaunchArgument('nvidia', default_value=str(os.path.exists(NVIDIA_EGL)).lower(),
                              description='Render on the NVIDIA GPU (PRIME offload)'),
        *nvidia_env,
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', models + os.pathsep + os.environ.get('GZ_SIM_RESOURCE_PATH', '')),
        gz_server, gz_gui, rviz, *map_nodes,
        OpaqueFunction(function=_fleet),
    ])
