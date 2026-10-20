# NRDA Fleet OS: the whole system in one command.
#
#   ros2 launch nrda_fleet_ros fleet.launch.py                       # 20 AMRs, NRDA-FA, Gazebo + RViz + web console
#
# Starts: Gazebo Harmonic (server; GUI with gui:=true) with the NRDS warehouse and AMR_1 ... AMR_N,
# the ros_gz bridge, RViz, the CPU lidar emulator (sensor_profile:=light), the independent safety
# monitor, the RViz fleet view, the web console gateway (http://localhost:8080) and one edge agent
# process per robot (/AMR_i/edge_agent). Nothing else publishes on /AMR_i/cmd_vel.
import os
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _agents(context):
    count = int(LaunchConfiguration('robots').perform(context))
    loss = float(LaunchConfiguration('comm_loss').perform(context))
    per = max(1, int(LaunchConfiguration('agents_per_process').perform(context)))
    ids = [f'AMR_{i}' for i in range(1, count + 1)]
    groups = []
    for k in range(0, len(ids), per):
        chunk = ids[k:k + per]
        if per == 1:
            node = Node(package='nrda_fleet_ros', executable='edge_agent', name='edge_agent', namespace=chunk[0],
                        output='log', parameters=[{'robot_id': chunk[0], 'comm_loss': loss,
                                                   'use_sim_time': True}])
        else:  # several robots' agents per process (each still its own /AMR_i/edge_agent node)
            # No `name=`: launch_ros turns that into `-r __node:=...`, which renames *every* node the
            # process creates. All four agents and the peer-bus hub then shared one node name, so the
            # graph showed /AMR_1/edge_agents_1 instead of the /AMR_1/edge_agent that the docs, this
            # file's own header and the runbook all refer to. Each node names itself instead.
            node = Node(package='nrda_fleet_ros', executable='edge_agents',
                        output='log', parameters=[{'robot_ids': ','.join(chunk),
                                                   'comm_loss': loss, 'use_sim_time': True}])
        groups.append(node)
    # the robots finish spawning ~6 s after launch; start their brains after that
    return [TimerAction(period=9.0 + 0.5 * i, actions=[g]) for i, g in enumerate(groups)]


def generate_launch_description():
    sim = get_package_share_directory('nrds_warehouse_sim')
    me = get_package_share_directory('nrda_fleet_ros')
    robots = LaunchConfiguration('robots')
    # one id per launch: the safety monitor's SQLite 'run' column and the gateway's ~/.nrda/runs/<id>.json
    default_run_id = time.strftime('%Y%m%d-%H%M%S')
    run_id = LaunchConfiguration('run_id')

    world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(sim, 'launch', 'nrds_warehouse.launch.py')),
        launch_arguments={'robots': robots, 'sensor_profile': LaunchConfiguration('sensor_profile'),
                          'gui': LaunchConfiguration('gui'), 'rviz': LaunchConfiguration('rviz'),
                          'gui_delay': LaunchConfiguration('gui_delay'),
                          'sim_speed': LaunchConfiguration('sim_speed'),
                          'rviz_config': os.path.join(me, 'config', 'fleet.rviz')}.items())
    common = {'use_sim_time': True}
    nodes = [
        Node(package='nrda_fleet_ros', executable='lidar_emulator', output='log',
             parameters=[{**common, 'robots': robots}],
             condition=LaunchConfigurationEquals('sensor_profile', 'light')),
        Node(package='nrda_fleet_ros', executable='safety_monitor', output='screen',
             parameters=[{**common, 'robots': robots, 'run_id': run_id,
                          'on_collision': LaunchConfiguration('on_collision')}]),
        Node(package='nrda_fleet_ros', executable='fleet_viz', output='log', parameters=[common]),
        Node(package='nrda_fleet_ros', executable='fleet_gateway', output='screen',
             parameters=[{**common, 'scenario': LaunchConfiguration('scenario'), 'port': LaunchConfiguration('port'),
                          'seed': LaunchConfiguration('seed'), 'run_id': run_id}],
             condition=IfCondition(LaunchConfiguration('console'))),
    ]
    browser = TimerAction(period=14.0, actions=[ExecuteProcess(
        cmd=['xdg-open', ['http://localhost:', LaunchConfiguration('port')]], output='log',
        condition=IfCondition(LaunchConfiguration('open_browser')))])

    return LaunchDescription([
        DeclareLaunchArgument('robots', default_value='20', description='Fleet size (AMR_1 ... AMR_N)'),
        DeclareLaunchArgument('scenario', default_value='none',
                              description='none (robots wait for tasks from the console) | demo | nominal | busy'),
        DeclareLaunchArgument('seed', default_value='1'),
        DeclareLaunchArgument('run_id', default_value=default_run_id,
                              description='Run id shared by the safety events db and ~/.nrda/runs/<id>.json'),
        DeclareLaunchArgument('sensor_profile', default_value='light', description='light | fleet | full'),
        DeclareLaunchArgument('gui', default_value='true', description='Gazebo GUI'),
        DeclareLaunchArgument('gui_delay', default_value='8.0',
                              description='s before the Gazebo GUI opens, so the fleet is already in the '
                                          'world when it reads the scene (raise it on a slower machine)'),
        DeclareLaunchArgument('sim_speed', default_value='1.0',
                              description='Target real-time factor of the simulation (1.0 = real time)'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('console', default_value='true', description='Web console + REST API'),
        DeclareLaunchArgument('port', default_value='8080'),
        DeclareLaunchArgument('open_browser', default_value='true'),
        DeclareLaunchArgument('on_collision', default_value='stop_pair', description='stop_pair | stop_fleet | log_only'),
        DeclareLaunchArgument('comm_loss', default_value='0.0', description='Emulated peer message loss 0..1'),
        DeclareLaunchArgument('agents_per_process', default_value='4',
                              description='Edge agents hosted per process (1 = one process per robot, like real robots)'),
        world, *nodes, OpaqueFunction(function=_agents), browser,
    ])
