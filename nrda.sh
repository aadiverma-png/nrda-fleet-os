#!/usr/bin/env bash
# NRDA Fleet OS: one entry point.
#
#   ./nrda.sh build                 build the ROS 2 workspace (core + warehouse + msgs + fleet runtime)
#   ./nrda.sh sim [launch args]     Gazebo + RViz + web console + 20 AMRs running NRDA-FA
#   ./nrda.sh sim sensor_profile:=fleet       Gazebo GPU lidars instead of the CPU lidar model
#   ./nrda.sh test                  unit + integration tests (ROS-free core)
#   ./nrda.sh bench [--quick]       scenario benchmark (2-D simulator) + KPI report
#   ./nrda.sh stop                  stop every simulation process
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$ROOT/ros2_ws"
ROS_DISTRO_SETUP="/opt/ros/jazzy/setup.bash"

ros_env() {
  # shellcheck disable=SC1090
  set +u; source "$ROS_DISTRO_SETUP"; [ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"; set -u
}

stop_all() {
  pkill -f "[g]z sim" 2>/dev/null || true
  pkill -f "[p]arameter_bridge" 2>/dev/null || true
  pkill -f "nrda_fleet_ros/lib/nrda_fleet_ros/[e]dge_agent" 2>/dev/null || true
  pkill -f "nrda_fleet_ros/lib/nrda_fleet_ros/[f]leet_gateway" 2>/dev/null || true
  pkill -f "nrda_fleet_ros/lib/nrda_fleet_ros/[l]idar_emulator" 2>/dev/null || true
  pkill -f "nrda_fleet_ros/lib/nrda_fleet_ros/[s]afety_monitor" 2>/dev/null || true
  pkill -f "nrda_fleet_ros/lib/nrda_fleet_ros/[f]leet_viz" 2>/dev/null || true
  pkill -f "nrds_warehouse_sim/[s]tatic_frames" 2>/dev/null || true
  pkill -f "[r]viz2" 2>/dev/null || true
  # the Gazebo server ignores SIGTERM while it is busy: a survivor keeps simulating the same world on the same
  # topics next to the new one (mixed clocks and poses). Give it 2 s, then kill what is left.
  sleep 2
  pkill -9 -f "[g]z sim" 2>/dev/null || true
  pkill -9 -f "nrda_fleet_ros/lib/nrda_fleet_ros/[e]dge_agent" 2>/dev/null || true
}

cmd="${1:-sim}"; shift || true
case "$cmd" in
  build)
    ros_env
    cd "$WS" && colcon build --symlink-install --base-paths src ../core "$@"
    # warm the lane-graph cache once so 20 agent processes start in milliseconds
    PYTHONPATH="$ROOT/core" python3 -c "from nrda_fleet.site import Site; from nrda_fleet.lane_graph import LaneGraph; LaneGraph.load_or_build(Site.load())"
    ;;
  sim)
    stop_all
    "$0" build >/dev/null  # incremental: a few seconds, and python edits always reach the install
    sleep 1
    ros_env
    exec ros2 launch nrda_fleet_ros fleet.launch.py "$@"
    ;;
  test)
    # The ROS-free core (core/visison, core/nrda_fleet) has no test suite: core/tests was deleted.
    # The runtime package's own tests. They are named file by file on purpose: ROS 2's pytest plugins
    # take over directory collection in ros2_ws/src/nrda_fleet_ros, where pointing pytest at the test
    # directory silently collects nothing at all (verified: even a trivial test file is dropped).
    ros_tests=("$WS"/src/nrda_fleet_ros/test/test_*.py)
    if [ -e "${ros_tests[0]}" ]; then
      echo "--- nrda_fleet_ros ---"
      # Sourced: these need rclpy and nrda_msgs, and an unsourced run turns their module-level
      # importorskip into a session-level skip that silently drops every other file with it.
      ros_env
      python3 -m pytest "${ros_tests[@]}" -q
    fi
    ;;
  bench)
    python3 "$ROOT/benchmarks/run_scenarios.py" "$@"
    python3 "$ROOT/benchmarks/report.py"
    ;;
  stop)
    stop_all
    ;;
  *)
    sed -n '2,9p' "$0"; exit 1 ;;
esac
