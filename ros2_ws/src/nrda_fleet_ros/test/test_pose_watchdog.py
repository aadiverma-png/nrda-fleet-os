"""The edge agent's pose-feed watchdog (needs the built workspace: rclpy + nrda_msgs).

A robot whose Gazebo model or ros_gz bridge goes away keeps its last pose for ever. Without this
watchdog the agent drove on that frozen pose and went on publishing a confident, non-stale
RobotState, so the console drew a robot where the simulator had none and peers kept honouring its
lane claims. POSE_LOST matches ClaimBook.peer_timeout and the gateway's `stale` threshold.
"""
import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("nrda_msgs")

from nrda_fleet_ros.edge_agent_node import POSE_LOST, pose_feed_age  # noqa: E402

POSE = (1.0, 2.0, 0.0)


def test_localiser_age_is_what_counts_not_odometry():
    """Ground truth is the pose the agent acts on, so its age is the one that decides."""
    # localiser fresh, odometry 10 s dead: the pose itself is still good
    assert pose_feed_age(100.0, 90.0, POSE, 99.9) == pytest.approx(0.1)
    # localiser 10 s dead, odometry fresh: the pose is stale even so
    assert pose_feed_age(100.0, 99.9, POSE, 90.0) == pytest.approx(10.0)


def test_falls_back_to_odometry_without_a_localiser():
    assert pose_feed_age(100.0, 97.5, None, 0.0) == pytest.approx(2.5)


def test_first_tick_before_any_odometry_reads_as_very_old():
    """odom_t is 0.0 until the first message: the arbiter must brake, not trust it."""
    assert pose_feed_age(5.0, 0.0, None, 0.0) == pytest.approx(5.0)


def test_a_live_feed_never_trips_the_watchdog():
    # 20 Hz ground truth: ages of one or two frames stay far below the threshold
    for frame in (0.05, 0.1, 0.5, 1.0, 2.9):
        assert pose_feed_age(100.0, 100.0, POSE, 100.0 - frame) < POSE_LOST


def test_a_dead_feed_trips_it_and_keeps_tripping():
    for gone in (3.01, 10.0, 600.0):
        assert pose_feed_age(1000.0, 1000.0, POSE, 1000.0 - gone) > POSE_LOST


def test_threshold_matches_the_peer_timeout_the_fleet_uses():
    from nrda_fleet.claims import ClaimBook
    assert POSE_LOST == ClaimBook("AMR_1").peer_timeout
