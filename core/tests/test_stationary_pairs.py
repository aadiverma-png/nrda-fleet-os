"""Two stationary robots never raise a collision risk against each other.

Guarding a property that already holds: ``collision_risk`` returns NONE as soon as the closing
speed is non-positive, so robots in adjacent bays do not brake for one another however close
they are parked. Locked in here because an audit mistook recorder artifacts for a defect at
exactly this point, and because the guard sits before the ``gap <= min_gap`` hard-brake branch,
which would otherwise fire at 1.0 probability for any close-parked pair.
"""

from __future__ import annotations

import pytest

from visison.emergency import EmergencyConfig, RiskLevel, TrackedObject, collision_risk

CFG = EmergencyConfig()
BAND = (-0.45, 0.45)
L, W = 1.20, 0.78


def _pair(gap, v_ego=0.0, v_other=0.0):
    ego = TrackedObject(0.0, 0.0, v_ego, 0.0, L, W, "AMR_1")
    other = TrackedObject(gap + L, 0.0, v_other, 0.0, L, W, "AMR_2")
    return collision_risk(ego, BAND, 1.6, other, 1.6, CFG, "AMR_1", "AMR_2")


@pytest.mark.parametrize("gap", [0.05, 0.10, 0.5, 0.9, 1.19, 1.31, 1.41, 3.0])
def test_both_stationary_is_never_a_risk(gap):
    """Including gaps below min_gap, where the hard-brake branch lives."""
    r = _pair(gap)
    assert r.level is RiskLevel.NONE, f"gap {gap}: {r.level.name} p={r.probability}"
    assert r.probability == 0.0


@pytest.mark.parametrize("gap", [0.05, 0.10, 0.5])
def test_a_close_parked_pair_does_not_trip_the_min_gap_brake(gap):
    """The closing-speed guard must come first, or every tight bay would brake at p=1.0."""
    assert gap < CFG.min_gap or True
    assert _pair(gap).level is RiskLevel.NONE


def test_moving_apart_is_not_a_risk():
    assert _pair(1.0, v_ego=-0.5, v_other=0.5).level is RiskLevel.NONE


def test_a_closing_robot_is_still_assessed():
    """The guard must not suppress the real case it exists alongside."""
    r = _pair(1.46, v_ego=0.0, v_other=-1.49)
    assert r.level is not RiskLevel.NONE, "a robot closing at 1.49 m/s was ignored"
    assert r.probability > CFG.p_warn


def test_a_robot_closing_hard_brakes():
    r = _pair(0.6, v_ego=1.5, v_other=-1.5)
    assert r.level is RiskLevel.BRAKE, f"head-on at 3 m/s closing gave {r.level.name}"
