"""The protective field follows the path the robot is on, not a straight rectangle (P8).

Live Gazebo run: AMR_13 full-braked at 1.50 m/s with its nearest peer 10.2 m away, AMR_4 at
1.19 m/s with 10.3 m, AMR_2 at 0.5 m/s with 8.8 m. At 1.5 m/s the straight band reaches 1.58 m
from the robot's centre, while a 3 m-radius turn has already carried the real path 0.45 m
sideways - a whole half-band - so rack faces beside a bend sat inside the field.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from nrda_fleet.amr import AMR01Spec
from nrda_fleet.safety import SafetySupervisor
from visison.motion_models import Command

AMR = AMR01Spec()
SUP = SafetySupervisor(AMR)
L = AMR.length / 2
W = AMR.width / 2 + SUP.side


def _arc_points(R, n=60, reach=4.0):
    """Points lying exactly on the arc of radius R (centre at (0, R)), ahead of the robot."""
    out = []
    for k in range(1, n + 1):
        s = reach * k / n
        phi = s / abs(R)
        sgn = math.copysign(1.0, R)
        out.append((abs(R) * math.sin(phi), R - sgn * abs(R) * math.cos(phi)))
    return np.array(out)


# ------------------------------------------------------------------ the real obstacle must still stop us

@pytest.mark.parametrize("w", [0.0, 0.1, -0.1, 0.4, -0.4])
def test_an_obstacle_on_the_path_still_brakes(w):
    v = 1.5
    if abs(w) <= SUP.arc_w:
        pts = np.array([[L + 0.3, 0.0]])
    else:
        R = v / w
        s = L + 0.3
        phi = s / abs(R)
        sgn = math.copysign(1.0, R)
        pts = np.array([[abs(R) * math.sin(phi), R - sgn * abs(R) * math.cos(phi)]])
    out = SUP.filter(Command(v, w), pts)
    assert out.brake, f"w={w}: an obstacle right on the path did not brake"


@pytest.mark.parametrize("w", [0.3, -0.3, 0.6, -0.6])
def test_points_along_the_arc_are_seen(w):
    """Everything on the arc is in the corridor, so the nearest one sets the gap."""
    v = 1.5
    pts = _arc_points(v / w)
    gap = SUP._path_gap(pts, v, w, 1.0, L, W)
    assert gap is not None, "the robot's own path was not covered"
    assert gap < 0.5, f"nearest arc point reported {gap:.2f} m away"


# ------------------------------------------------------------------ the false brake must stop happening

@pytest.mark.parametrize("w,ys", [(-0.5, 0.30), (0.5, -0.30)])
def test_a_rack_corner_beside_a_bend_no_longer_brakes(w, ys):
    """The live failure: a rack face ahead on the *outside* of the bend.

    Straight ahead it is 0.90 m past the bumper, inside the 0.98 m protective field at 1.5 m/s, so
    the robot full-braked. The arc it is actually driving curves away from it and never reaches it.
    """
    v = 1.5
    pts = np.array([[1.5, ys]])
    straight = SafetySupervisor(AMR)
    straight.arc_w = 1e9  # force the old straight-band behaviour
    old = straight.filter(Command(v, w), pts)
    new = SUP.filter(Command(v, w), pts)
    assert old.brake, "this case did not reproduce the old false brake"
    assert not new.brake, "the arc field still brakes for a corner beside the bend"


@pytest.mark.parametrize("w,ys", [(0.5, 0.30), (-0.5, -0.30)])
def test_the_same_corner_on_the_inside_of_the_bend_still_brakes(w, ys):
    """Turning *into* it, the arc does reach it, so the brake must stay."""
    out = SUP.filter(Command(1.5, w), np.array([[1.5, ys]]))
    assert out.brake, "turning towards an obstacle must still brake"


def test_a_wall_within_the_body_width_always_brakes():
    """Whatever the steering: 0.3 m off the centreline is inside a 0.78 m body."""
    pts = np.array([[x, 0.30] for x in np.arange(L + 0.05, 2.0, 0.05)])
    for w in (0.0, 0.3, -0.3, 0.6, -0.6):
        assert SUP.filter(Command(1.5, w), pts).brake, f"w={w} drove into a wall alongside it"


def test_straight_driving_is_unchanged():
    """With no turn the field is the original rectangle, to the centimetre."""
    v = 1.5
    pts = np.array([[L + 0.5, 0.2], [L + 2.0, 0.0], [L + 0.4, 3.0]])
    straight = SafetySupervisor(AMR)
    straight.arc_w = 1e9
    for w in (0.0, 0.01, 0.04):
        a = straight.filter(Command(v, w), pts)
        b = SUP.filter(Command(v, w), pts)
        assert a.brake == b.brake and abs(a.gap - b.gap) < 1e-9, f"w={w} changed straight behaviour"


def test_the_hard_stop_is_still_unconditional():
    """The 0.28 m ESPE stop is a straight-ahead guard and must not be weakened by any of this."""
    pts = np.array([[L + 0.1, 0.0]])
    for w in (0.0, 0.5, -0.5, 1.2):
        out = SUP.filter(Command(1.0, w), pts)
        assert out.brake, f"hard stop missed at w={w}"
        assert "hard stop" in out.reason


def test_reverse_on_an_arc_is_covered():
    v, w = -0.6, 0.4
    R = v / w
    s = L + 0.2
    phi = s / abs(R)
    sgn = math.copysign(1.0, R)
    pt = np.array([[-(abs(R) * math.sin(phi)), R - sgn * abs(R) * math.cos(phi)]])
    # reversing along the arc: the point sits behind the robot on its path
    gap = SUP._path_gap(pt, v, w, -1.0, L, W)
    assert gap is not None, "reversing arc path not covered"


def test_nothing_in_the_path_gives_no_gap():
    pts = np.array([[0.0, 6.0], [0.0, -6.0]])
    assert SUP._path_gap(pts, 1.5, 0.4, 1.0, L, W) is None
