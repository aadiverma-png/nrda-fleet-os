"""L2 bypass sessions must end: every controller phase is bounded (P1).

A session frozen in a manoeuvre phase used to live for ever. While it lives,
:meth:`~nrda_fleet.agent.EdgeAgent._l3` returns before re-planning, re-claiming and
re-deriving the wait edges, so the robot stood across both lanes of the aisle holding no
claims and gossiping a stale wait edge. Observed in Gazebo: 264 s, 0.000 m moved.
"""

from __future__ import annotations

import pytest

from nrda_fleet.bypass import HOLD_MAX, MANOEUVRE_MAX, STALL_MAX, BypassSession
from visison.controller import ControllerState


class FakeAgent:
    """Just enough agent for a session's deadline logic."""

    def __init__(self, pose=(0.0, 0.0, 0.0)):
        self.pose = pose
        self.t = 0.0
        self.rerouted: list[str] = []
        self.logs: list[str] = []

    def _reroute(self, why):
        self.rerouted.append(why)

    def _log(self, m):
        self.logs.append(m)

    def _skip_passed_legs(self):
        pass

    class _Book:
        def __init__(self):
            self.extra = {}

        def clear(self):
            self.extra.clear()

    book = None


class FakeCtl:
    def __init__(self, state):
        self.state = state
        self.ticks = 0

    def tick(self, t):
        self.ticks += 1


def session(state, agent=None, started=0.0):
    a = agent or FakeAgent()
    a.book = FakeAgent._Book()
    a.book.extra = {("H4/A", 2): 1}
    return BypassSession(a, FakeCtl(state), line=None, lane="A", obstacle_id="obs", started=started), a


MANOEUVRE_STATES = [
    ControllerState.PASSING,
    ControllerState.CHANGE_OUT,
    ControllerState.CHANGE_IN,
    ControllerState.WAITING,
    ControllerState.APPROACH,
    ControllerState.DETECTED,
    ControllerState.EVALUATING,
    ControllerState.REJOINING,
    ControllerState.EMERGENCY_STOP,
]


@pytest.mark.parametrize("state", MANOEUVRE_STATES)
def test_every_manoeuvre_phase_has_a_total_deadline(state):
    """No phase may outlive MANOEUVRE_MAX, even if the robot keeps inching along."""
    s, a = session(state)
    # creep just enough every tick that the no-progress rule never fires
    t = 0.0
    while t < MANOEUVRE_MAX + 1.0:
        t += 1.0
        a.t = t
        a.pose = (a.pose[0] + 0.5, 0.0, 0.0)
        s.expired(t)
    assert s.expired(t), f"{state.value} never expired after {t:.0f} s"


@pytest.mark.parametrize("state", MANOEUVRE_STATES)
def test_a_frozen_manoeuvre_is_given_up(state):
    """The live failure: stopped in a manoeuvre phase, never moving."""
    s, a = session(state)
    a.t = 1.0
    assert not s.expired(1.0), "should tolerate a brief stop"
    a.t = 1.0 + STALL_MAX + 1.0
    why = s.expired(a.t)
    assert why, f"{state.value} frozen for {STALL_MAX + 1:.0f} s was not given up"
    assert "stalled" in why


def test_the_observed_failure_ends_and_reroutes():
    """AMR_19's exact situation: PASSING, zero movement. It must reroute, not freeze."""
    s, a = session(ControllerState.PASSING)
    for t in range(0, 300, 1):  # 300 s standing perfectly still
        a.t = float(t)
        s.tick(float(t))
    assert a.rerouted, "a frozen PASSING session never handed the problem to L3"
    assert not s.active(), "the session is still active after giving up"
    assert s.ctl.state is ControllerState.REROUTING
    assert a.book.extra == {}, "the borrowed cells stayed booked"
    # and it happened promptly, not after 300 s
    assert "stalled" in a.rerouted[0]


def test_hold_deadline_is_unchanged():
    s, a = session(ControllerState.HOLDING)
    a.t = HOLD_MAX - 1.0
    assert not s.expired(a.t)
    a.t = HOLD_MAX + 1.0
    assert "hold timed out" in s.expired(a.t)


def test_progress_resets_the_stall_timer():
    """A slow but genuinely progressing manoeuvre is not cut off."""
    s, a = session(ControllerState.PASSING)
    for t in range(0, int(MANOEUVRE_MAX) - 5):
        a.t = float(t)
        a.pose = (t * 0.2, 0.0, 0.0)  # 0.2 m per tick: slow, but moving
        assert not s.expired(float(t)), f"cut off a progressing manoeuvre at t={t}"


def test_settling_and_rerouting_are_not_touched_by_the_new_rule():
    for state in (ControllerState.REJOINED, ControllerState.CRUISING, ControllerState.REROUTING):
        s, a = session(state)
        a.t = 10_000.0
        assert s.expired(a.t) == "", f"{state.value} must keep its own rule"
