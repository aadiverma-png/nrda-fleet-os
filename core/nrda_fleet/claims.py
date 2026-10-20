"""Owner-authoritative, gossiped space-time claims (NRDA-FA layer L3).

Every robot is the only authority over its *own* claims ("robot R needs
resource X during (start, end)"). It publishes its whole claim set as a
:class:`~nrda_fleet.bus.ClaimMsg` snapshot (4 Hz and on change). Each robot
keeps a replica: its own claims plus the latest snapshot of every peer.

Conflicts are settled by a **precedence order that every replica computes the
same way**:

1. a claim on a resource the robot's body physically covers right now wins;
2. otherwise the older claim wins (its first-claim stamp);
3. then the higher task priority;
4. then the lower AMR number.

A robot schedules around every peer claim that has precedence over it and
ignores the ones that lose (their owner will withdraw them at its next
replan). A robot moves only into resources whose claims are *confirmed*:
held for ``confirm`` seconds with no winning conflict.

Peers that fall silent for ``peer_timeout`` seconds lose every claim except
the resources their body covered in their last snapshot. Those stay blocked
(a failed robot is a static obstacle), as the health model requires
(:mod:`nrda_fleet.health`).

:class:`ClaimBook` also implements :class:`visison.reservation.ReservationProvider`,
so the visison bypass planner and controller use it unchanged.
"""

from __future__ import annotations

import math
from collections import defaultdict
from dataclasses import dataclass, field, replace
from typing import Callable, Iterable, Sequence

from visison.reservation import Reservation

from .bus import ClaimMsg
from .ids import id_key

Res = tuple[str, int]
STAMP_TIE = 0.05  # s: claims made this close together count as simultaneous


@dataclass(frozen=True)
class Claim:
    key: str
    idx: int
    start: float
    end: float
    stamp: float  # when this claim was first made (kept across replans)

    @property
    def res(self) -> Res:
        return self.key, self.idx

    def overlaps(self, start: float, end: float) -> bool:
        return self.start < end and start < self.end


@dataclass
class PeerClaims:
    robot_id: str
    heard: float  # local time the last snapshot arrived
    stamp: float  # snapshot stamp (sender clock = sim clock)
    seq: int
    priority: int
    by_res: dict[Res, list[Claim]]
    occupied: set[Res]


def precedence_key(stamp: float, priority: int, robot_id: str) -> tuple:
    """Smaller sorts first = wins. Stamps are bucketed so near-simultaneous claims tie on priority/id."""
    return (math.floor(stamp / STAMP_TIE), -priority, id_key(robot_id))


@dataclass
class ClaimBook:
    robot_id: str
    priority: Callable[[], int] = lambda: 2
    peer_timeout: float = 3.0
    confirm: float = 0.3
    own: dict[Res, list[Claim]] = field(default_factory=lambda: defaultdict(list))
    extra: dict[Res, list[Claim]] = field(default_factory=lambda: defaultdict(list))  # bypass (visison) claims
    occupied: set[Res] = field(default_factory=set)
    peers: dict[str, PeerClaims] = field(default_factory=dict)
    seq: int = 0
    now: float = 0.0

    # ----------------------------------------------------------------- peer snapshots

    def on_msg(self, msg: ClaimMsg, now: float) -> None:
        if msg.robot_id == self.robot_id:
            return
        old = self.peers.get(msg.robot_id)
        if old is not None and msg.seq <= old.seq and msg.stamp <= old.stamp:
            return  # out of order
        by_res: dict[Res, list[Claim]] = defaultdict(list)
        for key, idx, start, end, stamp in msg.claims:
            by_res[(key, idx)].append(Claim(key, idx, start, end, stamp))
        self._unindex(msg.robot_id)
        p = PeerClaims(msg.robot_id, now, msg.stamp, msg.seq, msg.priority, by_res, set(map(tuple, msg.occupied)))
        self.peers[msg.robot_id] = p
        idx = self.__dict__.setdefault("_index", defaultdict(dict))
        for res in by_res:
            idx[res][msg.robot_id] = p
        for res in p.occupied:
            idx[res][msg.robot_id] = p

    def _unindex(self, rid: str) -> None:
        old = self.peers.get(rid)
        idx = self.__dict__.get("_index")
        if old is None or idx is None:
            return
        for res in list(old.by_res) + list(old.occupied):
            d = idx.get(res)
            if d is not None:
                d.pop(rid, None)
                if not d:
                    del idx[res]

    def alive(self, rid: str, now: float | None = None) -> bool:
        p = self.peers.get(rid)
        return p is not None and (now if now is not None else self.now) - p.heard <= self.peer_timeout

    def forget(self, rid: str) -> None:
        self._unindex(rid)
        self.peers.pop(rid, None)

    # ----------------------------------------------------------------- precedence

    def _wins(self, mine: Claim, rid: str, theirs: Claim, their_prio: int) -> bool:
        """True if my claim beats ``theirs`` (so I may ignore it)."""
        tok = self.tokens.get(mine.res)
        if tok is not None and tok[1] > self.now and tok[0] in (self.robot_id, rid):
            # DRP anti-relapse token: the priority robot of a just-resolved deadlock goes first here
            return tok[0] == self.robot_id
        if mine.res in self.occupied:
            return True
        p = self.peers.get(rid)
        if p is not None and mine.res in p.occupied and self.alive(rid):
            return False
        if p is not None and self.core_map:
            # a robot whose body is inside a crossing core must be able to clear THAT crossing: it wins the
            # contest for that crossing's cells against a robot outside it (the same verdict on every replica:
            # bodies are gossiped). It wins nothing elsewhere: standing in one crossing used to out-rank an
            # older claim on another crossing metres away, and the verdict flipped back the moment the robot
            # left, so both robots accelerated into the second crossing in turn (collision 2026-09-30).
            xing = self.core_map.get(mine.res)
            if xing is not None and self.alive(rid):
                mine_in = any(self.core_map.get(o) == xing for o in self.occupied)
                theirs_in = any(self.core_map.get(o) == xing for o in p.occupied)
                if mine_in != theirs_in:
                    return mine_in
        if p is not None:
            # occupier goes first: if the peer is waiting for me because my body sits on its path, I must
            # move before it can - claim age cannot help it (this breaks the mutual-wait livelock)
            if self.waiting_on_me(rid) and any(o in p.by_res for o in self.occupied):
                return True
            if rid in self.my_blockers() and any(o in self.own or o in self.extra for o in p.occupied):
                return False
        return precedence_key(mine.stamp, self.priority(), self.robot_id) < \
            precedence_key(theirs.stamp, their_prio, rid)

    def blocking(self, res: Res, t_from: float = -math.inf, now: float | None = None,
                 ignore_losers: bool = True) -> list[tuple[str, float, float]]:
        """Peer intervals on ``res`` this robot must respect: (robot or obstacle, start, end)."""
        now = self.now if now is None else now
        out = []
        blocker = self.static_blocks.get(res)
        if blocker is not None:  # an unmapped obstacle sits there: blocked until it is cleared
            out.append((blocker, -math.inf, math.inf))
        for rid, p in self.__dict__.get("_index", {}).get(res, {}).items():  # only peers touching ``res``
            if now - p.heard > self.peer_timeout:  # silent peer: its body is a static obstacle
                if res in p.occupied:
                    out.append((rid, -math.inf, math.inf))
                continue
            if self.follow_ok(rid, res):
                continue  # same-direction traffic in a one-way lane: we follow it (ORCA headway + lidar fields)
            if self.impatient and not self.peer_moving(rid):
                # deadlock breaker: after a long wait, the bookings of peers that are themselves standing still
                # no longer block, only their bodies do. A moving peer's bookings are where it will be in a
                # moment, so they are always respected (separation must not depend on the lidar safety layer).
                if res in p.occupied:
                    out.append((rid, -math.inf, now + 1.0))
                continue
            t_behind = self._behind_me(p)
            for c in p.by_res.get(res, ()):
                if c.end <= t_from:
                    continue
                if c.start > t_behind and res not in self.occupied:
                    continue  # shadowed: the peer's path runs through my body first, it cannot get here before me
                # precedence is per resource, not per time slot: if we both need this cell, the better
                # claim goes first and the other schedules after it (no mutual "after you" livelock)
                if ignore_losers and any(self._wins(m, rid, c, p.priority) for m in self._mine(res)):
                    continue
                out.append((rid, c.start, c.end))
            if res in p.occupied and not p.by_res.get(res):
                out.append((rid, p.stamp, p.stamp + 0.5))  # covered but unclaimed: momentary block
        return out

    def _behind_me(self, p: PeerClaims) -> float:
        """Earliest time the peer's route enters a cell my body covers now (inf if it never does).

        Its claims that start later lie beyond me on its path, so they cannot block me: the peer has to get
        past my body first (by waiting for me, or by a bypass that re-plans its claims). This is what stops a
        robot queued behind another in a lane from holding the lane *ahead* of the robot in front.
        """
        t = math.inf
        for o in self.occupied:
            for c in p.by_res.get(o, ()):
                if c.start < t:
                    t = c.start
        return t + 0.05

    def _mine(self, res: Res) -> list[Claim]:
        return self.own.get(res, []) + self.extra.get(res, [])

    def lost(self) -> list[tuple[Claim, str]]:
        """Own claims that lose against a live peer claim (they must be withdrawn)."""
        out = []
        for res, mine in list(self.own.items()) + list(self.extra.items()):
            for m in mine:
                for rid, p in self.__dict__.get("_index", {}).get(res, {}).items():
                    if self.now - p.heard > self.peer_timeout:
                        continue
                    for c in p.by_res.get(res, ()):
                        if not self._wins(m, rid, c, p.priority):
                            out.append((m, rid))
        return out

    def confirmed(self, c: Claim) -> bool:
        """Usable claim: old enough that a racing peer claim would have arrived, or nobody can race us.

        A peer can only contest a resource we are about to enter if it is physically close to it,
        so with no live peer within ``race_radius`` of the robot every claim is usable at once.
        """
        return self.now - c.stamp >= self.confirm - 1e-6 or c.res in self.occupied or not self.contended

    race_radius: float = 4.0
    static_blocks: dict = field(default_factory=dict)  # resource -> obstacle name (from lidar / peer reports)
    waiting_on_me: Callable[[str], bool] = lambda rid: False  # set by the agent from the gossiped wait-for edges
    my_blockers: Callable[[], list] = lambda: []
    follow_ok: Callable[[str, Res], bool] = lambda rid, res: False
    contended: bool = True  # set each tick by the agent: a live peer is within race_radius
    impatient: bool = False  # set by the agent after a long wait: ignore the bookings of standing peers
    peer_moving: Callable[[str], bool] = lambda rid: True
    core_cells: frozenset = frozenset()  # lane cells of the crossing cores (set by the agent)
    core_map: dict = field(default_factory=dict)  # lane cell -> crossing whose core holds it (set by the agent)
    tokens: dict = field(default_factory=dict)  # resource -> (robot, until): DRP anti-relapse precedence

    # ----------------------------------------------------------------- own claims

    def set_route_claims(self, claims: Iterable[tuple[Res, float, float]], now: float,
                         tentative_after: float = math.inf) -> dict[Res, list[Claim]]:
        """Replace the route claims. A resource we already claimed keeps its original claim stamp.

        Claims starting after ``tentative_after`` (the end of the robot's first wait) are *tentative*: they
        take the current time as stamp, so they lose to every other claim. A robot that cannot move before
        its blocker clears must not hold precedence on cells beyond it - that is how a stuck robot's older
        bookings further along its route used to block the very robots that would unblock it.
        """
        new: dict[Res, list[Claim]] = defaultdict(list)
        for res, start, end in claims:
            if start >= tentative_after:
                new[res].append(Claim(res[0], res[1], start, end, now))
                continue
            # keep the age of an existing claim on this resource even if waiting shifted its interval:
            # precedence stays first-come-first-served for as long as the robot keeps needing the cell
            stamp = min((old.stamp for old in self.own.get(res, ()) if old.stamp < now - 1e-9), default=now)
            new[res].append(Claim(res[0], res[1], start, end, stamp))
        self.own = new
        return new

    def clear(self) -> None:
        self.own = defaultdict(list)
        self.extra = defaultdict(list)

    def signature(self) -> int:
        """Cheap change detector: claims quantised to 0.5 s plus the occupied set."""
        return hash((frozenset((c.key, c.idx, round(c.start * 2), round(c.end * 2) if math.isfinite(c.end) else -1)
                               for cs in (self.own, self.extra) for lst in cs.values() for c in lst),
                     frozenset(self.occupied)))

    def snapshot(self, now: float) -> ClaimMsg:
        self.seq += 1
        claims = [(c.key, c.idx, round(c.start, 3), round(c.end, 3) if math.isfinite(c.end) else 1e9,
                   round(c.stamp, 3))
                  for cs in (self.own, self.extra) for lst in cs.values() for c in lst if c.end > now - 1.0]
        return ClaimMsg(self.robot_id, now, self.seq, self.priority(), claims, sorted(self.occupied))

    def all_own(self) -> list[Claim]:
        return [c for cs in (self.own, self.extra) for lst in cs.values() for c in lst]

    def prune(self, now: float) -> None:
        self.now = now
        for cs in (self.own, self.extra):
            for res in list(cs):
                cs[res] = [c for c in cs[res] if c.end > now - 1.0]
                if not cs[res]:
                    del cs[res]

    # ----------------------------------------------------------------- visison ReservationProvider

    def query(self, lane: str, cells: Iterable[int], t_from: float, exclude_robot: str | None = None
              ) -> list[Reservation]:
        out = []
        for k in cells:
            for rid, s, e in self.blocking((lane, k), t_from):
                if rid != exclude_robot:
                    out.append(Reservation(rid, lane, k, s, e))
            if exclude_robot != self.robot_id:
                out += [Reservation(self.robot_id, lane, k, c.start, c.end) for c in self._mine((lane, k))
                        if c.end > t_from]
        return out

    def try_reserve(self, claims: Sequence[Reservation]) -> bool:
        for r in claims:
            for _rid, s, e in self.blocking((r.lane, r.cell), ignore_losers=False):
                if r.start < e and s < r.end:
                    return False
        for r in claims:
            self.extra[(r.lane, r.cell)].append(Claim(r.lane, r.cell, r.start, r.end, self.now))
        return True

    def release(self, claims: Sequence[Reservation]) -> None:
        for r in claims:
            lst = self.extra.get((r.lane, r.cell))
            if not lst:
                continue
            lst[:] = [c for c in lst if not (abs(c.start - r.start) < 1e-6 and abs(c.end - r.end) < 1e-6)]

    def shift(self, robot_id: str, delay: float, after: float, keep: Iterable[Reservation] = ()) -> list[Reservation]:
        if robot_id != self.robot_id:
            return []  # owner-authoritative: a robot can only move its own bookings
        keep_set = {(r.lane, r.cell, round(r.start, 6), round(r.end, 6)) for r in keep}
        for res, lst in self.own.items():
            lst[:] = [replace(c, start=c.start + delay, end=c.end + delay)
                      if c.start >= after and (c.key, c.idx, round(c.start, 6), round(c.end, 6)) not in keep_set
                      else c for c in lst]
        conflicts = []
        for res, lst in self.own.items():
            for c in lst:
                for rid, s, e in self.blocking(res):
                    if c.overlaps(s, e):
                        conflicts.append(Reservation(rid, res[0], res[1], s, e))
        return conflicts
