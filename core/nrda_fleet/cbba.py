"""Consensus-Based Bundle Algorithm (CBBA): decentralised task allocation.

Each robot greedily builds a bundle of tasks by marginal utility, then reaches
agreement with its peers through the CBBA consensus decision table over the
(winning bid, winning robot, timestamp) triple, the cascade-drop rule when a
task is out-bid, and a purge of a failed peer's tasks. There is no auctioneer
and no central dispatcher: every robot converges on the same conflict-free
assignment from gossiped bid vectors alone.

    Choi, Brunet and How, "Consensus-Based Decentralized Auctions for Robust
    Task Allocation", IEEE Transactions on Robotics 25(4):912-926, 2009.

:mod:`nrda_fleet.allocation` wraps this as layer L4: ``LaneCBBAAgent`` replaces
the straight-line task metric with lane-graph travel distance, and ``FA_CBBA``
holds the weights tuned for an 80 x 50 m warehouse.
"""

from dataclasses import dataclass, field
import math
from typing import Any, Dict, List, Optional, Set, Tuple


@dataclass
class CBBAConfig:
    """Configuration parameters for CBBA auction and consensus."""

    max_bundle_size: int = 4
    weight_priority: float = 100.0
    weight_distance: float = 10.0
    weight_late: float = 5.0
    discount_factor: float = 0.95
    nominal_speed: float = 0.5
    epsilon: float = 1e-6


@dataclass
class CBBALocalState:
    """Local belief state for a single CBBA agent."""

    robot_id: str
    bundle: List[str] = field(default_factory=list)
    path: List[Tuple[float, float]] = field(default_factory=list)
    winning_bids: Dict[str, float] = field(default_factory=dict)
    winning_robots: Dict[str, str] = field(default_factory=dict)
    timestamps: Dict[str, float] = field(default_factory=dict)
    iteration: int = 0
    is_converged: bool = False


class CBBAAgent:
    """Decentralized CBBA agent performing bundle construction and consensus."""

    def __init__(
        self,
        robot_id: str,
        config: Optional[CBBAConfig] = None,
        initial_position: Tuple[float, float] = (0.0, 0.0),
    ) -> None:
        """Initialize CBBA agent with identity and configuration."""
        self.robot_id = robot_id
        self.config = config or CBBAConfig()
        self.position = (float(initial_position[0]), float(initial_position[1]))
        self.state = CBBALocalState(robot_id=robot_id)

    @property
    def winning_robots(self) -> Dict[str, str]:
        """Return winning robots dictionary from local state."""
        return self.state.winning_robots

    @property
    def winning_bids(self) -> Dict[str, float]:
        """Return winning bids dictionary from local state."""
        return self.state.winning_bids

    @property
    def timestamps(self) -> Dict[str, float]:
        """Return timestamps dictionary from local state."""
        return self.state.timestamps

    def update_position(self, position: Tuple[float, float]) -> None:
        """Update current agent position."""
        self.position = (float(position[0]), float(position[1]))

    @staticmethod
    def _euclidean_dist(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
        """Compute Euclidean distance between two 2D points."""
        return math.hypot(p2[0] - p1[0], p2[1] - p1[1])

    def _calculate_route_distance(
        self,
        bundle: List[str],
        task_map: Dict[str, Any],
    ) -> float:
        """Calculate total transit distance for a sequential pickup-dropoff route."""
        total_dist = 0.0
        curr_pos = self.position
        for t_id in bundle:
            task = task_map.get(t_id)
            if not task:
                continue
            pickup = (float(task['pickup'][0]), float(task['pickup'][1]))
            dropoff = (float(task['dropoff'][0]), float(task['dropoff'][1]))
            total_dist += self._euclidean_dist(curr_pos, pickup)
            total_dist += self._euclidean_dist(pickup, dropoff)
            curr_pos = dropoff
        return total_dist

    def compute_marginal_utility(
        self,
        task: Dict[str, Any],
        task_map: Dict[str, Any],
        current_time: float = 0.0,
        min_insert_index: int = 0,
    ) -> Tuple[float, int]:
        """
        Compute marginal utility and optimal insertion index in the current bundle.

        :param task: Candidate task dictionary.
        :param task_map: Lookup map of all known task definitions.
        :param current_time: Current simulation or wall-clock epoch.
        :param min_insert_index: Minimum insertion index (to prevent preempting locked tasks).
        :return: Tuple of (maximum marginal utility, optimal insertion index).
        """
        raw_prio = task.get('priority', 2)
        if isinstance(raw_prio, (int, float)):
            priority_val = float(raw_prio)
        else:
            try:
                priority_val = float(raw_prio)
            except (ValueError, TypeError):
                prio_str = str(raw_prio).strip().upper()
                prio_map = {'LOW': 1.0, 'NORMAL': 2.0, 'HIGH': 3.0, 'CRITICAL': 4.0}
                priority_val = prio_map.get(prio_str, 2.0)

        base_reward = self.config.weight_priority * priority_val
        deadline = task.get('deadline')

        curr_bundle = self.state.bundle
        best_score = -float('inf')
        best_index = len(curr_bundle)

        # Evaluate all candidate insertion positions in the bundle (respecting min_insert_index)
        start_idx = max(0, min(min_insert_index, len(curr_bundle)))
        for m in range(start_idx, len(curr_bundle) + 1):
            cand_bundle = curr_bundle[:m] + [task['task_id']] + curr_bundle[m:]

            # Cumulative distance from robot position up to completion of candidate task
            cum_dist = 0.0
            curr_pos = self.position
            for t_id in cand_bundle:
                t = task_map.get(t_id)
                if not t:
                    continue
                p_up = (float(t['pickup'][0]), float(t['pickup'][1]))
                d_off = (float(t['dropoff'][0]), float(t['dropoff'][1]))
                cum_dist += (
                    self._euclidean_dist(curr_pos, p_up)
                    + self._euclidean_dist(p_up, d_off)
                )
                curr_pos = d_off
                if t_id == task['task_id']:
                    break

            travel_cost = self.config.weight_distance * cum_dist

            late_cost = 0.0
            if deadline is not None and deadline > 0.0:
                speed = max(0.1, self.config.nominal_speed)
                est_arrival = current_time + (cum_dist / speed)
                if est_arrival > deadline:
                    late_cost = self.config.weight_late * (est_arrival - deadline)

            net_utility = base_reward - travel_cost - late_cost
            discounted = (
                self.config.discount_factor ** len(curr_bundle)
            ) * max(0.01, net_utility)

            if discounted > best_score:
                best_score = discounted
                best_index = m

        return round(best_score, 4), best_index

    def _is_bid_higher(
        self,
        bid_a: float,
        agent_a: str,
        bid_b: float,
        agent_b: str,
    ) -> bool:
        """Determine if bid A beats bid B with deterministic lexicographical tie-break."""
        diff = bid_a - bid_b
        if abs(diff) > self.config.epsilon:
            return diff > 0
        # Deterministic tie-breaking on robot ID
        if not agent_b:
            return True
        if not agent_a:
            return False
        return agent_a < agent_b

    def build_bundle(
        self,
        task_map: Dict[str, Any],
        current_time: float = 0.0,
        locked_tasks: Optional[Set[str]] = None,
        full_task_map: Optional[Dict[str, Any]] = None,
    ) -> int:
        """
        Construct or expand task bundle up to max capacity using marginal scoring.

        :param task_map: Dictionary of available candidate tasks.
        :param current_time: Current simulation timestamp.
        :param locked_tasks: Set of task IDs that must not be preempted or reordered.
        :param full_task_map: Complete task definitions lookup (including assigned tasks).
        :return: Number of tasks successfully appended during this phase.
        """
        lookup_map = full_task_map if full_task_map is not None else task_map

        # Determine minimum insertion index to protect locked in-progress tasks
        min_insert_index = 0
        if locked_tasks:
            for t_id in self.state.bundle:
                if t_id in locked_tasks:
                    min_insert_index += 1
                else:
                    break

        added_count = 0
        while len(self.state.bundle) < self.config.max_bundle_size:
            best_task_id: Optional[str] = None
            best_utility = -float('inf')

            for t_id, task in task_map.items():
                if t_id in self.state.bundle:
                    continue

                # Filter by requested_robot allocation constraint (AUTO vs DIRECT)
                req_robot = task.get('requested_robot')
                if (
                    req_robot
                    and req_robot not in ('AUTO', 'NONE', '')
                    and req_robot != self.robot_id
                ):
                    continue

                util, _ = self.compute_marginal_utility(
                    task,
                    lookup_map,
                    current_time,
                    min_insert_index=min_insert_index,
                )
                curr_winning_bid = self.state.winning_bids.get(t_id, 0.0)
                curr_winning_robot = self.state.winning_robots.get(t_id, '')

                if self._is_bid_higher(util, self.robot_id, curr_winning_bid, curr_winning_robot):
                    if util > best_utility + self.config.epsilon:
                        best_utility = util
                        best_task_id = t_id
                    elif abs(util - best_utility) <= self.config.epsilon:
                        # Tie breaker between two tasks for same robot: lowest task ID
                        if best_task_id is None or t_id < best_task_id:
                            best_utility = util
                            best_task_id = t_id

            if best_task_id is None:
                break

            self.state.bundle.append(best_task_id)
            self.state.winning_bids[best_task_id] = best_utility
            self.state.winning_robots[best_task_id] = self.robot_id
            self.state.timestamps[best_task_id] = current_time
            added_count += 1

        self._rebuild_path(lookup_map)
        return added_count

    def _rebuild_path(self, task_map: Dict[str, Any]) -> None:
        """Reconstruct waypoints sequence from current task bundle."""
        new_path: List[Tuple[float, float]] = []
        for t_id in self.state.bundle:
            task = task_map.get(t_id)
            if task:
                new_path.append((float(task['pickup'][0]), float(task['pickup'][1])))
                new_path.append((float(task['dropoff'][0]), float(task['dropoff'][1])))
        self.state.path = new_path

    def resolve_conflicts(
        self,
        peer_id: str,
        peer_iteration: int,
        peer_winning_bids: Dict[str, float],
        peer_winning_robots: Dict[str, str],
        peer_timestamps: Dict[str, float],
        task_map: Dict[str, Any],
        current_time: float = 0.0,
        locked_tasks: Optional[Set[str]] = None,
    ) -> bool:
        """
        Execute CBBA consensus decision matrix to resolve bidding conflicts.

        Implements standard CBBA Table 1 conflict resolution rules between agent i (self)
        and agent k (peer), updating local beliefs and applying the cascade drop rule.

        :return: True if local state changed, False otherwise.
        """
        if peer_id == self.robot_id:
            return False

        prev_bundle = list(self.state.bundle)
        prev_winning_robots = dict(self.state.winning_robots)
        prev_winning_bids = dict(self.state.winning_bids)

        all_tasks = set(self.state.winning_bids.keys()) | set(peer_winning_bids.keys())

        for t_id in all_tasks:
            y_i = self.state.winning_bids.get(t_id, 0.0)
            z_i = self.state.winning_robots.get(t_id, '')
            s_i = self.state.timestamps.get(t_id, 0.0)

            y_k = peer_winning_bids.get(t_id, 0.0)
            z_k = peer_winning_robots.get(t_id, '')
            s_k = peer_timestamps.get(t_id, 0.0)

            # Protect locked in-progress tasks: cannot be outbid or yielded
            if locked_tasks and t_id in locked_tasks and z_i == self.robot_id:
                continue

            # CBBA consensus decision logic
            if z_k == peer_id:
                # Peer claims task
                if z_i == self.robot_id:
                    # Both agents claim the task: compare bids
                    if self._is_bid_higher(y_k, peer_id, y_i, self.robot_id):
                        self.state.winning_bids[t_id] = y_k
                        self.state.winning_robots[t_id] = peer_id
                        self.state.timestamps[t_id] = s_k
                elif z_i == peer_id:
                    # Peer updates its own bid
                    self.state.winning_bids[t_id] = y_k
                    self.state.timestamps[t_id] = s_k
                elif not z_i:
                    # Previously unassigned
                    self.state.winning_bids[t_id] = y_k
                    self.state.winning_robots[t_id] = peer_id
                    self.state.timestamps[t_id] = s_k
                else:
                    # z_i is another third-party robot m
                    if s_k > s_i or self._is_bid_higher(y_k, peer_id, y_i, z_i):
                        self.state.winning_bids[t_id] = y_k
                        self.state.winning_robots[t_id] = peer_id
                        self.state.timestamps[t_id] = s_k

            elif z_k == self.robot_id:
                # Peer agrees that self is the winner
                if z_i == peer_id:
                    # Self thought peer won, but peer yielded
                    self.state.winning_bids[t_id] = 0.0
                    self.state.winning_robots[t_id] = ''
                    self.state.timestamps[t_id] = s_k

            elif not z_k:
                # Peer believes nobody has won this task
                if z_i == peer_id:
                    # Peer released task
                    self.state.winning_bids[t_id] = 0.0
                    self.state.winning_robots[t_id] = ''
                    self.state.timestamps[t_id] = s_k
                elif z_i != self.robot_id and s_k > s_i:
                    self.state.winning_bids[t_id] = 0.0
                    self.state.winning_robots[t_id] = ''
                    self.state.timestamps[t_id] = s_k

            else:
                # Peer claims third party m is winner
                m = z_k
                if z_i == self.robot_id:
                    if self._is_bid_higher(y_k, m, y_i, self.robot_id):
                        self.state.winning_bids[t_id] = y_k
                        self.state.winning_robots[t_id] = m
                        self.state.timestamps[t_id] = s_k
                elif z_i == peer_id:
                    self.state.winning_bids[t_id] = y_k
                    self.state.winning_robots[t_id] = m
                    self.state.timestamps[t_id] = s_k
                elif not z_i:
                    self.state.winning_bids[t_id] = y_k
                    self.state.winning_robots[t_id] = m
                    self.state.timestamps[t_id] = s_k
                elif z_i == m:
                    if s_k >= s_i:
                        self.state.winning_bids[t_id] = y_k
                        self.state.timestamps[t_id] = s_k
                else:
                    # z_i is another third party m'
                    if s_k > s_i or self._is_bid_higher(y_k, m, y_i, z_i):
                        self.state.winning_bids[t_id] = y_k
                        self.state.winning_robots[t_id] = m
                        self.state.timestamps[t_id] = s_k

        # Apply CBBA Cascade Drop Rule
        # Find earliest index in bundle where self is no longer the winning robot
        first_outbid_idx: Optional[int] = None
        for idx, t_id in enumerate(self.state.bundle):
            if locked_tasks and t_id in locked_tasks:
                continue
            if self.state.winning_robots.get(t_id) != self.robot_id:
                first_outbid_idx = idx
                break

        if first_outbid_idx is not None:
            # Drop task at first_outbid_idx and all subsequent tasks
            dropped_tasks = self.state.bundle[first_outbid_idx:]
            self.state.bundle = self.state.bundle[:first_outbid_idx]

            for d_id in dropped_tasks:
                if self.state.winning_robots.get(d_id) == self.robot_id:
                    self.state.winning_robots[d_id] = ''
                    self.state.winning_bids[d_id] = 0.0
                    self.state.timestamps[d_id] = current_time

        self._rebuild_path(task_map)

        # Return True if bundle or task beliefs (winner / bid) changed
        if self.state.bundle != prev_bundle:
            return True

        all_checked = set(self.state.winning_robots.keys()) | set(prev_winning_robots.keys())
        for t in all_checked:
            if self.state.winning_robots.get(t, '') != prev_winning_robots.get(t, ''):
                return True
            diff = abs(
                self.state.winning_bids.get(t, 0.0) - prev_winning_bids.get(t, 0.0)
            )
            if diff > self.config.epsilon:
                return True

        return False

    def is_consistent_with(self, peer_agent: 'CBBAAgent') -> bool:
        """Check if winning allocations and bids match between self and peer."""
        all_tasks = set(self.state.winning_robots.keys()) | set(
            peer_agent.state.winning_robots.keys()
        )
        for t_id in all_tasks:
            z_i = self.state.winning_robots.get(t_id, '')
            z_k = peer_agent.state.winning_robots.get(t_id, '')
            if z_i != z_k:
                return False
            y_i = self.state.winning_bids.get(t_id, 0.0)
            y_k = peer_agent.state.winning_bids.get(t_id, 0.0)
            if abs(y_i - y_k) > self.config.epsilon:
                return False
        return True

    def purge_failed_peer_tasks(
        self,
        failed_robot_id: str,
        current_time: float = 0.0,
    ) -> List[str]:
        """
        Purge beliefs for all tasks held by a failed peer robot.

        When a peer robot is confirmed failed, resets winning_robots to ''
        and winning_bids to 0.0 for all tasks that the peer was believed to hold.
        This enables local robot to consider those tasks as available for re-bidding.
        Returns the list of task IDs that were freed.
        """
        freed_tasks: List[str] = []
        for t_id, winner in list(self.state.winning_robots.items()):
            if winner == failed_robot_id:
                self.state.winning_robots[t_id] = ''
                self.state.winning_bids[t_id] = 0.0
                self.state.timestamps[t_id] = current_time
                freed_tasks.append(t_id)
        return freed_tasks

    def reconcile_reconnection(
        self,
        peer_winning_robots: Dict[str, str],
        peer_timestamps: Dict[str, float],
        peer_winning_bids: Optional[Dict[str, float]] = None,
    ) -> List[str]:
        """
        Reconcile local beliefs when reconnecting after communication loss.

        If a task was reallocated to another robot while self was disconnected
        (indicated by a strictly newer peer timestamp and non-self winner),
        yields the task from self's bundle to prevent stale task resurrection
        or duplicate ownership. Returns list of yielded tasks.
        """
        yielded_tasks: List[str] = []
        for t_id in list(self.state.bundle):
            peer_winner = peer_winning_robots.get(t_id, '')
            peer_ts = peer_timestamps.get(t_id, 0.0)
            local_ts = self.state.timestamps.get(t_id, 0.0)

            if peer_winner and peer_winner != self.robot_id and peer_ts > local_ts:
                self.state.bundle.remove(t_id)
                if t_id in self.state.path:
                    self.state.path.remove(t_id)
                self.state.winning_robots[t_id] = peer_winner
                if peer_winning_bids and t_id in peer_winning_bids:
                    self.state.winning_bids[t_id] = peer_winning_bids[t_id]
                self.state.timestamps[t_id] = peer_ts
                yielded_tasks.append(t_id)

        return yielded_tasks
