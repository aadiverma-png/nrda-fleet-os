"""Wait-for graph: cyclic-wait detection for the deadlock resolution protocol.

Nodes are robots. A directed edge u -> v means robot u is waiting for robot v
to release a resource, so a cycle in this graph is a circular wait: the
classical necessary condition for deadlock in a resource-allocation system.

:meth:`WaitForGraph.find_cycles` enumerates the *elementary cycles* by
depth-first search, recording one on each back edge into the current stack and
canonicalising it by rotating the lowest-numbered member to the front. Every
robot therefore derives the same cycle set from the same gossiped edges, with
no negotiation. Note this is cycle enumeration, not a strongly-connected-
component decomposition: a clique of five mutually waiting robots yields every
elementary cycle among them, not one component of five, and
``agent._check_deadlock`` depends on that per-cycle behaviour.

    Holt, "Some Deadlock Properties of Computer Systems", ACM Computing
    Surveys 4(3):179-196, 1972 (the wait-for graph and circular wait).
    Knapp, "Deadlock Detection in Distributed Databases", ACM Computing
    Surveys 19(4):303-328, 1987 (detection from a replicated graph).

The fleet rebuilds this graph each planning tick from the ``waiting_for`` edges
gossiped on :class:`nrda_fleet.bus.RobotStateMsg`. Persistence, grouping,
classification and recovery are :mod:`nrda_fleet.deadlock` (the DRP).
"""

import time
from typing import Any, Dict, List, Optional, Set, Tuple

Position = Tuple[int, int]
"""Edge metadata only. The fleet passes ``(0, 0)``: NRDA-FA keys waits by
robot, not by grid cell, so there is no grid behind this type."""


class WaitForGraph:
    """
    Directed Wait-For Graph (WFG) representing resource dependencies.

    Nodes: Robot IDs.
    Directed edge (u -> v): Robot u is waiting for Robot v to vacate or release a resource.
    """

    def __init__(self) -> None:
        """Initialize adjacency lists and edge metadata."""
        # Map: waiter_id -> Dict[blocking_id, Dict[str, Any]]
        self._adj: Dict[str, Dict[str, Dict[str, Any]]] = {}

    def add_wait(
        self,
        waiter_id: str,
        blocking_id: str,
        resource: Position,
        time_step: int,
        timestamp: Optional[float] = None,
    ) -> None:
        """Add or update a directed waiting edge waiter_id -> blocking_id."""
        if waiter_id == blocking_id:
            return
        t_now = timestamp if timestamp is not None else time.time()
        self._adj.setdefault(waiter_id, {})[blocking_id] = {
            'resource': resource,
            'time_step': time_step,
            'created_at': t_now,
        }

    def remove_wait(self, waiter_id: str, blocking_id: Optional[str] = None) -> None:
        """Remove waiting edge(s) for waiter_id."""
        if waiter_id not in self._adj:
            return
        if blocking_id is None:
            self._adj.pop(waiter_id, None)
        else:
            self._adj[waiter_id].pop(blocking_id, None)
            if not self._adj[waiter_id]:
                self._adj.pop(waiter_id, None)

    def clear_robot(self, robot_id: str) -> None:
        """Remove all incoming and outgoing edges involving robot_id."""
        self._adj.pop(robot_id, None)
        for waiter in list(self._adj.keys()):
            self._adj[waiter].pop(robot_id, None)
            if not self._adj[waiter]:
                self._adj.pop(waiter, None)

    def get_waiting_for(self, waiter_id: str) -> Optional[str]:
        """Return the primary blocking robot ID that waiter_id is waiting on."""
        outgoing = self._adj.get(waiter_id)
        if not outgoing:
            return None
        return next(iter(outgoing.keys()))

    def get_edges(self) -> List[Tuple[str, str, Position, int]]:
        """Return list of all directed edges as (waiter, blocker, resource, time_step)."""
        edges = []
        for u, targets in self._adj.items():
            for v, meta in targets.items():
                edges.append((u, v, meta['resource'], meta['time_step']))
        return edges

    def find_cycles(self) -> List[List[str]]:
        """
        Find all simple elementary cycles in the WFG using DFS.

        Returns cycles in canonical order (minimum element first) without duplicates.
        """
        visited: Set[str] = set()
        in_stack: Set[str] = set()
        stack: List[str] = []
        cycles: List[List[str]] = []
        seen_canonical: Set[Tuple[str, ...]] = set()

        def dfs(u: str) -> None:
            visited.add(u)
            in_stack.add(u)
            stack.append(u)

            for v in self._adj.get(u, {}).keys():
                if v not in visited:
                    dfs(v)
                elif v in in_stack:
                    # Cycle detected: extract subpath from v to end of stack
                    idx = stack.index(v)
                    cycle = stack[idx:]
                    # Canonical rotation: shift smallest robot_id to index 0
                    min_idx = cycle.index(min(cycle))
                    canonical = tuple(cycle[min_idx:] + cycle[:min_idx])
                    if canonical not in seen_canonical and len(canonical) >= 2:
                        seen_canonical.add(canonical)
                        cycles.append(list(canonical))

            stack.pop()
            in_stack.remove(u)

        all_nodes = set(self._adj.keys())
        for targets in self._adj.values():
            all_nodes.update(targets.keys())

        for node in sorted(all_nodes):
            if node not in visited:
                dfs(node)

        return cycles
