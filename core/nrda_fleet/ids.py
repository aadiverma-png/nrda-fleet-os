"""Robot identifiers: ``AMR_1`` ... ``AMR_N`` everywhere (model names, namespaces, frames, messages)."""

from __future__ import annotations

import re

_PATTERN = re.compile(r"^AMR_([1-9][0-9]*)$")


def robot_id(index: int) -> str:
    """``robot_id(7) == "AMR_7"``. Indices start at 1."""
    if index < 1:
        raise ValueError(f"robot index must be >= 1, got {index}")
    return f"AMR_{index}"


def parse_robot_id(rid: str) -> int:
    """Numeric index of a robot id; raises ``ValueError`` for anything that is not ``AMR_<n>``."""
    m = _PATTERN.match(rid)
    if not m:
        raise ValueError(f"not a robot id: {rid!r} (expected AMR_<n>)")
    return int(m.group(1))


def is_robot_id(rid: str) -> bool:
    return bool(_PATTERN.match(rid))


def id_key(rid: str) -> tuple[int, str]:
    """Sort key: numeric order (AMR_2 before AMR_10); non-robot ids sort last."""
    m = _PATTERN.match(rid)
    return (int(m.group(1)), "") if m else (1 << 30, rid)


def fleet_ids(count: int) -> list[str]:
    return [robot_id(i) for i in range(1, count + 1)]
