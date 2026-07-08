"""Full-map master run: valid global upper bound (and sometimes a better incumbent)."""

from __future__ import annotations

import math


def run_bound(grid, master, incumbent_walls, time_limit,
              incumbent_anchors=None):
    """Solve the full map with the incumbent as MIP start.

    The returned result's `bound` (Gurobi ObjBound of the master, which is a
    relaxation of the true problem restricted to generated cuts) is a valid
    upper bound on the true maximin. `walls`/`maximin` may improve on the
    incumbent — the caller should keep the better of the two.
    """
    warm = set(incumbent_walls) if incumbent_walls else None
    return master.solve(time_limit=time_limit, warm_start=warm,
                        warm_anchors=incumbent_anchors)


def gap(incumbent, bound):
    """Relative optimality gap; inf when there is no meaningful incumbent."""
    if not incumbent or incumbent <= 0:
        return math.inf
    return (bound - incumbent) / incumbent
