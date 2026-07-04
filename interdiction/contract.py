"""Exact portal contraction of a rectangular window against a fixed outside.

Any spawn->target path alternates outside segments (through the fixed part
of the map) and inside segments (through the window). Outside segments are
collapsed into weighted edges between "terminals" — portals (outside cells
adjacent to the window), spawns outside the window, and the target if
outside. Window cells stay explicit. Shortest distances in the contracted
graph therefore equal full-grid BFS distances for every assignment of walls
to window cells.
"""

from __future__ import annotations

import heapq
from collections import deque
from dataclasses import dataclass, field

Cell = tuple[int, int]


@dataclass
class ContractedWindow:
    grid: object
    window: frozenset
    free: frozenset                 # window cells that may hold a wall
    adj: dict = field(default_factory=dict)   # cell -> [(cell, weight)]

    def dijkstra(self, window_walls):
        """Per-spawn (distance, window cells on one shortest path).

        (None, None) when the spawn cannot reach the target. Only window
        cells can be walls — outside walls are already baked into the graph.
        """
        blocked = set(window_walls)
        out = []
        for s in self.grid.spawns:
            dist, prev = self._from(s, blocked)
            d = dist.get(self.grid.target)
            if d is None:
                out.append((None, None))
                continue
            cells = []
            cur = self.grid.target
            while cur != s:
                if cur in self.window:
                    cells.append(cur)
                cur = prev[cur]
            if s in self.window:
                cells.append(s)
            out.append((d, cells))
        return out

    def _from(self, src, blocked):
        dist = {src: 0}
        prev = {}
        pq = [(0, src)]
        while pq:
            d, u = heapq.heappop(pq)
            if d > dist.get(u, float("inf")):
                continue
            for v, w in self.adj.get(u, ()):
                if v in blocked:
                    continue
                nd = d + w
                if nd < dist.get(v, float("inf")):
                    dist[v] = nd
                    prev[v] = u
                    heapq.heappush(pq, (nd, v))
        return dist, prev


def _bfs(grid, src, allowed):
    dist = {src: 0}
    q = deque([src])
    while q:
        u = q.popleft()
        for v in grid.neighbors(u):
            if v in allowed and v not in dist:
                dist[v] = dist[u] + 1
                q.append(v)
    return dist


def contract(grid, window_cells, outside_walls) -> ContractedWindow:
    window = frozenset(window_cells)
    inside = window & grid.walkable
    free = window & grid.buildable
    outside_walls = frozenset(outside_walls) - free
    out_open = grid.walkable - window - outside_walls

    portals = {n for v in inside for n in grid.neighbors(v) if n in out_open}

    # terminals: endpoints of outside path segments
    spawn_out = {s for s in grid.spawns if s not in window}
    terminals = set(portals) | spawn_out
    if grid.target not in window:
        terminals.add(grid.target)

    dout = {t: _bfs(grid, t, out_open) for t in terminals}

    adj: dict = {}

    def add(a, b, w):
        adj.setdefault(a, []).append((b, w))
        adj.setdefault(b, []).append((a, w))

    for v in inside:
        for n in grid.neighbors(v):
            if n in inside:
                if v < n:
                    add(v, n, 1)
            elif n in portals:
                add(v, n, 1)

    terms = sorted(terminals)
    for i, a in enumerate(terms):
        for b in terms[i + 1:]:
            if a in spawn_out and b in spawn_out:
                continue            # a path never runs spawn -> spawn
            d = dout[a].get(b)
            if d:
                add(a, b, d)

    return ContractedWindow(grid, window, free, adj)
