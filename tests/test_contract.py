import random

from interdiction.contract import contract
from interdiction.grid import parse_map


def _window(grid, r0, c0, size):
    return {(r, c)
            for r in range(max(r0, 0), min(r0 + size, grid.rows))
            for c in range(max(c0, 0), min(c0 + size, grid.cols))}


def _random_map(make_map, rng, i):
    while True:
        rows, cols = rng.randint(4, 8), rng.randint(4, 8)
        cells = [["." for _ in range(cols)] for _ in range(rows)]
        for r in range(rows):
            for c in range(cols):
                if rng.random() < 0.12:
                    cells[r][c] = "#"
        cells[0][0] = "S"
        if rng.random() < 0.4:
            cells[rows - 1][0] = "S"
        cells[rows - 1][cols - 1] = "T"
        text = "\n".join("".join(row) for row in cells)
        try:
            return parse_map(make_map(text, name=f"case{i}.txt"))
        except ValueError:
            continue    # unreachable spawn etc. — draw again


def test_contraction_exactness_property(make_map):
    """THE invariant: contracted Dijkstra == full-grid BFS, always."""
    rng = random.Random(42)
    checked = 0
    for i in range(150):
        grid = _random_map(make_map, rng, i)
        window = _window(grid, rng.randint(-1, grid.rows - 2),
                         rng.randint(-1, grid.cols - 2), rng.randint(2, 4))
        free = window & grid.buildable
        walls = {v for v in grid.buildable if rng.random() < 0.25}
        outside_walls = walls - free
        window_walls = walls & free

        cw = contract(grid, window, outside_walls)
        got = cw.dijkstra(window_walls)
        dist = grid.dist_field(walls)
        for k, s in enumerate(grid.spawns):
            expected = dist.get(s)
            assert got[k][0] == expected, (
                f"case {i}: spawn {s} contracted={got[k][0]} bfs={expected}")
            checked += 1
    assert checked >= 150


def test_contraction_path_cells_are_window_cells_of_shortest_path(make_map):
    grid = parse_map(make_map("""
        S....
        .....
        .....
        ....T
    """))
    window = _window(grid, 1, 1, 2)
    cw = contract(grid, window, set())
    (d, cells), = cw.dijkstra(frozenset())
    assert d == 7
    assert all(v in window for v in cells)
    # blocking every reported window cell must not break exactness
    got = cw.dijkstra(frozenset(cells))
    assert got[0][0] == grid.dist_field(set(cells))[grid.spawns[0]]


def test_contraction_with_target_inside_window(make_map):
    grid = parse_map(make_map("""
        S....
        .....
        ....T
    """))
    window = _window(grid, 1, 3, 2)          # covers T
    cw = contract(grid, window, set())
    (d, _), = cw.dijkstra(frozenset())
    assert d == 6


def test_contraction_reports_disconnect(make_map):
    grid = parse_map(make_map("""
        S..
        ...
        ..T
    """))
    window = _window(grid, 0, 0, 3)
    cw = contract(grid, window, set())
    got = cw.dijkstra(frozenset({(0, 1), (1, 0)}))   # seals the spawn
    assert got[0] == (None, None)
