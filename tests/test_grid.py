import pytest

from interdiction.grid import parse_map, parse_solution, write_solution


def test_parse_symbols_and_derived_sets(make_map):
    g = parse_map(make_map("""
        S.X#.
        ..W.T
    """))
    assert (g.rows, g.cols) == (2, 5)
    assert g.spawns == ((0, 0),)
    assert g.target == (1, 4)
    assert g.obstacles == frozenset({(0, 3)})
    assert g.unbuildables == frozenset({(0, 2)})
    assert g.preset_walls == frozenset({(1, 2)})
    # walkable = everything except '#'
    assert g.walkable == frozenset(
        (r, c) for r in range(2) for c in range(5)) - {(0, 3)}
    # buildable = walkable minus spawns, target, unbuildables
    assert g.buildable == g.walkable - {(0, 0), (1, 4), (0, 2)}
    assert g.preset_walls <= g.buildable


def test_neighbors_respect_bounds_and_obstacles(make_map):
    g = parse_map(make_map("""
        S.#
        ..T
    """))
    assert set(g.neighbors((0, 0))) == {(0, 1), (1, 0)}
    assert set(g.neighbors((0, 1))) == {(0, 0), (1, 1)}  # (0,2) is '#'


def test_dist_field_and_evaluate(make_map):
    g = parse_map(make_map("""
        S....
        .....
        ..#..
        .....
        ....T
    """))
    dist = g.dist_field(set())
    assert dist[g.target] == 0
    assert dist[(0, 0)] == 8
    assert (2, 2) not in dist          # obstacle not in field
    maximin, per = g.evaluate(set())
    assert (maximin, per) == (8, (8,))


def test_evaluate_with_walls_and_disconnect(make_map):
    g = parse_map(make_map("""
        S....
        .....
        T....
    """))
    assert g.evaluate(set())[0] == 2
    assert g.evaluate({(1, 0)})[0] == 4               # one detour
    assert g.evaluate({(1, 0), (1, 1)})[0] == 6       # longer detour
    maximin, per = g.evaluate({(1, 0), (0, 1)})       # spawn sealed off
    assert maximin is None and per == (None,)


def test_evaluate_multi_spawn_maximin(make_map):
    g = parse_map(make_map("""
        S...T
        .....
        S....
    """))
    maximin, per = g.evaluate(set())
    assert per == (4, 6)
    assert maximin == 4


def test_shortest_path_descends_dist_field(make_map):
    g = parse_map(make_map("""
        S....
        .....
        ..#..
        .....
        ....T
    """))
    path = g.shortest_path(set(), (0, 0))
    assert path[0] == (0, 0) and path[-1] == g.target
    assert len(path) - 1 == 8
    # consecutive cells adjacent
    for a, b in zip(path, path[1:]):
        assert abs(a[0] - b[0]) + abs(a[1] - b[1]) == 1
    assert g.shortest_path({(0, 1), (1, 0)}, (0, 0)) is None


def test_shortest_path_rng_sampling(make_map):
    import random
    g = parse_map(make_map("""
        S...
        ....
        ...T
    """))
    rng = random.Random(0)
    seen = {tuple(g.shortest_path(set(), (0, 0), rng=rng)) for _ in range(50)}
    assert len(seen) > 1                      # samples multiple shortest paths
    assert all(len(p) - 1 == 5 for p in seen)  # all optimal


def test_manhattan_parity(make_map):
    g = parse_map(make_map("""
        S...T
        .....
        S....
    """))
    assert g.manhattan_parity((0, 0)) == 0    # distance 4
    assert g.manhattan_parity((2, 0)) == 0    # distance 6


def test_solution_roundtrip(make_map, tmp_path):
    g = parse_map(make_map("""
        S....
        .....
        ....T
    """))
    walls = {(0, 2), (1, 2)}
    out = str(tmp_path / "sol.txt")
    write_solution(g, walls, out)
    assert (tmp_path / "sol.txt").read_text() == "S.W..\n..W..\n....T\n"
    assert parse_solution(g, out) == walls


def test_parse_solution_hash_convention_ignores_base_obstacles(make_map):
    base = parse_map(make_map("""
        S.#..
        .....
        ....T
    """, name="base.txt"))
    # annealing/genetic convention: placed walls written as '#'
    sol = make_map("""
        S.#.#
        ..#..
        ....T
    """, name="sol.txt")
    # (0,2) is a base obstacle -> not a wall; (0,4) and (1,2) are walls
    assert parse_solution(base, sol) == {(0, 4), (1, 2)}


@pytest.mark.parametrize("text,msg", [
    ("....\n.T..", "no spawn"),
    ("S...\n....", "no target"),
    ("S.T.\n.T..", "multiple T"),
    ("S?T.", "unknown character"),
    ("S#T", "unreachable"),
    ("S.T\n....", "rectangular"),
])
def test_parse_errors(make_map, text, msg):
    with pytest.raises(ValueError, match=msg):
        parse_map(make_map(text))
