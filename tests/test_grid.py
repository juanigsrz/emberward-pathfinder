import pytest

from interdiction.grid import parse_map


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
