import textwrap
from itertools import combinations

import pytest


def brute_force_opt(grid):
    """Exhaustive optimum over all wall subsets. Only for tiny maps."""
    cells = sorted(grid.buildable)
    assert len(cells) <= 14, "brute force limited to 2^14 subsets"
    best_val, best_walls = -1, set()
    for r in range(len(cells) + 1):
        for combo in combinations(cells, r):
            val, _ = grid.evaluate(set(combo))
            if val is not None and val > best_val:
                best_val, best_walls = val, set(combo)
    return best_val, best_walls


@pytest.fixture
def make_map(tmp_path):
    """Write a dedented map string to a temp file, return its path."""
    def _make(text, name="m.txt"):
        f = tmp_path / name
        f.write_text(textwrap.dedent(text).strip() + "\n")
        return str(f)
    return _make
