import os

from interdiction.cli import main
from interdiction.grid import parse_map, parse_solution


def test_eval_only(make_map, capsys):
    path = make_map("""
        S....
        ..W..
        ....T
    """)
    assert main([path, "--eval-only"]) == 0
    out = capsys.readouterr().out
    assert "maximin" in out


def test_exact_solve_writes_solution(make_map, tmp_path, capsys):
    path = make_map("""
        S....
        .....
        ....T
    """)
    out_file = str(tmp_path / "sol.txt")
    assert main([path, "--exact", "--time", "30", "--out", out_file]) == 0
    assert os.path.exists(out_file)
    grid = parse_map(path)
    walls = parse_solution(grid, out_file)
    val, _ = grid.evaluate(walls)
    printed = capsys.readouterr().out
    assert f"maximin: {val}" in printed
    assert "bound:" in printed and "gap:" in printed


def test_lns_pipeline_smoke(make_map, tmp_path, capsys):
    path = make_map("""
        S......
        .......
        .......
        ......T
    """)
    out_file = str(tmp_path / "sol.txt")
    assert main([path, "--time", "20", "--bound-frac", "0.5",
                 "--subsolve-time", "4", "--rng-seed", "1",
                 "--out", out_file]) == 0
    grid = parse_map(path)
    walls = parse_solution(grid, out_file)
    val, _ = grid.evaluate(walls)
    baseline, _ = grid.evaluate(set())
    assert val >= baseline
