"""Command line entry point.

    myenv/bin/python -m interdiction maps/endless.txt \
        --seed maps/endless_annealing_solution.txt --time 3600
"""

from __future__ import annotations

import argparse
import os
import random
import sys

from interdiction.bound import gap, run_bound
from interdiction.grid import parse_map, parse_solution, write_solution
from interdiction.lns import run_lns
from interdiction.master import MasterSolver


def _summary(grid, walls, bound_val=None):
    val, per = grid.evaluate(walls)
    lines = [f"walls: {len(walls)}"]
    for s, d in zip(grid.spawns, per):
        lines.append(f"spawn {s}: distance {d}")
    lines.append(f"maximin: {val}")
    if bound_val is not None:
        lines.append(f"bound: {bound_val:.1f}")
        lines.append(f"gap: {gap(val, bound_val):.3f}")
    return "\n".join(lines)


def main(argv=None) -> int:
    p = argparse.ArgumentParser(prog="interdiction")
    p.add_argument("map")
    p.add_argument("--seed", help="solution file to seed initial walls from")
    p.add_argument("--time", type=float, default=3600.0,
                   help="total wall-clock budget in seconds")
    p.add_argument("--bound-frac", type=float, default=0.25,
                   help="fraction of budget for the final full-map bound run")
    p.add_argument("--subsolve-time", type=float, default=15.0)
    p.add_argument("--no-corridor-hint", action="store_true",
                   help="disable corridor-structure constraints in LNS windows")
    p.add_argument("--window-sizes", default="12,16,20",
                   help="comma-separated LNS window sizes")
    p.add_argument("--rng-seed", type=int, default=0)
    p.add_argument("--out", help="solution output path")
    p.add_argument("--exact", action="store_true",
                   help="single full-map exact solve, no LNS")
    p.add_argument("--eval-only", action="store_true",
                   help="evaluate seed/preset walls and exit")
    args = p.parse_args(argv)

    grid = parse_map(args.map)
    walls = set(grid.preset_walls)
    if args.seed:
        walls |= parse_solution(grid, args.seed)
    val, _ = grid.evaluate(walls)

    if args.eval_only:
        if val is None:
            print("error: initial walls disconnect a spawn", file=sys.stderr)
            return 2
        print(_summary(grid, walls))
        return 0

    if val is None:
        # preset/seed walls are only warm-start hints — recover, don't abort
        print("warning: initial walls disconnect a spawn — "
              "starting from empty walls", file=sys.stderr)
        walls = set()

    rng = random.Random(args.rng_seed)
    master = MasterSolver(grid, rng=rng, gurobi_seed=args.rng_seed,
                          output=args.exact)
    out = args.out or os.path.splitext(args.map)[0] + "_milp_solution.txt"

    best, bound_val = walls, None
    try:
        if args.exact:
            res = master.solve(time_limit=args.time,
                               warm_start=walls or None)
            if res.walls is not None:
                best = res.walls
            bound_val = res.bound
        else:
            window_sizes = tuple(
                int(x) for x in args.window_sizes.split(","))
            lns = run_lns(grid, walls,
                          total_time=args.time * (1 - args.bound_frac),
                          subsolve_time=args.subsolve_time, rng=rng,
                          corridor_hint=not args.no_corridor_hint,
                          window_sizes=window_sizes)
            best = lns.walls
            for elapsed, it, v in lns.trajectory:
                print(f"[lns] t={elapsed:7.1f}s iter={it:4d} maximin={v}")
            if not lns.interrupted:
                bres = run_bound(grid, master, best,
                                 time_limit=args.time * args.bound_frac)
                bound_val = bres.bound
                if bres.maximin is not None and \
                        bres.maximin > grid.evaluate(best)[0]:
                    best = bres.walls
    except KeyboardInterrupt:
        print("\ninterrupted — writing best solution so far", file=sys.stderr)

    write_solution(grid, best, out)
    print(_summary(grid, best, bound_val))
    print(f"solution written to {out}")
    return 0
