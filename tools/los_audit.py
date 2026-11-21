#!/usr/bin/env python3
"""Audit line-of-sight gating: does the bot withhold fire on shots that are
actually clear?

M2 added `los = map_.shot_los(muzzle, centre)`, and no local check covers it --
verify_map.py tests terrain and solid but never shot_los. On the recorded stream
that gate cut firing on in-range enemies from 100% of ticks to 67%, and the
submitted score fell from 1.42 to 0.27 in the same step. This measures the gate
against independent geometry.

Ground truth here is a fine-sampled ray against the Python prism model
(0.5 m steps, point test), accelerated by a bounding-box grid so it is fast
enough to run the whole stream.

    python3 tools/los_audit.py [--bots PATH=PATH ...]
"""
import argparse
import gzip
import json
import math
import os
import subprocess
import sys
from collections import defaultdict

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from trace_assert import Geom

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CELL = 16.0


class Grid:
    """Prism containment with a bounding-box grid, so a point test touches a
    handful of candidate prisms instead of all ~259."""

    def __init__(self, g):
        self.g = g
        self.cells = defaultdict(list)
        self.prisms = []
        for base, top, ring, holes in g.prisms:
            xs = [ring[2 * i] for i in range(len(ring) // 2)]
            ys = [ring[2 * i + 1] for i in range(len(ring) // 2)]
            idx = len(self.prisms)
            self.prisms.append((base, top, ring, holes, min(xs), max(xs), min(ys), max(ys)))
            for ci in range(int(math.floor(min(xs) / CELL)), int(math.floor(max(xs) / CELL)) + 1):
                for cj in range(int(math.floor(min(ys) / CELL)), int(math.floor(max(ys) / CELL)) + 1):
                    self.cells[(ci, cj)].append(idx)

    def solid(self, x, y, z, eps=1e-4):
        g = self.g
        for i in self.cells.get((int(math.floor(x / CELL)), int(math.floor(y / CELL))), ()):
            base, top, ring, holes, x0, x1, y0, y1 = self.prisms[i]
            if x < x0 or x > x1 or y < y0 or y > y1:
                continue
            if not (base < z + eps and top > z - eps):
                continue
            if not g._in_poly(ring, x, y):
                continue
            if any(g._in_poly(hh, x, y) for hh in holes):
                continue
            return True
        return False


def los_fine(grid, g, a, b, step=0.5):
    """Fine-sampled independent truth: is any solid on the segment?"""
    L = math.dist(a, b)
    n = max(1, int(L / step))
    for i in range(1, n + 1):
        t = i / n
        x = a[0] + (b[0] - a[0]) * t
        y = a[1] + (b[1] - a[1]) * t
        z = a[2] + (b[2] - a[2]) * t
        if z <= g.terrain_h(x, y):
            return False
        if grid.solid(x, y, z):
            return False
    return True


def los_cpp_rule(grid, g, a, b):
    """src/map.cpp Map::shot_los: ~2 m point sampling, endpoints skipped."""
    dx, dy, dz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
    L = math.sqrt(dx * dx + dy * dy + dz * dz)
    if L < 1e-6:
        return True
    steps = max(1, int(L / 2.0))
    for i in range(1, steps):
        t = i / steps
        x, y, z = a[0] + dx * t, a[1] + dy * t, a[2] + dz * t
        if z <= g.terrain_h(x, y):
            return False
        if grid.solid(x, y, z):
            return False
    return True


def run_bot(bot, hello, obs):
    p = subprocess.Popen([bot], stdin=subprocess.PIPE, stdout=subprocess.PIPE, bufsize=0)

    def send(m):
        p.stdin.write((json.dumps(m, separators=(",", ":")) + "\n").encode())
        p.stdin.flush()

    out = []
    send(hello)
    if p.stdout.readline() is None:
        return None
    for o in obs:
        send(o)
        line = p.stdout.readline()
        out.append(json.loads(line) if line else None)
    try:
        p.stdin.close()
        p.wait(timeout=3)
    except Exception:
        p.kill()
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bots", nargs="*", default=[])
    ap.add_argument("--stream", default=os.path.join(ROOT, "check/stream.jsonl.gz"))
    ap.add_argument("--arena", default=os.path.join(ROOT, "arena"))
    a = ap.parse_args()

    g = Geom(a.arena)
    grid = Grid(g)
    msgs = [json.loads(l) for l in gzip.open(a.stream, "rt") if l.strip()]
    hello, obs = msgs[0], [m for m in msgs if m.get("type") == "obs"]
    hello["map"]["dir"] = os.path.abspath(a.arena)
    gr = hello["config"]["weapons"]["gun"]["range"]

    bots = dict(b.split("=", 1) for b in a.bots) if a.bots else {"HEAD": os.path.join(ROOT, "build/bot")}
    acts = {k: run_bot(v, hello, obs) for k, v in bots.items()}

    rows = []
    mism = []
    for i, o in enumerate(obs):
        s = o["self"]
        mz = s["pos"][2] + s["stats"].get("muzzle_height", 7.0)
        muz = (s["pos"][0], s["pos"][1], mz)
        for e in o["mechs"]:
            if e.get("npc"):
                continue
            sc = 1.0 + 0.05 * max(0.0, e.get("cubes") or 0.0)
            pt = (e["pos"][0], e["pos"][1], e["pos"][2] + 4.5 * sc)
            if math.dist(muz, pt) > gr:
                continue
            fine = los_fine(grid, g, muz, pt)
            rule = los_cpp_rule(grid, g, muz, pt)
            if fine != rule and len(mism) < 6:
                mism.append((round(o["t"], 1), round(math.dist(muz, pt), 1), rule, fine))
            fired = {}
            for k, arr in acts.items():
                ac = arr[i] if arr and arr[i] else {}
                f = ac.get("fire", [False, False])
                fired[k] = bool(f[0] or f[1])
            rows.append((fine, fired))

    print(f"in-range enemy ticks: {len(rows)}")
    print()
    print(f"{'rev':10} {'clear+fired':>12} {'clear+NOFIRE':>13} {'blocked+fired':>14} {'blocked+NOFIRE':>15}")
    for k in bots:
        c = {"cf": 0, "cn": 0, "bf": 0, "bn": 0}
        for fine, fired in rows:
            c[("c" if fine else "b") + ("f" if fired[k] else "n")] += 1
        print(f"{k:10} {c['cf']:12} {c['cn']:13} {c['bf']:14} {c['bn']:15}")
    print()
    print("'clear+NOFIRE' is withheld fire on a shot that is geometrically open.")
    if mism:
        print(f"\n2 m rule vs 0.5 m truth disagreements (t, dist, rule, fine): {mism}")
    else:
        print("\n2 m rule agrees with 0.5 m fine sampling on every in-range tick.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
