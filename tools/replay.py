#!/usr/bin/env python3
"""Replay the recorded observation stream through a bot binary and report what it
decided. The stream does not react, so this measures BEHAVIOUR, not outcome --
but it is the only way to compare two revisions against identical inputs, which
is what makes a "the newer bot is worse" claim checkable without a submission.

    python3 tools/replay.py --bot ./build/bot [--stream check/stream.jsonl.gz]

Reports, per run:
  fire          ticks firing each slot, and how many of those had a mech in range
  return_fire   fraction of "enemy visible and inside gun range" ticks on which we
                fired at it at all -- a bot that never fires back is a sitting duck
  jitter        mean |turn| of the move vector between ticks. Large values mean the
                steering is oscillating rather than committing to a heading
  dist/moved    net displacement vs path length: a low ratio means going in circles
"""
import argparse
import gzip
import json
import math
import os
import subprocess
from collections import Counter


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bot", default="./build/bot")
    ap.add_argument("--stream", default="check/stream.jsonl.gz")
    ap.add_argument("--arena", default="arena")
    ap.add_argument("--label", default="")
    a = ap.parse_args()

    msgs = [json.loads(l) for l in gzip.open(a.stream, "rt") if l.strip()]
    hello, obs = msgs[0], [m for m in msgs if m.get("type") == "obs"]
    hello["map"]["dir"] = os.path.abspath(a.arena)

    p = subprocess.Popen([a.bot], stdin=subprocess.PIPE, stdout=subprocess.PIPE, bufsize=0)

    def send(m):
        p.stdin.write((json.dumps(m, separators=(",", ":")) + "\n").encode())
        p.stdin.flush()

    def recv():
        line = p.stdout.readline()
        return json.loads(line) if line else None

    send(hello)
    if recv() is None:
        print("FAIL: no ready")
        return 1

    gun_range = hello["config"]["weapons"]["gun"]["range"]
    fire0 = fire1 = 0
    enemy_in_range = 0
    fired_while_enemy = 0
    turns, steps = [], []
    labels = Counter()
    path = 0.0
    prev_move = None
    prev_pos = None
    first_pos = last_pos = None

    for o in obs:
        send(o)
        act = recv()
        if act is None:
            print("FAIL: no answer at tick", o["tick"])
            return 1
        f = act.get("fire", [False, False])
        fire0 += bool(f[0])
        fire1 += bool(f[1])
        lab = (act.get("debug") or {}).get("label", "")
        for tok in lab.split():
            if tok.isalpha() and tok not in ("hp", "c", "z", "a", "su"):
                labels[tok] += 1

        me = o["self"]["pos"]
        mx, my, mz = me[0], me[1], me[2] + o["self"]["stats"].get("muzzle_height", 7.0)
        near = [e for e in o["mechs"]
                if not e.get("npc")
                and math.dist((mx, my, mz), (e["pos"][0], e["pos"][1], e["pos"][2] + 4.5)) <= gun_range]
        if near:
            enemy_in_range += 1
            if f[0] or f[1]:
                fired_while_enemy += 1

        mv = act.get("move", [0.0, 0.0])
        L = math.hypot(mv[0], mv[1])
        if L > 0.05:
            u = (mv[0] / L, mv[1] / L)
            if prev_move is not None:
                dot = max(-1.0, min(1.0, u[0] * prev_move[0] + u[1] * prev_move[1]))
                turns.append(math.degrees(math.acos(dot)))
            prev_move = u
        pos = o["self"]["pos"]
        if prev_pos is not None:
            path += math.dist(pos, prev_pos)
        else:
            first_pos = pos
        prev_pos, last_pos = pos, pos

    try:
        p.stdin.close()
        p.wait(timeout=3)
    except Exception:
        p.kill()

    net = math.dist(first_pos[:2], last_pos[:2]) if first_pos and last_pos else 0.0
    tag = a.label or a.bot
    print(f"=== {tag} ===")
    print(f"  fire slot0/slot1        : {fire0} / {fire1}")
    print(f"  mech in gun range       : {enemy_in_range} ticks")
    print(f"  return fire             : {fired_while_enemy}/{enemy_in_range}"
          f"  ({100.0 * fired_while_enemy / max(1, enemy_in_range):.0f}%)")
    if turns:
        print(f"  move-vector turn        : mean {sum(turns) / len(turns):5.1f} deg  max {max(turns):5.1f}")
    print(f"  net displacement / path : {net:6.1f} / {path:7.1f} m  (straightness "
          f"{100.0 * net / max(1e-6, path):.0f}%)")
    print(f"  label targets           : {dict(labels)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
