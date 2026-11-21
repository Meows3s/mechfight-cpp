#!/usr/bin/env python3
"""Behavioural audit of the bot over the recorded observation stream.

check/check.py proves protocol compliance but never looks at what the bot
decided. This replays the same recorded stream and asserts invariants about the
DECISIONS, using an independent Python reimplementation of the map geometry
(the same one cross-checked by tools/verify_map.py). Because the stream does not
react, these are self-consistency checks: given this observation, is this action
sane?

    python3 tools/trace_assert.py [--bot ./build/bot] [--arena arena]
"""
import argparse
import gzip
import json
import math
import os
import queue
import struct
import subprocess
import sys
import threading

# ------------------------------------------------------------------ geometry
# Independent reimplementation of src/map.cpp, used to judge the bot's choices.


class Geom:
    def __init__(self, arena):
        d = json.load(open(os.path.join(arena, "v2/world.json")))
        t = d["terrain"]
        self.minx, self.miny, self.step = t["minX"], t["minY"], t["step"]
        self.nx, self.ny, self.scale = t["nx"], t["ny"], t["scale"]
        raw = open(os.path.join(arena, "v2", t["file"]), "rb").read()
        self.h = struct.unpack(f"<{self.nx * self.ny}h", raw[: self.nx * self.ny * 2])
        self.maxstep = d["rules"]["maxStep"]
        self.maxslope = d["rules"]["maxSlopeDeg"]
        self.prisms = []
        for p in d["prisms"]:
            if len(p["ring"]) < 6:
                continue
            holes = [hh for hh in (p.get("holes") or []) if len(hh) >= 6]
            self.prisms.append((p["base"], p["top"], p["ring"], holes))

    def terrain_h(self, x, y):
        fx, fy = (x - self.minx) / self.step, (y - self.miny) / self.step
        i, j = math.floor(fx), math.floor(fy)
        tx, ty = fx - i, fy - j

        def at(ii, jj):
            ii = min(max(ii, 0), self.nx - 1)
            jj = min(max(jj, 0), self.ny - 1)
            return self.h[jj * self.nx + ii] * self.scale

        a, b, c, e = at(i, j), at(i + 1, j), at(i, j + 1), at(i + 1, j + 1)
        return (a * (1 - tx) + b * tx) * (1 - ty) + (c * (1 - tx) + e * tx) * ty

    @staticmethod
    def _in_poly(ring, x, y):
        n = len(ring) // 2
        inside, j = False, n - 1
        for i in range(n):
            xi, yi = ring[2 * i], ring[2 * i + 1]
            xj, yj = ring[2 * j], ring[2 * j + 1]
            if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
                inside = not inside
            j = i
        return inside

    def _inside_prism(self, p, x, y):
        base, top, ring, holes = p
        if not self._in_poly(ring, x, y):
            return False
        return not any(self._in_poly(hh, x, y) for hh in holes)

    def prism_blocks(self, x, y, zlo, zhi):
        if zhi <= zlo:
            return False
        return any(p[0] < zhi and p[1] > zlo and self._inside_prism(p, x, y) for p in self.prisms)

    def surface_h(self, x, y, z, step_up):
        h = self.terrain_h(x, y)
        for p in self.prisms:
            if p[1] <= z + step_up and p[1] > h and self._inside_prism(p, x, y):
                h = p[1]
        return h

    def walkable(self, x, y, z, height):
        s = self.surface_h(x, y, z, self.maxstep)
        if s - z > self.maxstep:
            return False
        return not self.prism_blocks(x, y, z + self.maxstep, z + height)

    def clearance(self, x, y, z, dx, dy, max_len, height):
        tan_max = math.tan(min(self.maxslope, 89.0) * math.pi / 180.0)
        prev, travelled, ds = self.surface_h(x, y, z, self.maxstep), 0.0, 1.0
        while travelled + ds <= max_len:
            nx, ny = x + dx * (travelled + ds), y + dy * (travelled + ds)
            if not self.walkable(nx, ny, z, height):
                return travelled
            nh = self.surface_h(nx, ny, z, self.maxstep)
            if abs(nh - prev) / ds > tan_max:
                return travelled
            prev, travelled = nh, travelled + ds
        return travelled


# ------------------------------------------------------------------ bot runner


class Bot:
    def __init__(self, cmd):
        self.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, bufsize=0)
        self.lines = queue.Queue()
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        for raw in self.proc.stdout:
            self.lines.put(raw)

    def send(self, msg):
        self.proc.stdin.write((json.dumps(msg, separators=(",", ":")) + "\n").encode())
        self.proc.stdin.flush()

    def next_line(self, timeout=5.0):
        try:
            return self.lines.get(timeout=timeout)
        except queue.Empty:
            return None



# ------------------------------------------------------------------ synthetic
# The recorded stream never puts the mech outside the zone or next to a pickup,
# so those two policy paths are never exercised by it. These mutated
# observations force them, and check the answer is the intended one.


def synthetic(bot, base, g):
    out = {}

    # 1. Well outside the zone, nothing else to do: must run for the centre.
    o = json.loads(json.dumps(base))
    o["tick"] = 900001
    o["t"] = 100.0
    o["mechs"] = []
    o["pickups"] = []
    o["offer"] = None
    o["events"] = []
    o["zone"] = {"center": [0.0, 0.0], "radius": 60.0, "next_center": [0.0, 0.0],
                 "next_radius": 40.0, "shrink_start": 100.0, "shrink_end": 130.0, "dps": 20.0}
    px, py = 100.0, 0.0
    o["self"]["pos"] = [px, py, g.terrain_h(px, py)]
    o["self"]["in_zone"] = False
    bot.send(o)
    act = json.loads(bot.next_line())
    to_c = (-px, -py)
    mv = act.get("move", [0.0, 0.0])
    denom = math.hypot(*to_c) * max(1e-9, math.hypot(*mv))
    dot = (mv[0] * to_c[0] + mv[1] * to_c[1]) / denom if denom else 0.0
    out["zone_run_dot"] = round(dot, 4)
    out["zone_run_boost"] = bool(act.get("boost"))
    out["zone_run_pass"] = dot > 0.9 and bool(act.get("boost"))

    # 2. A real weapon within pickup_radius and an empty slot: must take it.
    o = json.loads(json.dumps(base))
    o["tick"] = 900002
    o["offer"] = None
    x, y, z = o["self"]["pos"]
    o["self"]["weapons"] = [o["self"]["weapons"][0], None]
    o["pickups"] = [{"id": 8123, "kind": "laser", "pos": [x + 2.0, y, z]}]
    bot.send(o)
    act = json.loads(bot.next_line())
    out["take_id"] = act.get("take")
    out["take_pass"] = act.get("take") == 8123

    # 3. An offer where the best choice is NOT index 0 ("armor" outranks
    #    "sensors" and "jets"): must return the index, not the id.
    o = json.loads(json.dumps(base))
    o["tick"] = 900003
    o["offer"] = {"id": 2, "expires": o["t"] + 5.0, "choices": [
        {"id": "sensors", "name": "Sensor suite", "desc": ""},
        {"id": "armor", "name": "Reactive armor", "desc": ""},
        {"id": "jets", "name": "Efficient jets", "desc": ""}]}
    bot.send(o)
    act = json.loads(bot.next_line())
    out["pick"] = act.get("pick")
    out["pick_pass"] = act.get("pick") == 1

    # 4. A wall directly ahead: the chosen move must not go into it, or the
    #    probe must have found nothing at all (walled in, which shouldn't
    #    happen anywhere reachable).
    o = json.loads(json.dumps(base))
    o["tick"] = 900004
    o["mechs"] = []
    o["pickups"] = []
    o["offer"] = None
    o["zone"] = {"center": [0.0, 0.0], "radius": 600.0, "next_center": [0.0, 0.0],
                 "next_radius": 600.0, "shrink_start": 100.0, "shrink_end": 130.0, "dps": 20.0}
    o["self"]["in_zone"] = True
    bot.send(o)
    act = json.loads(bot.next_line())
    mv = act.get("move", [0.0, 0.0])
    L = math.hypot(*mv)
    if L > 0.05:
        sx, sy, sz = o["self"]["pos"]
        h = o["self"]["stats"].get("height", 9.0)
        out["wall_clearance"] = g.clearance(sx, sy, sz, mv[0] / L, mv[1] / L, 6.0, h)
        out["wall_pass"] = out["wall_clearance"] >= 1.5
    else:
        out["wall_pass"] = False
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bot", default="./build/bot")
    ap.add_argument("--arena", default="arena")
    ap.add_argument("--stream", default="check/stream.jsonl.gz")
    a = ap.parse_args()

    g = Geom(a.arena)
    msgs = [json.loads(l) for l in gzip.open(a.stream, "rt") if l.strip()]
    hello = msgs[0]
    obs = [m for m in msgs if m.get("type") == "obs"]
    hello["map"]["dir"] = os.path.abspath(a.arena)
    cfg = hello["config"]
    pickup_r = cfg["loot"]["pickup_radius"]
    min_pitch, max_pitch = cfg["mech"]["min_pitch"], cfg["mech"]["max_pitch"]

    bot = Bot([a.bot])
    bot.send(hello)
    if bot.next_line() is None:
        print("FAIL: no ready")
        return 1

    counts = {k: 0 for k in
              ("checked", "bad_tick", "nonfinite", "bad_yaw", "bad_pitch", "bad_move_len",
               "take_out_of_range", "take_unknown", "fire_no_target", "walled_move",
               "ignored_zone", "no_map_flag",
               # "did this check actually run?" counters, so a zero means something
               "ran_move_probe", "ran_zone_run", "ran_take", "ran_fire0", "ran_aim")}
    walled_examples, zone_examples = [], []

    for o in obs:
        bot.send(o)
        raw = bot.next_line()
        if raw is None:
            print(f"FAIL: no answer at tick {o['tick']}")
            return 1
        act = json.loads(raw)
        counts["checked"] += 1
        me = o["self"]
        x, y, z = me["pos"]
        height = me["stats"].get("height", 9.0)
        radius = me["stats"].get("radius", 2.5)

        if act.get("tick") != o["tick"]:
            counts["bad_tick"] += 1

        nums = []
        for k in ("move", "aim"):
            if k in act:
                nums += list(act[k])
        for k in ("boost", "jump", "reload", "pick", "take", "drop"):
            if k in act and isinstance(act[k], (int, float)):
                nums.append(act[k])
        if any(not math.isfinite(float(v)) for v in nums):
            counts["nonfinite"] += 1

        if "aim" in act:
            yaw, pitch = act["aim"]
            if not (-math.pi - 0.01 <= yaw <= math.pi + 0.01):
                counts["bad_yaw"] += 1
            if not (min_pitch - 0.01 <= pitch <= max_pitch + 0.01):
                counts["bad_pitch"] += 1

        if "move" in act:
            L = math.hypot(*act["move"])
            if L > 1.0001:
                counts["bad_move_len"] += 1
        lab = (act.get("debug") or {}).get("label", "")
        if "NOMAP" in lab:
            counts["no_map_flag"] += 1
        if "aim" in act:
            counts["ran_aim"] += 1
        if "take" in act:
            counts["ran_take"] += 1
        if act.get("fire", [False, False])[0]:
            counts["ran_fire0"] += 1

        if "take" in act:
            hit = [p for p in o["pickups"] if p["id"] == act["take"]]
            if not hit:
                counts["take_unknown"] += 1
            elif math.dist(hit[0]["pos"][:2], (x, y)) > pickup_r + 0.01:
                counts["take_out_of_range"] += 1

        if act.get("fire", [False, False])[0] and not o["mechs"]:
            counts["fire_no_target"] += 1

        # Did we walk into a wall? Probe the chosen direction against the map.
        if "move" in act and math.hypot(*act["move"]) > 0.05:
            counts["ran_move_probe"] += 1
            dx, dy = act["move"]
            L = math.hypot(dx, dy)
            dx, dy = dx / L, dy / L
            clr = g.clearance(x, y, z, dx, dy, 6.0, height)
            if clr < 1.5:
                counts["walled_move"] += 1
                if len(walled_examples) < 5:
                    walled_examples.append((o["tick"], round(x, 1), round(y, 1), round(clr, 2)))

        # Ignoring the zone? If we should be running for it, do we head that way?
        zn = o["zone"]
        span = zn["shrink_end"] - zn["shrink_start"]
        u = 0.0 if span <= 1e-6 else min(1.0, max(0.0, (o["t"] + 4.0 - zn["shrink_start"]) / span))
        cx = zn["center"][0] + (zn["next_center"][0] - zn["center"][0]) * u
        cy = zn["center"][1] + (zn["next_center"][1] - zn["center"][1]) * u
        cr = zn["radius"] + (zn["next_radius"] - zn["radius"]) * u
        margin = radius + 6.0
        if math.dist((x, y), (cx, cy)) > cr - margin:
            counts["ran_zone_run"] += 1
            to_c = (cx - x, cy - y)
            n = math.hypot(*to_c)
            if "move" in act and n > 1e-6:
                dot = (act["move"][0] * to_c[0] + act["move"][1] * to_c[1]) / (n * max(1e-9, math.hypot(*act["move"])))
                if dot < 0.2:
                    counts["ignored_zone"] += 1
                    if len(zone_examples) < 5:
                        zone_examples.append((o["tick"], round(math.dist((x, y), (cx, cy)), 1), round(cr, 1), round(dot, 2)))

    syn = synthetic(bot, obs[len(obs) // 2], g)
    print("synthetic policy checks (paths the recorded stream never reaches):")
    for k in ("zone_run_dot", "zone_run_boost", "zone_run_pass", "take_id", "take_pass",
              "pick", "pick_pass", "wall_clearance", "wall_pass"):
        if k in syn:
            print(f"  {k:18s} {syn[k]}")
    print("  SYNTHETIC", "PASS" if all(v for k, v in syn.items() if k.endswith("_pass")) else "FAIL")

    bot.send({"type": "end", "place": 0, "kills": 0})
    try:
        bot.proc.stdin.close()
        bot.proc.wait(timeout=2)
    except Exception:
        bot.proc.kill()

    print(f"decisions checked: {counts['checked']}")
    for k, v in counts.items():
        if k != "checked":
            print(f"  {k:22s} {v}")
    if walled_examples:
        print(f"  walled_move examples (tick,x,y,clearance): {walled_examples}")
    if zone_examples:
        print(f"  ignored_zone examples (tick,dist,radius,dot): {zone_examples}")

    hard = ("bad_tick", "nonfinite", "bad_yaw", "bad_pitch", "bad_move_len",
            "take_out_of_range", "take_unknown")
    failed = {k: counts[k] for k in hard if counts[k]}
    if not all(v for k, v in syn.items() if k.endswith("_pass")):
        failed["synthetic"] = 1
    if failed:
        print("FAIL", failed)
        return 1
    print("PASS (hard invariants); soft counters are informational")
    return 0


if __name__ == "__main__":
    sys.exit(main())
