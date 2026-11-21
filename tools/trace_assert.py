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


def ground_base(obs, g):
    """An observation where the mech stands on real ground with 10 m of clear
    space to the east. The recorded stream has the mech on rooftops much of the
    time, and a crate placed at a rooftop's height is correctly judged
    unreachable, so the farming tests need a ground-level anchor."""
    for o in obs:
        x, y, z = o["self"]["pos"]
        if not (-512 <= x <= 555 and -333 <= y <= 388):
            continue
        if abs(z - g.terrain_h(x, y)) >= 1.0:
            continue
        if g.clearance(x, y, z, 1.0, 0.0, 10.0, 9.0) < 10.0:
            continue
        return o
    return obs[len(obs) // 2]


def synthetic(bot, obs, g, cfg):
    out = {}
    base = ground_base(obs, g)
    bx, by, bz = base["self"]["pos"]
    out["base_tick"] = base["tick"]
    out["base_on_ground"] = abs(bz - g.terrain_h(bx, by)) < 1.0

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

    # 2. RUN AND HIDE takes nothing: even a free weapon at our feet is a detour
    #    the strategy refuses -- its whole value is arriving at the final melt
    #    untouched.
    o = json.loads(json.dumps(base))
    o["tick"] = 900002
    o["offer"] = None
    x, y, z = o["self"]["pos"]
    o["self"]["weapons"] = [o["self"]["weapons"][0], None]
    o["pickups"] = [{"id": 8123, "kind": "laser", "pos": [x + 2.0, y, z]}]
    bot.send(o)
    act = json.loads(bot.next_line())
    out["take_id"] = act.get("take")
    out["take_pass"] = act.get("take") is None

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

    # 5. Silent: a crate, a cube and a weapon pickup all in view, nobody
    #    shooting -- no fire, no take. A shot broadcasts a bearing to a squad
    #    that shares vision, and loot means crossing open ground.
    o = json.loads(json.dumps(base))
    o["tick"] = 900005
    o["mechs"] = []
    o["offer"] = None
    o["self"]["cubes"] = 0
    o["self"]["weapons"] = [o["self"]["weapons"][0], None]
    o["zone"] = {"center": [0.0, 0.0], "radius": 600.0, "next_center": [0.0, 0.0],
                 "next_radius": 600.0, "shrink_start": 100.0, "shrink_end": 130.0, "dps": 20.0}
    o["self"]["in_zone"] = True
    x, y, z = o["self"]["pos"]
    o["crates"] = [{"id": 1007, "pos": [x + 6.0, y, g.terrain_h(x + 6.0, y)], "hp": 300.0}]
    o["cubes"] = [{"id": 5012, "pos": [x + 10.0, y, z]}]
    o["pickups"] = [{"id": 8124, "kind": "laser", "pos": [x + 4.0, y, z]}]
    bot.send(o)
    act = json.loads(bot.next_line())
    f = act.get("fire", [False, False])
    out["silent_fire"] = [bool(f[0]), bool(f[1])]
    out["silent_take"] = act.get("take")
    out["silent_pass"] = not f[0] and not f[1] and act.get("take") is None

    # 6. Under fire: hit on the last event, an enemy in the open in range --
    #    return fire. Quiet is already spent once they are shooting at us.
    o = json.loads(json.dumps(base))
    o["tick"] = 900006
    o["offer"] = None
    o["crates"] = []
    o["cubes"] = []
    o["pickups"] = []
    o["self"]["cubes"] = 0
    o["self"]["in_zone"] = True
    o["self"]["hp"] = 1400.0
    o["self"]["aim"] = [0.0, 0.0]
    o["zone"] = {"center": [0.0, 0.0], "radius": 600.0, "next_center": [0.0, 0.0],
                 "next_radius": 600.0, "shrink_start": 100.0, "shrink_end": 130.0, "dps": 20.0}
    x, y, z = o["self"]["pos"]
    o["events"] = [{"type": "damaged", "amount": 45, "weapon": "gun",
                    "bearing": 0.0, "by": 77, "rear": False}]
    o["mechs"] = [{"id": 77, "npc": False, "pos": [x + 60.0, y, g.terrain_h(x + 60.0, y)],
                   "vel": [0.0, 0.0, 0.0], "aim": [0.0, 0.0], "hp": 1400.0,
                   "max_hp": 1400.0, "cubes": 0.0, "weapons": ["gun"]}]
    bot.send(o)
    act = json.loads(bot.next_line())
    out["return_fire"] = bool(act.get("fire", [False, False])[0])
    out["return_fire_pass"] = out["return_fire"]

    # 7. Quiet: the same enemy in the open in range, but nobody has hit us
    #    recently -- this observation runs 10 s after test 6's hit, so the
    #    under-fire window (3 s) has lapsed. Hold fire: a shot from hiding
    #    reveals us for nothing.
    o = json.loads(json.dumps(base))
    o["tick"] = 900007
    o["t"] = o["t"] + 10.0
    o["offer"] = None
    o["crates"] = []
    o["cubes"] = []
    o["pickups"] = []
    o["events"] = []
    o["self"]["cubes"] = 0
    o["self"]["in_zone"] = True
    o["self"]["hp"] = 1400.0
    o["self"]["aim"] = [0.0, 0.0]
    o["zone"] = {"center": [0.0, 0.0], "radius": 600.0, "next_center": [0.0, 0.0],
                 "next_radius": 600.0, "shrink_start": 100.0, "shrink_end": 130.0, "dps": 20.0}
    x, y, z = o["self"]["pos"]
    o["mechs"] = [{"id": 77, "npc": False, "pos": [x + 60.0, y, g.terrain_h(x + 60.0, y)],
                   "vel": [0.0, 0.0, 0.0], "aim": [0.0, 0.0], "hp": 1400.0,
                   "max_hp": 1400.0, "cubes": 0.0, "weapons": ["gun"]}]
    bot.send(o)
    act = json.loads(bot.next_line())
    out["quiet_fire"] = bool(act.get("fire", [False, False])[0])
    out["quiet_pass"] = not out["quiet_fire"]

    # ---------------------------------------------------------- M4 weapons
    # The mortar arc, the laser charge/release, and the fight maths all have
    # exact answers, so assert against them rather than against "it fired".

    def mkweapon(kind, damage, cd, speed, rng, charge=-1.0, mag=0.0, reload_s=0.0):
        return {"kind": kind, "cooldown": 0.0, "ammo": -1.0, "reload": -1.0, "charge": charge,
                "stats": {"damage": damage, "cooldown": cd, "speed": speed, "range": rng,
                          "spread_deg": 0.0, "mag": mag, "reload": reload_s}}

    def duel(tick, weapons, dist_east, enemy_hp, extra_lasers=0, hot=False):
        d = json.loads(json.dumps(base))
        d["tick"] = tick
        d["t"] = 100.0
        d["offer"] = None
        # `hot`: we were hit on the last event, which is the only condition the
        # hide strategy fires under. The weapon tests below are about aiming,
        # so they all run hot.
        d["events"] = ([{"type": "damaged", "amount": 45, "weapon": "gun",
                         "bearing": 0.0, "by": 77, "rear": False}] if hot else [])
        d["pickups"] = []
        d["crates"] = []
        d["cubes"] = []
        d["self"]["cubes"] = 0
        d["self"]["hp"] = 1400.0
        d["self"]["energy"] = 100.0
        d["self"]["in_zone"] = True
        d["self"]["aim"] = [0.0, 0.0]
        d["self"]["weapons"] = weapons
        d["zone"] = {"center": [0.0, 0.0], "radius": 600.0, "next_center": [0.0, 0.0],
                     "next_radius": 600.0, "shrink_start": 100.0, "shrink_end": 130.0,
                     "dps": 20.0}
        x, y, z = d["self"]["pos"]

        def enemy(i, ex, ey, hp, kinds):
            return {"id": i, "npc": False, "pos": [ex, ey, g.terrain_h(ex, ey)],
                    "vel": [0.0, 0.0, 0.0], "aim": [0.0, 0.0], "hp": hp, "max_hp": 1400.0,
                    "cubes": 0.0, "weapons": kinds}

        d["mechs"] = [enemy(77, x + dist_east, y, enemy_hp, ["gun"])]
        for i in range(extra_lasers):
            d["mechs"].append(enemy(80 + i, x + dist_east, y - 20.0 - 5.0 * i, 1400.0, ["laser"]))
        return d

    gun0 = base["self"]["weapons"][0]
    mcfg = (cfg.get("weapons") or {}).get("mortar") or {}
    mv, mg = mcfg.get("speed", 72.0), mcfg.get("gravity", 22.0)
    lc = ((cfg.get("weapons") or {}).get("laser") or {}).get("charge", 0.75)

    def arc_pitch(obs_like, dist_east, lob):
        """The launch pitch the mortar should return, computed from the closed
        form independently of the C++ (src/ballistics.hpp)."""
        x, y, z = obs_like["self"]["pos"]
        mz = z + obs_like["self"]["stats"].get("muzzle_height", 7.0)
        ex, ey = x + dist_east, y
        tz = g.terrain_h(ex, ey) + 4.5  # centre() of a mech with no cubes
        dd = math.hypot(ex - x, ey - y)
        dz = tz - mz
        A = mg * dd * dd / (2.0 * mv * mv)
        disc = dd * dd - 4.0 * A * (dz + A)
        if disc < 0.0:
            return None
        root = math.sqrt(disc)
        T = (dd + root) / (2.0 * A) if lob else 2.0 * (dz + A) / (dd + root)
        return math.atan(T)

    mortar = mkweapon("mortar", 200.0, 2.6, mv, 230.0)

    # 8. Holding a mortar at range: it outranks the gun, and the pitch returned
    #    must be a genuine arc that reaches the target -- matched against the
    #    closed form above, so a mortar aimed like a gun fails this.
    o = duel(900008, [gun0, mortar], 60.0, 100.0, hot=True)
    bot.send(o)
    act = json.loads(bot.next_line())
    f = act.get("fire", [False, False])
    out["mortar_fire"] = [bool(f[0]), bool(f[1])]
    pitch = act.get("aim", [0.0, 0.0])[1]
    cands = [p for p in (arc_pitch(o, 60.0, False), arc_pitch(o, 60.0, True)) if p is not None]
    out["mortar_pitch_err"] = round(min(abs(pitch - p) for p in cands), 4) if cands else None
    out["mortar_pass"] = bool(cands) and bool(f[1]) and not bool(f[0]) and \
        out["mortar_pitch_err"] < 0.02

    # 9. A target inside the mortar's own blast radius: must fall through to the
    #    gun instead of shelling itself.
    o = duel(900009, [gun0, mortar], 8.0, 100.0, hot=True)
    bot.send(o)
    act = json.loads(bot.next_line())
    f = act.get("fire", [False, False])
    out["mortar_close_fire"] = [bool(f[0]), bool(f[1])]
    out["mortar_close_pass"] = bool(f[0]) and not bool(f[1])

    # 10-12. Laser: hold to charge, release to fire. Idle and aligned it must
    #    start charging; part-way it must keep holding; near full it must let go.
    hold = []
    for i, (chg, want) in enumerate(((0.0, True), (0.30, True), (0.90 * lc + 0.05, False))):
        o = duel(900010 + i, [gun0, mkweapon("laser", 315.0, 1.65, 0.0, 280.0, charge=chg)],
                 8.0, 100.0, hot=True)
        bot.send(o)
        act = json.loads(bot.next_line())
        f = act.get("fire", [False, False])
        hold.append((round(chg, 3), bool(f[1]), want))
    out["laser_charge_hold"] = hold
    out["laser_pass"] = all(got == want for _, got, want in hold)

    # 13. Under fire and out-gunned: still answer -- a hiding bot that is being
    #     shot at has already been found, so refusing to shoot costs HP for
    #     nothing -- but do not close on a losing trade; relocate instead.
    o = duel(900013, [gun0], 60.0, 1400.0, extra_lasers=5, hot=True)
    bot.send(o)
    act = json.loads(bot.next_line())
    f = act.get("fire", [False, False])
    mv = act.get("move", [0.0, 0.0])
    # the enemy sits due east (+x) of us, so a positive radial is closing on it
    rad = mv[0] / max(1e-9, math.hypot(*mv))
    out["lose_fire"] = [bool(f[0]), bool(f[1])]
    out["lose_approach"] = round(rad, 3)
    out["lose_pass"] = bool(f[0]) and not bool(f[1]) and rad < 0.6

    # ---- rooftop pathing --------------------------------------------------
    # A roof crate used to be discarded outright (">3 m above the ground under
    # it"). Now it is a target if a tank of jet can reach it, and the bot climbs
    # by pressing into the wall, jumping, then jetting.

    def find_roof(lo=8.0, hi=22.0):
        """A climbable building, plus a standing spot outside its wall and a
        crate spot just inside the roof near that wall."""
        for base, top, ring, holes in g.prisms:
            if base > 0.5 or top < lo:
                continue
            pts = [(ring[2 * i], ring[2 * i + 1]) for i in range(len(ring) // 2)]
            cx = sum(p[0] for p in pts) / len(pts)
            cy = sum(p[1] for p in pts) / len(pts)
            if not (lo <= top - g.terrain_h(cx, cy) <= hi):
                continue
            for q in pts:
                dx, dy = q[0] - cx, q[1] - cy
                n = math.hypot(dx, dy) or 1.0
                ax, ay = q[0] + dx / n * 4.5, q[1] + dy / n * 4.5
                gz = g.terrain_h(ax, ay)
                if g.prism_blocks(ax, ay, gz + 1.5, gz + 9.0):
                    continue                                   # wall in the way
                sx, sy = q[0] - dx / n * 2.5, q[1] - dy / n * 2.5
                if not g.prism_blocks(sx, sy, top - 0.5, top):
                    continue                                   # not roof surface
                if g.prism_blocks(sx, sy, top + 0.1, top + 1.0):
                    continue                                   # something above it
                return (top, ax, ay, sx, sy, gz)
        return None

    def find_tower(lo=58.0):
        for base, top, ring, holes in g.prisms:
            if base > 0.5 or top < 10.0:
                continue
            pts = [(ring[2 * i], ring[2 * i + 1]) for i in range(len(ring) // 2)]
            cx = sum(p[0] for p in pts) / len(pts)
            cy = sum(p[1] for p in pts) / len(pts)
            if top - g.terrain_h(cx, cy) > lo:
                return (cx, cy, top)
        return None

    def roof_obs(tick, roof, energy, grounded, crate_xy, dz=0.0):
        top, ax, ay, sx, sy, gz = roof
        d = json.loads(json.dumps(base))
        d["tick"] = tick
        d["t"] = 100.0
        d["offer"] = None
        d["events"] = []
        d["mechs"] = []
        d["pickups"] = []
        d["cubes"] = []
        d["self"]["cubes"] = 0
        d["self"]["hp"] = 1400.0
        d["self"]["energy"] = energy
        d["self"]["max_energy"] = 100.0
        d["self"]["grounded"] = grounded
        d["self"]["in_zone"] = True
        d["self"]["aim"] = [0.0, 0.0]
        d["self"]["pos"] = [ax, ay, gz + dz]
        d["zone"] = {"center": [0.0, 0.0], "radius": 600.0, "next_center": [0.0, 0.0],
                     "next_radius": 600.0, "shrink_start": 100.0, "shrink_end": 130.0, "dps": 20.0}
        d["crates"] = [{"id": 4242, "pos": [crate_xy[0], crate_xy[1], top], "hp": 300.0}]
        return d

    # 14. RUN AND HIDE farms nothing and climbs nothing: a roof crate at the
    #     wall and a tower crate beyond reach are both ignored -- no jump, no
    #     jet, no shot. An airborne mech is loud and sky-lined.
    roof = find_roof()
    out["roof_found"] = roof is not None
    if roof:
        top, ax, ay, sx, sy, gz = roof
        out["roof_climb_m"] = round(top - gz, 1)
        o = roof_obs(900014, roof, 100.0, True, (sx, sy))
        o["t"] = 110.0  # 10 s after test 13's hit: quiet, so no sprint either
        bot.send(o)
        act = json.loads(bot.next_line())
        f = act.get("fire", [False, False])
        out["no_climb_jump"] = bool(act.get("jump"))
        out["no_climb_boost"] = bool(act.get("boost"))
        out["no_climb_fire"] = bool(f[0]) or bool(f[1])
        out["no_climb_pass"] = (not out["no_climb_jump"] and not out["no_climb_boost"]
                                and not out["no_climb_fire"])

    # 18. A laser 120 m east while unarmed: not taken, not chased, not shot at.
    #     The hide strategy takes nothing -- even a free weapon is a detour
    #     through the open for a bot whose whole value is arriving untouched.
    o = json.loads(json.dumps(base))
    o["tick"] = 900018
    o["t"] = 120.0  # quiet: 10 s after the roof test, nobody shooting
    o["mechs"] = []
    o["offer"] = None
    o["events"] = []
    o["cubes"] = []
    o["crates"] = []
    o["self"]["cubes"] = 0
    o["self"]["in_zone"] = True
    o["self"]["weapons"] = [o["self"]["weapons"][0], None]
    x, y, z = o["self"]["pos"]
    o["zone"] = {"center": [x, y], "radius": 600.0, "next_center": [x, y],
                 "next_radius": 600.0, "shrink_start": 100.0, "shrink_end": 130.0, "dps": 20.0}
    o["pickups"] = [{"id": 8200, "kind": "laser",
                     "pos": [x + 120.0, y, g.terrain_h(x + 120.0, y)]}]
    bot.send(o)
    act = json.loads(bot.next_line())
    f = act.get("fire", [False, False])
    mv = act.get("move", [0.0, 0.0])
    out["arm_east"] = round(mv[0] / max(1e-9, math.hypot(*mv)), 3)
    out["arm_pass"] = act.get("take") is None and not f[0] and not f[1]

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

        # Firing with nothing visible at all is a bug. Firing with no MECH
        # visible is legitimate: the bot shoots crates to farm cubes.
        if act.get("fire", [False, False])[0] and not (o["mechs"] or o["crates"]):
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

    syn = synthetic(bot, obs, g, cfg)
    print("synthetic policy checks (paths the recorded stream never reaches):")
    for k in ("base_tick", "base_on_ground",
              "zone_run_dot", "zone_run_boost", "zone_run_pass", "take_id", "take_pass",
              "pick", "pick_pass", "wall_clearance", "wall_pass",
              "silent_fire", "silent_take", "silent_pass",
              "return_fire", "return_fire_pass", "quiet_fire", "quiet_pass",
              "mortar_fire", "mortar_pitch_err", "mortar_pass",
              "mortar_close_fire", "mortar_close_pass",
              "laser_charge_hold", "laser_pass", "lose_fire", "lose_approach", "lose_pass",
              "roof_found", "roof_climb_m",
              "no_climb_jump", "no_climb_boost", "no_climb_fire", "no_climb_pass",
              "arm_east", "arm_pass"):
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
