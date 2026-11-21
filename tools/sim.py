#!/usr/bin/env python3
"""Partial simulator for Mech Royale: movement, jets, collision, shots, vision.

The kit ships no simulator (check/check.py only replays a recorded stream), so
nothing here has been able to verify that a rooftop climb actually lands, or that
a mortar really clears a roof edge. This closes that gap for the cases we care
about, using the live numbers out of hello.config and the map geometry that
tools/verify_map.py already cross-checks against src/map.cpp.

Two modes:

  probe   static geometry question: from a rooftop, which ground targets are
          visible, which can the gun/laser reach, and which can a lobbed mortar
          hit without line of sight?

  climb   closed loop: run the real bot binary against this physics, aiming at a
          crate on a roof, and report whether it ends up standing on the roof.

Approximations, stated because they matter:
  - Wall sliding is "try the full move, then x-only, then y-only", which is the
    documented behaviour but not the server's exact sweep.
  - Roof-edge handling is centre-support: you land on the surface under your
    centre, which is what protocol.md section 7 describes ("you land on the edge
    instead of dropping into the wall below").
  - Drone/NPC, sound, zone damage, and supply drops are not simulated.

    python3 tools/sim.py probe [--roof-index N]
    python3 tools/sim.py climb [--bot ./build/bot]
"""
import argparse
import gzip
import json
import math
import os
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from trace_assert import Geom  # the geometry already verified against src/map.cpp

DT = 0.05          # 20 ticks/s (protocol section 2)
TICKS_PER_DECISION = 2   # a decision holds for 2 ticks, like the real server
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def load_cfg():
    p = os.path.join(ROOT, "check", "stream.jsonl.gz")
    msgs = [json.loads(l) for l in gzip.open(p, "rt") if l.strip()]
    return msgs[0]


# ------------------------------------------------------------------ physics


class Mech:
    def __init__(self, g, config, pos, hp=None):
        m = config["mech"]
        self.g, self.config, self.m = g, config, m
        self.pos = [float(pos[0]), float(pos[1]), float(pos[2])]
        self.vel = [0.0, 0.0, 0.0]
        self.aim = [0.0, 1.9]           # yaw, pitch (rad); pitch 0 = level, > 0 up
        self.energy = float(m["energy"])
        self.max_hp = float(m["hp"])
        self.hp = float(hp if hp is not None else m["hp"])
        self.cubes = 0.0
        self.height = float(m["height"])
        self.radius = float(m["radius"])
        self.grounded = True
        self._since_spend = 999.0
        self.moved = 0.0

    # -- helpers
    def muzzle(self, which="gun"):
        z = self.pos[2] + (float(self.m["muzzle_height"]) if which == "gun" else self.height + 0.5)
        return [self.pos[0], self.pos[1], z]

    def dir3(self):
        cy, sy = math.cos(self.aim[0]), math.sin(self.aim[0])
        cp, sp = math.cos(self.aim[1]), math.sin(self.aim[1])
        return [cp * cy, cp * sy, sp]

    # -- one tick
    def step(self, act):
        m = self.m
        mv = act.get("move", [0.0, 0.0])
        ln = math.hypot(mv[0], mv[1])
        want_dir = (mv[0] / ln, mv[1] / ln) if ln > 1e-9 else (0.0, 0.0)
        boost = bool(act.get("boost"))
        jump = bool(act.get("jump"))
        self.moved = min(1.0, ln)

        # vertical
        if jump and self.grounded:
            self.vel[2] = float(m["jump_speed"])
            self.grounded = False
        spent = 0.0
        if not self.grounded:
            self.vel[2] -= float(m["gravity"]) * DT
            if boost:
                self.vel[2] = min(float(m["max_rise_speed"]),
                                  self.vel[2] + float(m["jet_accel"]) * DT)
                spent = float(m["jet_drain"]) * DT
        else:
            self.vel[2] = 0.0

        # horizontal: accelerate toward the commanded speed
        cap = (float(m["boost_speed"]) if (self.grounded and boost) else float(m["walk_speed"]))
        acc = float(m["ground_accel"]) if self.grounded else float(m["air_accel"])
        tx, ty = want_dir[0] * cap, want_dir[1] * cap
        for i in (0, 1):
            dv = (tx if i == 0 else ty) - self.vel[i]
            step = acc * DT
            self.vel[i] += max(-step, min(step, dv))
        if self.grounded and boost:
            spent += float(m["boost_drain"]) * DT

        # energy
        if spent > 0.0:
            self.energy = max(0.0, self.energy - spent)
            self._since_spend = 0.0
            if self.energy <= 0.0 and boost:
                # out of energy: the jet dies, you fall
                pass
        else:
            self._since_spend += DT
            if self._since_spend >= float(m["energy_regen_delay"]):
                self.energy = min(float(m["energy"]), self.energy + float(m["energy_regen"]) * DT)

        # horizontal move with wall sliding
        self._move_h(self.vel[0] * DT, self.vel[1] * DT)

        # vertical move + landing (centre support)
        z_new = self.pos[2] + self.vel[2] * DT
        surf = self.g.surface_h(self.pos[0], self.pos[1], self.pos[2], float(m["step_up"]))
        if z_new <= surf and self.vel[2] <= 0.0:
            self.pos[2] = surf
            self.vel[2] = 0.0
            self.grounded = True
        else:
            self.pos[2] = z_new
            self.grounded = False

    def _free(self, x, y, z):
        return not self.g.prism_blocks(x, y, z + 0.05, z + self.height)

    def _move_h(self, dx, dy):
        step_up = float(self.m["step_up"])
        for ddx, ddy in ((dx, dy), (dx, 0.0), (0.0, dy)):
            if abs(ddx) < 1e-12 and abs(ddy) < 1e-12:
                continue
            nx, ny = self.pos[0] + ddx, self.pos[1] + ddy
            surf = self.g.surface_h(nx, ny, self.pos[2], step_up)
            if surf - self.pos[2] > step_up:          # a ledge too tall to walk up
                continue
            if not self._free(nx, ny, max(self.pos[2], surf)):
                continue
            self.pos[0], self.pos[1] = nx, ny
            if self.grounded and surf > self.pos[2]:
                self.pos[2] = surf                    # step up
            return


# ------------------------------------------------------------------ shots


def ray_hits_solid(g, a, b, step=0.4):
    """First solid hit along the segment a->b, or None. Terrain and prisms."""
    L = math.dist(a, b)
    n = max(1, int(L / step))
    for i in range(1, n + 1):
        t = i / n
        x = a[0] + (b[0] - a[0]) * t
        y = a[1] + (b[1] - a[1]) * t
        z = a[2] + (b[2] - a[2]) * t
        if z <= g.terrain_h(x, y):
            return (x, y, z)
        if g.prism_blocks(x, y, z, z + 0.001):
            return (x, y, z)
    return None


def aim_at(a, b):
    dx, dy, dz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
    return [math.atan2(dy, dx), math.atan2(dz, math.hypot(dx, dy))]


def _dir3(aim):
    return [math.cos(aim[1]) * math.cos(aim[0]), math.cos(aim[1]) * math.sin(aim[0]),
            math.sin(aim[1])]


def _origin(shooter, which, aim):
    """Shots leave radius + 0.5 = 3 m out along the aim (protocol section 5)."""
    mu = shooter.muzzle(which)
    d = _dir3(aim)
    return [mu[i] + d[i] * (shooter.radius + 0.5) for i in range(3)]


def _in_target(q, target):
    if not (target.pos[2] - 0.1 <= q[2] <= target.pos[2] + target.height + 0.1):
        return False
    return math.hypot(q[0] - target.pos[0], q[1] - target.pos[1]) <= target.radius + 0.3


def _fly(g, o, v, gravity, ttl, target):
    """Walk a projectile in 0.2 m sub-steps and report the FIRST thing it meets:
    "target", "solid" or None. Ordering matters -- testing a whole step at once
    lets a target standing behind a wall register as a hit."""
    t, prev = 0.0, o[:]
    while t <= ttl:
        t += 0.01
        p = [o[0] + v[0] * t, o[1] + v[1] * t, o[2] + v[2] * t - 0.5 * gravity * t * t]
        L = math.dist(prev, p)
        n = max(1, int(L / 0.2) + 1)
        for i in range(1, n + 1):
            q = [prev[j] + (p[j] - prev[j]) * i / n for j in range(3)]
            if _in_target(q, target):
                return "target"
            if q[2] <= g.terrain_h(q[0], q[1]) or g.prism_blocks(q[0], q[1], q[2], q[2] + 0.001):
                return "solid"
        prev = p
    return None


def ray_hit_target(g, shooter, aim, target, rng=230.0, speed=252.0):
    """The gun flies straight with no drop, and the laser is hitscan: both are a
    straight line along the aim, so one function covers them."""
    o = _origin(shooter, "gun", aim)
    return _fly(g, o, [c * speed for c in _dir3(aim)], 0.0, rng / speed, target) == "target"


def solve_pitch(d, h, v, gr, lob):
    if d <= 1e-6:
        return None
    A = gr * d * d / (2.0 * v * v)
    disc = d * d - 4.0 * A * (h + A)
    if disc < 0.0:
        return None
    root = math.sqrt(disc)
    if lob:
        if A < 1e-12:
            return None
        T = (d + root) / (2.0 * A)
    else:
        T = 2.0 * (h + A) / (d + root)
    return math.atan(T)


def shell_hit_target(g, shooter, aim, target, mcfg):
    o = _origin(shooter, "mortar", aim)
    d = _dir3(aim)
    return _fly(g, o, [c * mcfg["speed"] for c in d], mcfg["gravity"],
                mcfg.get("ttl", 7.0), target) == "target"


def mortar_lands(g, shooter, target, mcfg):
    """Try the flat arc then the lob; "flat"/"lob" if the shell reaches the
    target's cylinder, else None. This is the 'fire with no line of sight'
    question: the flat arc needs a clear path, the lob goes over the top."""
    v, gr = mcfg["speed"], mcfg["gravity"]
    muzzle = shooter.muzzle("mortar")
    dz = (target.pos[2] + target.height * 0.5) - muzzle[2]
    d = math.hypot(target.pos[0] - muzzle[0], target.pos[1] - muzzle[1])
    yaw = math.atan2(target.pos[1] - muzzle[1], target.pos[0] - muzzle[0])
    for lob in (False, True):
        p = solve_pitch(d, dz, v, gr, lob)
        if p is None:
            continue
        if not (-0.7 <= p <= shooter.m.get("max_pitch", 1.3)):
            continue
        if shell_hit_target(g, shooter, [yaw, p], target, mcfg):
            return ("lob" if lob else "flat")
    return None


def visible(shooter, target, config):
    """protocol.md section 8: eye LOS to the target's centre or top, and either
    within near_range any direction, or within range inside the fov yaw wedge."""
    vs = config["vision"]
    eye = [shooter.pos[0], shooter.pos[1], shooter.pos[2] + float(shooter.m["eye_height"])]
    ctr = [target.pos[0], target.pos[1], target.pos[2] + target.height * 0.5]
    top = [target.pos[0], target.pos[1], target.pos[2] + target.height]
    los = (ray_hits_solid(shooter.g, eye, ctr) is None
           or ray_hits_solid(shooter.g, eye, top) is None)
    if not los:
        return False
    d = math.dist(eye[:2], ctr[:2])
    if d <= float(vs["near_range"]):
        return True                              # proximity: any direction
    if d > float(vs["range"]):
        return False
    # horizontal-only yaw wedge (pitch is irrelevant; you see straight up/down)
    bear = math.atan2(ctr[1] - eye[1], ctr[0] - eye[0])
    dyaw = abs((bear - shooter.aim[0] + math.pi) % (2 * math.pi) - math.pi)
    return dyaw <= math.radians(float(vs["fov_deg"]) / 2.0)


# ------------------------------------------------------------------ probe


def in_bounds(g, x, y):
    return (g.minx <= x <= g.minx + (g.nx - 1) * g.step
            and g.miny <= y <= g.miny + (g.ny - 1) * g.step)


def find_roofs(g, cfg, lo=8.0, hi=40.0):
    out = []
    for base, top, ring, holes in g.prisms:
        if base > 0.5 or top < lo:
            continue
        pts = [(ring[2 * i], ring[2 * i + 1]) for i in range(len(ring) // 2)]
        cx = sum(p[0] for p in pts) / len(pts)
        cy = sum(p[1] for p in pts) / len(pts)
        ground = g.terrain_h(cx, cy)
        if not (lo <= top - ground <= hi):
            continue
        out.append((cx, cy, top, ground, pts))
    return out


def probe(args):
    cfg = load_cfg()
    g = Geom(os.path.join(ROOT, "arena"))
    mcfg = cfg["config"]["weapons"]["mortar"]
    roofs = find_roofs(g, cfg["config"])
    if not roofs:
        print("no roofs found")
        return 1
    roofs.sort(key=lambda r: -(r[2] - r[3]))
    cx, cy, top, ground, pts = roofs[args.roof_index]
    # Stand back from the edge: the mean of the roof and its centre-ish, i.e.
    # the roof centre itself.
    shooter = Mech(g, cfg["config"], [cx, cy, top])
    shooter.aim = [0.0, 0.0]
    print(f"shooter on a {top - ground:.1f} m roof at ({cx:.0f},{cy:.0f}), top z={top:.1f}")
    print(f"  roof footprint: {min(p[0] for p in pts):.0f}..{max(p[0] for p in pts):.0f} x, "
          f"{min(p[1] for p in pts):.0f}..{max(p[1] for p in pts):.0f} y")

    print(f"\n{'dist':>6} {'bear':>6} {'seen':>5} {'gun':>5} {'laser':>6} {'mortar':>7}  note")
    rows = []
    for dist in (20, 30, 45, 60, 80, 110, 150, 190):
        for bear_deg in range(0, 360, 45):
            br = math.radians(bear_deg)
            tx = cx + math.cos(br) * dist
            ty = cy + math.sin(br) * dist
            if not in_bounds(g, tx, ty):
                continue
            tz = g.terrain_h(tx, ty)
            if g.prism_blocks(tx, ty, tz + 0.1, tz + 9.0):
                continue                          # inside a building, not a target
            t = Mech(g, cfg["config"], [tx, ty, tz])
            shooter.aim = [math.atan2(ty - cy, tx - cx), 0.0]
            seen = visible(shooter, t, cfg["config"])
            ctr = [tx, ty, tz + t.height * 0.5]
            shoot_aim = aim_at(shooter.muzzle("gun"), ctr)
            gun = ray_hit_target(g, shooter, shoot_aim, t, rng=230.0)
            las = ray_hit_target(g, shooter, shoot_aim, t, rng=280.0)
            mor = mortar_lands(g, shooter, t, mcfg)
            note = ""
            if seen and not gun:
                note = "SEEN BUT NOT SHOOTABLE"
            if not seen and mor:
                note = "MORTAR ONLY (no LOS)"
            rows.append((dist, bear_deg, seen, gun, las, mor, note))
            print(f"{dist:6d} {bear_deg:6d} {str(seen):>5} {str(gun):>5} {str(las):>6} "
                  f"{str(mor):>7}  {note}")

    seen_not_shootable = [r for r in rows if r[2] and not r[3]]
    mortar_only = [r for r in rows if not r[2] and r[5]]
    print(f"\ntotal probes: {len(rows)}")
    print(f"  visible but a gun cannot reach: {len(seen_not_shootable)}")
    print(f"  mortar reachable with no line of sight: {len(mortar_only)}")
    return 0


# ------------------------------------------------------------------ climb


class BotProc:
    def __init__(self, cmd):
        self.p = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, bufsize=0)

    def send(self, msg):
        self.p.stdin.write((json.dumps(msg, separators=(",", ":")) + "\n").encode())
        self.p.stdin.flush()

    def recv(self):
        line = self.p.stdout.readline()
        return json.loads(line) if line else None


def make_obs(tick, t, me, crates, cfg, zone):
    m = me.m
    return {
        "type": "obs", "tick": tick, "t": t, "alive": 16,
        "self": {
            "id": 1, "pos": list(me.pos), "vel": list(me.vel), "aim": list(me.aim),
            "hp": me.hp, "max_hp": me.max_hp, "energy": me.energy,
            "max_energy": float(m["energy"]), "grounded": me.grounded,
            "cubes": me.cubes, "upgrades": [], "in_zone": True,
            "weapons": [{"kind": "gun", "cooldown": 0.0, "ammo": 12, "reload": 0,
                         "stats": {"damage": 45, "cooldown": 0.24, "speed": 252,
                                   "range": 230, "spread_deg": 0.6, "mag": 12, "reload": 2.4}}],
            "stats": {k: m[k] for k in ("height", "radius", "step_up", "walk_speed",
                                        "turn_rate", "pitch_rate") if k in m},
        },
        "mechs": [], "projectiles": [], "crates": crates, "cubes": [], "pickups": [],
        "sounds": [], "beams": [], "events": [], "drops": [], "zone": zone, "offer": None,
    }


def run_climb(bot, g, config, me, crate, zone, top, tick0, seconds=45, verbose=False):
    """Drive the real bot against the physics until it is standing on `top`."""
    for n, tick in enumerate(range(tick0, tick0 + 20 * seconds, TICKS_PER_DECISION)):
        t = tick * DT
        bot.send(make_obs(tick, t, me, crate, config, zone))
        act = bot.recv()
        if act is None:
            return False, tick, "bot stopped answering"
        if verbose and n % 5 == 0:
            mv = act.get("move", [0.0, 0.0])
            print(f"    t={n * TICKS_PER_DECISION * DT:5.1f}s pos=({me.pos[0]:7.1f},{me.pos[1]:7.1f},"
                  f"{me.pos[2]:6.1f}) e={me.energy:5.1f} gnd={int(me.grounded)} "
                  f"jmp={int(bool(act.get('jump')))} bst={int(bool(act.get('boost')))} "
                  f"|mv|={math.hypot(*mv):.2f} '{(act.get('debug') or {}).get('label', '')}'")
        for _ in range(TICKS_PER_DECISION):
            me.step(act)
        if me.grounded and abs(me.pos[2] - top) < 0.5 and \
                g.prism_blocks(me.pos[0], me.pos[1], top - 0.5, top):
            return True, tick, ""
    return False, tick, "never reached the roof"


def climb(args):
    hello = load_cfg()
    g = Geom(os.path.join(ROOT, "arena"))
    hello["map"]["dir"] = os.path.join(ROOT, "arena")
    config = hello["config"]

    all_roofs = find_roofs(g, config, lo=6.0, hi=50.0)
    if not all_roofs:
        print("no roofs found")
        return 1
    # Keep only roofs you can actually stand below and climb: find a wall vertex
    # whose outside spot is clear and genuinely lower than the roof.
    cands = []
    for cx, cy, top, ground, pts in all_roofs:
        for q in pts:
            dx, dy = q[0] - cx, q[1] - cy
            n = math.hypot(dx, dy) or 1.0
            ax, ay = q[0] + dx / n * 4.5, q[1] + dy / n * 4.5
            az = g.terrain_h(ax, ay)
            if top - az < 6.0:
                continue                      # roof is not above the standing spot
            if g.prism_blocks(ax, ay, az + 1.5, az + 9.0):
                continue                      # wall in the way
            cands.append((cx, cy, top, ground, pts, q, ax, ay, az))
            break
    if not cands:
        print("no climbable roofs found")
        return 1
    cands.sort(key=lambda c: c[2] - c[8])
    # a spread of heights, so the tank budget actually gets exercised
    uniq, seen = [], set()
    for w in (0.05, 0.30, 0.55, 0.80, 0.98):
        c = cands[min(len(cands) - 1, int(w * len(cands)))]
        k = (round(c[5][0]), round(c[5][1]))
        if k not in seen:
            seen.add(k)
            uniq.append(c)

    bot = BotProc([args.bot])
    bot.send(hello)
    if bot.recv() is None:
        print("bot failed to start")
        return 1

    zone = {"center": [0.0, 0.0], "radius": 900.0, "next_center": [0.0, 0.0],
            "next_radius": 900.0, "shrink_start": 1e9, "shrink_end": 1e9 + 1.0, "dps": 0.0}
    print("climb test: stand outside a wall, target a crate on the roof\n")
    print(f"{'climb':>7} {'time':>7} {'energy':>8} {'result':>7}  detail")
    tick, fails = 0, 0
    for ci, (cx, cy, top, ground, pts, q, ax, ay, az) in enumerate(uniq):
        if args.only is not None and ci != args.only:
            continue
        dx, dy = q[0] - cx, q[1] - cy
        n = math.hypot(dx, dy) or 1.0
        crate = [{"id": 900, "pos": [q[0] - dx / n * 3.0, q[1] - dy / n * 3.0, top], "hp": 300.0}]
        me = Mech(g, config, [ax, ay, az])
        if args.verbose:
            print(f"  [{ci}] roof top={top:.1f} ground={az:.1f} climb={top - az:.1f} "
                  f"mech=({ax:.1f},{ay:.1f},{az:.1f}) crate=({crate[0]['pos'][0]:.1f},"
                  f"{crate[0]['pos'][1]:.1f},{top:.1f})")
        e0, tk0 = me.energy, tick
        ok, tick, why = run_climb(bot, g, config, me, crate, zone, top, tick, verbose=args.verbose)
        used = e0 - me.energy
        print(f"{top - az:6.1f}m {(tick - tk0) * DT:6.1f}s {used:7.0f} {('PASS' if ok else 'FAIL'):>7}  "
              f"{why}{'' if ok else f' final z={me.pos[2]:.1f} grounded={me.grounded}'}")
        if not ok:
            fails += 1

    bot.p.stdin.close()
    try:
        bot.p.wait(timeout=2)
    except Exception:
        bot.p.kill()
    print(f"\n{len(uniq) - fails}/{len(uniq)} rooftops reached")
    print("CLIMB", "PASS" if fails == 0 else "FAIL")
    return 0 if fails == 0 else 1


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="mode", required=True)
    p1 = sub.add_parser("probe")
    p1.add_argument("--roof-index", type=int, default=0)
    p2 = sub.add_parser("climb")
    p2.add_argument("--bot", default="./build/bot")
    p2.add_argument("--only", type=int, default=None)
    p2.add_argument("--verbose", action="store_true")
    a = ap.parse_args()
    return probe(a) if a.mode == "probe" else climb(a)


if __name__ == "__main__":
    sys.exit(main())
