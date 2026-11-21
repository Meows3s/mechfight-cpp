#!/usr/bin/env python3
"""Survival harness: run the real bot through a whole match against a hunter
squad and the closing zone.

Why this exists: every submission so far died in the opening ~40 s of a 415 s
match, and the zone does not even start moving until t=60. Placement is decided
entirely by whether the bot survives the first hunter -- and nothing local
measured that. check.py replays a frozen stream that never reacts; sim.py climb
tests one mechanic in isolation. This runs the match.

    python3 tools/survival.py --bot ./build/bot --matches 6
    python3 tools/survival.py --baseline center           # sanity: does it discriminate?

Fidelity, stated because it decides how much to trust the numbers:
  - One hunter engages from t=0, a second from t=90, a third from t=180, all
    fifteen after t=300. The docs say "one early, more as you thin them out, all
    of them late"; the schedule between those is INVENTED here.
  - Hunters share vision, remember our last seen position for vision.memory_s,
    hold a standoff, and fire on line of sight.
  - All fire is resolved as hitscan, for hunters and for us. Real shots have a
    spread and travel time, so the hunters are more accurate than reality and
    kills against us are an upper bound -- survival times are a LOWER bound.
  - Zone holds its documented hold/shrink schedule and dps, with the centre
    interpolated to one fixed end point.
  - No supply drops, no drones, no cube pickups, and hunters never take damage
    from the zone, so this cannot produce a placement; it produces time survived
    and kills. Placement also depends on how many hunters the zone kills, which
    is not modelled.
"""
import argparse
import math
import os
import random
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from trace_assert import Geom
from los_audit import Grid
from sim import Mech, DT, TICKS_PER_DECISION, load_cfg

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOS_STEP = 1.0        # coarse on purpose: this runs thousands of times per match
LOS_EVERY = 4         # ticks between line-of-sight recomputes


# ------------------------------------------------------------------ line of sight


def los_clear(grid, g, a, b, step=LOS_STEP, limit=320.0):
    """Any solid between a and b? Coarse but ~20x cheaper than the fine sampler,
    which matters when it runs 15 hunters x 4000 ticks."""
    dx, dy, dz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
    L = math.sqrt(dx * dx + dy * dy + dz * dz)
    if L < 1e-6:
        return True
    if L > limit:
        return False
    n = max(1, int(L / step))
    for i in range(1, n):
        t = i / n
        x, y, z = a[0] + dx * t, a[1] + dy * t, a[2] + dz * t
        if z <= g.terrain_h(x, y):
            return False
        if grid.solid(x, y, z):
            return False
    return True


# ------------------------------------------------------------------ zone


class Zone:
    def __init__(self, cfg):
        z = cfg["zone"]
        self.c0 = [float(z["start_center"][0]), float(z["start_center"][1])]
        self.r0 = float(z["start_radius"])
        self.phases = z["phases"]
        # one fixed end point for the drift; the real centre is seeded per match
        self.c1 = [-42.12, 7.11]
        self.total = float(sum(p["hold_s"] + p["shrink_s"] for p in self.phases))

    def at(self, t):
        r = self.r0
        el = 0.0
        for p in self.phases:
            if t < el + p["hold_s"]:
                return self._centre(t) + [r, float(p["dps"])]
            el += p["hold_s"]
            if t < el + p["shrink_s"]:
                u = (t - el) / p["shrink_s"]
                return self._centre(t) + [r + (float(p["radius"]) - r) * u, float(p["dps"])]
            el += p["shrink_s"]
            r = float(p["radius"])
        return self._centre(t) + [r, float(self.phases[-1]["dps"])]

    def _centre(self, t):
        u = min(1.0, max(0.0, t / max(1e-6, self.total)))
        return [self.c0[0] + (self.c1[0] - self.c0[0]) * u,
                self.c0[1] + (self.c1[1] - self.c0[1]) * u]


# ------------------------------------------------------------------ actor


class Actor:
    """Our mech or a hunter: a Mech plus combat state."""

    def __init__(self, g, config, pos, aid, weapons, is_bot=False):
        self.mech = Mech(g, config, pos)
        self.id = aid
        self.weapons = list(weapons)
        self.is_bot = is_bot
        self.cfg = config["weapons"]
        self.cd = {w: 0.0 for w in self.weapons}
        self.charge = 0.0
        self.alive = True
        self.since_hit = 999.0
        self.hit_flash = 0.0
        self._act = None

    @property
    def pos(self):
        return self.mech.pos

    @property
    def vel(self):
        return self.mech.vel

    def centre(self):
        return [self.pos[0], self.pos[1], self.pos[2] + 4.5]

    def muzzle(self):
        return [self.pos[0], self.pos[1], self.pos[2] + self.mech.m["muzzle_height"]]

    def aim(self):
        return self.mech.aim


# ------------------------------------------------------------------ hunters


class Hunter(Actor):
    STANDOFF = 110.0

    def __init__(self, g, config, pos, hid):
        super().__init__(g, config, pos, hid, ("gun", "laser"))
        self.last_seen = None       # [x, y, z, t] of our last observed position
        self.memory = float(config["vision"]["memory_s"])

    def memory_of(self, t):
        if self.last_seen is None:
            return None
        if t - self.last_seen[3] > self.memory:
            return None
        return self.last_seen[:3]

    def step_ai(self, dt, t, g, grid, config, targets, known_pos, fallback=None,
                has_los=False):
        m = self.mech.m
        if known_pos is not None:
            self.last_seen = [known_pos[0], known_pos[1], known_pos[2], t]
        # A hunter that has never seen us still has to go somewhere: it converges
        # on the zone centre, which is both what keeps it alive and how a squad
        # eventually finds a lone mech. Without this they simply stop, and a
        # motionless bot outlives them -- which is not what the scoreboard says.
        goal = self.memory_of(t) or known_pos or fallback
        act = {"move": [0.0, 0.0], "boost": False, "jump": False}
        if goal is not None:
            to = [goal[0] - self.pos[0], goal[1] - self.pos[1]]
            L = math.hypot(*to) or 1.0
            u = [to[0] / L, to[1] / L]
            d = math.dist(self.pos[:2], goal[:2])
            # Holding a standoff only makes sense when it can actually shoot from
            # there. Behind a building it has to close until a line opens, or it
            # parks at 110 m forever -- which made the harness score a motionless
            # bot as long-lived, the opposite of the real scoreboard.
            standoff = self.STANDOFF if has_los else 45.0
            radial = 1.0 if d > standoff else (-1.0 if d < standoff * 0.7 else 0.0)
            act["move"] = [u[0] * radial - u[1] * 0.9, u[1] * radial + u[0] * 0.9]
            n = math.hypot(*act["move"]) or 1.0
            act["move"] = [act["move"][0] / n, act["move"][1] / n]
            nz = math.hypot(act["move"][0], act["move"][1]) or 1.0
            self.mech.aim = [math.atan2(u[1], u[0]), 0.0]
            # aim at the target's centre, with the pitch that reaches it
            for tg in targets:
                v = [tg.centre()[0] - self.muzzle()[0], tg.centre()[1] - self.muzzle()[1],
                     tg.centre()[2] - self.muzzle()[2]]
                self.mech.aim = [math.atan2(v[1], v[0]), math.atan2(v[2], math.hypot(v[0], v[1]))]
                break
        self.mech.step(act)
        for w in self.weapons:
            self.cd[w] = max(0.0, self.cd[w] - dt)

    def try_fire(self, g, grid, targets, dt):
        """Hitscan on line of sight. Returns list of (target, damage, rear)."""
        out = []
        if not targets:
            return out
        tgt = targets[0]
        d = math.dist(self.muzzle(), tgt.centre())
        for w in self.weapons:
            if self.cd[w] > 0.0:
                continue
            cfg = self.cfg[w]
            if d > float(cfg.get("range", 230.0)):
                continue
            if not los_clear(grid, g, self.muzzle(), tgt.centre()):
                continue
            dmg = float(cfg["damage"])
            # a hit from outside the front arc takes the rear multiplier
            bear = math.atan2(tgt.pos[1] - self.pos[1], tgt.pos[0] - self.pos[0])
            rel = abs((bear - tgt.mech.aim[0] + math.pi) % (2 * math.pi) - math.pi)
            rear = rel > math.radians(90.0)
            if rear:
                dmg *= float(self.mech.m.get("rear_mult", 1.5))
            out.append((tgt, dmg, rear))
            self.cd[w] = float(cfg.get("cooldown", 1.0)) + float(cfg.get("charge", 0.0))
        return out


# ------------------------------------------------------------------ obs for the bot


def make_obs(tick, t, me, hunters, zone, config, kills):
    """Only what the bot could legitimately see. Vision needs eye line of sight
    to the target's centre or top, then either near_range in any direction or
    range inside the fov yaw wedge."""
    vs = config["vision"]
    eye = [me.pos[0], me.pos[1], me.pos[2] + me.mech.m["eye_height"]]
    vis_mechs = []
    for h in hunters:
        if not h.alive:
            continue
        ctr = h.centre()
        d = math.dist(eye[:2], ctr[:2])
        if d > float(vs["range"]):
            continue
        bear = math.atan2(ctr[1] - eye[1], ctr[0] - eye[0])
        dyaw = abs((bear - me.mech.aim[0] + math.pi) % (2 * math.pi) - math.pi)
        in_wedge = (d <= float(vs["near_range"])
                    or dyaw <= math.radians(float(vs["fov_deg"]) / 2.0))
        if not in_wedge:
            continue
        # the bot can only know this itself through its own map, but feeding it
        # only wedge geometry is the honest approximation here
        vis_mechs.append({
            "id": h.id, "kind": "mech", "npc": False,
            "pos": list(h.pos), "vel": list(h.vel), "aim": list(h.mech.aim),
            "hp": h.mech.hp, "max_hp": h.mech.max_hp, "cubes": 0.0,
            "weapons": list(h.weapons), "charging": False,
        })
    m = me.mech.m
    return {
        "type": "obs", "tick": tick, "t": t, "alive": 16 - kills,
        "self": {
            "id": 0, "pos": list(me.pos), "vel": list(me.vel), "aim": list(me.mech.aim),
            "hp": me.mech.hp, "max_hp": me.mech.max_hp, "energy": me.mech.energy,
            "max_energy": float(m["energy"]), "grounded": me.mech.grounded,
            "cubes": 0.0, "upgrades": [], "in_zone": True,
            "weapons": [{"kind": "gun", "cooldown": 0.0, "ammo": 12, "reload": 0,
                         "stats": {"damage": 45, "cooldown": 0.24, "speed": 252,
                                   "range": 230, "spread_deg": 0.6, "mag": 12, "reload": 2.4}},
                        None],
            "stats": {k: m[k] for k in ("height", "radius", "step_up", "walk_speed",
                                        "turn_rate", "pitch_rate", "eye_height",
                                        "muzzle_height", "gravity") if k in m},
        },
        "mechs": vis_mechs, "projectiles": [], "crates": [], "cubes": [], "pickups": [],
        "sounds": [], "beams": [], "events": [], "drops": [],
        "zone": {"center": [zone[0], zone[1]], "radius": zone[2],
                 "next_center": [zone[0], zone[1]], "next_radius": zone[2],
                 "shrink_start": t, "shrink_end": t + 30.0, "dps": zone[3]},
        "offer": None,
    }


# ------------------------------------------------------------------ one match


def run_match(bot_cmd, seed, greedy=False, verbose=False, trace=False):
    rng = random.Random(seed)
    cfg = load_cfg()
    config = cfg["config"]
    g = Geom(os.path.join(ROOT, "arena"))
    grid = Grid(g)
    cfg["map"]["dir"] = os.path.join(ROOT, "arena")

    zone = Zone(config)
    mm = config["mech"]
    # our spawn: inside the start circle, on ground
    for _ in range(400):
        a = rng.uniform(0, 2 * math.pi)
        r = rng.uniform(0.0, 0.55) * float(config["zone"]["start_radius"])
        x = float(config["zone"]["start_center"][0]) + math.cos(a) * r
        y = float(config["zone"]["start_center"][1]) + math.sin(a) * r
        z = g.terrain_h(x, y)
        if not grid.solid(x, y, z + 0.5) and not g.prism_blocks(x, y, z + 0.1, z + 9.0):
            break
    me = Actor(g, config, [x, y, z], 0, ("gun",), is_bot=True)

    # hunters: at least spawn_min_separation away, on ground
    hunters = []
    sep = float(config["match"]["spawn_min_separation"])
    tries = 0
    while len(hunters) < 15 and tries < 4000:
        tries += 1
        a = rng.uniform(0, 2 * math.pi)
        d = rng.uniform(sep, sep * 2.6)
        hx = me.pos[0] + math.cos(a) * d
        hy = me.pos[1] + math.sin(a) * d
        hz = g.terrain_h(hx, hy)
        if grid.solid(hx, hy, hz + 0.5):
            continue
        if g.prism_blocks(hx, hy, hz + 0.1, hz + 9.0):
            continue
        hunters.append(Hunter(g, config, [hx, hy, hz], len(hunters) + 1))

    # nearest first: `hunters[:active]` must mean the nearest hunters, not an
    # arbitrary spawn order -- otherwise the one "early" attacker can start
    # 338 m away on the far side of the map and take 30 s to even arrive.
    hunters.sort(key=lambda h: math.dist(h.pos[:2], me.pos[:2]))
    for i, h in enumerate(hunters):
        h.id = i + 1
    # "One early": the squad dispatches a hunter straight to our drop point.
    # Perfect memory for that one only; the rest obey vision.memory_s.
    hunters[0].memory = 1e9
    hunters[0].last_seen = [me.pos[0], me.pos[1], me.pos[2], 0.0]

    proc = None
    if bot_cmd:
        proc = subprocess.Popen(bot_cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, bufsize=0)
        proc.stdin.write((__import__("json").dumps(cfg, separators=(",", ":")) + "\n").encode())
        proc.stdin.flush()
        if proc.stdout.readline() is None:
            return None

    active = 1
    exposure_ticks = 0
    total_ticks = 0
    kills = 0
    dmg_taken = 0.0
    regen_wait = 0.0
    last_los = {}
    death_t = None
    hazard = 0.0

    duration = float(config["match"]["duration_s"])
    n_ticks = int(duration / DT)
    for tick in range(n_ticks + 1):
        t = tick * DT
        zx, zy, zr, zdps = zone.at(t)
        total_ticks += 1

        # scheduled arrivals ("one early, more as you thin them out, all late")
        if t >= 300.0:
            active = 15
        else:
            active = 1 + int(t // 90.0)

        # --- our bot decides every TICKS_PER_DECISION
        if tick % TICKS_PER_DECISION == 0:
            if proc:
                proc.stdin.write((__import__("json").dumps(
                    make_obs(tick, t, me, hunters, [zx, zy, zr, zdps], config, kills),
                    separators=(",", ":")) + "\n").encode())
                proc.stdin.flush()
                line = proc.stdout.readline()
                try:
                    act = __import__("json").loads(line) if line else {}
                except Exception:
                    act = {}
            elif greedy:
                to = [zx - me.pos[0], zy - me.pos[1]]
                n = math.hypot(*to) or 1.0
                act = {"move": [to[0] / n, to[1] / n], "boost": False, "jump": False,
                       "aim": list(me.mech.aim), "fire": [False, False]}
            else:  # freeze baseline
                act = {"move": [0.0, 0.0], "boost": False, "jump": False,
                       "aim": list(me.mech.aim), "fire": [False, False]}
            me._act = act
        act = getattr(me, "_act", {"move": [0.0, 0.0]})

        if "aim" in act:
            me.mech.aim = list(act["aim"])
        me.mech.step(act)

        if trace and tick % 40 == 0:
            lab = (act.get("debug") or {}).get("label", "")
            nd = min((math.dist(h.pos[:2], me.pos[:2]) for h in hunters[:active] if h.alive),
                     default=-1)
            mv = act.get("move", [0.0, 0.0])
            print(f"      t={t:6.1f} hp={me.mech.hp:6.0f} e={me.mech.energy:5.0f} "
                  f"h1d={nd:6.1f} mv=({mv[0]:5.2f},{mv[1]:5.2f}) {lab}")

        # our fire: hitscan at whatever the bot aimed at
        if act.get("fire", [False, False])[0]:
            aim = me.mech.aim
            d = [math.cos(aim[1]) * math.cos(aim[0]), math.cos(aim[1]) * math.sin(aim[0]),
                 math.sin(aim[1])]
            gc = config["weapons"]["gun"]
            for h in hunters[:active]:
                if not h.alive:
                    continue
                # does the ray pass through the hunter's cylinder?
                for s in range(1, 60):
                    L = 4.0 * s
                    p = [me.muzzle()[0] + d[0] * L, me.muzzle()[1] + d[1] * L,
                         me.muzzle()[2] + d[2] * L]
                    if p[2] <= g.terrain_h(p[0], p[1]) or grid.solid(p[0], p[1], p[2]):
                        break
                    if math.dist(p[:2], h.pos[:2]) <= h.mech.radius + 0.3 \
                            and h.pos[2] - 0.2 <= p[2] <= h.pos[2] + h.mech.height:
                        h.mech.hp -= float(gc["damage"])
                        if h.mech.hp <= 0:
                            h.alive = False
                            kills += 1
                        break

        # --- hunters
        known = None
        for h in hunters[:active]:
            if not h.alive:
                continue
            if tick % LOS_EVERY == 0:
                last_los[h.id] = los_clear(grid, g, h.muzzle(), me.centre())
            if last_los.get(h.id) and known is None:
                known = list(me.pos[:2]) + [me.pos[2]]
        # every hunter moves; only the `active` nearest ones get our position
        # and may shoot. Distant inactive hunters get a cheap analytic roam --
        # full Mech physics for 15 idle hunters is what made a match take
        # minutes (Geom.surface_h scans every prism on each call).
        for h in hunters:
            if not h.alive:
                continue
            is_active = h.id <= active
            # full Mech physics only for the active hunters (the ramp the docs
            # describe); the rest roam analytically. 338 m spawn distances mean
            # a distance cut-off would still catch all 15 and cost minutes.
            if is_active:
                h.step_ai(DT, t, g, grid, config, [me] if me.mech.hp > 0 else [],
                          known if is_active else None,
                          fallback=[zx, zy, g.terrain_h(zx, zy)],
                          has_los=bool(last_los.get(h.id)))
                if is_active:
                    for tgt, dmg, _rear in h.try_fire(g, grid,
                                                      [me] if me.mech.hp > 0 else [], DT):
                        tgt.mech.hp -= dmg
                        tgt.since_hit = 0.0
                        dmg_taken += dmg
            elif tick % 20 == h.id % 20:
                to = [zx - h.pos[0], zy - h.pos[1]]
                n = math.hypot(*to) or 1.0
                step = float(h.mech.m["walk_speed"]) * 20 * DT
                h.pos[0] += to[0] / n * step
                h.pos[1] += to[1] / n * step
                h.pos[2] = g.terrain_h(h.pos[0], h.pos[1])

        # --- exposure bookkeeping (are we being shot at, or hidden?)
        if any(last_los.get(h.id) for h in hunters[:active] if h.alive):
            exposure_ticks += 1

        # --- zone damage
        if math.dist(me.pos[:2], (zx, zy)) > zr:
            me.mech.hp -= zdps * DT
            me.since_hit = 0.0

        # --- hp regen
        me.since_hit += DT
        if me.mech.hp > 0 and me.since_hit >= float(mm["regen_delay"]):
            me.mech.hp = min(me.mech.max_hp, me.mech.hp + float(mm["regen_per_s"]) * DT)

        if me.mech.hp <= 0 and death_t is None:
            death_t = t
            break

    if proc:
        try:
            proc.stdin.close()
            proc.wait(timeout=3)
        except Exception:
            proc.kill()

    if verbose:
        print(f"    spawn=({me.pos[0]:.0f},{me.pos[1]:.0f}) hunters_los_seen={len(last_los)}")
    return {"death_t": death_t if death_t is not None else duration,
            "kills": kills,
            "exposure": exposure_ticks / max(1, total_ticks),
            "dmg_taken": dmg_taken}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bot", default=os.path.join(ROOT, "build/bot"))
    ap.add_argument("--matches", type=int, default=6)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--baseline", choices=["none", "center", "freeze"], default="none",
                    help="bypass the bot: 'center' runs at the zone centre, 'freeze' stands still")
    ap.add_argument("--verbose", action="store_true")
    ap.add_argument("--trace", action="store_true", help="per-2s line: hp, nearest hunter, move, label")
    a = ap.parse_args()

    if a.baseline == "none":
        cmd = [a.bot]
        label = os.path.relpath(a.bot, ROOT)
    elif a.baseline == "center":
        cmd = None
        label = "baseline: run at zone centre"
    else:
        cmd = None
        label = "baseline: freeze"

    rows = []
    for i in range(a.matches):
        r = run_match(cmd, a.seed + i, greedy=(a.baseline == "center"),
                      verbose=a.verbose, trace=a.trace)
        if r is None:
            print("bot failed to start")
            return 1
        rows.append(r)
        print(f"  match {i + 1}: died t={r['death_t']:6.1f}s  kills={r['kills']}  "
              f"exposed {100 * r['exposure']:3.0f}% of ticks  dmg={r['dmg_taken']:8.0f}")

    n = len(rows)
    print(f"\n{label}: {n} matches")
    print(f"  median survival   {sorted(r['death_t'] for r in rows)[n // 2]:6.1f} s "
          f"(of 480 s)")
    print(f"  mean survival     {sum(r['death_t'] for r in rows) / n:6.1f} s")
    print(f"  mean exposure     {100 * sum(r['exposure'] for r in rows) / n:6.1f} % of ticks")
    print(f"  mean kills        {sum(r['kills'] for r in rows) / n:.2f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
