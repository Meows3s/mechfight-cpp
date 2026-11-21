# Strategy

Notes to work from, not gospel. Numbers come from `hello.config` at runtime; the
values quoted here are the ones in the kit's recorded match.

## The metric decides almost everything

Score = mean over a run of `16 - place`, where `place` is the order mechs die in
and `place` = the number of mechs alive (including you) at the moment you die.

- Dying first: `place 16` -> **0 points**.
- Last alive: `place 1` -> **15 points**.
- Survivors when the clock runs out are ranked by **HP**.

So the score is exactly "how many of the other 15 were already dead when I
died", averaged. That includes the ones we killed ourselves: `alive` counts
every mech still standing, so each kill of ours moves our own place up by one at
the moment we die. "Placement counts, not kills" means **do not die for a kill**
— it does not mean kills are worthless. A kill is worth a place point plus the
victim's cubes + 2, provided we live past it.

Baseline to beat: **1.15** (the kit's Python example). That is roughly `place
14.8` on average — the starter usually dies first or near-first against a
coordinated 15-bot squad.

Corollary: **variance matters as much as mean.** One early death costs ~15 points
in that match. A policy that trades a rare fast death for a slightly worse
average placement is usually worth it.

## What practice actually is

48 matches, each vs 15 house bots acting as **one squad**:

- 4 easy, 4 medium, 4 hard, 3 expert.
- They **share vision**, **never fight each other**, and **hunt you weakest
  first**: one early, more as you thin them out, all of them late.

This is a 15v1 survival problem, not a free-for-all. Three consequences:

1. **You are always the target.** There is no "let them fight each other".
2. **Killing escalates the threat — but a kill is also a place point.** `alive`
   counts every mech still standing, so anything we kill (like anything the zone
   kills) is one more mech already dead when we die: worth exactly one place,
   plus the victim's cubes. The countervailing cost is the squad committing more
   bots to us ("more as you thin them out"). Those two channels pull opposite
   ways and the net is **not measurable locally** — only a submitted practice run
   will say which dominates. Hence the policy rule is "engage only when the trade
   is predicted winnable", not "never engage".
3. **"Weakest first" is a death spiral risk.** If the squad's notion of weakest
   is low HP, being hurt draws more attackers, which hurts you more. Keeping HP
   topped up (regen is 45/s after 5 s out of combat) is defensive *and*
   de-escalating.

The zone is the other killer. It shrinks to radius 0 by ~6:55 with damage
escalating to 260 HP/s, and zone damage also resets the 5 s regen timer. House
bots that mis-position die to it. **Every mech the zone kills is a place
gained for free.** Outliving rather than outfighting is the cheapest points on
the board.

## Levers, roughly in order of expected value per hour of work

1. **Never be out of the zone.** Zone damage is unarmored, stops regen, and is
   the single most likely way to lose a place for free. Predictable: the obs
   gives the current circle, the next circle, and the shrink window, and motion
   between them is linear. Pre-positioning during `hold_s` is free.
2. **Face the threat.** Rear hits take `rear_mult` = 1.5x damage, and the front
   arc is 180 deg wide. Keeping the torso pointed at whatever is shooting you is
   a ~33% damage reduction on incoming fire, for free. Attackers are visible:
   `projectiles`, `beams`, `sounds` (bearing + range band), and `damaged`
   events (bearing + attacker id) all point at them.
3. **Map model + obstacle-aware movement.** No walls, no ledges, no cover
   without it. Right now the bot walks into buildings and relies on the
   server's sliding. Needed for: reaching loot, breaking line of sight, not
   getting stuck, and using rooftops (a quarter of the crates are up there).
4. **Dodge.** Enemy projectiles are listed in `obs.projectiles` with position and
   velocity. A lateral step beats a bullet travelling at 252 m/s at range.
   Lasers are telegraphed (`charging: true` + their aim) and make a sound.
5. **Weapon discipline.** We currently fire the gun and the lightning and
   nothing else. The laser is the highest-damage weapon (315 at full charge,
   hitscan, 280 m) and it is currently dead weight in slot 1: it needs
   charge-then-release on target. The mortar needs a pitch-for-range solver
   (`v^2/g` ~ 235 m at 45 deg) to lob over buildings.
6. **Cube scaling.** +4% max HP and +4% damage per cube to a cap of 10, plus
   +4% damage *and* 5% bigger per cube with no cap on size. ~10 cubes is the
   sweet spot: 40% more HP and damage, and only 50% larger. Crates (36/match,
   1 cube) and NPC drones (12/match, 2 cubes each, 350 HP) are the safe farm.
7. **Supply drops.** 3 per match (70/140/210 s), each worth 3 cubes + full HP +
   full energy, for standing alone in an 8 m ring for 4 s. High value, high
   risk — announced to everyone and it pins you in place.
8. **Escalation timing.** Given (2) above, the plan is not "fight early". Farm
   cubes and position, let the zone thin the herd, and be strong and inside the
   final circle when the brawl starts.

## The bottleneck: no local simulator

`check/check.py` verifies protocol compliance, not play. Its stream is recorded
and does not react to actions. So today the *only* way to measure a policy
change is to submit: **4 submissions/hour**, each taking minutes to score.

Options, in increasing cost:

- **(a) Trace assertions (cheap, hours).** Replay `check/stream.jsonl.gz`
  through the decision function and assert invariants: never out of the zone,
  never idle, always facing the current threat, never aim beyond weapon range.
  Catches regressions without a submission. Does not measure placement.
- **(b) Partial simulator (medium, a day).** Terrain grid + prisms + movement +
  the zone + gun ballistics + a crude hunter model. Enough to test navigation,
  zone timing, and dodge logic. Fidelity risk is real; the squad AI and exact
  collision rules are not fully specified.
- **(c) Full reimplementation (expensive).** Effectively rewriting the server.
  Not worth it inside a 33-hour window.
- **(d) Submit and read the score.** Ground truth, but 4/hour.

Recommended: **(a) now, (b) only if time allows.** Batch policy changes so each
submission tests one idea, and keep the best-scoring submission as the locked
entry.

## Timeline

Cutoff is **Sun 11:00 EST** (~33 h from setup). All 16 finalists then play a
64-match series on new seeds. Growth needs to be aimed at *general* play, not
overfit to the 48 private practice seeds — the series is on new seeds and the
final 4 matches are live.

## Roadmap

- **M1 (done).** Protocol-correct C++ bot, passes `check.py` natively and in the
  match sandbox. Zone-seeking, one weapon pickup, gun with lead.
- **M2 (done).** Map model (terrain grid + prisms, spatial-indexed, verified
  against an independent reimplementation), obstacle-aware steering, predictive
  zone seeking, threat-facing, shot line-of-sight. `tools/trace_assert.py` now
  audits every recorded decision and forces the paths the stream never reaches.
- **M3 (done).** Map-aware farming: best crate or drone per unit distance,
  filtered to reachable ground inside the target circle; walks over loose cubes;
  stops at the 10-cube stat cap; engages only inside 110 m.
- **M4.** Threat/dodge model and low-HP disengage (farming is loud, and nothing
  dodges incoming fire yet), then laser charge-and-release and a mortar arc
  solver.
- **M5.** Submit early, measure, tune; lock the best entry well before the cutoff.
