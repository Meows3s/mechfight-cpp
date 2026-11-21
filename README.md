# mechroyale — C++ bot for Mech Royale (Builders Buildathon)

A `mechfight/1` bot written in C++17. It reads observations on stdin, writes one
action per observation on stdout, and answers well inside the 50 ms/tick budget.

The kit ships Python and TypeScript starters only, so the protocol wrapper here
(`src/mf.cpp`) is written from scratch against `protocol.md`. `/arena` and
`check/` are copied from the official kit for local verification; everything
under `src/` is ours.

## Status

| Check | Result |
| --- | --- |
| `check/check.py`, native | **PASS** — 300/300 in time, p99 2.7 ms, max 4.3 ms |
| `check/check.py`, match sandbox (1 CPU, 512 MB, read-only, uid 65534, no network) | **PASS** — 300/300 in time, p99 5.2 ms, max 6.8 ms |
| `tools/verify_map.py` — map model vs. an independent reimplementation | **PASS** — 660 terrain queries 0 mismatches, 3960 prism queries 0 mismatches |
| `tools/verify_ballistics.py` — mortar arc vs. numerical integration | **PASS** — 862 solvable cases, worst trajectory error 2.4e-8 m; all 738 no-solution cases genuinely out of range |
| `tools/trace_assert.py` — audit of all 300 recorded decisions | **PASS** — 0 wall-bound moves out of 300 probes |
| `tools/trace_assert.py` — forced synthetic policy paths | **PASS** — 13 paths the stream never reaches (zone run, loot take, upgrade pick, wall probe, crate farm, cube collect, cube cap, mortar arc, mortar self-blast refusal, laser charge/hold/release, out-gunned hold-fire) |
| Build time in Docker | ~100 s (limit 10 min) |
| Practice score | not submitted yet |

Answer-time headroom is ~7x on one core, and the map queries cost ~2 ms of it,
so real search in `act()` is still affordable.

## Layout

```
Dockerfile            multi-stage build; runtime image holds only the binary
CMakeLists.txt        C++17, -O2, warnings on
src/mf.hpp  src/mf.cpp   protocol layer: messages, parsing, the stdin/stdout loop
src/map.hpp src/map.cpp  arena geometry: terrain grid, prisms, walkability, LOS
src/bot.hpp src/bot.cpp  strategy
src/main.cpp             wires the strategy into the loop
tools/                   offline checks, never shipped in the image
third_party/nlohmann/json.hpp   vendored JSON (v3.11.3, pinned)
check/                kit conformance check + recorded observation stream
arena/                kit map at /arena
protocol.md, mechfight.schema.json   the rules, for reference
```

## Build and test

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
python3 check/check.py -- ./build/bot
```

Offline checks. These are what make a policy change measurable without burning
a submission (4/hour); `check.py` only proves protocol compliance.

```sh
# map model vs. a brute-force reimplementation of the same geometry
g++ -std=c++17 -O2 -Isrc -Ithird_party tools/mapprobe.cpp src/map.cpp -o build/mapprobe
./build/mapprobe arena > /tmp/probe.txt && python3 tools/verify_map.py /tmp/probe.txt

# audit every decision in the recorded stream, plus forced synthetic paths
python3 tools/trace_assert.py --bot ./build/bot --arena arena
```

The scorer's sandbox check — this is the one that decides acceptance:

```sh
docker build -t mechroyale .
python3 check/check.py --map-dir /arena -- docker run --rm -i --network none \
  --cpus 1 --memory 512m --read-only --tmpfs /tmp:rw,size=64m \
  --user 65534:65534 -v "$PWD/arena:/arena:ro" mechroyale
```

## Protocol notes worth not re-learning

- **Answer the newest observation, never an obsolete one.** The loop drains stdin
  until it goes quiet (~250 µs) before answering, so a burst of queued
  observations collapses to the newest tick. Bounded at 64 lines per answer so a
  continuous stream can never stall the bot.
- **stdout is the protocol.** Logs go to stderr. Every line is flushed.
- **Read live numbers from `hello.config`**, never hard-code balance. The current
  bot does this throughout.
- **Never emit NaN/Inf.** nlohmann's default (strict) error handler throws on
  them, which would kill the answer; `fin()` sanitizes every number written.
- **`act()` may throw** — the wrapper keeps coasting on the last move/aim/boost
  instead of letting the mech idle.

## What the bot does today

Priority order in `act()`:

1. **Stay inside the circle the zone is closing to, 4 s ahead.** The zone moves
   linearly from the current circle to the next between `shrink_start` and
   `shrink_end`, so the future position is computed exactly rather than guessed.
   Boosts only when it is actually outside.
2. **Never walk into a wall.** Steering probes 14 headings from ±0 to 180 deg,
   asks the map how far each is clear for, and takes the clearest one that still
   points at the goal. Walls, ledges taller than `step_up`, and slopes over
   `max_slope_deg` all count as blocked.
3. **Farm power cubes** while it is safe: the best crate or NPC drone per unit
   distance, inside the circle it is heading to and on reachable ground. Crates
   on rooftops are filtered out, because walking to a crate 10 m up is wasted
   time. Drones are weighted higher (2 cubes) but crates win when a weapon slot
   is empty, since half of them drop a weapon.
4. **Walk over loose cubes** — they are free and already on the ground — and
   detour for a weapon only when a slot is empty.
5. **Stop at 10 cubes.** That is `cubes.stat_cap`: past it, cubes still scale the
   mech 5% bigger each with no HP or damage gain, which is pure downside.
6. **Take a fight only when the trade is predicted winnable.** Time-to-kill each
   way, from visible HP and a cube-scaled DPS estimate that amortises gun reload
   and laser charge; every visible mech inside 300 m counts as incoming, because
   the squad shares vision and will converge. It must win by 1.35x to commit,
   otherwise it keeps farming. Placement is the number of mechs dead when you
   die, so a kill is worth a place and a kill is worth cubes -- but only if it
   does not get us killed, which is what the margin is for.
7. **Pick the cheapest kill.** Among enemies actually in range, the lowest
   HP/our-DPS, lightly penalised by distance: a hurt mech costs least to finish.
   Then hold standoff range and slide sideways, flipping direction every 2-4 s
   off `hello.seed` so it is not a predictable orbit.
8. **Face the threat when idle.** Rear hits take `mech.rear_mult` (1.5x), so the
   torso turns toward whatever last damaged us (`damaged.bearing`).
9. **One weapon at a time.** Both slots fire along the single torso aim, so a
   mortar and a direct-fire gun can never be used together -- one would always
   miss. It picks the most preferred usable weapon: mortar, then laser, then
   lightning, then the gun. The mortar solves its launch pitch in closed form
   (`src/ballistics.hpp`), lobbing over buildings when the flat arc is blocked,
   iterating once on time-of-flight, and refusing to fire inside its own blast
   radius. The laser holds to charge and releases near full. Shots go out only
   with a clear map line, leading by muzzle velocity; the gun reloads when dry.

## Known gaps (milestone 4+)

- **Rooftops are still unreachable.** Navigation is local, not global: steering
  avoids walls but has no path plan, so it can get stuck in a concave corner and
  cannot route up onto a roof. A quarter of the crates are up there, and the
  strategy notes want it fighting from height. This is the biggest gap against
  the strategy file, and the `reachable()` filter in `pick_farm` deliberately
  discards those crates until it is fixed.
- The fight estimate prices enemy weapons at **stock** config numbers scaled by
  their cubes, and assumes unknown armour is zero; it sees their weapons only as
  kinds. A heavily upgraded opponent will be misjudged.
- No incoming-fire model: projectiles are in the observation but nothing jinks or
  dodges them, and there is no low-HP disengage. Farming makes noise — a gunshot
  carries 320 m — so this is the most likely thing to get us killed next.
- Zone handling is a 4 s lookahead on the circle, not a time-to-safety estimate
  against the shrink deadline.
- No supply-drop play. Drops are worth 3 cubes plus a full heal, but they pin
  you in an 8 m ring for 4 s and are announced to everyone, which against a
  squad that hunts you is a good way to die.
- Threat facing only remembers the last `damaged` bearing for 4 s; it does not
  fuse `sounds` or `beams` into a picture of where enemies are.
- Local checks measure behaviour, not placement. See `docs/strategy.md`.
