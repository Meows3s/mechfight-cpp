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
| `check/check.py`, native | **PASS** — 300/300 in time, p99 5.4 ms |
| `check/check.py`, match sandbox (1 CPU, 512 MB, read-only, uid 65534, no network) | **PASS** — 300/300 in time, p99 6.3 ms, max 6.6 ms |
| `tools/verify_map.py` — map model vs. an independent reimplementation | **PASS** — 660 terrain queries 0 mismatches, 3960 prism queries 0 mismatches |
| `tools/trace_assert.py` — audit of all 300 recorded decisions | **PASS** — 0 wall-bound moves out of 300 probes, plus 4 forced synthetic paths |
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
3. **Detour for loot** only when a weapon slot is empty and the pickup sits
   comfortably inside the circle we are heading to.
4. **Hold station at standoff range** from the nearest visible mech and slide
   sideways, flipping direction every 2-4 s off `hello.seed` so it is not a
   predictable orbit.
5. **Face the threat when idle.** Rear hits take `mech.rear_mult` (1.5x), so the
   torso turns toward whatever last damaged us (`damaged.bearing`).
6. **Only fire with a clear shot** — the map's line-of-sight test rejects shots
   that would go into a building — leading the target by the gun's muzzle
   velocity and reloading when the magazine is dry.

## Known gaps (milestone 3+)

- Navigation is local, not global: steering avoids walls but has no path plan,
  so it can get stuck in a concave corner and cannot route onto rooftops (a
  quarter of the crates are up there).
- The laser is never charged/released and the mortar has no ballistic solver —
  only the gun and lightning are fired. Slot 1 will happily pick up a laser and
  then never use it.
- No supply-drop play, no crate or NPC farming, so no cube scaling.
- No incoming-fire model: projectiles are in the observation but nothing dodges
  them yet, and there is no low-HP disengage.
- Threat facing only remembers the last `damaged` bearing for 4 s; it does not
  fuse `sounds` or `beams` into a picture of where enemies are.
- Local checks measure behaviour, not placement. See `docs/strategy.md`.
