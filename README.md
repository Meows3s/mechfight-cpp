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
| `check/check.py`, native | **PASS** — 300/300 in time, p99 3.1 ms |
| `check/check.py`, match sandbox (1 CPU, 512 MB, read-only, uid 65534, no network) | **PASS** — 300/300 in time, p99 2.8 ms, max 4.2 ms |
| Build time in Docker | ~100 s (limit 10 min) |
| Practice score | not submitted yet |

Answer-time headroom is ~12x, so there is room for real search in `act()`.

## Layout

```
Dockerfile            multi-stage build; runtime image holds only the binary
CMakeLists.txt        C++17, -O2, warnings on
src/mf.hpp  src/mf.cpp   protocol layer: messages, parsing, the stdin/stdout loop
src/bot.hpp src/bot.cpp  strategy
src/main.cpp             wires the strategy into the loop
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

1. Take the best offered upgrade (ranked preference list).
2. Fill the one empty weapon slot with the nearest non-gun pickup in reach.
3. Stay in the zone; boost toward the centre only when it actually matters.
4. Hold a standoff against the nearest visible mech and strafe around it.
5. Lead the target with the arm gun's muzzle velocity and fire inside range;
   reload when the magazine is dry.

## Known gaps (milestone 2+)

- No map model: no obstacle avoidance, no line-of-sight tests, no use of
  buildings as cover. Ledges and walls are handled by the server's sliding, not
  by the bot.
- The laser is never charged/released and the mortar has no ballistic solver —
  only the gun and lightning are fired. Slot 1 picks up laser/mortar and then
  does nothing with them.
- No supply-drop play, no crate farming, no NPC farming, no cube scaling.
- No threat model: it does not disengage at low HP, dodge incoming fire, or
  respect rear armor.
- There is no local simulator, so policy changes are only measurable by
  submitting (4/hour). See `docs/strategy.md`.
