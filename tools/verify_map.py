#!/usr/bin/env python3
"""Independent check of src/map.cpp against a brute-force reimplementation.

Reads `arena/v2/world.json` + `terrain.i16` directly and recomputes every query
`build/mapprobe` prints, with no spatial index and no shared code. Any mismatch
means the C++ map model disagrees with the map files.

    g++ -std=c++17 -O2 -Isrc -Ithird_party tools/mapprobe.cpp src/map.cpp -o build/mapprobe
    ./build/mapprobe arena > /tmp/probe.txt
    python3 tools/verify_map.py /tmp/probe.txt
"""
import json
import math
import struct
import sys

KIT = "arena"


def main(path):
    d = json.load(open(f"{KIT}/v2/world.json"))
    t = d["terrain"]
    minx, miny, step = t["minX"], t["minY"], t["step"]
    nx, ny, scale = t["nx"], t["ny"], t["scale"]
    raw = open(f"{KIT}/v2/terrain.i16", "rb").read()
    h = struct.unpack(f"<{nx * ny}h", raw[: nx * ny * 2])

    def terrain_h(x, y):
        fx = (x - minx) / step
        fy = (y - miny) / step
        i, j = math.floor(fx), math.floor(fy)
        tx, ty = fx - i, fy - j

        def at(ii, jj):
            ii = min(max(ii, 0), nx - 1)
            jj = min(max(jj, 0), ny - 1)
            return h[jj * nx + ii] * scale

        a, b, c, e = at(i, j), at(i + 1, j), at(i, j + 1), at(i + 1, j + 1)
        return (a * (1 - tx) + b * tx) * (1 - ty) + (c * (1 - tx) + e * tx) * ty

    def in_poly(ring, x, y):
        n = len(ring) // 2
        if n < 3:
            return False
        inside = False
        j = n - 1
        for i in range(n):
            xi, yi = ring[2 * i], ring[2 * i + 1]
            xj, yj = ring[2 * j], ring[2 * j + 1]
            if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
                inside = not inside
            j = i
        return inside

    prisms = []
    for p in d["prisms"]:
        ring = p["ring"]
        if len(ring) < 6:
            continue
        holes = [hh for hh in (p.get("holes") or []) if len(hh) >= 6]
        prisms.append((p["base"], p["top"], ring, holes))
    print(f"python sees {len(prisms)} prisms (C++ reported: see META line)")

    def inside_prism(p, x, y):
        base, top, ring, holes = p
        if not in_poly(ring, x, y):
            return False
        for hh in holes:
            if in_poly(hh, x, y):
                return False
        return True

    def prism_blocks(x, y, zlo, zhi):
        if zhi <= zlo:
            return False
        for p in prisms:
            if p[0] < zhi and p[1] > zlo and inside_prism(p, x, y):
                return True
        return False

    bad_t = bad_s = 0
    n_t = n_s = 0
    worst = 0.0
    for line in open(path):
        f = line.split()
        if not f:
            continue
        if f[0] == "T":
            x, y, got = float(f[1]), float(f[2]), float(f[3])
            want = terrain_h(x, y)
            n_t += 1
            err = abs(want - got)
            worst = max(worst, err)
            if err > 1e-6:
                bad_t += 1
                if bad_t <= 5:
                    print(f"  TERRAIN mismatch at ({x:.1f},{y:.1f}): cpp {got:.6f} py {want:.6f}")
        elif f[0] == "S":
            x, y, z = float(f[1]), float(f[2]), float(f[3])
            got = int(f[4])
            want = 1 if prism_blocks(x, y, z - 0.5, z + 0.5) else 0
            n_s += 1
            if got != want:
                bad_s += 1
                if bad_s <= 8:
                    print(f"  SOLID mismatch at ({x:.1f},{y:.1f},{z:.1f}): cpp {got} py {want}")

    print(f"terrain: {n_t} queries, {bad_t} mismatches, worst abs error {worst:.3e}")
    print(f"solid  : {n_s} queries, {bad_s} mismatches")
    ok = bad_t == 0 and bad_s == 0
    print("VERIFY", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1] if len(sys.argv) > 1 else "/tmp/probe.txt"))
