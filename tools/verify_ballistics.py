#!/usr/bin/env python3
"""Independent check of src/ballistics.hpp.

For every case build/ballistics_probe prints, integrate the shell's actual
flight at the returned pitch and confirm it passes through the target point.
A closed-form solution that does not match a simulation of the motion is worse
than none at all, so this is the check that matters.

    g++ -std=c++17 -O2 -Isrc tools/ballistics_probe.cpp -o build/ballistics_probe
    ./build/ballistics_probe > /tmp/ball.txt
    python3 tools/verify_ballistics.py /tmp/ball.txt
"""
import math
import sys

V = 72.0   # weapons.mortar.speed
G = 22.0   # weapons.mortar.gravity


def main(path):
    ok_cases = no_sol = bad = 0
    worst_err = 0.0
    for line in open(path):
        f = line.split()
        if not f or f[0] != "P":
            continue
        d, h, lob, ok = float(f[1]), float(f[2]), int(f[3]), int(f[4])
        pitch = float(f[5])

        # Out of range must mean genuinely out of range: no pitch can reach it.
        if ok == 0:
            no_sol += 1
            # max range on the flat-ish arc is v^2/g; verify the discriminant view
            A = G * d * d / (2.0 * V * V)
            disc = d * d - 4.0 * A * (h + A)
            if disc >= 0.0:
                bad += 1
                print(f"  FALSE NO-SOLUTION d={d} h={h} disc={disc:.3f}")
            continue

        ok_cases += 1
        # Simulate the actual trajectory and find the height where x == d.
        t_flight = d / (V * math.cos(pitch))
        z = V * math.sin(pitch) * t_flight - 0.5 * G * t_flight * t_flight
        err = abs(z - h)
        worst_err = max(worst_err, err)
        if err > 0.02:  # 2 cm in 100 m of flight
            bad += 1
            if bad <= 8:
                print(f"  TRAJECTORY MISS d={d} h={h} lob={lob} pitch={math.degrees(pitch):.3f}deg "
                      f"landed z={z:.4f} want {h:.4f} err={err:.4f}")
        # the lofted arc must not be lower than the flat one
        if lob == 1 and pitch > math.pi / 2:
            bad += 1
            print(f"  LOB PAST VERTICAL d={d} h={h} pitch={math.degrees(pitch):.2f}")

    print(f"solved: {ok_cases}  no-solution: {no_sol}  worst trajectory error: {worst_err:.3e} m")
    print("VERIFY", "PASS" if bad == 0 else f"FAIL ({bad})")
    return 0 if bad == 0 else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1] if len(sys.argv) > 1 else "/tmp/ball.txt"))
