// ballistics_probe - dumps solve_pitch() results for an independent check.
//   g++ -std=c++17 -O2 -Isrc tools/ballistics_probe.cpp -o build/ballistics_probe
//   ./build/ballistics_probe > /tmp/ball.txt && python3 tools/verify_ballistics.py
#include <cstdio>

#include "ballistics.hpp"

int main() {
  const double v = 72.0, g = 22.0;  // hello.config weapons.mortar
  int n = 0;
  for (double d = 5.0; d <= 400.0; d += 5.0) {
    for (double h = -30.0; h <= 60.0; h += 10.0) {
      for (int lob = 0; lob < 2; ++lob) {
        double p = 0.0;
        const bool ok = mech::solve_pitch(d, h, v, g, lob == 1, p);
        std::printf("P %.6f %.6f %d %d %.12f\n", d, h, lob, ok ? 1 : 0, ok ? p : 0.0);
        ++n;
      }
    }
  }
  std::fprintf(stderr, "%d cases\n", n);
  return 0;
}
