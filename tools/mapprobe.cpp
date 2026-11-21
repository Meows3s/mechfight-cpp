// mapprobe - offline map-checking tool. Not part of the bot image; built ad hoc:
//   g++ -std=c++17 -O2 -Isrc -Ithird_party tools/mapprobe.cpp src/map.cpp -o build/mapprobe
// Dumps deterministic queries so they can be diffed against an independent
// implementation (tools/verify_map.py).
#include <cstdio>
#include <string>
#include <vector>

#include "map.hpp"

int main(int argc, char** argv) {
  const std::string dir = argc > 1 ? argv[1] : "arena";
  mech::Map m;
  if (!m.load(dir, {"v2/world.json", "v2/terrain.i16"})) {
    std::printf("LOAD_FAIL %s\n", m.error().c_str());
    return 1;
  }
  std::printf("META prisms %zu maxstep %.4f maxslope %.4f\n", m.prism_count(), m.rule_max_step(),
              m.rule_max_slope_deg());

  const double bx0 = -512.38, bx1 = 555.08, by0 = -333.09, by1 = 388.64;
  const int NX = 30, NY = 22;
  for (int i = 0; i < NX; ++i) {
    for (int j = 0; j < NY; ++j) {
      const double x = bx0 + (bx1 - bx0) * i / (NX - 1);
      const double y = by0 + (by1 - by0) * j / (NY - 1);
      std::printf("T %.9f %.9f %.12f\n", x, y, m.terrain_h(x, y));
    }
  }

  const double zs[] = {-6.0, -2.0, 0.5, 5.0, 12.0, 30.0};
  for (int i = 0; i < NX; ++i) {
    for (int j = 0; j < NY; ++j) {
      const double x = bx0 + (bx1 - bx0) * i / (NX - 1);
      const double y = by0 + (by1 - by0) * j / (NY - 1);
      for (double z : zs)
        std::printf("S %.9f %.9f %.9f %d\n", x, y, z, m.prism_blocks(x, y, z - 0.5, z + 0.5) ? 1 : 0);
    }
  }

  // A few walkability / clearance / los spot checks from real observation positions.
  const double probes[][3] = {{-217.98, -22.04, -4.73}, {-208.24, -7.7, -4.7}, {10.2, 55.1, 0.3},
                              {41.3, -72.8, 0.0},   {14.0, -41.0, 0.1},  {520.0, 122.0, 4.0}};
  for (const auto& p : probes) {
    std::printf("W %.2f %.2f %.2f %d\n", p[0], p[1], p[2],
                m.walkable(p[0], p[1], p[2], 9.0, 1.5) ? 1 : 0);
  }
  for (const auto& p : probes) {
    mech::Vec3 a{p[0], p[1], p[2] + 7.0};
    mech::Vec3 b{p[0] + 40.0, p[1] + 25.0, p[2] + 7.0};
    std::printf("L %.2f %.2f %.2f %d\n", p[0], p[1], p[2], m.shot_los(a, b) ? 1 : 0);
  }
  // Clearance north/east from each spot, as a crude "is the bot walled in" signal.
  for (const auto& p : probes) {
    mech::Vec3 a{p[0], p[1], p[2]};
    std::printf("C %.2f %.2f %.2f %.2f %.2f\n", p[0], p[1], p[2],
                m.clearance(a, {1.0, 0.0}, 30.0, 9.0, 1.5, 50.0),
                m.clearance(a, {0.0, 1.0}, 30.0, 9.0, 1.5, 50.0));
  }
  return 0;
}
