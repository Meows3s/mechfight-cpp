// ballistics.hpp - mortar arc solution.
//
// A shell launched at `speed` m/s along the aim, falling under `gravity`, must
// pass through a point at horizontal distance `d` and height `h` above the
// muzzle. With x = v*cos(p)*t and z = v*sin(p)*t - g*t^2/2, eliminating t gives
//
//     h = d*T - (g*d^2)/(2*v^2) * (1 + T^2)          where T = tan(pitch)
//
// i.e. A*T^2 - d*T + (h + A) = 0 with A = g*d^2 / (2*v^2). Header-only so it can
// be unit-tested directly (tools/ballistics_probe.cpp).
#pragma once

#include <cmath>

namespace mech {

// Returns false when the target is out of range (no real root). `lob` picks the
// high arc, which clears obstacles the flat arc cannot. The low root is
// evaluated in its numerically stable form so it stays accurate at short range,
// where A -> 0 and the naive (d - sqrt(disc)) / (2A) is 0/0.
inline bool solve_pitch(double d, double h, double v, double g, bool lob, double& pitch_out) {
  if (d <= 1e-6 || v <= 0.0 || g <= 0.0) return false;
  const double A = g * d * d / (2.0 * v * v);
  const double disc = d * d - 4.0 * A * (h + A);
  if (disc < 0.0) return false;
  const double root = std::sqrt(disc);
  double T;
  if (lob) {
    if (A < 1e-12) return false;  // degenerate: an arbitrarily high arc
    T = (d + root) / (2.0 * A);
  } else {
    T = 2.0 * (h + A) / (d + root);  // stable form of (d - root) / (2A)
  }
  pitch_out = std::atan(T);
  return true;
}

}  // namespace mech
