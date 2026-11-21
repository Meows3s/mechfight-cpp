// map.hpp - static arena geometry, loaded once from /arena (protocol.md section 11).
//
// The map is the same every match, so it is parsed at `hello` and queried after.
// Prisms (buildings, stadium, bridges) are indexed in a uniform grid; without it
// a single solidity test would walk all 9786 ring vertices.
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "mf.hpp"

namespace mech {

using mf::json;
using mf::Vec2;
using mf::Vec3;

class Map {
 public:
  // `dir` is hello.map.dir ("/arena"); `files` is hello.map.files. Falls back to
  // the conventional names when `files` is empty.
  bool load(const std::string& dir, const std::vector<std::string>& files);

  bool ok() const { return ok_; }
  const std::string& error() const { return err_; }
  std::size_t prism_count() const { return prisms_.size(); }

  double rule_max_step() const { return rule_max_step_; }
  double rule_max_slope_deg() const { return rule_max_slope_deg_; }

  // Terrain height (bilinear) at (x,y); clamped at the grid edge.
  double terrain_h(double x, double y) const;

  // Highest surface reachable from feet height `z` at (x,y): the terrain, or a
  // prism roof no higher than z + step_up.
  double surface_h(double x, double y, double z, double step_up) const;

  // Does a solid prism overlap the mech cylinder [z_lo, z_hi] at (x,y)?
  bool prism_blocks(double x, double y, double z_lo, double z_hi) const;

  bool in_bounds(double x, double y) const;

  // Arena bounds, from world.json's `bounds` (the same numbers in_bounds uses).
  double bound_x0() const { return bx0_; }
  double bound_x1() const { return bx1_; }
  double bound_y0() const { return by0_; }
  double bound_y1() const { return by1_; }

  // May a mech standing at feet height `z` occupy (x,y)? Rejects walls, steps
  // taller than step_up, and out-of-bounds cells.
  bool walkable(double x, double y, double z, double height, double step_up) const;

  // How far a horizontal sightline from (x, y, z) toward (dx, dy) runs before
  // terrain or a building stops it, capped at `cap`. The arena edge counts as
  // blocked. Coarse by design: this scores how exposed a standing spot is, at
  // hello time, over a grid -- not a per-tick precision query.
  double sight_run(double x, double y, double z, double dx, double dy,
                   double cap, double step = 3.0) const;

  // How far along `dir` (unit, horizontal) can the mech travel before it is
  // blocked or the slope exceeds max_slope_deg? Capped at max_len.
  double clearance(const Vec3& from, const Vec2& dir, double max_len, double height,
                   double step_up, double max_slope_deg) const;

  // Line of sight for a shot: prisms and terrain block, trees do not.
  bool shot_los(const Vec3& a, const Vec3& b) const;

 private:
  struct Prism {
    double base = 0.0, top = 0.0;
    std::vector<double> ring;  // flat x0,y0,x1,y1,...
    std::vector<std::vector<double>> holes;
  };

  static bool in_poly(const std::vector<double>& ring, double x, double y);
  bool inside_prism(const Prism& p, double x, double y) const;
  bool solid_point(double x, double y, double z) const;
  const std::vector<int>& cell(int ci, int cj) const;

  std::vector<Prism> prisms_;
  std::vector<int16_t> h_;
  double minx_ = 0, miny_ = 0, hstep_ = 1.0, hscale_ = 0.01;
  int nx_ = 0, ny_ = 0;
  double bx0_ = 0, bx1_ = 0, by0_ = 0, by1_ = 0;
  double rule_max_step_ = 1.5, rule_max_slope_deg_ = 50.0;

  double csz_ = 16.0;
  int ncx_ = 0, ncy_ = 0;
  std::vector<std::vector<int>> cells_;

  bool ok_ = false;
  std::string err_;
};

}  // namespace mech
