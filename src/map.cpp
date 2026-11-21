#include "map.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <sstream>

namespace mech {
namespace {

constexpr double kPi = 3.14159265358979323846;

double clampd(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }

bool read_file(const std::string& path, std::string& out) {
  std::ifstream f(path, std::ios::binary);
  if (!f) return false;
  std::ostringstream ss;
  ss << f.rdbuf();
  out = ss.str();
  return true;
}

}  // namespace

// ---------------------------------------------------------------- helpers

bool Map::in_poly(const std::vector<double>& ring, double x, double y) {
  const std::size_t n = ring.size() / 2;
  if (n < 3) return false;
  bool inside = false;
  for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
    const double xi = ring[2 * i], yi = ring[2 * i + 1];
    const double xj = ring[2 * j], yj = ring[2 * j + 1];
    if (((yi > y) != (yj > y)) && (x < (xj - xi) * (y - yi) / (yj - yi) + xi)) inside = !inside;
  }
  return inside;
}

bool Map::inside_prism(const Prism& p, double x, double y) const {
  if (!in_poly(p.ring, x, y)) return false;
  for (const auto& hole : p.holes)
    if (in_poly(hole, x, y)) return false;
  return true;
}

const std::vector<int>& Map::cell(int ci, int cj) const {
  static const std::vector<int> kEmpty;
  if (ci < 0 || cj < 0 || ci >= ncx_ || cj >= ncy_) return kEmpty;
  return cells_[static_cast<std::size_t>(cj) * ncx_ + ci];
}

bool Map::in_bounds(double x, double y) const {
  return x >= bx0_ && x <= bx1_ && y >= by0_ && y <= by1_;
}

// ---------------------------------------------------------------- load

bool Map::load(const std::string& dir, const std::vector<std::string>& files) {
  ok_ = false;
  if (dir.empty()) {
    err_ = "no map dir in hello";
    return false;
  }

  std::string world_rel = "v2/world.json", terr_rel = "v2/terrain.i16";
  bool saw_world = false, saw_terr = false;
  for (const auto& f : files) {
    if (f.size() >= 10 && f.compare(f.size() - 10, 10, "world.json") == 0) {
      world_rel = f;
      saw_world = true;
    } else if (f.size() >= 4 && f.compare(f.size() - 4, 4, ".i16") == 0) {
      terr_rel = f;
      saw_terr = true;
    }
  }
  (void)saw_world;
  (void)saw_terr;

  std::string world_txt;
  if (!read_file(dir + "/" + world_rel, world_txt)) {
    err_ = "cannot read " + dir + "/" + world_rel;
    return false;
  }

  json w = json::parse(world_txt, nullptr, false);
  if (w.is_discarded() || !w.is_object()) {
    err_ = "world.json is not valid JSON";
    return false;
  }

  if (auto b = w.find("bounds"); b != w.end() && b->is_object()) {
    bx0_ = (*b).value("minX", 0.0);
    bx1_ = (*b).value("maxX", 0.0);
    by0_ = (*b).value("minY", 0.0);
    by1_ = (*b).value("maxY", 0.0);
  }
  if (auto r = w.find("rules"); r != w.end() && r->is_object()) {
    rule_max_step_ = (*r).value("maxStep", rule_max_step_);
    rule_max_slope_deg_ = (*r).value("maxSlopeDeg", rule_max_slope_deg_);
  }

  const json* t = nullptr;
  if (auto tt = w.find("terrain"); tt != w.end() && tt->is_object()) {
    t = &*tt;
    minx_ = t->value("minX", 0.0);
    miny_ = t->value("minY", 0.0);
    hstep_ = t->value("step", 1.0);
    hscale_ = t->value("scale", 0.01);
    nx_ = t->value("nx", 0);
    ny_ = t->value("ny", 0);
    if (auto fn = t->find("file"); fn != t->end() && fn->is_string()) {
      const std::string f = fn->get<std::string>();
      if (!f.empty() && !saw_terr) terr_rel = "v2/" + f;
    }
  }
  if (t == nullptr || nx_ <= 0 || ny_ <= 0) {
    err_ = "world.json has no usable terrain grid";
    return false;
  }

  std::string terr_bytes;
  if (!read_file(dir + "/" + terr_rel, terr_bytes)) {
    err_ = "cannot read " + dir + "/" + terr_rel;
    return false;
  }
  const std::size_t want = static_cast<std::size_t>(nx_) * ny_ * sizeof(int16_t);
  if (terr_bytes.size() < want) {
    err_ = "terrain.i16 is short (have " + std::to_string(terr_bytes.size()) + ", want " +
           std::to_string(want) + ")";
    return false;
  }
  h_.resize(static_cast<std::size_t>(nx_) * ny_);
  std::memcpy(h_.data(), terr_bytes.data(), want);

  if (auto p = w.find("prisms"); p != w.end() && p->is_array()) {
    prisms_.reserve(p->size());
    for (auto& src : *p) {
      if (!src.is_object()) continue;
      Prism pp;
      pp.base = src.value("base", 0.0);
      pp.top = src.value("top", 0.0);
      if (auto ring = src.find("ring"); ring != src.end() && ring->is_array())
        for (auto& v : *ring)
          if (v.is_number()) pp.ring.push_back(v.get<double>());
      if (pp.ring.size() < 6) continue;  // need a triangle
      if (auto hs = src.find("holes"); hs != src.end() && hs->is_array()) {
        for (auto& hh : *hs) {
          if (!hh.is_array()) continue;
          std::vector<double> flat;
          for (auto& v : hh)
            if (v.is_number()) flat.push_back(v.get<double>());
          if (flat.size() >= 6) pp.holes.push_back(std::move(flat));
        }
      }
      prisms_.push_back(std::move(pp));
    }
  }

  // Uniform grid over prism AABBs so a solidity test touches a handful of prisms.
  ncx_ = std::max(1, static_cast<int>(std::ceil((bx1_ - bx0_) / csz_)) + 1);
  ncy_ = std::max(1, static_cast<int>(std::ceil((by1_ - by0_) / csz_)) + 1);
  cells_.assign(static_cast<std::size_t>(ncx_) * ncy_, {});
  for (std::size_t i = 0; i < prisms_.size(); ++i) {
    double lo_x = 1e18, hi_x = -1e18, lo_y = 1e18, hi_y = -1e18;
    const auto& ring = prisms_[i].ring;
    for (std::size_t k = 0; k + 1 < ring.size(); k += 2) {
      lo_x = std::min(lo_x, ring[k]);
      hi_x = std::max(hi_x, ring[k]);
      lo_y = std::min(lo_y, ring[k + 1]);
      hi_y = std::max(hi_y, ring[k + 1]);
    }
    const int ci0 = std::max(0, static_cast<int>((lo_x - bx0_) / csz_));
    const int ci1 = std::min(ncx_ - 1, static_cast<int>((hi_x - bx0_) / csz_));
    const int cj0 = std::max(0, static_cast<int>((lo_y - by0_) / csz_));
    const int cj1 = std::min(ncy_ - 1, static_cast<int>((hi_y - by0_) / csz_));
    for (int cj = cj0; cj <= cj1; ++cj)
      for (int ci = ci0; ci <= ci1; ++ci)
        cells_[static_cast<std::size_t>(cj) * ncx_ + ci].push_back(static_cast<int>(i));
  }

  ok_ = true;
  return true;
}

// ---------------------------------------------------------------- queries

double Map::terrain_h(double x, double y) const {
  if (h_.empty()) return 0.0;
  const double fx = (x - minx_) / hstep_;
  const double fy = (y - miny_) / hstep_;
  const int i = static_cast<int>(std::floor(fx));
  const int j = static_cast<int>(std::floor(fy));
  const double tx = fx - i, ty = fy - j;
  auto at = [&](int ii, int jj) -> double {
    ii = std::min(std::max(ii, 0), nx_ - 1);
    jj = std::min(std::max(jj, 0), ny_ - 1);
    return h_[static_cast<std::size_t>(jj) * nx_ + ii] * hscale_;
  };
  const double a = at(i, j), b = at(i + 1, j), c = at(i, j + 1), d = at(i + 1, j + 1);
  return (a * (1 - tx) + b * tx) * (1 - ty) + (c * (1 - tx) + d * tx) * ty;
}

double Map::surface_h(double x, double y, double z, double step_up) const {
  double h = terrain_h(x, y);
  const int ci = static_cast<int>((x - bx0_) / csz_);
  const int cj = static_cast<int>((y - by0_) / csz_);
  for (int idx : cell(ci, cj)) {
    const Prism& p = prisms_[static_cast<std::size_t>(idx)];
    if (p.top <= z + step_up && p.top > h && inside_prism(p, x, y)) h = p.top;
  }
  return h;
}

bool Map::prism_blocks(double x, double y, double z_lo, double z_hi) const {
  if (z_hi <= z_lo) return false;
  const int ci = static_cast<int>((x - bx0_) / csz_);
  const int cj = static_cast<int>((y - by0_) / csz_);
  for (int idx : cell(ci, cj)) {
    const Prism& p = prisms_[static_cast<std::size_t>(idx)];
    if (p.base < z_hi && p.top > z_lo && inside_prism(p, x, y)) return true;
  }
  return false;
}

bool Map::solid_point(double x, double y, double z) const {
  if (z < terrain_h(x, y)) return true;  // under the ground
  return prism_blocks(x, y, z - 1e-4, z + 1e-4);
}

bool Map::walkable(double x, double y, double z, double height, double step_up) const {
  if (!in_bounds(x, y)) return false;
  const double s = surface_h(x, y, z, step_up);
  if (s - z > step_up) return false;                       // step too tall to climb
  if (prism_blocks(x, y, z + step_up, z + height)) return false;  // body hits a wall
  return true;
}

double Map::clearance(const Vec3& from, const Vec2& dir, double max_len, double height,
                      double step_up, double max_slope_deg) const {
  const double tan_max = std::tan(clampd(max_slope_deg, 0.0, 89.0) * kPi / 180.0);
  const double ds = 1.0;
  const double z = from[2];
  double prev = surface_h(from[0], from[1], z, step_up);
  double travelled = 0.0;
  while (travelled + ds <= max_len) {
    const double nx = from[0] + dir[0] * (travelled + ds);
    const double ny = from[1] + dir[1] * (travelled + ds);
    if (!walkable(nx, ny, z, height, step_up)) return travelled;
    const double nh = surface_h(nx, ny, z, step_up);
    if (std::fabs(nh - prev) / ds > tan_max) return travelled;
    prev = nh;
    travelled += ds;
  }
  return travelled;
}

bool Map::shot_los(const Vec3& a, const Vec3& b) const {
  const double dx = b[0] - a[0], dy = b[1] - a[1], dz = b[2] - a[2];
  const double len = std::sqrt(dx * dx + dy * dy + dz * dz);
  if (len < 1e-6) return true;
  const int steps = std::max(1, static_cast<int>(len / 2.0));  // ~2 m sampling
  for (int i = 1; i < steps; ++i) {
    const double t = static_cast<double>(i) / steps;
    if (solid_point(a[0] + dx * t, a[1] + dy * t, a[2] + dz * t)) return false;
  }
  return true;
}

double Map::sight_run(double x, double y, double z, double dx, double dy,
                      double cap, double step) const {
  for (double d = step; d <= cap; d += step) {
    const double px = x + dx * d;
    const double py = y + dy * d;
    if (!in_bounds(px, py)) return d;           // the arena edge is a wall
    if (terrain_h(px, py) >= z) return d;       // rising ground closes the line
    if (solid_point(px, py, z)) return d;       // a building closes the line
  }
  return cap;
}

}  // namespace mech
