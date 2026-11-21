// bot.cpp - the policy.
//
// Priorities, in order:
//   1. stay inside the circle the zone is closing to, a few seconds ahead
//   2. never walk into a wall: steering probes the map and picks the clearest
//      direction that still points at the goal
//   3. farm power cubes (crates and NPC drones) while it is safe to do so --
//      cubes are +4% max HP and +4% damage each up to 10, which is what decides
//      the late brawl
//   4. take a fight only when the damage trade is predicted winnable, and then
//      pick the cheapest kill going
//   5. face whatever is shooting at us, so its fire lands in the front arc
//      instead of the 1.5x rear arc
//   6. shoot only with a clear shot, tested against the map
//
// Firing: both weapon slots shoot along the ONE torso aim, so a mortar and a
// direct-fire gun can never be used on the same decision -- one of them would
// always be mis-aimed. So a single active weapon is chosen per decision,
// preferring mortar, then laser, then lightning, then the gun, among those
// actually usable at that range.
//
// Everything reads live numbers from hello.config. The map is loaded once.
#include "bot.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <string>

#include "ballistics.hpp"
#include "map.hpp"

namespace mech {
namespace {

constexpr double kPi = 3.14159265358979323846;

double hypot2(double x, double y) { return std::sqrt(x * x + y * y); }

double dist2(const Vec2& a, const Vec2& b) { return hypot2(a[0] - b[0], a[1] - b[1]); }

Vec2 xy(const Vec3& p) { return {p[0], p[1]}; }

Vec2 norm_or(const Vec2& v, const Vec2& fallback) {
  const double L = hypot2(v[0], v[1]);
  if (L < 1e-9) return fallback;
  return {v[0] / L, v[1] / L};
}

Vec2 rot(const Vec2& v, double ang) {
  const double c = std::cos(ang), s = std::sin(ang);
  return {v[0] * c - v[1] * s, v[0] * s + v[1] * c};
}

double angle_diff(double a, double b) {
  double d = a - b;
  while (d > kPi) d -= 2.0 * kPi;
  while (d < -kPi) d += 2.0 * kPi;
  return std::fabs(d);
}

// Lower rank = better. Unknown upgrades sink to the bottom.
int upgrade_rank(const std::string& id) {
  static const char* kOrder[] = {"armor",  "plating", "servos",  "capacitor", "reactor",
                                 "nanites", "gyros",  "jets",    "sensors"};
  for (int i = 0; i < static_cast<int>(sizeof(kOrder) / sizeof(kOrder[0])); ++i)
    if (id == kOrder[i]) return i;
  return 1000;
}

double weapon_range(const Weapon& w) { return w.range > 1.0 ? w.range : 230.0; }

// One decimal, for the debug label.
std::string f1(double v) {
  char b[32];
  std::snprintf(b, sizeof b, "%.1f", v);
  return b;
}

// hello.config["weapons"][w][k], or `d`.
inline double cfgw(const mf::json& c, const char* w, const char* k, double d) {
  const auto a = c.find("weapons");
  if (a == c.end() || !a->is_object()) return d;
  const auto b = a->find(w);
  if (b == a->end() || !b->is_object()) return d;
  const auto v = b->find(k);
  if (v == b->end() || !v->is_number()) return d;
  return v->get<double>();
}

class MechBot final : public Bot {
 public:
  explicit MechBot(const Hello& hello)
      : pickup_radius_(hello.pickup_radius()),
        min_pitch_(hello.cfg("mech", "min_pitch", -0.7)),
        max_pitch_(hello.cfg("mech", "max_pitch", 1.3)),
        standoff_(hello.cfg("sound", "near", 60.0) + 20.0),
        cube_radius_(hello.cfg("cubes", "pickup_radius", 3.5)),
        crate_drop_(hello.cfg("crates", "height", 2.4) * 0.5),
        dmg_per_cube_(hello.cfg("cubes", "damage_per_cube", 0.04)),
        stat_cap_(hello.cfg("cubes", "stat_cap", 10.0)),
        armor_(hello.cfg("mech", "armor", 0.0)),
        mortar_gravity_(cfgw(hello.config, "mortar", "gravity", 22.0)),
        splash_radius_(cfgw(hello.config, "mortar", "splash_radius", 7.0)),
        lightning_cost_(cfgw(hello.config, "lightning", "energy", 3.0)),
        laser_charge_max_(cfgw(hello.config, "laser", "charge", 0.75)),
        mech_height_(hello.cfg("mech", "height", 9.0)),
        mech_radius_(hello.cfg("mech", "radius", 2.5)),
        rng_(static_cast<std::uint32_t>(hello.seed) | 1u) {
    for (int i = 0; i < 4; ++i) {
      const char* n = weapon_name(static_cast<WeaponKind>(i));
      wc_[i].damage = cfgw(hello.config, n, "damage", 0.0);
      wc_[i].cd = cfgw(hello.config, n, "cooldown", 0.0);
      wc_[i].range = cfgw(hello.config, n, "range", 0.0);
      wc_[i].speed = cfgw(hello.config, n, "speed", 0.0);
      wc_[i].mag = cfgw(hello.config, n, "mag", 0.0);
      wc_[i].reload = cfgw(hello.config, n, "reload", 0.0);
    }
    mortar_min_range_ = splash_radius_ + 6.0;  // closer than this and we eat our own splash

    // The server walks and jets by config.mech, NOT by world.json's rules:
    // protocol.md section 11 says rules.maxStep/maxSlopeDeg are the *viewer's*
    // numbers. They disagree -- rules.maxStep is 1.5 while mech.step_up is 1.2 --
    // and using the map's copy made the planner claim it could walk up 1.4 m
    // ledges that are really walls.
    step_up_ = hello.cfg("mech", "step_up", 1.2);
    max_slope_ = hello.cfg("mech", "max_slope_deg", 50.0);
    // Jet, for the rooftop climb.
    jump_speed_ = hello.cfg("mech", "jump_speed", 11.0);
    gravity_ = hello.cfg("mech", "gravity", 22.0);
    jet_accel_ = hello.cfg("mech", "jet_accel", 34.0);
    jet_drain_ = hello.cfg("mech", "jet_drain", 15.0);
    max_rise_ = hello.cfg("mech", "max_rise_speed", 10.0);
    max_energy_ = hello.cfg("mech", "energy", 100.0);

    have_map_ = map_.load(hello.map_dir, hello.map_files);
  }

  std::string name() const override { return "apex"; }

  Action act(const Obs& o) override {
    Action a;
    const Self& me = o.self;
    const Vec2 me_xy = xy(me.pos);
    const double dt = (last_t_ >= 0.0) ? std::max(0.0, o.t - last_t_) : 0.0;
    last_t_ = o.t;

    mech_height_ = me.stat("height", mech_height_);
    mech_radius_ = me.stat("radius", mech_radius_);
    cur_max_energy_ = me.max_energy > 0.0 ? me.max_energy : max_energy_;

    take_upgrade(a, o);
    take_loot(a, o, me_xy);

    // Remember who last hit us and from where: that is what we face when idle.
    for (const auto& e : o.events)
      if (e.type == "damaged" && e.has_bearing) {
        threat_bearing_ = e.bearing;
        threat_age_ = 0.0;
      }
    threat_age_ += dt;

    const Circle zone = predicted_zone(o);
    const double margin = mech_radius_ + 6.0;
    const bool settled = dist2(me_xy, zone.center) <= zone.radius - margin;

    // Cheapest kill first (low HP costs least to finish). `engage` -- whether the
    // trade is predicted to go our way -- decides ONLY whether we close on it.
    const Actor* enemy = pick_enemy(o, me);
    const bool engage = enemy != nullptr && winnable(o, me, *enemy);

    const Farm farm = (!engage && me.cubes < kCubeTarget) ? pick_farm(o, zone, margin) : Farm{};

    steer(a, o, me, me_xy, zone, settled, engage ? enemy : nullptr, farm);
    // Fire at the enemy whether or not we are closing on it. Refusing to shoot
    // while being shot at is strictly dominated: the gun costs no energy to fire,
    // the squad shares vision so we are already located, and every kill is a place
    // point. tools/los_audit.py measured the old wiring withholding fire on 87 of
    // 112 geometrically open in-range shots, which is how a bot that armed itself
    // sensibly still died first. The line-of-sight gate inside shoot() still
    // stops us wasting rounds into walls.
    shoot(a, me, enemy, farm);
    // Sprint while someone actually has a shot line on us and the trade is not
    // winnable. Boost is 19.5 m/s against a hunter's 11.7, so a few seconds of
    // it opens ~47 m they must re-close while buildings keep breaking their
    // line -- and it costs no aim and no shots. In the air boost *is* the jet,
    // so only ever sprint grounded.
    if (!engage && enemy && have_map_ && me.grounded &&
        map_.shot_los({enemy->pos[0], enemy->pos[1], enemy->pos[2] + 7.0},  // muzzle_height
                      {me.pos[0], me.pos[1], me.pos[2] + 4.5})) {
      a.boost = true;
    }
    draw(a, o, me, zone, enemy, farm);
    return a;
  }

 private:
  struct Circle {
    Vec2 center{};
    double radius = 0.0;
  };

  struct WeaponCfg {
    double damage = 0, cd = 0, range = 0, speed = 0, mag = 0, reload = 0;
  };

  struct Farm {
    enum class Kind { None, Crate, Drone };
    Kind kind = Kind::None;
    const Actor* drone = nullptr;
    const Item* crate = nullptr;
    Vec3 point{};
    double dist = 1e18;
  };

  struct Shot {
    bool valid = false;
    int slot = -1;
    bool fire = false;
    Vec2 aim{};
    double dist = 1e18;
  };

  static constexpr double kCubeTarget = 10.0;   // cubes.stat_cap: HP/damage stop here
  static constexpr double kFarmRadius = 110.0;  // how far to detour for cubes
  static constexpr double kWinMargin = 1.35;    // demand to win by this much to engage
  static constexpr double kAligned = 0.14;      // rad: "the torso is on the target"
  static constexpr double kThreatRange = 300.0; // beyond this nobody can be shooting us
  static constexpr double kClimbReserve = 0.18;   // fraction of a tank kept back from a climb
  static constexpr double kEnergyReserve = 12.0;  // energy always kept back for a fight
  static constexpr double kClimbSafeRange = 90.0; // never climb with anyone this close
  static constexpr double kClimbStartRange = 12.0; // start a climb within this of the goal

  // ---------------------------------------------------------------- geometry

  static int wi(WeaponKind k) { return static_cast<int>(k); }

  static int weapon_pref(WeaponKind k) {
    switch (k) {
      case WeaponKind::Mortar: return 0;
      case WeaponKind::Laser: return 1;
      case WeaponKind::Lightning: return 2;
      case WeaponKind::Gun: return 3;
    }
    return 3;
  }

  static Vec3 muzzle(const Self& me) {
    return {me.pos[0], me.pos[1], me.pos[2] + me.muzzle_height()};
  }

  static Vec3 centre(const Actor& a) {
    const double scale = 1.0 + 0.05 * std::max(0.0, a.cubes);
    return {a.pos[0], a.pos[1], a.pos[2] + 4.5 * scale};
  }

  static double dist3(const Vec3& a, const Vec3& b) {
    const double dx = a[0] - b[0], dy = a[1] - b[1], dz = a[2] - b[2];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  std::uint32_t next_rand() {
    rng_ = rng_ * 1664525u + 1013904223u;
    return rng_;
  }

  Circle predicted_zone(const Obs& o) const {
    const Zone& z = o.zone;
    double u = 0.0;
    const double span = z.shrink_end - z.shrink_start;
    if (span > 1e-6) u = std::clamp((o.t + lookahead_ - z.shrink_start) / span, 0.0, 1.0);
    Circle c;
    c.center = {z.center[0] + (z.next_center[0] - z.center[0]) * u,
                z.center[1] + (z.next_center[1] - z.center[1]) * u};
    c.radius = z.radius + (z.next_radius - z.radius) * u;
    return c;
  }

  // ---------------------------------------------------------------- damage model

  double cube_mult(double cubes) const {
    return 1.0 + dmg_per_cube_ * std::min(std::max(0.0, cubes), stat_cap_);
  }

  // Sustained damage per second. Taking raw damage/cooldown flatters both the
  // gun (reload downtime) and the laser (charge time), so amortise each.
  static double sustained_dps(double damage, double cd, double extra) {
    if (damage <= 0.0 || cd <= 0.0) return 0.0;
    return damage / (cd + extra);
  }

  double our_dps(const Self& me) const {
    const double mult = cube_mult(me.cubes);
    double dps = 0.0;
    for (int i = 0; i < 2; ++i) {
      if (!me.has_weapon[i]) continue;
      const Weapon& w = me.weapons[i];
      double extra = 0.0;
      if (w.kind == WeaponKind::Gun && w.mag > 0.0 && w.reload_s > 0.0)
        extra = w.reload_s * w.cd / w.mag;  // reload amortised across the magazine
      else if (w.kind == WeaponKind::Laser)
        extra = laser_charge_max_;
      dps += sustained_dps(w.damage, w.cd, extra) * mult;
    }
    return dps;
  }

  // Their loadout is visible only as weapon kinds, so use stock config numbers
  // scaled by their cubes. Unknown armour is assumed to be none.
  double enemy_dps(const Actor& a) const {
    const double mult = cube_mult(a.cubes);
    double dps = 0.0;
    for (WeaponKind k : a.weapons) {
      const WeaponCfg& c = wc_[wi(k)];
      double extra = 0.0;
      if (k == WeaponKind::Gun && c.mag > 0.0 && c.reload > 0.0)
        extra = c.reload * c.cd / c.mag;
      else if (k == WeaponKind::Laser)
        extra = laser_charge_max_;
      dps += sustained_dps(c.damage, c.cd, extra) * mult;
    }
    if (dps <= 0.0) {
      const WeaponCfg& g = wc_[wi(WeaponKind::Gun)];
      const double extra = (g.mag > 0.0 && g.cd > 0.0) ? g.reload * g.cd / g.mag : 0.0;
      dps = sustained_dps(g.damage, g.cd, extra) * mult;
    }
    return dps;
  }

  // ---------------------------------------------------------------- decisions

  void take_upgrade(Action& a, const Obs& o) const {
    if (!o.offer || o.offer->choices.empty()) return;
    int best = 0, best_rank = 1 << 30;
    for (int i = 0; i < static_cast<int>(o.offer->choices.size()); ++i) {
      const int r = upgrade_rank(o.offer->choices[i].id);
      if (r < best_rank) {
        best_rank = r;
        best = i;
      }
    }
    a.pick = best;
  }

  // Best weapon pickup within `radius`, by the mortar > laser > lightning
  // preference and then by distance. Only worth taking if it beats what slot 1
  // already holds; holding the same kind twice is illegal anyway.
  const Pickup* best_pickup(const Obs& o, const Vec2& me_xy, double radius, const Circle* zone,
                            double margin) const {
    int held = 999;
    if (o.self.has_weapon[1]) held = weapon_pref(o.self.weapons[1].kind);
    const Pickup* best = nullptr;
    int best_pref = 999;
    double best_d = 1e18;
    for (const auto& p : o.pickups) {
      if (p.kind == WeaponKind::Gun) continue;  // never swap the gun out
      const int pr = weapon_pref(p.kind);
      if (pr >= held) continue;                 // no upgrade over what we hold
      const double d = dist2(xy(p.pos), me_xy);
      if (d > radius) continue;
      if (zone && dist2(xy(p.pos), zone->center) > zone->radius - margin) continue;
      if (pr < best_pref || (pr == best_pref && d < best_d)) {
        best_pref = pr;
        best_d = d;
        best = &p;
      }
    }
    return best;
  }

  void take_loot(Action& a, const Obs& o, const Vec2& me_xy) const {
    const Pickup* p = best_pickup(o, me_xy, pickup_radius_, nullptr, 0.0);
    if (!p) return;
    a.take_id = p->id;
    // Slot 1 is occupied, so this is a swap: name the slot being replaced.
    if (o.self.has_weapon[1]) a.drop_slot = 1;
  }

  // Least expensive kill that is actually in range: estimated time for us to
  // finish it, with a mild penalty for distance.
  const Actor* pick_enemy(const Obs& o, const Self& me) const {
    const double dps = our_dps(me);
    if (dps <= 0.0) return nullptr;
    double reach = 0.0;
    for (int i = 0; i < 2; ++i)
      if (me.has_weapon[i]) reach = std::max(reach, weapon_range(me.weapons[i]));
    const Vec3 m = muzzle(me);
    const Actor* best = nullptr;
    double best_score = 1e18;
    for (const auto& a : o.mechs) {
      if (a.npc) continue;
      const double d = dist3(m, centre(a));
      if (d > reach) continue;
      const double hp = a.hp > 0.0 ? a.hp : (a.max_hp > 0.0 ? a.max_hp : 1400.0);
      const double score = (hp / dps) * (1.0 + d / 600.0);
      if (score < best_score) {
        best_score = score;
        best = &a;
      }
    }
    return best;
  }

  // Do we finish them before they -- and everyone else who can see us -- finish
  // us? Every visible mech in range counts as incoming, because the squad
  // shares vision and will converge.
  bool winnable(const Obs& o, const Self& me, const Actor& target) const {
    const double dps = our_dps(me);
    if (dps <= 0.0) return false;
    const double their_hp = target.hp > 0.0 ? target.hp : 1400.0;
    const double ttk_us = their_hp / dps;

    const Vec3 m = muzzle(me);
    double incoming = 0.0;
    for (const auto& a : o.mechs) {
      if (a.npc) continue;
      if (dist3(m, centre(a)) > kThreatRange) continue;
      incoming += enemy_dps(a);
    }
    incoming *= (1.0 - std::clamp(armor_, 0.0, 0.9));
    const double ttk_them = me.hp / std::max(1e-6, incoming);
    return ttk_us * kWinMargin < ttk_them;
  }

  Farm pick_farm(const Obs& o, const Circle& zone, double margin) const {
    const bool want_weapon = !o.self.has_weapon[1];
    const Vec3 m = muzzle(o.self);
    Farm best;
    double best_score = 1e18;

    auto consider = [&](Farm::Kind kind, const Actor* drone, const Item* crate, const Vec3& aim,
                        double value) {
      if (!affordable_climb(aim)) return;
      if (dist2(xy(aim), zone.center) > zone.radius - margin) return;
      const double d = dist3(m, aim);
      if (d > kFarmRadius) return;
      const double score = d / value;
      if (score < best_score) {
        best_score = score;
        best.kind = kind;
        best.drone = drone;
        best.crate = crate;
        best.point = aim;
        best.dist = d;
      }
    };

    for (const auto& c : o.crates)
      consider(Farm::Kind::Crate, nullptr, &c, {c.pos[0], c.pos[1], c.pos[2] + crate_drop_},
               want_weapon ? 1.6 : 1.0);
    for (const auto& a : o.mechs)
      if (a.npc) consider(Farm::Kind::Drone, &a, nullptr, centre(a), want_weapon ? 2.4 : 2.0);
    return best;
  }

  // ---------------------------------------------------------------- jet / climb

  // Seconds of jet to rise `h` metres: first `t_acc` accelerating to max_rise at
  // (jet_accel - gravity), then flat out.
  double jet_seconds_for(double h) const {
    const double a = std::max(1.0, jet_accel_ - gravity_);
    const double t_acc = max_rise_ / a;
    const double h_acc = 0.5 * a * t_acc * t_acc;
    if (h <= h_acc) return std::sqrt(std::max(0.0, 2.0 * h / a));
    return t_acc + (h - h_acc) / max_rise_;
  }

  double energy_for_climb(double h) const { return jet_seconds_for(h) * jet_drain_; }

  // Height one tank buys once kClimbReserve is kept back: the inverse of
  // jet_seconds_for. On this map that is ~50 m, which covers all but the towers
  // (roofs need a median 15 m climb / 29 energy).
  double climb_budget_m() const {
    const double t = (cur_max_energy_ * (1.0 - kClimbReserve)) / std::max(0.1, jet_drain_);
    const double a = std::max(1.0, jet_accel_ - gravity_);
    const double t_acc = max_rise_ / a;
    const double h_acc = 0.5 * a * t_acc * t_acc;
    if (t <= t_acc) return 0.5 * a * t * t;
    return h_acc + max_rise_ * (t - t_acc);
  }

  // May we get up to `pos`? Walkable if it is at ground level, otherwise the jet
  // has to reach it. A local height test, not a route search.
  bool affordable_climb(const Vec3& pos) const {
    if (!have_map_) return true;
    const double need = pos[2] - map_.terrain_h(pos[0], pos[1]);
    if (need <= 3.0) return true;  // at ground level, just walk
    return need <= climb_budget_m();
  }

  // Commit to a climb only if the tank covers it *now*, plus a reserve, and
  // nothing is shooting at us: rising at 10 m/s with air_accel 12 is a sitting
  // duck, and running dry halfway up just drops us.
  bool climb_allowed(const Obs& o, const Self& me, double need) const {
    if (me.energy < energy_for_climb(need) + kEnergyReserve) return false;
    const Vec3 m = muzzle(me);
    for (const auto& e : o.mechs)
      if (dist3(m, centre(e)) <= kClimbSafeRange) return false;
    return true;
  }

  void climb(Action& a, const Obs& o, const Self& me, const Vec2& me_xy, const Vec2& goal_xy,
             double need) {
    if (!climb_allowed(o, me, need)) {
      a.move = {0.0, 0.0};  // hold still and regenerate rather than waste the tank
      return;
    }
    const double goal_z = me.pos[2] + need;  // `need` is the remaining rise, not a height
    // Press STRAIGHT at the goal, with no avoid() probing. While climbing we
    // want to be shoved into the wall, not steered around it: wall sliding keeps
    // us alongside it and we rise cleanly, whereas avoid() -- which seeks the
    // clearest heading -- makes the mech orbit the building and burn the tank in
    // open air (found by tools/sim.py: x oscillated -98 <-> -107 while rising).
    a.move = norm_or({goal_xy[0] - me_xy[0], goal_xy[1] - me_xy[1]}, {1.0, 0.0});
    a.jump = me.grounded;   // leave the ground; jumping needs it
    // Jet only while short of the roofline. Holding the jet past it sails us over
    // the roof and we never come down: tools/sim.py showed the mech reach the
    // crate's x,y at z=14.8 with the roofline at 14.9, then climb to 17.6, drift
    // off the side and oscillate for the rest of the match. Cutting at roofline +
    // 0.5 is the clearance needed to cross the lip, and step_up (1.2) then walks
    // us on as soon as our centre is over it.
    a.boost = !me.grounded && me.pos[2] < goal_z + 0.5;
    climb_until_ = o.t + 0.5;  // remember we are mid-climb, for the draw label
  }

  // ---------------------------------------------------------------- steering

  // Probe headings around the desired one and take the clearest that still
  // points roughly at it. Cheap local avoidance; the server's sliding covers
  // the rest.
  Vec2 avoid(const Vec3& from, const Vec2& desired, double probe) const {
    if (!have_map_) return desired;
    static const double kAngles[] = {0.0,  0.35,  -0.35, 0.70, -0.70, 1.05, -1.05,
                                     1.40, -1.40, 1.75,  -1.75, 2.35, -2.35, kPi};
    Vec2 best = desired;
    double best_score = -1e18;
    for (double ang : kAngles) {
      const Vec2 d = rot(desired, ang);
      const double clr = map_.clearance(from, d, probe, mech_height_, step_up_, max_slope_);
      if (clr < 1.5) continue;  // immediately blocked: not a real option
      const double score = std::min(clr, probe) - 5.0 * std::fabs(ang);
      if (score > best_score) {
        best_score = score;
        best = d;
      }
    }
    return best;
  }

  void steer(Action& a, const Obs& o, const Self& me, const Vec2& me_xy, const Circle& zone,
             bool settled, const Actor* engage, const Farm& farm) {
    if (!settled) {
      // Priority 1: get inside the circle the zone is closing to.
      a.move = avoid(me.pos, norm_or({zone.center[0] - me_xy[0], zone.center[1] - me_xy[1]},
                                     {1.0, 0.0}),
                     12.0);
      a.boost = !me.in_zone && dist2(me_xy, zone.center) > 25.0;
      return;
    }

    // An empty slot 1 is the biggest single upgrade on the board: a second
    // weapon nearly doubles return-fire damage (gun 156 -> gun+laser 287 dps,
    // the same as a hunter), and every kill is a place point plus the victim's
    // cubes. So while unarmed, search far and take the weapon before free
    // cubes -- +4% stats is not worth a fraction of +84% damage. Armed, a
    // swap is only worth a short opportunistic detour.
    if (const Pickup* loot = best_pickup(o, me_xy, me.has_weapon[1] ? 60.0 : 180.0,
                                         &zone, mech_radius_ + 6.0)) {
      a.move = avoid(me.pos, norm_or({loot->pos[0] - me_xy[0], loot->pos[1] - me_xy[1]},
                                     {1.0, 0.0}),
                     12.0);
      return;
    }

    // A cube already on the ground is free: it only needs walking over.
    if (const Item* cube = nearest_cube(o, me_xy, zone)) {
      a.move = avoid(me.pos, norm_or({cube->pos[0] - me_xy[0], cube->pos[1] - me_xy[1]},
                                     {1.0, 0.0}),
                     12.0);
      return;
    }

    if (engage) {
      // Hold station at standoff range and slide sideways: a moving target is
      // harder to lead.
      if (o.t >= strafe_until_) {
        strafe_sign_ = (next_rand() & 1u) ? 1.0 : -1.0;
        strafe_until_ = o.t + 2.0 + 2.0 * ((next_rand() >> 8) * (1.0 / 16777216.0));
      }
      const Vec2 to{engage->pos[0] - me_xy[0], engage->pos[1] - me_xy[1]};
      const Vec2 u = norm_or(to, {1.0, 0.0});
      const double L = hypot2(to[0], to[1]);
      const double radial = L > standoff_ + 12.0 ? 1.0 : (L < standoff_ - 12.0 ? -1.0 : 0.0);
      a.move = avoid(me.pos,
                     norm_or({u[0] * radial - u[1] * strafe_sign_ * 0.9,
                              u[1] * radial + u[0] * strafe_sign_ * 0.9},
                             u),
                     12.0);
      return;
    }

    if (farm.kind != Farm::Kind::None) {
      const Vec2 goal_xy{farm.point[0], farm.point[1]};
      const Vec2 to_goal = norm_or({goal_xy[0] - me_xy[0], goal_xy[1] - me_xy[1]}, {1.0, 0.0});
      const double need = farm.point[2] - me.pos[2];
      // A target above us that we cannot walk to is a roof. The trigger is
      // "blocked, and the goal is up there" rather than a distance test, so it
      // also works when the crate sits deep inside a big roof: press into the
      // wall, get stopped by it, then jump and jet.
      const double ahead = have_map_ ? map_.clearance(me.pos, to_goal, 6.0, mech_height_, step_up_,
                                                      max_slope_)
                                     : 6.0;
      // Keep climbing until the goal is *reached*, not merely until it is within
      // one step_up. The old `need > step_up_ + 0.2` handed control back to this
      // farm branch while we were still 1.4 m under the roofline, and that branch
      // steers with avoid() -- which swung the mech back off the edge and dropped
      // it (tools/sim.py: z climbed to 16.8 then fell to 6.7, over and over).
      // While airborne we also hold the climb for a metre *past* the goal, so the
      // push that carries us over the lip survives through to the landing.
      const bool climbing = need > 0.2 || (!me.grounded && need > -1.0);
      if (climbing && (ahead < 3.0 || dist2(goal_xy, me_xy) <= kClimbStartRange)) {
        climb(a, o, me, me_xy, goal_xy, need);
        return;
      }
      a.move = avoid(me.pos, to_goal, 12.0);
      return;
    }

    a.move = avoid(me.pos,
                   norm_or({zone.center[0] - me_xy[0], zone.center[1] - me_xy[1]}, {1.0, 0.0}),
                   12.0);
  }

  const Item* nearest_cube(const Obs& o, const Vec2& me_xy, const Circle& zone) const {
    const Item* best = nullptr;
    double best_d = 45.0;
    for (const auto& c : o.cubes) {
      if (dist2(xy(c.pos), zone.center) > zone.radius) continue;
      const double d = dist2(xy(c.pos), me_xy);
      if (d <= cube_radius_) continue;  // already in pickup range; the server collects it
      if (d < best_d) {
        best_d = d;
        best = &c;
      }
    }
    return best;
  }

  // ---------------------------------------------------------------- shooting

  static Vec2 aim_at(const Vec3& m, const Vec3& p) {
    const double hd = hypot2(p[0] - m[0], p[1] - m[1]);
    return {std::atan2(p[1] - m[1], p[0] - m[0]), std::atan2(p[2] - m[2], hd)};
  }

  // Mortar: solve the launch pitch, iterating on time-of-flight so the shell
  // lands where the target will be. Prefers the flat arc, but lobs over the top
  // when a building is in the way -- which is what a mortar is for.
  bool mortar_aim(const Vec3& m, const Vec3& pt, const Vec3& vel, bool leadable, Vec2& aim) const {
    const WeaponCfg& mc = wc_[wi(WeaponKind::Mortar)];
    const double v = mc.speed > 1.0 ? mc.speed : 72.0;
    const double g = mortar_gravity_;
    Vec3 t = pt;
    double pitch = 0.0;
    for (int it = 0; it < 3; ++it) {
      const double dx = t[0] - m[0], dy = t[1] - m[1], dz = t[2] - m[2];
      const double d = hypot2(dx, dy);
      if (d < 1.0) return false;
      double flat = 0.0, lob = 0.0;
      const bool has_flat =
          solve_pitch(d, dz, v, g, false, flat) && flat >= min_pitch_ && flat <= max_pitch_;
      const bool has_lob =
          solve_pitch(d, dz, v, g, true, lob) && lob >= min_pitch_ && lob <= max_pitch_;
      const bool straight = !have_map_ || map_.shot_los(m, t);
      if (has_flat && straight) {
        pitch = flat;
      } else if (has_lob) {
        pitch = lob;  // over the top
      } else if (has_flat) {
        pitch = flat;
      } else {
        return false;
      }
      if (!leadable) break;
      const double tof = d / (v * std::cos(pitch));
      t = {pt[0] + vel[0] * tof, pt[1] + vel[1] * tof, pt[2] + vel[2] * tof};
    }
    aim = {std::atan2(t[1] - m[1], t[0] - m[0]), pitch};
    return true;
  }

  // Hold to charge, let go to fire. Releasing fires the beam wherever we happen
  // to be pointing, so once charging we are committed: hold until the torso is
  // on the target and the charge is nearly full.
  bool laser_trigger(const Weapon& w, const Self& me, bool aligned) const {
    (void)me;
    if (w.charge <= 0.0) return aligned;                     // start only when on target
    if (w.charge >= laser_charge_max_ * 0.90) return false;  // let go: near-full beam
    return true;                                             // already committed; keep charging
  }

  Shot plan_shot(const Self& me, const Vec3& pt, const Vec3& vel, bool leadable,
                 bool allow_fire) const {
    Shot s;
    const Vec3 m = muzzle(me);
    for (int slot = 0; slot < 2; ++slot) {
      if (!me.has_weapon[slot]) continue;
      const Weapon& w = me.weapons[slot];
      const double d = dist3(m, pt);
      const bool los = !have_map_ || map_.shot_los(m, pt);
      Vec2 aim{};
      bool fire = false;

      switch (w.kind) {
        case WeaponKind::Gun: {
          if (d > weapon_range(w)) continue;
          const double sp = w.speed > 1.0 ? w.speed : 252.0;
          const double tof = leadable ? d / sp : 0.0;
          aim = aim_at(m, Vec3{pt[0] + vel[0] * tof, pt[1] + vel[1] * tof, pt[2] + vel[2] * tof});
          fire = los;
          break;
        }
        case WeaponKind::Lightning: {
          if (d > weapon_range(w) || me.energy < lightning_cost_) continue;
          aim = aim_at(m, pt);  // instant along the aim: no lead
          fire = los;
          break;
        }
        case WeaponKind::Laser: {
          if (d > weapon_range(w)) continue;
          aim = aim_at(m, pt);  // hitscan: no lead
          fire = laser_trigger(w, me, los && angle_diff(me.aim[0], aim[0]) < kAligned);
          break;
        }
        case WeaponKind::Mortar: {
          if (d < mortar_min_range_) continue;  // this close, its own splash hits us
          if (!mortar_aim(m, pt, vel, leadable, aim)) continue;
          fire = true;
          break;
        }
      }
      fire = fire && allow_fire;
      if (!s.valid || weapon_pref(w.kind) < weapon_pref(me.weapons[s.slot].kind)) {
        s.valid = true;
        s.slot = slot;
        s.fire = fire;
        s.aim = aim;
        s.dist = d;
      }
    }
    return s;
  }

  void shoot(Action& a, const Self& me, const Actor* enemy, const Farm& farm) const {
    Vec3 pt{};
    Vec3 vel{0.0, 0.0, 0.0};
    bool leadable = false;
    bool allow_fire = false;

    if (enemy) {
      pt = centre(*enemy);
      vel = enemy->vel;
      leadable = true;
      allow_fire = true;
    } else if (farm.kind != Farm::Kind::None) {
      pt = farm.point;
      if (farm.drone) {
        vel = farm.drone->vel;
        leadable = true;
      }
      allow_fire = true;
    } else {
      a.aim = (threat_age_ < 4.0) ? Vec2{threat_bearing_, me.aim[1]} : me.aim;
      return;
    }

    const Shot s = plan_shot(me, pt, vel, leadable, allow_fire);
    if (!s.valid) {
      // Nothing can be brought to bear (out of range, or no line). Still point
      // the torso at it, so incoming fire meets the front arc rather than the back.
      Vec2 aim = aim_at(muzzle(me), pt);
      aim[1] = std::clamp(aim[1], min_pitch_, max_pitch_);
      a.aim = aim;
      return;
    }
    a.aim = s.aim;
    a.fire[s.slot] = s.fire;
    const Weapon& w = me.weapons[s.slot];
    if (w.kind == WeaponKind::Gun && w.ammo <= 0.0 && w.reload <= 0.0) a.reload = true;
  }

  // ---------------------------------------------------------------- drawing

  void draw(Action& a, const Obs& o, const Self& me, const Circle& zone, const Actor* enemy,
            const Farm& farm) const {
    for (int i = 0; i < 12; ++i) {
      const double th = i * (2.0 * kPi / 12.0);
      a.draw.point({zone.center[0] + std::cos(th) * zone.radius,
                    zone.center[1] + std::sin(th) * zone.radius, me.pos[2]},
                   "#40c0ff", 4.0);
    }
    if (enemy) {
      a.draw.line(muzzle(me), centre(*enemy), "#ff5040");
      a.draw.text("E" + std::to_string(enemy->id), centre(*enemy), "#ffd060");
    }
    if (farm.kind != Farm::Kind::None) {
      a.draw.line(muzzle(me), farm.point, "#40ff90");
      a.draw.point(farm.point, "#40ff90", 8.0);
    }
    const char* mode = farm.kind == Farm::Kind::Drone ? "drone"
                       : farm.kind == Farm::Kind::Crate ? "crate"
                                                        : "";
    const char* held = me.has_weapon[1] ? weapon_name(me.weapons[1].kind) : "-";
    a.draw.label("hp " + std::to_string(static_cast<int>(me.hp)) + " c" +
                 std::to_string(static_cast<int>(me.cubes)) + "/" +
                 std::to_string(static_cast<int>(kCubeTarget)) + " z" +
                 std::to_string(static_cast<int>(zone.radius)) + " a" +
                 std::to_string(o.alive) + " " + held + " " + mode +
                 (have_map_ ? "" : " NOMAP") + " su" + f1(step_up_) +
                 (o.t < climb_until_ ? " CLIMB" : ""));
  }

  Map map_;
  bool have_map_ = false;

  WeaponCfg wc_[4];

  double pickup_radius_;
  double min_pitch_;
  double max_pitch_;
  double standoff_;
  double cube_radius_;
  double crate_drop_;
  double dmg_per_cube_;
  double stat_cap_;
  double armor_;
  double mortar_gravity_;
  double splash_radius_;
  double mortar_min_range_ = 13.0;
  double lightning_cost_;
  double laser_charge_max_;
  double mech_height_ = 9.0;
  double mech_radius_ = 2.5;
  double step_up_ = 1.2;  // config.mech.step_up, NOT world.json rules.maxStep (1.5)
  double max_slope_ = 50.0;
  double lookahead_ = 4.0;
  // Jet, for the rooftop climb.
  double jump_speed_ = 11.0;
  double gravity_ = 22.0;
  double jet_accel_ = 34.0;
  double jet_drain_ = 15.0;
  double max_rise_ = 10.0;
  double max_energy_ = 100.0;
  double cur_max_energy_ = 100.0;  // live me.max_energy: upgrades raise the climb ceiling
  double climb_until_ = -1.0;  // o.t until which we consider ourselves mid-climb

  double last_t_ = -1.0;
  double threat_bearing_ = 0.0;
  double threat_age_ = 1e9;
  double strafe_sign_ = 1.0;
  double strafe_until_ = 0.0;
  std::uint32_t rng_ = 1;
};

}  // namespace

std::unique_ptr<Bot> make_bot(const Hello& hello) { return std::make_unique<MechBot>(hello); }

}  // namespace mech
