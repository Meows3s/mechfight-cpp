// bot.cpp - the policy.
//
// M2 priorities, in order:
//   1. stay inside the circle the zone is closing to, a few seconds ahead
//   2. never walk into a wall: steering probes the map and picks the clearest
//      direction that still points at the goal
//   3. face whatever is shooting at us, so its fire lands in the front arc
//      instead of the 1.5x rear arc
//   4. only shoot when the map says the shot is not going into a building
//
// Everything reads live numbers from hello.config. The map is loaded once.
#include "bot.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>

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

// Lower rank = better. Unknown upgrades sink to the bottom.
int upgrade_rank(const std::string& id) {
  static const char* kOrder[] = {"armor",  "plating", "servos",  "capacitor", "reactor",
                                 "nanites", "gyros",  "jets",    "sensors"};
  for (int i = 0; i < static_cast<int>(sizeof(kOrder) / sizeof(kOrder[0])); ++i)
    if (id == kOrder[i]) return i;
  return 1000;
}

double weapon_range(const Weapon& w) { return w.range > 1.0 ? w.range : 230.0; }

class MechBot final : public Bot {
 public:
  explicit MechBot(const Hello& hello)
      : pickup_radius_(hello.pickup_radius()),
        min_pitch_(hello.cfg("mech", "min_pitch", -0.7)),
        max_pitch_(hello.cfg("mech", "max_pitch", 1.3)),
        standoff_(hello.cfg("sound", "near", 60.0) + 20.0),
        rng_(static_cast<std::uint32_t>(hello.seed) | 1u) {
    have_map_ = map_.load(hello.map_dir, hello.map_files);
    if (have_map_) {
      step_up_ = map_.rule_max_step();
      max_slope_ = map_.rule_max_slope_deg();
    }
  }

  std::string name() const override { return "apex"; }

  Action act(const Obs& o) override {
    Action a;
    const Self& me = o.self;
    const Vec2 me_xy = xy(me.pos);
    const double dt = (last_t_ >= 0.0) ? std::max(0.0, o.t - last_t_) : 0.0;
    last_t_ = o.t;

    mech_height_ = me.stat("height", 9.0);
    mech_radius_ = me.stat("radius", 2.5);

    take_upgrade(a, o);
    take_loot(a, o, me_xy);

    // Remember who last hit us and from where: that is what we face when idle.
    for (const auto& e : o.events)
      if (e.type == "damaged" && e.has_bearing) {
        threat_bearing_ = e.bearing;
        threat_age_ = 0.0;
      }
    threat_age_ += dt;

    const Actor* target = pick_target(o, me_xy);
    const double dist = target ? dist3(muzzle(me), centre(*target)) : 0.0;
    const bool los = target && (!have_map_ || map_.shot_los(muzzle(me), centre(*target)));

    steer(a, o, me, me_xy, target);
    aim_and_fire(a, me, target, dist, los);
    draw(a, o, me, target);
    return a;
  }

 private:
  struct Circle {
    Vec2 center{};
    double radius = 0.0;
  };

  // ---------------------------------------------------------------- geometry

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

  // Where the zone circle will be `lookahead_` seconds from now. The circle
  // moves linearly from the current one to the next between shrink_start and
  // shrink_end, so this is exact rather than a guess.
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

  void take_loot(Action& a, const Obs& o, const Vec2& me_xy) const {
    if (o.self.has_weapon[1]) return;
    int want = -1;
    double best = pickup_radius_;
    for (const auto& p : o.pickups) {
      if (p.kind == WeaponKind::Gun) continue;  // never swap the gun out
      const double d = dist2(xy(p.pos), me_xy);
      if (d <= best) {
        best = d;
        want = p.id;
      }
    }
    if (want >= 0) a.take_id = want;
  }

  const Actor* pick_target(const Obs& o, const Vec2& me_xy) const {
    const Actor* best = nullptr;
    double best_d = 1e18;
    for (const auto& m : o.mechs) {
      if (m.npc) continue;
      const double d = dist2(xy(m.pos), me_xy);
      if (d < best_d) {
        best_d = d;
        best = &m;
      }
    }
    if (best) return best;
    for (const auto& m : o.mechs) {
      const double d = dist2(xy(m.pos), me_xy);
      if (d < best_d) {
        best_d = d;
        best = &m;
      }
    }
    return best;
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

  void steer(Action& a, const Obs& o, const Self& me, const Vec2& me_xy, const Actor* target) {
    const Circle zone = predicted_zone(o);
    const double to_centre = dist2(me_xy, zone.center);
    const double margin = mech_radius_ + 6.0;
    const bool settled = to_centre <= zone.radius - margin;

    Vec2 desired{};
    if (!settled) {
      // Priority 1: get inside the circle the zone is closing to.
      desired = norm_or({zone.center[0] - me_xy[0], zone.center[1] - me_xy[1]}, {1.0, 0.0});
      a.boost = !me.in_zone && to_centre > 25.0;
    } else if (const Pickup* loot = want_loot(o, zone, me_xy, margin)) {
      desired = norm_or({loot->pos[0] - me_xy[0], loot->pos[1] - me_xy[1]}, {1.0, 0.0});
    } else if (target) {
      // Hold station at standoff range and keep sliding sideways: a moving
      // target is harder to lead.
      if (o.t >= strafe_until_) {
        strafe_sign_ = (next_rand() & 1u) ? 1.0 : -1.0;
        strafe_until_ = o.t + 2.0 + 2.0 * ((next_rand() >> 8) * (1.0 / 16777216.0));
      }
      const Vec2 to{target->pos[0] - me_xy[0], target->pos[1] - me_xy[1]};
      const Vec2 u = norm_or(to, {1.0, 0.0});
      const double L = hypot2(to[0], to[1]);
      const double radial = L > standoff_ + 12.0 ? 1.0 : (L < standoff_ - 12.0 ? -1.0 : 0.0);
      desired = norm_or({u[0] * radial - u[1] * strafe_sign_ * 0.9,
                         u[1] * radial + u[0] * strafe_sign_ * 0.9}, u);
    } else {
      desired = norm_or({zone.center[0] - me_xy[0], zone.center[1] - me_xy[1]}, {1.0, 0.0});
    }

    a.move = avoid(me.pos, desired, 12.0);
  }

  // A pickup worth detouring for: free slot, a real weapon, and comfortably
  // inside the circle we are heading to.
  const Pickup* want_loot(const Obs& o, const Circle& zone, const Vec2& me_xy,
                          double margin) const {
    if (o.self.has_weapon[1]) return nullptr;
    const Pickup* best = nullptr;
    double best_d = 60.0;
    for (const auto& p : o.pickups) {
      if (p.kind == WeaponKind::Gun) continue;
      if (dist2(xy(p.pos), zone.center) > zone.radius - margin) continue;
      const double d = dist2(xy(p.pos), me_xy);
      if (d < best_d) {
        best_d = d;
        best = &p;
      }
    }
    return best;
  }

  // ---------------------------------------------------------------- aiming

  void aim_and_fire(Action& a, const Self& me, const Actor* target, double dist, bool los) const {
    if (!target) {
      // Nothing to shoot: face whatever last shot at us, so its next hit lands
      // in the front arc rather than the 1.5x rear one.
      a.aim = (threat_age_ < 4.0) ? Vec2{threat_bearing_, me.aim[1]} : me.aim;
      return;
    }

    const Vec3 m = muzzle(me);
    const Vec3 c = centre(*target);
    const double speed =
        (me.has_weapon[0] && me.weapons[0].speed > 1.0) ? me.weapons[0].speed : 252.0;
    const double tof = dist / speed;
    const double px = c[0] + target->vel[0] * tof;
    const double py = c[1] + target->vel[1] * tof;
    const double pz = c[2] + target->vel[2] * tof;
    const double hd = hypot2(px - m[0], py - m[1]);
    double pitch = std::atan2(pz - m[2], hd);
    pitch = std::clamp(pitch, min_pitch_, max_pitch_);
    a.aim = {std::atan2(py - m[1], px - m[0]), pitch};

    // Slot 0: the arm gun. Fire inside range, and only with a clear shot.
    if (me.has_weapon[0]) {
      const Weapon& w = me.weapons[0];
      a.fire[0] = los && dist <= weapon_range(w);
      if (w.kind == WeaponKind::Gun && w.ammo <= 0.0 && w.reload <= 0.0) a.reload = true;
    }
    // Slot 1: only the lightning is safe to hold. Laser and mortar need real
    // charge/arc handling, which is milestone 3.
    if (me.has_weapon[1] && me.weapons[1].kind == WeaponKind::Lightning)
      a.fire[1] = dist <= weapon_range(me.weapons[1]);
  }

  // ---------------------------------------------------------------- drawing

  void draw(Action& a, const Obs& o, const Self& me, const Actor* target) const {
    const Circle zone = predicted_zone(o);
    for (int i = 0; i < 12; ++i) {
      const double th = i * (2.0 * kPi / 12.0);
      a.draw.point({zone.center[0] + std::cos(th) * zone.radius,
                    zone.center[1] + std::sin(th) * zone.radius, me.pos[2]},
                   "#40c0ff", 4.0);
    }
    if (target) {
      a.draw.line(muzzle(me), centre(*target), "#ff5040");
      a.draw.text("T" + std::to_string(target->id), centre(*target), "#ffd060");
    }
    a.draw.label("hp " + std::to_string(static_cast<int>(me.hp)) + " z" +
                 std::to_string(static_cast<int>(zone.radius)) + " alive " +
                 std::to_string(o.alive) + (have_map_ ? "" : " NOMAP"));
  }

  Map map_;
  bool have_map_ = false;

  double pickup_radius_;
  double min_pitch_;
  double max_pitch_;
  double standoff_;
  double mech_height_ = 9.0;
  double mech_radius_ = 2.5;
  double step_up_ = 1.5;
  double max_slope_ = 50.0;
  double lookahead_ = 4.0;

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
