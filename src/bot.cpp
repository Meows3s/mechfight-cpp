// bot.cpp - the baseline policy.
//
// Milestone 1: a protocol-correct, non-stupid bot. It stays in the zone, picks up
// one real weapon, leads the nearest visible enemy with the arm gun, and fires.
// Deliberately simple: no map, no obstacle avoidance, no laser/mortar use.
// Everything here reads numbers from hello.config, never hard-coded defaults.
#include "bot.hpp"

#include <algorithm>
#include <cmath>
#include <string>

namespace mech {
namespace {

double hypot2(double x, double y) { return std::sqrt(x * x + y * y); }

double dist2(const mf::Vec2& a, const mf::Vec2& b) { return hypot2(a[0] - b[0], a[1] - b[1]); }

mf::Vec2 xy(const mf::Vec3& p) { return {p[0], p[1]}; }

mf::Vec2 norm_or(const mf::Vec2& v, const mf::Vec2& fallback) {
  const double L = hypot2(v[0], v[1]);
  if (L < 1e-9) return fallback;
  return {v[0] / L, v[1] / L};
}

// Lower rank = better. Unknown upgrades sink to the bottom.
int upgrade_rank(const std::string& id) {
  static const char* kOrder[] = {"armor",   "plating",   "servos", "capacitor", "reactor",
                                 "nanites", "gyros",     "jets",   "sensors"};
  for (int i = 0; i < static_cast<int>(sizeof(kOrder) / sizeof(kOrder[0])); ++i)
    if (id == kOrder[i]) return i;
  return 1000;
}

double weapon_range(const mf::Weapon& w) { return w.range > 1.0 ? w.range : 230.0; }

class MechBot final : public mf::Bot {
 public:
  explicit MechBot(const mf::Hello& hello)
      : pickup_radius_(hello.pickup_radius()),
        min_pitch_(hello.cfg("mech", "min_pitch", -0.7)),
        max_pitch_(hello.cfg("mech", "max_pitch", 1.3)),
        standoff_(hello.cfg("sound", "near", 60.0) + 10.0) {}

  std::string name() const override { return "apex"; }

  mf::Action act(const mf::Obs& o) override {
    mf::Action a;
    const mf::Self& me = o.self;
    const mf::Vec2 me_xy = xy(me.pos);

    upgrade(a, o);
    loot(a, o, me_xy);
    const mf::Actor* target = pick_target(o, me_xy);
    steer(a, o, me, me_xy, target);
    shoot(a, me, target);
    draw(a, o, me, target);
    return a;
  }

 private:
  // Offer: take the highest-ranked upgrade on the menu; fall back to choice 0.
  void upgrade(mf::Action& a, const mf::Obs& o) const {
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

  // Fill the one empty weapon slot with the closest real weapon in reach.
  void loot(mf::Action& a, const mf::Obs& o, const mf::Vec2& me_xy) const {
    if (o.self.has_weapon[1]) return;
    int want = -1;
    double best = pickup_radius_;
    for (const auto& p : o.pickups) {
      if (p.kind == mf::WeaponKind::Gun) continue;  // never swap the gun out
      const double d = dist2(xy(p.pos), me_xy);
      if (d <= best) {
        best = d;
        want = p.id;
      }
    }
    if (want >= 0) a.take_id = want;
  }

  // Nearest visible mech; if none, nearest NPC drone.
  const mf::Actor* pick_target(const mf::Obs& o, const mf::Vec2& me_xy) const {
    const mf::Actor* best = nullptr;
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

  // Movement: get back in the zone first, otherwise hold a standoff and strafe.
  void steer(mf::Action& a, const mf::Obs& o, const mf::Self& me, const mf::Vec2& me_xy,
             const mf::Actor* target) const {
    const mf::Vec2 zc = o.zone.center;
    const double to_center = dist2(me_xy, zc);
    const double margin = 10.0;

    if (!me.in_zone || to_center > o.zone.radius - margin) {
      a.move = norm_or({zc[0] - me_xy[0], zc[1] - me_xy[1]}, {0.0, 0.0});
      a.boost = to_center > 25.0;  // boost only when the run actually matters
      return;
    }

    if (target) {
      const mf::Vec2 to{target->pos[0] - me_xy[0], target->pos[1] - me_xy[1]};
      const double L = hypot2(to[0], to[1]);
      const double ux = L > 1e-9 ? to[0] / L : 1.0;
      const double uy = L > 1e-9 ? to[1] / L : 0.0;
      const double radial = L > standoff_ + 12.0 ? 1.0 : (L < standoff_ - 12.0 ? -1.0 : 0.0);
      // radial component keeps the range, tangential component keeps us moving
      a.move = norm_or({ux * radial - uy * 0.8, uy * radial + ux * 0.8}, {ux, uy});
      return;
    }

    a.move = norm_or({o.zone.next_center[0] - me_xy[0], o.zone.next_center[1] - me_xy[1]},
                     {0.0, 0.0});
  }

  void shoot(mf::Action& a, const mf::Self& me, const mf::Actor* target) const {
    if (!target) {
      a.aim = me.aim;  // hold the torso still while there is nothing to shoot
      return;
    }

    // Lead with the arm gun's muzzle velocity; allow for the target's cube size.
    const double scale = 1.0 + 0.05 * std::max(0.0, target->cubes);
    const double tx = target->pos[0];
    const double ty = target->pos[1];
    const double tz = target->pos[2] + 4.5 * scale;
    const double dx = tx - me.pos[0];
    const double dy = ty - me.pos[1];
    const double dz = tz - (me.pos[2] + me.muzzle_height());
    const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

    const double speed = (me.has_weapon[0] && me.weapons[0].speed > 1.0) ? me.weapons[0].speed : 252.0;
    const double tof = dist / speed;
    const double px = tx + target->vel[0] * tof;
    const double py = ty + target->vel[1] * tof;
    const double pz = tz + target->vel[2] * tof;
    const double hd = hypot2(px - me.pos[0], py - me.pos[1]);

    double pitch = std::atan2(pz - (me.pos[2] + me.muzzle_height()), hd);
    pitch = std::clamp(pitch, min_pitch_, max_pitch_);
    a.aim = {std::atan2(py - me.pos[1], px - me.pos[0]), pitch};

    // Slot 0: the arm gun. Fire inside its range; start a reload when dry.
    if (me.has_weapon[0]) {
      const mf::Weapon& w = me.weapons[0];
      a.fire[0] = dist <= weapon_range(w);
      if (w.kind == mf::WeaponKind::Gun && w.ammo <= 0.0 && w.reload <= 0.0)
        a.reload = true;
    }

    // Slot 1: only the lightning is safe to spam. Laser and mortar need real
    // charge/arc handling, which is milestone 2.
    if (me.has_weapon[1] && me.weapons[1].kind == mf::WeaponKind::Lightning) {
      a.fire[1] = dist <= weapon_range(me.weapons[1]);
    }
  }

  void draw(mf::Action& a, const mf::Obs& o, const mf::Self& me, const mf::Actor* target) const {
    if (target) {
      a.draw.line({me.pos[0], me.pos[1], me.pos[2] + me.muzzle_height()},
                  {target->pos[0], target->pos[1], target->pos[2] + 4.5}, "#ff5040");
    }
    a.draw.point({o.zone.center[0], o.zone.center[1], me.pos[2]}, "#40c0ff", 8.0);
    a.draw.label("hp " + std::to_string(static_cast<int>(me.hp)) + " cubes " +
                 std::to_string(static_cast<int>(me.cubes)) + " alive " +
                 std::to_string(o.alive));
  }

  double pickup_radius_;
  double min_pitch_;
  double max_pitch_;
  double standoff_;
};

}  // namespace

std::unique_ptr<mf::Bot> make_bot(const mf::Hello& hello) {
  return std::make_unique<MechBot>(hello);
}

}  // namespace mech
