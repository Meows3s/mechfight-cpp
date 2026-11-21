// mf.hpp - mechfight/1 protocol layer (protocol.md sections 2, 5, 6).
//
// Everything in this header is ground truth from protocol.md: message shapes,
// units, and the stdin/stdout contract. Strategy lives in bot.cpp, not here.
#pragma once

#include <array>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace mf {

using json = nlohmann::json;

using Vec2 = std::array<double, 2>;  // [yaw, pitch] or a horizontal direction
using Vec3 = std::array<double, 3>;  // [x east, y north, z up], meters

enum class WeaponKind { Gun, Mortar, Laser, Lightning };

const char* weapon_name(WeaponKind k);          // "gun", "mortar", ...
WeaponKind weapon_kind(const std::string& s);   // unknown -> Gun

// ---------------------------------------------------------------- observation

struct Weapon {
  WeaponKind kind = WeaponKind::Gun;
  double cooldown = 0.0;  // seconds until it can fire
  double ammo = -1.0;     // gun only
  double reload = -1.0;   // gun only: seconds of reload left (0 = not reloading)
  double charge = -1.0;   // laser only: seconds charged so far
  // this weapon's numbers after your upgrades
  double damage = 0.0, cd = 0.0, speed = 0.0, range = 0.0;
  double spread_deg = 0.0, mag = 0.0, reload_s = 0.0, max_charge = 0.0;
};

struct Actor {  // another mech (npc == false) or an NPC drone
  int id = -1;
  bool npc = false;
  Vec3 pos{}, vel{};
  Vec2 aim{};
  double hp = -1.0, max_hp = -1.0, cubes = -1.0;
  std::vector<WeaponKind> weapons;
  bool charging = false;
};

struct Projectile { int id = -1; WeaponKind kind = WeaponKind::Gun; Vec3 pos{}, vel{}; };
struct Item { int id = -1; Vec3 pos{}; double hp = -1.0; };
struct Pickup { int id = -1; WeaponKind kind = WeaponKind::Gun; Vec3 pos{}; };

enum class Dist { Near, Mid, Far };
struct Sound { std::string kind; double bearing = 0.0; Dist dist = Dist::Far; double age = 0.0; };

struct Beam { int by = -1; std::string kind; std::vector<Vec3> path; double age = 0.0; };

// One entry of obs.events (protocol.md section 6). Fields not carried by a given
// event keep their defaults; `type` says which ones are meaningful.
struct Event {
  std::string type;
  double amount = 0.0;
  std::string weapon;
  double bearing = 0.0;
  bool has_bearing = false;
  int by = -1;
  bool has_by = false;
  bool rear = false;
  int target = -1;
  std::string effect;
  int id = -1;      // drop / took / dropped / take_failed
  std::string sid;  // upgrade id
  double cubes = -1.0;
  std::string kind;
  int slot = -1;
  std::string reason;
};

struct Drop {
  int id = -1;
  Vec3 pos{};
  double radius = 0.0, lands_in = 0.0, capture_s = 0.0, progress = 0.0;
  int holder = -1;
  bool has_holder = false;
};

struct Zone {
  Vec2 center{}, next_center{};
  double radius = 0.0, next_radius = 0.0;
  double shrink_start = 0.0, shrink_end = 0.0, dps = 0.0;
};

struct OfferChoice { std::string id, name, desc; };
struct Offer { int id = -1; double expires = 0.0; std::vector<OfferChoice> choices; };

struct Self {
  int id = -1;
  Vec3 pos{}, vel{};
  Vec2 aim{};  // actual (slewed) aim
  double hp = 0.0, max_hp = 0.0, energy = 0.0, max_energy = 0.0;
  bool grounded = false;
  double cubes = 0.0;
  std::vector<std::string> upgrades;
  bool in_zone = false;
  bool has_weapon[2] = {false, false};
  Weapon weapons[2];
  std::map<std::string, double> stats;  // your [mech] numbers after upgrades

  double stat(const std::string& key, double fallback) const;
  double muzzle_height() const;
};

struct Obs {
  long tick = 0;
  double t = 0.0;  // match time, seconds
  int alive = 0;   // mechs alive (NPCs not counted)
  Self self;
  std::vector<Actor> mechs;
  std::vector<Projectile> projectiles;
  std::vector<Item> crates, cubes;
  std::vector<Pickup> pickups;
  std::vector<Sound> sounds;
  std::vector<Beam> beams;
  std::vector<Event> events;
  std::vector<Drop> drops;
  Zone zone;
  std::optional<Offer> offer;

  static Obs parse(const json& j);
};

// ---------------------------------------------------------------- hello

struct Hello {
  int slot = -1;
  long long seed = 0;
  std::string map_dir;
  std::vector<std::string> map_files;
  json config;  // the complete live rule set; read numbers from here

  // config[section][key], or `fallback` when absent.
  double cfg(const char* section, const char* key, double fallback) const;
  double pickup_radius() const;
};

// ---------------------------------------------------------------- action

// Debug drawing for one decision. World coordinates; only your team sees it and
// it never affects the match. The server silently truncates over the caps.
struct Draw {
  json lines = json::array();
  json points = json::array();
  json texts = json::array();
  std::optional<std::string> label_text;

  void clear();
  bool empty() const;
  void line(const Vec3& a, const Vec3& b, const std::string& color = "");
  void point(const Vec3& at, const std::string& color = "", double size = 4.0);
  void text(const std::string& s, const std::optional<Vec3>& at = std::nullopt,
            const std::string& color = "");
  void label(const std::string& s);  // one short status line above your mech

  json take() const;  // the pending drawing, or null when there is nothing to draw
};

struct Action {
  std::optional<Vec2> move;  // world-frame direction, length <= 1
  bool boost = false;
  bool jump = false;
  std::optional<Vec2> aim;  // desired [yaw, pitch]
  bool fire[2] = {false, false};
  bool reload = false;
  std::optional<int> pick;
  std::optional<int> take_id;
  std::optional<int> drop_slot;
  Draw draw;

  json to_json(long tick) const;
};

// ---------------------------------------------------------------- the bot

class Bot {
public:
  virtual ~Bot() = default;
  virtual std::string name() const { return "bot"; }
  virtual void on_hello(const Hello& hello) { (void)hello; }
  virtual Action act(const Obs& obs) = 0;
};

// Reads stdin, writes stdout, answers only the newest observation, exits on `end`.
// act() may take 50 ms of wall time; the wrapper handles crashing and coasting.
int run(const std::function<std::unique_ptr<Bot>(const Hello&)>& make_bot);

void log(const std::string& s);  // stderr, line-buffered

}  // namespace mf
