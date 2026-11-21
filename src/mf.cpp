// mf.cpp - the mechfight/1 stdin/stdout loop (protocol.md section 2).
//
// Hard requirements implemented here:
//  - `ready` within hello_budget_ms after `hello`
//  - one action per observation, echoing that observation's tick
//  - answer only the NEWEST observation when several are queued
//  - flush stdout after every line; keep stdout protocol-clean (logs go to stderr)
//  - exit cleanly after `end`
//  - if act() throws, keep coasting on the last move/aim/boost instead of dying
#include "mf.hpp"

#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <iostream>
#include <deque>
#include <mutex>
#include <thread>

namespace mf {

// ---------------------------------------------------------------- small helpers

static double num(const json& j, const char* key, double fallback = 0.0) {
  auto it = j.find(key);
  if (it == j.end() || it->is_null() || !it->is_number()) return fallback;
  return it->get<double>();
}

static long inum(const json& j, const char* key, long fallback = 0) {
  return static_cast<long>(num(j, key, static_cast<double>(fallback)));
}

static Vec2 vec2(const json& j) {
  Vec2 v{0.0, 0.0};
  if (j.is_array()) {
    for (std::size_t i = 0; i < 2 && i < j.size(); ++i)
      if (j[i].is_number()) v[i] = j[i].get<double>();
  }
  return v;
}

static Vec3 vec3(const json& j) {
  Vec3 v{0.0, 0.0, 0.0};
  if (j.is_array()) {
    for (std::size_t i = 0; i < 3 && i < j.size(); ++i)
      if (j[i].is_number()) v[i] = j[i].get<double>();
  }
  return v;
}

static double fin(double v) { return std::isfinite(v) ? v : 0.0; }  // never emit NaN/Inf

// ---------------------------------------------------------------- weapon names

const char* weapon_name(WeaponKind k) {
  switch (k) {
    case WeaponKind::Gun: return "gun";
    case WeaponKind::Mortar: return "mortar";
    case WeaponKind::Laser: return "laser";
    case WeaponKind::Lightning: return "lightning";
  }
  return "gun";
}

WeaponKind weapon_kind(const std::string& s) {
  if (s == "mortar") return WeaponKind::Mortar;
  if (s == "laser") return WeaponKind::Laser;
  if (s == "lightning") return WeaponKind::Lightning;
  return WeaponKind::Gun;
}

// ---------------------------------------------------------------- self

double Self::stat(const std::string& key, double fallback) const {
  auto it = stats.find(key);
  return it == stats.end() ? fallback : it->second;
}

double Self::muzzle_height() const { return stat("muzzle_height", 7.0); }

// ---------------------------------------------------------------- observation parsing

static Weapon parse_weapon(const json& w) {
  Weapon out;
  out.kind = weapon_kind(w.value("kind", std::string("gun")));
  out.cooldown = num(w, "cooldown");
  out.ammo = num(w, "ammo", -1.0);
  out.reload = num(w, "reload", -1.0);
  out.charge = num(w, "charge", -1.0);
  auto s = w.find("stats");
  if (s != w.end() && s->is_object()) {
    out.damage = num(*s, "damage");
    out.cd = num(*s, "cooldown");
    out.speed = num(*s, "speed");
    out.range = num(*s, "range");
    out.spread_deg = num(*s, "spread_deg");
    out.mag = num(*s, "mag");
    out.reload_s = num(*s, "reload");
    out.max_charge = num(*s, "charge");
  }
  return out;
}

Obs Obs::parse(const json& j) {
  Obs o;
  o.tick = inum(j, "tick");
  o.t = num(j, "t");
  o.alive = static_cast<int>(inum(j, "alive"));

  auto s = j.find("self");
  if (s != j.end() && s->is_object()) {
    const json& sj = *s;
    Self& me = o.self;
    me.id = static_cast<int>(inum(sj, "id", -1));
    if (sj.contains("pos")) me.pos = vec3(sj["pos"]);
    if (sj.contains("vel")) me.vel = vec3(sj["vel"]);
    if (sj.contains("aim")) me.aim = vec2(sj["aim"]);
    me.hp = num(sj, "hp");
    me.max_hp = num(sj, "max_hp");
    me.energy = num(sj, "energy");
    me.max_energy = num(sj, "max_energy");
    me.grounded = sj.value("grounded", false);
    me.cubes = num(sj, "cubes");
    me.in_zone = sj.value("in_zone", true);
    if (auto u = sj.find("upgrades"); u != sj.end() && u->is_array())
      for (auto& e : *u) me.upgrades.push_back(e.get<std::string>());
    if (auto w = sj.find("weapons"); w != sj.end() && w->is_array()) {
      for (std::size_t i = 0; i < 2 && i < w->size(); ++i) {
        if ((*w)[i].is_object()) {
          me.has_weapon[i] = true;
          me.weapons[i] = parse_weapon((*w)[i]);
        }
      }
    }
    if (auto st = sj.find("stats"); st != sj.end() && st->is_object())
      for (auto& [k, v] : st->items())
        if (v.is_number()) me.stats[k] = v.get<double>();
  }

  if (auto m = j.find("mechs"); m != j.end() && m->is_array()) {
    o.mechs.reserve(m->size());
    for (auto& e : *m) {
      if (!e.is_object()) continue;
      Actor a;
      a.id = static_cast<int>(inum(e, "id", -1));
      a.npc = e.value("kind", std::string("mech")) == "npc";
      if (e.contains("pos")) a.pos = vec3(e["pos"]);
      if (e.contains("vel")) a.vel = vec3(e["vel"]);
      if (e.contains("aim")) a.aim = vec2(e["aim"]);
      a.hp = num(e, "hp", -1.0);
      a.max_hp = num(e, "max_hp", -1.0);
      a.cubes = num(e, "cubes", -1.0);
      a.charging = e.value("charging", false);
      if (auto w = e.find("weapons"); w != e.end() && w->is_array())
        for (auto& k : *w) a.weapons.push_back(weapon_kind(k.get<std::string>()));
      o.mechs.push_back(std::move(a));
    }
  }

  if (auto p = j.find("projectiles"); p != j.end() && p->is_array()) {
    o.projectiles.reserve(p->size());
    for (auto& e : *p) {
      if (!e.is_object()) continue;
      Projectile x;
      x.id = static_cast<int>(inum(e, "id", -1));
      x.kind = weapon_kind(e.value("kind", std::string("gun")));
      if (e.contains("pos")) x.pos = vec3(e["pos"]);
      if (e.contains("vel")) x.vel = vec3(e["vel"]);
      o.projectiles.push_back(x);
    }
  }

  auto read_items = [](const json& src, std::vector<Item>& out) {
    if (!src.is_array()) return;
    out.reserve(src.size());
    for (auto& e : src) {
      if (!e.is_object()) continue;
      Item it;
      it.id = static_cast<int>(inum(e, "id", -1));
      if (e.contains("pos")) it.pos = vec3(e["pos"]);
      it.hp = num(e, "hp", -1.0);
      out.push_back(it);
    }
  };
  if (auto c = j.find("crates"); c != j.end()) read_items(*c, o.crates);
  if (auto c = j.find("cubes"); c != j.end()) read_items(*c, o.cubes);

  if (auto p = j.find("pickups"); p != j.end() && p->is_array()) {
    o.pickups.reserve(p->size());
    for (auto& e : *p) {
      if (!e.is_object()) continue;
      Pickup x;
      x.id = static_cast<int>(inum(e, "id", -1));
      x.kind = weapon_kind(e.value("kind", std::string("gun")));
      if (e.contains("pos")) x.pos = vec3(e["pos"]);
      o.pickups.push_back(x);
    }
  }

  if (auto snd = j.find("sounds"); snd != j.end() && snd->is_array()) {
    o.sounds.reserve(snd->size());
    for (auto& e : *snd) {
      if (!e.is_object()) continue;
      Sound x;
      x.kind = e.value("kind", std::string());
      x.bearing = num(e, "bearing");
      std::string d = e.value("dist", std::string("far"));
      x.dist = d == "near" ? Dist::Near : (d == "mid" ? Dist::Mid : Dist::Far);
      x.age = num(e, "age");
      o.sounds.push_back(std::move(x));
    }
  }

  if (auto b = j.find("beams"); b != j.end() && b->is_array()) {
    o.beams.reserve(b->size());
    for (auto& e : *b) {
      if (!e.is_object()) continue;
      Beam x;
      x.by = static_cast<int>(inum(e, "by", -1));
      x.kind = e.value("kind", std::string());
      x.age = num(e, "age");
      if (auto p = e.find("path"); p != e.end() && p->is_array())
        for (auto& pt : *p) x.path.push_back(vec3(pt));
      o.beams.push_back(std::move(x));
    }
  }

  if (auto ev = j.find("events"); ev != j.end() && ev->is_array()) {
    o.events.reserve(ev->size());
    for (auto& e : *ev) {
      if (!e.is_object()) continue;
      Event x;
      x.type = e.value("type", std::string());
      x.amount = num(e, "amount");
      x.weapon = e.value("weapon", std::string());
      if (auto bt = e.find("bearing"); bt != e.end() && bt->is_number()) {
        x.bearing = bt->get<double>();
        x.has_bearing = true;
      }
      if (auto by = e.find("by"); by != e.end() && by->is_number()) {
        x.by = static_cast<int>(by->get<double>());
        x.has_by = true;
      }
      x.rear = e.value("rear", false);
      x.target = static_cast<int>(inum(e, "target", -1));
      x.effect = e.value("effect", std::string());
      x.id = static_cast<int>(inum(e, "id", -1));
      x.sid = e.value("id", std::string());
      x.cubes = num(e, "cubes", -1.0);
      x.kind = e.value("kind", std::string());
      x.slot = static_cast<int>(inum(e, "slot", -1));
      x.reason = e.value("reason", std::string());
      o.events.push_back(std::move(x));
    }
  }

  if (auto d = j.find("drops"); d != j.end() && d->is_array()) {
    o.drops.reserve(d->size());
    for (auto& e : *d) {
      if (!e.is_object()) continue;
      Drop x;
      x.id = static_cast<int>(inum(e, "id", -1));
      if (e.contains("pos")) x.pos = vec3(e["pos"]);
      x.radius = num(e, "radius");
      x.lands_in = num(e, "lands_in");
      x.capture_s = num(e, "capture_s");
      x.progress = num(e, "progress");
      if (auto h = e.find("holder"); h != e.end() && h->is_number()) {
        x.holder = static_cast<int>(h->get<double>());
        x.has_holder = true;
      }
      o.drops.push_back(x);
    }
  }

  if (auto z = j.find("zone"); z != j.end() && z->is_object()) {
    if (z->contains("center")) o.zone.center = vec2((*z)["center"]);
    if (z->contains("next_center")) o.zone.next_center = vec2((*z)["next_center"]);
    o.zone.radius = num(*z, "radius");
    o.zone.next_radius = num(*z, "next_radius");
    o.zone.shrink_start = num(*z, "shrink_start");
    o.zone.shrink_end = num(*z, "shrink_end");
    o.zone.dps = num(*z, "dps");
  }

  if (auto of = j.find("offer"); of != j.end() && of->is_object()) {
    Offer x;
    x.id = static_cast<int>(inum(*of, "id", -1));
    x.expires = num(*of, "expires");
    if (auto ch = of->find("choices"); ch != of->end() && ch->is_array())
      for (auto& c : *ch) {
        if (!c.is_object()) continue;
        OfferChoice oc;
        oc.id = c.value("id", std::string());
        oc.name = c.value("name", std::string());
        oc.desc = c.value("desc", std::string());
        x.choices.push_back(std::move(oc));
      }
    o.offer = std::move(x);
  }

  return o;
}

// ---------------------------------------------------------------- hello

double Hello::cfg(const char* section, const char* key, double fallback) const {
  auto s = config.find(section);
  if (s == config.end() || !s->is_object()) return fallback;
  return num(*s, key, fallback);
}

double Hello::pickup_radius() const { return cfg("loot", "pickup_radius", 4.0); }

// ---------------------------------------------------------------- draw

void Draw::clear() {
  lines = json::array();
  points = json::array();
  texts = json::array();
  label_text.reset();
}

bool Draw::empty() const {
  return lines.empty() && points.empty() && texts.empty() && !label_text.has_value();
}

void Draw::line(const Vec3& a, const Vec3& b, const std::string& color) {
  json l = {{"a", {fin(a[0]), fin(a[1]), fin(a[2])}}, {"b", {fin(b[0]), fin(b[1]), fin(b[2])}}};
  if (!color.empty()) l["color"] = color;
  lines.push_back(std::move(l));
}

void Draw::point(const Vec3& at, const std::string& color, double size) {
  json p = {{"at", {fin(at[0]), fin(at[1]), fin(at[2])}}};
  if (!color.empty()) p["color"] = color;
  p["size"] = fin(size);
  points.push_back(std::move(p));
}

void Draw::text(const std::string& s, const std::optional<Vec3>& at, const std::string& color) {
  json t = {{"text", s}};
  if (at) t["at"] = {fin((*at)[0]), fin((*at)[1]), fin((*at)[2])};
  if (!color.empty()) t["color"] = color;
  texts.push_back(std::move(t));
}

void Draw::label(const std::string& s) { label_text = s; }

json Draw::take() const {
  if (empty()) return json();
  json d = json::object();
  if (label_text) d["label"] = *label_text;
  if (!texts.empty()) d["text"] = texts;
  if (!lines.empty()) d["lines"] = lines;
  if (!points.empty()) d["points"] = points;
  return d;
}

// ---------------------------------------------------------------- action

json Action::to_json(long tick) const {
  json a = json::object();
  a["tick"] = tick;
  if (move) a["move"] = {fin((*move)[0]), fin((*move)[1])};
  if (boost) a["boost"] = true;
  if (jump) a["jump"] = true;
  if (aim) a["aim"] = {fin((*aim)[0]), fin((*aim)[1])};
  if (fire[0] || fire[1]) a["fire"] = {fire[0], fire[1]};
  if (reload) a["reload"] = true;
  if (pick) a["pick"] = *pick;
  if (take_id) a["take"] = *take_id;
  if (drop_slot) a["drop"] = *drop_slot;
  json d = draw.take();
  if (!d.is_null()) a["debug"] = std::move(d);
  return a;
}

// ---------------------------------------------------------------- logging

void log(const std::string& s) {
  std::fwrite(s.data(), 1, s.size(), stderr);
  std::fputc('\n', stderr);
  std::fflush(stderr);
}

// ---------------------------------------------------------------- the loop

namespace {

// A reader thread blocks on stdin and hands complete lines to the main thread.
// Blocking (not spinning) matters: the match container has one CPU.
struct LineQueue {
  std::mutex m;
  std::condition_variable cv;
  std::deque<std::string> q;
  bool done = false;

  void push(std::string s) {
    {
      std::lock_guard<std::mutex> g(m);
      q.push_back(std::move(s));
    }
    cv.notify_one();
  }

  bool pop_blocking(std::string& out) {
    std::unique_lock<std::mutex> l(m);
    cv.wait(l, [&] { return !q.empty() || done; });
    if (q.empty()) return false;
    out = std::move(q.front());
    q.pop_front();
    return true;
  }

  // Waits up to `window` for the next line; false means the stream went quiet.
  bool pop_for(std::string& out, std::chrono::microseconds window) {
    std::unique_lock<std::mutex> l(m);
    if (!cv.wait_for(l, window, [&] { return !q.empty() || done; })) return false;
    if (q.empty()) return false;
    out = std::move(q.front());
    q.pop_front();
    return true;
  }

  void finish() {
    {
      std::lock_guard<std::mutex> g(m);
      done = true;
    }
    cv.notify_all();
  }
};

void send(const json& msg) {
  const std::string s = msg.dump() + "\n";
  std::fwrite(s.data(), 1, s.size(), stdout);
  std::fflush(stdout);
}

}  // namespace

int run(const std::function<std::unique_ptr<Bot>(const Hello&)>& make_bot) {
  LineQueue lq;
  std::thread reader([&lq] {
    std::string line;
    while (std::getline(std::cin, line)) {
      if (line.empty()) continue;
      lq.push(std::move(line));
    }
    lq.finish();
  });
  reader.detach();  // dies with the process; nothing to join, nothing to hang on

  std::unique_ptr<Bot> bot;
  Action last;  // continuous parts to coast on if act() throws
  std::vector<std::string> batch;
  std::optional<json> latest;
  int errors = 0;

  while (true) {
    std::string first;
    if (!lq.pop_blocking(first)) break;  // stdin closed
    batch.clear();
    batch.push_back(std::move(first));
    // Observations can arrive in bursts (protocol.md section 2; check.py's
    // queued-observation test). Keep pulling until the stream goes quiet, so we
    // answer the NEWEST observation and never an obsolete one. Bounded, so a
    // continuous stream can never stall us.
    std::string more;
    while (batch.size() < 64 && lq.pop_for(more, std::chrono::microseconds(250)))
      batch.push_back(std::move(more));

    latest.reset();
    for (auto& raw : batch) {
      json msg = json::parse(raw, nullptr, false);
      if (msg.is_discarded() || !msg.is_object()) continue;  // bad line: ignore
      const std::string kind = msg.value("type", std::string());
      if (kind == "hello") {
        Hello hello;
        hello.slot = static_cast<int>(inum(msg, "slot", -1));
        hello.seed = static_cast<long long>(num(msg, "seed"));
        if (auto m = msg.find("map"); m != msg.end() && m->is_object()) {
          hello.map_dir = m->value("dir", std::string());
          if (auto f = m->find("files"); f != m->end() && f->is_array())
            for (auto& e : *f) hello.map_files.push_back(e.get<std::string>());
        }
        if (auto c = msg.find("config"); c != msg.end()) hello.config = *c;
        try {
          bot = make_bot(hello);
          if (bot) bot->on_hello(hello);
        } catch (const std::exception& e) {
          log(std::string("bot construction failed: ") + e.what());
        }
        json ready = {{"type", "ready"}, {"name", bot ? bot->name() : std::string("bot")}};
        send(ready);
      } else if (kind == "obs") {
        latest = std::move(msg);
      } else if (kind == "end") {
        return 0;  // protocol: stdin closes right after `end`, so just exit
      }
    }

    if (!latest) continue;

    Action a;
    try {
      Obs obs = Obs::parse(*latest);
      if (bot) {
        a = bot->act(obs);
      }
      if (a.move) last.move = a.move;
      if (a.aim) last.aim = a.aim;
      last.boost = a.boost;
    } catch (const std::exception& e) {
      if (++errors <= 5 || errors % 100 == 0)
        log(std::string("act() failed (") + std::to_string(errors) + "): " + e.what());
      a = Action{};
      a.move = last.move;
      a.aim = last.aim;
      a.boost = last.boost;
    } catch (...) {
      if (++errors <= 5 || errors % 100 == 0) log("act() failed: unknown exception");
      a = Action{};
      a.move = last.move;
      a.aim = last.aim;
      a.boost = last.boost;
    }
    send(a.to_json(latest->value("tick", 0L)));
  }

  return 0;
}

}  // namespace mf
