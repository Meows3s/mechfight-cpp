// bot.hpp - the strategy bot factory used by main.cpp.
#pragma once

#include <memory>

#include "mf.hpp"

namespace mech {

using mf::Actor;
using mf::Action;
using mf::Bot;
using mf::Hello;
using mf::Item;
using mf::Obs;
using mf::Pickup;
using mf::Self;
using mf::Vec2;
using mf::Vec3;
using mf::Weapon;
using mf::WeaponKind;
using mf::Zone;

std::unique_ptr<mf::Bot> make_bot(const mf::Hello& hello);

}  // namespace mech
