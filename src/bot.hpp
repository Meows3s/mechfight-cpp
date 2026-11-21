// bot.hpp - the strategy bot factory used by main.cpp.
#pragma once

#include <memory>

#include "mf.hpp"

namespace mech {

std::unique_ptr<mf::Bot> make_bot(const mf::Hello& hello);

}  // namespace mech
