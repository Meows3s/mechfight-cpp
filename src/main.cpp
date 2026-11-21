#include "bot.hpp"

int main() {
  return mf::run([](const mf::Hello& hello) { return mech::make_bot(hello); });
}
