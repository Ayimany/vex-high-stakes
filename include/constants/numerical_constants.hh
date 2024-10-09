#pragma once

#include "units.hh"

#include <numbers>

using namespace units::literals;

namespace vtx::constants {

    constexpr units::angle::radian_t half_turn { std::numbers::pi };
    constexpr units::angle::radian_t full_turn = 2.0 * half_turn;

} // namespace vtx::constants
