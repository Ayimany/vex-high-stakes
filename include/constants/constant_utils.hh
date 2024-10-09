#pragma once

#include <cstdint>

namespace vtx {
    struct pid_coefficients {
        double p;
        double i;
        double d;
    };

} // namespace vtx

namespace vtx::constants {

    constexpr std::int8_t INVERTED     = -1;
    constexpr std::int8_t NOT_INVERTED = 1;

} // namespace vtx::constants
