#pragma once

#include <cstdint>

namespace vtx {

    /**
     * @brief Holds the PID coefficients of a PID controller
     */
    struct pid_coefficients {
        double p;
        double i;
        double d;
    };

} // namespace vtx

namespace vtx::constants {

    /**
     * @brief Helper to aid in visualizing whether or not a motor is inverted.
     * Inverted.
     */
    constexpr std::int8_t INVERTED = -1;

    /**
     * @brief Helper to aid in visualizing whether or not a motor is inverted.
     * Not inverted.
     */
    constexpr std::int8_t NOT_INVERTED = 1;

} // namespace vtx::constants
