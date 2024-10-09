#pragma once

#include "constants/constant_utils.hh"
#include "subsystems/chassis.hh"

#include <numbers>

namespace vtx::constants {

    constexpr std::int8_t front_right_motor_port = 20 * NOT_INVERTED;
    constexpr std::int8_t front_left_motor_port  = 10 * INVERTED;
    constexpr std::int8_t back_left_motor_port   = 1 * INVERTED;
    constexpr std::int8_t back_right_motor_port  = 11 * NOT_INVERTED;

    constexpr std::pair<std::uint8_t, std::uint8_t> left_encoder_port
        = std::make_pair('E', 'F');

    constexpr std::pair<std::uint8_t, std::uint8_t> right_encoder_port
        = std::make_pair('G', 'H');

    constexpr std::pair<std::uint8_t, std::uint8_t> strafe_encoder_port
        = std::make_pair('C', 'D');

    constexpr std::int32_t encoder_ppr { 360 };

    constexpr std::uint8_t imu_port = 8;

    constexpr units::length::meter_t wheel_diameter
        = units::length::inch_t { 4.0 };

    constexpr units::length::meter_t wheel_circumference
        = wheel_diameter * std::numbers::pi;

    constexpr double turn_p = -0.65;
    constexpr double turn_i = 0.0;
    constexpr double turn_d = 0.0;

    constexpr double drive_p = 1.1;
    constexpr double drive_i = 0.0;
    constexpr double drive_d = 0.0;

    constexpr drivetrain_config drivetrain_configuration {
        { .front_right_id = front_right_motor_port,
         .front_left_id  = front_left_motor_port,
         .back_left_id   = back_left_motor_port,
         .back_right_id  = back_right_motor_port },
        { .left_encoder_ids   = left_encoder_port,
         .right_encoder_ids  = right_encoder_port,
         .strafe_encoder_ids = strafe_encoder_port },
        { .imu_port = imu_port },
        { .wheel_circumference = wheel_circumference,
         .encoder_ppr         = encoder_ppr },
        { turn_p, turn_i, turn_d },
        { drive_p, drive_i, drive_d }
    };

} // namespace vtx::constants
