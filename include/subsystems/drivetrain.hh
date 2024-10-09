#pragma once

#include "constants/drivetrain_constants.hh"
#include "control/pid_controller.hh"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "units.hh"

#include <cstdint>
#include <utility>

namespace vtx {

    class drivetrain {
    public:
        explicit drivetrain(const config::drivetrain_config &config);

        auto
        drive(std::int32_t x_velocity, std::int32_t a_velocity) -> void;

        auto
        turn_to_angle(units::angle::radian_t angle) -> void;

        auto
        move_to_distance(units::length::meter_t distance) -> void;

        [[nodiscard]]
        auto
        get_left_power()
            -> std::pair<units::power::watt_t, units::power::watt_t>;

        [[nodiscard]]
        auto
        get_right_power()
            -> std::pair<units::power::watt_t, units::power::watt_t>;

        [[nodiscard]]
        auto
        get_left_displacement() -> units::length::meter_t;

        [[nodiscard]]
        auto
        get_right_displacement() -> units::length::meter_t;

        [[nodiscard]]
        auto
        get_side_displacement() -> units::length::meter_t;

        [[nodiscard]]
        auto
        get_heading() -> units::angle::radian_t;

    private:
        pros::MotorGroup   left_motors;
        pros::MotorGroup   right_motors;
        pros::adi::Encoder left_encoder;
        pros::adi::Encoder right_encoder;
        pros::adi::Encoder sideways_encoder;
        pros::IMU          imu;

        pid_controller _turn_controler;
        pid_controller _movement_controler;

        const config::drivetrain_config &config;
    };

} // namespace vtx
