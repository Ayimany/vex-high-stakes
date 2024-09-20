#pragma once

#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "units.hh"

#include <cstdint>
#include <utility>

namespace vtx {

    struct drivetrain_config {
        struct motor_ports {
            std::int8_t front_right_id;
            std::int8_t front_left_id;
            std::int8_t back_left_id;
            std::int8_t back_right_id;
        };

        struct encoder_ports {
            std::pair<std::uint8_t, std::uint8_t> left_encoder_ids;
            std::pair<std::uint8_t, std::uint8_t> right_encoder_ids;
            std::pair<std::uint8_t, std::uint8_t> strafe_encoder_ids;
        };

        struct peripheral_ports {
            std::uint8_t imu_port;
        };

        struct physical_measurements {
            units::length::meter_t wheel_circumference;
            std::int32_t           encoder_ppr;
        };

        motor_ports           motor_config;
        encoder_ports         encoder_config;
        peripheral_ports      peripheral_config;
        physical_measurements physical_description;
    };

    class drivetrain {
    public:
        explicit drivetrain(const drivetrain_config &config);

        auto
        drive(std::int32_t x_velocity, std::int32_t a_velocity) -> void;

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

        const drivetrain_config &config;
    };

} // namespace vtx
