#pragma once

#include "constants/constant_utils.hh"
#include "units.hh"

#include <numbers>
#include <utility>

namespace vtx::config {
    /**
     * @brief A struct to hold the configuration parameters of the drivetrain in
     * a single, localized container
     */
    struct drivetrain_config {
        /**
         * @brief Represents the port ID assignments of each drivetrain motor
         * according to its position
         */
        struct motor_ports {
            std::int8_t front_right_id;
            std::int8_t front_left_id;
            std::int8_t back_left_id;
            std::int8_t back_right_id;
        };

        /**
         * @brief Represents the port ID assignments of each drivetrain encoder
         * according to its placement
         *
         */
        struct encoder_ports {
            std::pair<std::uint8_t, std::uint8_t> left_encoder_ids;
            std::pair<std::uint8_t, std::uint8_t> right_encoder_ids;
            std::pair<std::uint8_t, std::uint8_t> strafe_encoder_ids;
        };

        /**
         * @brief Represents the port ID assignment for each peripheral device
         * mounted on the drivetrain
         */
        struct peripheral_ports {
            std::uint8_t imu_port;
        };

        /**
         * @brief Describes the physical properties of the chassis and its
         * components
         */
        struct physical_measurements {
            /**
             * @brief The circumference of the wheels
             */
            units::length::meter_t wheel_circumference;

            /**
             * @brief The encoder pulses per rotation
             */
            std::int32_t encoder_ppr;
        };

        /**
         * @brief The port config of the motors
         */
        motor_ports motor_config;

        /**
         * @brief The port config of the encoders
         */
        encoder_ports encoder_config;

        /**
         * @brief The port config of the peripherals
         */
        peripheral_ports peripheral_config;

        /**
         * @brief The physical description of the drivetrain
         */
        physical_measurements physical_description;

        /**
         * @brief The PID coefficients of the robot's turning motion
         */
        pid_coefficients turn_coefficients;

        /**
         * @brief The PID coefficients of the robot's driving motion
         */
        pid_coefficients drive_coefficients;
    };

} // namespace vtx::config

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

    constexpr config::drivetrain_config drivetrain_configuration {
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
