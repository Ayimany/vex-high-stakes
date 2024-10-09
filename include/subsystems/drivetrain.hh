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
        /**
         * @brief Construct a new drivetrain object
         *
         * @param config The config for the object
         */
        explicit drivetrain(const config::drivetrain_config &config);

        /**
         * @brief Causes the robot to move with the desired velocities
         *
         * @param x_velocity The linear velocity along the x-axis
         * @param a_velocity The angular velocity around the z-axis
         */
        auto
        drive(std::int32_t x_velocity, std::int32_t a_velocity) -> void;

        /**
         * @brief Causes the robot to attempt to reach a specific angle. Must be
         * called repeatedly
         *
         * @param angle The angle ro reach
         */
        auto
        turn_to_angle(units::angle::radian_t angle) -> void;

        /**
         * @brief Causes the robot to attempt to reach a specific distance
         * relative to the point in which it was last reset. Must be called
         * repeatedly
         *
         * @param distance The distance to reach
         */
        auto
        move_to_distance(units::length::meter_t distance) -> void;

        /**
         * @brief Get the left power output
         *
         * @return std::pair<units::power::watt_t, units::power::watt_t>
         */
        [[nodiscard]]
        auto
        get_left_power()
            -> std::pair<units::power::watt_t, units::power::watt_t>;

        /**
         * @brief Get the right power output
         *
         * @return std::pair<units::power::watt_t, units::power::watt_t>
         */
        [[nodiscard]]
        auto
        get_right_power()
            -> std::pair<units::power::watt_t, units::power::watt_t>;

        /**
         * @brief Get the left encoder's displacement
         *
         * @return units::length::meter_t
         */
        [[nodiscard]]
        auto
        get_left_displacement() -> units::length::meter_t;

        /**
         * @brief Get the right encoder's displacement
         *
         * @return units::length::meter_t
         */
        [[nodiscard]]
        auto
        get_right_displacement() -> units::length::meter_t;

        /**
         * @brief Get the strafe encoder's displacement
         *
         * @return units::length::meter_t
         */
        [[nodiscard]]
        auto
        get_strafe_displacement() -> units::length::meter_t;

        /**
         * @brief Get the displacement average of the left and right
         * displacements
         *
         * @return units::length::meter_t
         */
        [[nodiscard]]
        auto
        get_side_displacement_average() -> units::length::meter_t;

        /**
         * @brief Get the robot's heading
         *
         * @return units::angle::radian_t
         */
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
