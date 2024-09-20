#include "subsystems/chassis.hh"

#include "units.hh"

namespace vtx {

    drivetrain::drivetrain(const drivetrain_config &config) :
    left_motors { config.motor_config.front_left_id,
                  config.motor_config.back_left_id },
    right_motors { config.motor_config.front_right_id,
                   config.motor_config.back_right_id },
    left_encoder { config.encoder_config.left_encoder_ids.first,
                   config.encoder_config.left_encoder_ids.second },
    right_encoder { config.encoder_config.right_encoder_ids.first,
                    config.encoder_config.right_encoder_ids.second,
                    true },
    sideways_encoder { config.encoder_config.strafe_encoder_ids.first,
                       config.encoder_config.strafe_encoder_ids.second },
    imu { config.peripheral_config.imu_port },
    config { config } {
        imu.tare();
    }

    auto
    drivetrain::drive(std::int32_t x_velocity,
                      std::int32_t a_velocity) -> void {
        std::int32_t left_speed  = x_velocity - a_velocity;
        std::int32_t right_speed = x_velocity + a_velocity;

        left_motors.move_velocity(left_speed);
        right_motors.move_velocity(right_speed);
    }

    auto
    drivetrain::get_left_power()
        -> std::pair<units::power::watt_t, units::power::watt_t> {
        return std::make_pair(
            units::power::watt_t { left_motors.get_power(0) },
            units::power::watt_t { left_motors.get_power(1) });
    }

    auto
    drivetrain::get_right_power()
        -> std::pair<units::power::watt_t, units::power::watt_t> {
        return std::make_pair(
            units::power::watt_t { right_motors.get_power(0) },
            units::power::watt_t { right_motors.get_power(1) });
    }

    auto
    drivetrain::get_left_displacement() -> units::length::meter_t {
        return config.physical_description.wheel_circumference
             * (static_cast<double>(this->left_encoder.get_value())
                / static_cast<double>(config.physical_description.encoder_ppr));
    }

    auto
    drivetrain::get_right_displacement() -> units::length::meter_t {
        return config.physical_description.wheel_circumference
             * (static_cast<double>(this->right_encoder.get_value())
                / static_cast<double>(config.physical_description.encoder_ppr));
    }

    auto
    drivetrain::get_side_displacement() -> units::length::meter_t {
        return config.physical_description.wheel_circumference
             * (static_cast<double>(this->sideways_encoder.get_value())
                / static_cast<double>(config.physical_description.encoder_ppr));
    }

    auto
    drivetrain::get_heading() -> units::angle::radian_t {
        return units::angle::degree_t { imu.get_heading() };
    }

} // namespace vtx
