#include "constants/numerical_constants.hh"
#include "subsystems/drivetrain.hh"
#include "units.hh"

#include <algorithm>

namespace vtx {

    drivetrain::drivetrain(const config::drivetrain_config &config) :
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
    config { config },
    _turn_controler { config.turn_coefficients.p,
                      config.turn_coefficients.i,
                      config.turn_coefficients.d },
    _movement_controler { config.drive_coefficients.p,
                          config.drive_coefficients.i,
                          config.drive_coefficients.d } {
        imu.tare();
        _turn_controler.set_output_limits(-1.0, 1.0);
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
    drivetrain::turn_to_angle(units::angle::radian_t angle) -> void {
        angle = units::math::abs(angle - constants::half_turn);
        units::angle::radian_t heading = get_heading() - constants::half_turn;

        units::angle::radian_t shortest_distance
            = units::math::fmod(angle - heading + constants::half_turn,
                                constants::full_turn)
            - constants::half_turn;

        double rotational_output
            = std::clamp(_turn_controler.get_output(
                             0.0,
                             units::math::abs(shortest_distance).value()),
                         -1.0,
                         1.0);

        drive(0.0,
              static_cast<std::int32_t>(
                  rotational_output * 127.0
                  * (shortest_distance < 0_rad ? -1.0 : 1.0)));
    }

    auto
    drivetrain::move_to_distance(units::length::meter_t distance) -> void {
        units::length::meter_t current_distance
            = get_side_displacement_average();

        double output = std::clamp(
            _movement_controler.get_output(current_distance.value(),
                                           distance.value()),
            -1.0,
            1.0);

        drive(output * 127.0, 0.0);
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
    drivetrain::get_strafe_displacement() -> units::length::meter_t {
        return config.physical_description.wheel_circumference
             * (static_cast<double>(this->sideways_encoder.get_value())
                / static_cast<double>(config.physical_description.encoder_ppr));
    }

    auto
    drivetrain::get_side_displacement_average() -> units::length::meter_t {
        return (get_left_displacement() + get_right_displacement()) / 2.0;
    }

    auto
    drivetrain::get_heading() -> units::angle::radian_t {
        return units::angle::degree_t { imu.get_heading() };
    }

} // namespace vtx
