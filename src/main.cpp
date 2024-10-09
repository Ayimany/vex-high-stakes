#include "main.h"

#include "constants/drivetrain_constants.hh"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "subsystems/chassis.hh"
#include "units.hh"

#include <numbers>

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void
initialize() {
    pros::lcd::initialize();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void
disabled() { }

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void
competition_initialize() { }

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void
autonomous() { }

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void
opcontrol() {
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    vtx::drivetrain  _drivetrain { vtx::constants::drivetrain_configuration };

    while (true) {
        std::int32_t x_velocity       = master.get_analog(ANALOG_LEFT_Y);
        std::int32_t angular_velocity = -master.get_analog(ANALOG_RIGHT_X);

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A) > 0) {
            _drivetrain.turn_to_angle(units::angle::radian_t { 0 });
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X) > 0) {
            _drivetrain.move_to_distance(units::length::meter_t { 1.0 });
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B) > 0) {
            _drivetrain.move_to_distance(units::length::meter_t { -1.0 });
        } else {
            _drivetrain.drive(x_velocity, angular_velocity);
        }

        pros::lcd::print(0,
                         "Left: %f m",
                         _drivetrain.get_left_displacement().value());

        pros::lcd::print(1,
                         "Right: %f m",
                         _drivetrain.get_right_displacement().value());

        pros::lcd::print(2,
                         "Strafe: %f",
                         _drivetrain.get_side_displacement().value());
        pros::lcd::print(3, "vx: %d", x_velocity);
        pros::lcd::print(4, "theta: %d", angular_velocity);

        pros::lcd::print(
            6,
            "Theta: %f °",
            _drivetrain.get_heading().convert<units::angle::degree>().value());

        pros::delay(20); // Run for 20 ms then update
    }
}
