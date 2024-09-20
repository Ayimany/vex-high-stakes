#include "mecanum_chassis.hh"

#include "pros/abstract_motor.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>

namespace vtx {

    mecanum_chassis::mecanum_chassis(int fr_motor_id,
                                     int fl_motor_id,
                                     int bl_motor_id,
                                     int br_motor_id) :
    _front_right(fr_motor_id),
    _front_left(fl_motor_id),
    _back_left(bl_motor_id),
    _back_right(br_motor_id) {
        _front_right.set_brake_mode(pros::MotorBrake::coast);
        _front_left.set_brake_mode(pros::MotorBrake::coast);
        _back_left.set_brake_mode(pros::MotorBrake::coast);
        _back_right.set_brake_mode(pros::MotorBrake::coast);
    }

    auto
    mecanum_chassis::drive(const vector2i32 &movement_vector,
                           std::int32_t      rx) -> void {
        const std::int32_t  x = movement_vector.x;
        const std::uint32_t y = movement_vector.y;

        _front_left.move_velocity(x + y + rx);
        _back_left.move_velocity(x - y + rx);
        _front_right.move_velocity(x - y - rx);
        _back_right.move_velocity(x + y - rx);
    }

} // namespace vtx
