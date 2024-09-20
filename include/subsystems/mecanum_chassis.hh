#pragma once

#include "pros/motors.hpp"
#include "util/vector2.hh"

namespace vtx {

    class mecanum_chassis {
    public:
        explicit mecanum_chassis(int fr_motor_id,
                                 int fl_motor_id,
                                 int bl_motor_id,
                                 int br_motor_id);

        auto
        drive(const vector2i32 &movement_vector, std::int32_t rx) -> void;

    private:
        pros::Motor _front_right;
        pros::Motor _front_left;
        pros::Motor _back_right;
        pros::Motor _back_left;
    };

} // namespace vtx
