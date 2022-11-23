#ifndef BASE_LOCAL_PLANNER_COMMON_H
#define BASE_LOCAL_PLANNER_COMMON_H

#include<cmath>

namespace base_local_planner {

inline double normalize_angle(double value, double min=-M_PI, double max=M_PI)
{
    double cycle = max - min;
    double angle = std::fmod(value - min, cycle) + min;

    while (angle < min)
    {
        angle += cycle;
    }

    return angle;
}

inline void calc_cargo_rear_position(double x, double y, double global_cargo_angle, double cargo_length, double &cargo_rear_x, double &cargo_rear_y)
{
    cargo_rear_x = x + std::cos(global_cargo_angle) * cargo_length;
    cargo_rear_y = y + std::sin(global_cargo_angle) * cargo_length;
}

inline void calc_cargo_delta_angle(double x0, double y0, double x1, double y1, double cargo_rear_x, double cargo_rear_y, double &delta_angle)
{
    double cur_cargo_th = std::atan2(y0 - cargo_rear_y, x0 - cargo_rear_x);
    double next_cargo_th = std::atan2(y1 - cargo_rear_y, x1 - cargo_rear_x);
    delta_angle = base_local_planner::normalize_angle(next_cargo_th - cur_cargo_th);
}

}
#endif