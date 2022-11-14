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
}
#endif