#ifndef BASE_LOCAL_PLANNER_COMMON_H
#define BASE_LOCAL_PLANNER_COMMON_H

#include<cmath>

namespace base_local_planner {

inline double normalize(double value, double min=-M_PI, double max=M_PI)
{
    double range = std::fabs(max - min);
    if (value < min)
    {
        return value + range * std::ceil(std::abs(value - min) / range);
    }
    else if (max <= value)
    {
        return value - range * (std::floor(std::abs(value - max) / range) + 1);
    }
    return value;
}
}
#endif