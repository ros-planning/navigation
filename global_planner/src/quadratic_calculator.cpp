#include <global_planner/quadratic_calculator.h>

namespace global_planner {
float QuadraticCalculator::calculatePotential(float* potential, unsigned char cost, int n) {
    // get neighbors
    float u, d, l, r;
    l = potential[n - 1];
    r = potential[n + 1];
    u = potential[n - nx_];
    d = potential[n + nx_];
    //  ROS_INFO("[Update] c: %f  l: %f  r: %f  u: %f  d: %f\n",
    //	 potential[n], l, r, u, d);
    //  ROS_INFO("[Update] cost: %d\n", costs[n]);

    // find lowest, and its lowest neighbor
    float ta, tc;
    if (l < r)
        tc = l;
    else
        tc = r;
    if (u < d)
        ta = u;
    else
        ta = d;

    float hf = cost; // traversability factor
    float dc = tc - ta;		// relative cost between ta,tc
    if (dc < 0) 		// tc is lowest
            {
        dc = -dc;
        ta = tc;
    }

    // calculate new potential
    if (dc >= hf)		// if too large, use ta-only update
        return ta + hf;
    else			// two-neighbor interpolation update
    {
        // use quadratic approximation
        // might speed this up through table lookup, but still have to
        //   do the divide
        float d = dc / hf;
        float v = -0.2301 * d * d + 0.5307 * d + 0.7040;
        return ta + hf * v;
    }
}
}

