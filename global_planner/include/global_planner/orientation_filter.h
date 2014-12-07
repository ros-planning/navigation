/*********************************************************************
 * Author: David V. Lu!!
 *********************************************************************/
#ifndef _ORIENTATION_FILTER_H
#define _ORIENTATION_FILTER_H
#include <nav_msgs/Path.h>

namespace global_planner {

class OrientationFilter {
    public:
        virtual void processPath(const geometry_msgs::PoseStamped& start,
                                 std::vector<geometry_msgs::PoseStamped>& path);
};

} //end namespace global_planner
#endif
