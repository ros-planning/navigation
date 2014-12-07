/*********************************************************************
 * Author: David V. Lu!!
 *********************************************************************/
#ifndef _ORIENTATION_FILTER_H
#define _ORIENTATION_FILTER_H
#include <nav_msgs/Path.h>

namespace global_planner {

enum OrientationMode { NONE, FORWARD, INTERPOLATE };

class OrientationFilter {
    public:
        OrientationFilter() : omode_(NONE) {}
    
    
        virtual void processPath(const geometry_msgs::PoseStamped& start,
                                 std::vector<geometry_msgs::PoseStamped>& path);
                                 
        void pointToNext(std::vector<geometry_msgs::PoseStamped>& path, int index);
        void interpolate(std::vector<geometry_msgs::PoseStamped>& path, 
                         int start_index, int end_index);
                         
        void setMode(OrientationMode new_mode){ omode_ = new_mode; }
        void setMode(int new_mode){ setMode((OrientationMode) new_mode); }
    protected:
        OrientationMode omode_;        
};

} //end namespace global_planner
#endif
