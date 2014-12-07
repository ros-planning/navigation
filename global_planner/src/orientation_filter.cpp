
#include <global_planner/orientation_filter.h>
#include <tf/tf.h>

namespace global_planner {

void set_angle(geometry_msgs::PoseStamped* pose, double angle)
{
    pose->pose.orientation = tf::createQuaternionMsgFromYaw(angle); 
}

void OrientationFilter::processPath(const geometry_msgs::PoseStamped& start, 
                                    std::vector<geometry_msgs::PoseStamped>& path)
{
    for(int i=0;i<path.size()-1;i++){
        pointToNext(path, i);
    }
}
    
void OrientationFilter::pointToNext(std::vector<geometry_msgs::PoseStamped>& path, int index)
{
  double x0 = path[ index ].pose.position.x, 
         y0 = path[ index ].pose.position.y,
         x1 = path[index+1].pose.position.x,
         y1 = path[index+1].pose.position.y;
         
  double angle = atan2(y1-y0,x1-x0);
  set_angle(&path[index], angle);
}
};
