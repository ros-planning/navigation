
#include <global_planner/orientation_filter.h>
#include <tf/tf.h>
#include <angles/angles.h>

namespace global_planner {

void set_angle(geometry_msgs::PoseStamped* pose, double angle)
{
    pose->pose.orientation = tf::createQuaternionMsgFromYaw(angle); 
}

void OrientationFilter::processPath(const geometry_msgs::PoseStamped& start, 
                                    std::vector<geometry_msgs::PoseStamped>& path)
{
    switch(omode_) {
        case FORWARD:
            for(int i=0;i<path.size()-1;i++){
                pointToNext(path, i);
            }
            break;
        case INTERPOLATE:
            path[0].pose.orientation = start.pose.orientation;
            interpolate(path, 0, path.size()-1);
            break;
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

void OrientationFilter::interpolate(std::vector<geometry_msgs::PoseStamped>& path, 
                                    int start_index, int end_index)
{
    double start_yaw = tf::getYaw(path[start_index].pose.orientation),
           end_yaw   = tf::getYaw(path[end_index  ].pose.orientation);
    double diff = angles::shortest_angular_distance(start_yaw, end_yaw);
    double increment = diff/(end_index-start_index);
    for(int i=start_index; i<=end_index; i++){
        double angle = start_yaw + increment * i;
        set_angle(&path[i], angle);
    }
}
                                   

};
