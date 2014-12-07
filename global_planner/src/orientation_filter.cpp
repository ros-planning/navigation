
#include <global_planner/orientation_filter.h>
#include <tf/tf.h>
#include <angles/angles.h>

namespace global_planner {

void set_angle(geometry_msgs::PoseStamped* pose, double angle)
{
    pose->pose.orientation = tf::createQuaternionMsgFromYaw(angle); 
}

double getYaw(geometry_msgs::PoseStamped pose)
{
    return tf::getYaw(pose.pose.orientation);
}

void OrientationFilter::processPath(const geometry_msgs::PoseStamped& start, 
                                    std::vector<geometry_msgs::PoseStamped>& path)
{
    int n = path.size();
    switch(omode_) {
        case FORWARD:
            for(int i=0;i<n-1;i++){
                pointToNext(path, i);
            }
            break;
        case INTERPOLATE:
            path[0].pose.orientation = start.pose.orientation;
            interpolate(path, 0, n-1);
            break;
        case MIXTURE:
            for(int i=0;i<n-1;i++){
                pointToNext(path, i);
            }
            
            int i=n-3;
            double last = getYaw(path[i]);
            while( i>0 ){
                double new_angle = getYaw(path[i-1]);
                double diff = fabs(angles::shortest_angular_distance(new_angle, last));
                // ROS_INFO("%f %f (%d)", new_angle, diff, i);
                if( diff>0.35)
                    break;
                else
                    i--;
            }
            // ROS_INFO("%f %f", path[i].pose.position.x, path[i].pose.position.y);
            
            path[0].pose.orientation = start.pose.orientation;
            interpolate(path, i, n-1);
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
    double start_yaw = getYaw(path[start_index]),
           end_yaw   = getYaw(path[end_index  ]);
    double diff = angles::shortest_angular_distance(start_yaw, end_yaw);
    double increment = diff/(end_index-start_index);
    for(int i=start_index; i<=end_index; i++){
        double angle = start_yaw + increment * i;
        set_angle(&path[i], angle);
    }
}
                                   

};
