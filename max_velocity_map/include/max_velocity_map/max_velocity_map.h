#include <ros/ros.h>
#include <amcl/map/map.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>

#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

class MaxVelocityMap
{
  public:
    MaxVelocityMap();
    ~MaxVelocityMap();

    private:
    ros::NodeHandle nh_;
    ros::Subscriber amcl_sub_;
    ros::Subscriber map_sub_;
    ros::Publisher max_vel_x_pub_;
    ros::Publisher min_vel_x_pub_;
    ros::Publisher max_vel_y_pub_;
    ros::Publisher min_vel_y_pub_;
    map_t* map_;
    dynamic_reconfigure::ReconfigureRequest srv_req_;
    dynamic_reconfigure::ReconfigureResponse srv_resp_;
    dynamic_reconfigure::DoubleParameter double_param_;
    dynamic_reconfigure::Config conf_;
    std::string local_planner_;
    float max_vel_x_initial_;
    float min_vel_x_initial_;
    float max_vel_y_initial_;
    float min_vel_y_initial_;
    void freeMapDependentMemory();
    void amclCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void mapReceived(const nav_msgs::OccupancyGridConstPtr& msg);
    void handleMapMessage(const nav_msgs::OccupancyGrid& msg);
    map_t* convertMap (const nav_msgs::OccupancyGrid& map_msg);
};

MaxVelocityMap* mvm;
void reset_params(int sig);
