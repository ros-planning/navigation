#include <max_velocity_map/max_velocity_map.h>
#include <algorithm>
#include <signal.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "max_velocity_map_node");
  mvm = new MaxVelocityMap();
  signal(SIGINT, reset_params);  // reset rosparam when Ctrl-C is pressed
  ros::spin();
  return(0);
}

MaxVelocityMap::MaxVelocityMap()
{
  // specify local planner
  ros::param::get("~local_planner", local_planner_);
  ROS_INFO("%s", local_planner_.c_str());
  // get rosparam
  if(nh_.hasParam(local_planner_ + "/max_vel_x")) {
    nh_.getParam(local_planner_ + "/max_vel_x", max_vel_x_initial_);
    ROS_INFO("max_vel_x_initial_: %f", max_vel_x_initial_);
  }
  if(nh_.hasParam(local_planner_ + "/min_vel_x")) {
    nh_.getParam(local_planner_ + "/min_vel_x", min_vel_x_initial_);
    ROS_INFO("min_vel_x_initial_: %f", min_vel_x_initial_);
  }
  if(nh_.hasParam(local_planner_ + "/max_vel_y")) {
    nh_.getParam(local_planner_ + "/max_vel_y", max_vel_y_initial_);
    ROS_INFO("max_vel_y_initial_: %f", max_vel_y_initial_);
  }
  if(nh_.hasParam(local_planner_ + "/min_vel_x")) {
    nh_.getParam(local_planner_ + "/min_vel_y", min_vel_y_initial_);
    ROS_INFO("min_vel_y_initial_: %f", min_vel_y_initial_);
  }
  // register subscriber and publisher
  amcl_sub_ = nh_.subscribe("amcl_pose", 100, &MaxVelocityMap::amclCallback, this);
  map_sub_ = nh_.subscribe("max_velocity_map", 1, &MaxVelocityMap::mapReceived, this);
  max_vel_x_pub_ = nh_.advertise<std_msgs::Float32>("max_vel_x", 1000);
  min_vel_x_pub_ = nh_.advertise<std_msgs::Float32>("min_vel_x", 1000);
  max_vel_y_pub_ = nh_.advertise<std_msgs::Float32>("max_vel_y", 1000);
  min_vel_y_pub_ = nh_.advertise<std_msgs::Float32>("min_vel_y", 1000);
}

MaxVelocityMap::~MaxVelocityMap()
{
  freeMapDependentMemory();
  // restore max_vel parameters
  conf_.doubles.clear();
  double_param_.name = "max_vel_x";
  double_param_.value = max_vel_x_initial_;
  conf_.doubles.push_back(double_param_);
  double_param_.name = "min_vel_x";
  double_param_.value = min_vel_x_initial_;
  conf_.doubles.push_back(double_param_);
  double_param_.name = "max_vel_y";
  double_param_.value = max_vel_y_initial_;
  conf_.doubles.push_back(double_param_);
  double_param_.name = "min_vel_y";
  double_param_.value = min_vel_y_initial_;
  conf_.doubles.push_back(double_param_);
  srv_req_.config = conf_;
  ros::service::call(local_planner_ + "/set_parameters", srv_req_, srv_resp_);
}

void MaxVelocityMap::freeMapDependentMemory()
{
  if( map_ != NULL ) {
    map_free( map_ );
    map_ = NULL;
  }
}

void MaxVelocityMap::amclCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  float pos_x = msg->pose.pose.position.x;
  float pos_y = msg->pose.pose.position.y;
  float pos_z = msg->pose.pose.position.z;

  if( map_ != NULL ) {
    // get map pixel value of current robot position
    int index_x = (int)((pos_x - map_->origin_x) / map_->scale);
    int index_y = (int)((pos_y - map_->origin_y) / map_->scale);
    // update max_vel parameters
    float max_vel_ratio = std::max(map_->cells[index_x + index_y * map_->size_x].occ_state / 255.0, 0.2); // avoid too slow movement
    std_msgs::Float32 msg;
    // max_vel_x
    conf_.doubles.clear();
    double_param_.name = "max_vel_x";
    double_param_.value = max_vel_x_initial_ * max_vel_ratio;
    conf_.doubles.push_back(double_param_);
    msg.data = double_param_.value;
    max_vel_x_pub_.publish(msg);
    // min_vel_x
    double_param_.name = "min_vel_x";
    double_param_.value = min_vel_x_initial_ * max_vel_ratio;
    conf_.doubles.push_back(double_param_);
    msg.data = double_param_.value;
    min_vel_x_pub_.publish(msg);
    // max_vel_y
    double_param_.name = "max_vel_y";
    double_param_.value = max_vel_y_initial_ * max_vel_ratio;
    conf_.doubles.push_back(double_param_);
    msg.data = double_param_.value;
    max_vel_y_pub_.publish(msg);
    // min_vel_y
    double_param_.name = "min_vel_y";
    double_param_.value = min_vel_y_initial_ * max_vel_ratio;
    conf_.doubles.push_back(double_param_);
    msg.data = double_param_.value;
    min_vel_y_pub_.publish(msg);
    // call dynamic reconfigure
    srv_req_.config = conf_;
    ros::service::call(local_planner_ + "/set_parameters", srv_req_, srv_resp_);
  }
}

void MaxVelocityMap::mapReceived(const nav_msgs::OccupancyGridConstPtr& msg)
{
  handleMapMessage( *msg );
}

void MaxVelocityMap::handleMapMessage(const nav_msgs::OccupancyGrid& msg)
{
  ROS_INFO("Received a %d X %d map @ %.3f m/pix\n",
           msg.info.width,
           msg.info.height,
           msg.info.resolution);

  freeMapDependentMemory();

  map_ = convertMap(msg);
}

map_t* MaxVelocityMap::convertMap( const nav_msgs::OccupancyGrid& map_msg )
{
  map_t* map = map_alloc();
  ROS_ASSERT(map);

  map->size_x = map_msg.info.width;
  map->size_y = map_msg.info.height;
  map->scale = map_msg.info.resolution;
  map->origin_x = map_msg.info.origin.position.x;
  map->origin_y = map_msg.info.origin.position.y;
  // convert map data
  map->cells = (map_cell_t*)malloc(sizeof(map_cell_t)*map->size_x*map->size_y);
  ROS_ASSERT(map->cells);
  for(int i=0;i<map->size_x * map->size_y;i++)
  {
    map->cells[i].occ_state = (unsigned char)map_msg.data[i]; // black: 0, white: 255
  }

  return map;
}

void reset_params(int sig)
{
  delete mvm; // reset rosparam
  ros::shutdown();
}
