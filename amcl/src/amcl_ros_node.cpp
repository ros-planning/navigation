#include "amcl_ros.h"

using namespace amcl;
#define USAGE "USAGE: amcl"

boost::shared_ptr<AmclRosNode> amcl_node_ptr;

void sigintHandler(int sig)
{
  // Delete latest pose as we're shutting down.
  amcl_node_ptr->deletePoseFromServer();
  ros::shutdown();
}

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "amcl");
  ros::NodeHandle nh;

  // Override default sigint handler
  signal(SIGINT, sigintHandler);

  // Make our node available to sigintHandler
  amcl_node_ptr.reset(new AmclRosNode());

  if (argc == 1)
  {
    // run using ROS input
    ros::spin();
  }

  // Without this, our boost locks are not shut down nicely
  amcl_node_ptr.reset();

  // To quote Morgan, Hooray!
  return(0);
}