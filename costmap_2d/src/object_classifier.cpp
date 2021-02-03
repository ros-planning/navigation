#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <math.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>

#define MAP_FRAME       "map"           // name of /map frame
#define STATIC_OBJECT   100             // value of key from vector map for areas where static objects are expected
#define MAX_DIST        61.5            // TODO: distance output from LiDAR when no object is detected
#define RATE            40              // the rate at which scan data are processed

bool is_ready = false;
sensor_msgs::LaserScan current_scan;
nav_msgs::OccupancyGrid vector_map_msg;
geometry_msgs::TransformStamped transformStamped;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    // Constant values that do not change. Need to be run only once
    current_scan.angle_min = msg->angle_min;
    current_scan.angle_max = msg->angle_max;
    current_scan.angle_increment = msg->angle_increment;
    current_scan.scan_time = msg->scan_time;
    current_scan.range_max = msg->range_max;
    current_scan.range_min = msg->range_min;

    current_scan.header = msg->header;
    current_scan.ranges = msg->ranges;
    current_scan.intensities = msg->intensities;
}

void vectormapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    // Constant values that do not change. Need to be run only once
    vector_map_msg.info.resolution = msg->info.resolution;
    vector_map_msg.info.height = msg->info.height;
    vector_map_msg.info.width = msg->info.width;
    vector_map_msg.info.origin.position.x = msg->info.origin.position.x;
    vector_map_msg.info.origin.position.y = msg->info.origin.position.y;
    is_ready = true;

    vector_map_msg.data = msg->data;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "obstacle_classifier");
    ros::NodeHandle n;
    ros::Subscriber sub_scan = n.subscribe("scan", 1, scanCallback);
    ros::Subscriber sub_vectormap = n.subscribe("vector_map", 1, vectormapCallback);
    ros::Publisher pub_filtered_scan = n.advertise<sensor_msgs::LaserScan>("scan_dynamic", 1);
    ros::Rate rate(RATE);

    float angle;
    int i, j, k;
    double x, y;
    std::string scan_frame_name;
    n.param<std::string>("scan_frame_name", scan_frame_name, "laser");
    geometry_msgs::Pose v;
    geometry_msgs::Pose tv;
    sensor_msgs::LaserScan filtered_dynamic;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    while(ros::ok())
    {
        i = 0;
        j = 0;
        k = 0;

        // Constant values that do not change. Need to be run only once
        filtered_dynamic.range_max = current_scan.range_max;
        filtered_dynamic.range_min = current_scan.range_min;
        filtered_dynamic.angle_max = current_scan.angle_max;
        filtered_dynamic.angle_min = current_scan.angle_min;
        filtered_dynamic.angle_increment = current_scan.angle_increment;

        filtered_dynamic.header = current_scan.header;
        filtered_dynamic.ranges = current_scan.ranges;
        filtered_dynamic.intensities = current_scan.intensities;

        try
        {
            ros::Time now = filtered_dynamic.header.stamp;
            transformStamped = tfBuffer.lookupTransform(MAP_FRAME, scan_frame_name, now, ros::Duration(0.5));
        }
        catch(tf::TransformException& ex)
        {
            ROS_INFO("%s", ex.what());
        }

        if(is_ready)
        {
            for(angle = current_scan.angle_min; angle < current_scan.angle_max; angle += current_scan.angle_increment)
            {                
                v.position.x = filtered_dynamic.ranges[i] * cos(angle);
                v.position.y = filtered_dynamic.ranges[i] * sin(angle);
                v.orientation.w = 1.0;

                try
                {
                    tf2::doTransform(v, tv, transformStamped);
                    x = tv.position.x;
                    y = tv.position.y;
                }
                catch(tf::TransformException& ex)
                {
                    ROS_INFO("%s", ex.what());
                    break;
                }
                
                // Transform 2D coordinate system from the map to 1D
                j = floor((x - vector_map_msg.info.origin.position.x) / vector_map_msg.info.resolution) + floor((y - vector_map_msg.info.origin.position.y) / vector_map_msg.info.resolution) * vector_map_msg.info.width;
                
                // Anything in the unoccupied and unknown zone is considered to be dynamic objects
                // Anything in the static object zone will be filtered out
                if(vector_map_msg.data[j] == STATIC_OBJECT)
                {
                    filtered_dynamic.ranges[i] = MAX_DIST;
                    k++;
                }
                i++; 
            }
            
            ROS_INFO("Number of static objects filtered out: %i", k);
            pub_filtered_scan.publish(filtered_dynamic);
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
