#include <ros/ros.h>
#include <tf/tf.h>
#include <tf2/buffer_core.h>
#include <math.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/TransformStamped.h>

#define UNOCCUPIED 0            // value of key from vector map for areas where static objects are expected
#define MAX_DIST 61.5           // TODO: distance output from LiDAR when no object is detected
#define RATE 40                 // the rate at which scan data are processed

sensor_msgs::LaserScan current_scan;
nav_msgs::OccupancyGrid vector_map_msg;
bool is_ready = false;

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

    vector_map_msg.data = msg->data;

    is_ready = true;
    ROS_INFO("I am called %i", vector_map_msg.data[0]);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "obstacle_classifier");
    ros::NodeHandle n;
    ros::Subscriber sub_scan = n.subscribe("scan", 1, scanCallback);
    ros::Subscriber sub_vectormap = n.subscribe("vector_map", 1, vectormapCallback);
    ros::Publisher pub_filtered_scan = n.advertise<sensor_msgs::LaserScan>("scan_dynamic", 1);
    ros::Rate rate(RATE);

    sensor_msgs::LaserScan filtered_dynamic;
    // vector_map_msg.data = {0};
    float angle;
    int i, j, k;
    double x, y;

    while(ros::ok())
    {
        i = 0;
        j = 0;
        k = 0;

        // TODO: need to run only once
        filtered_dynamic.range_max = current_scan.range_max;
        filtered_dynamic.range_min = current_scan.range_min;
        filtered_dynamic.angle_max = current_scan.angle_max;
        filtered_dynamic.angle_min = current_scan.angle_min;
        filtered_dynamic.angle_increment = current_scan.angle_increment;

        filtered_dynamic.header = current_scan.header;
        filtered_dynamic.ranges = current_scan.ranges;
        filtered_dynamic.intensities = current_scan.intensities;

        if(is_ready)
        {
            for(angle = current_scan.angle_min; angle < current_scan.angle_max; angle += current_scan.angle_increment)
            {
                /* tf transform from scan frame to map frame
                tf2::BufferCore buffer_core;
                geometry_msgs::TransformStamped ts1;
                ts1.header.frame_id = "map";
                ts1.child_frame_id = "hokuyo_laser";        // TODO: fix
                ts1.transform.translation.x = filtered_dynamic.ranges[i] * cos(angle);
                ts1.transform.translation.y = filtered_dynamic.ranges[i] * sin(angle);
                ts1.transform.translation.z = 0;
                ts1.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
                buffer_core.setTransform(ts1, "default_authority", true);
                geometry_msgs::TransformStamped ts_lookup;
                ts_lookup = buffer_core.lookupTransform("map", "hokuyo_link", ros::Time(0));

                x = ts_lookup.transform.translation.x;
                y = ts_lookup.transform.translation.y;
                */
                
                x = filtered_dynamic.ranges[i] * cos(angle);
                y = filtered_dynamic.ranges[i] * sin(angle);

                // vector_map.data is a 1D array.
                // We need to transform 2D coordinate system of the map to 1D
                j = floor((x - vector_map_msg.info.origin.position.x) / vector_map_msg.info.resolution) + floor((y - vector_map_msg.info.origin.position.y) / vector_map_msg.info.resolution) * vector_map_msg.info.width;
                
                // Not unoccupied means either unknown or obstacle. Anything in the "not unoccupied" zone is considered to be dynamic objects
                if(vector_map_msg.data[j] != UNOCCUPIED)
                {
                    filtered_dynamic.ranges[i] = MAX_DIST;
                    k++;
                }

                i++;
                if(i % 100 == 0)
                    ROS_INFO("Process running %i", i); 
            }
            
            ROS_INFO("Number of dynamic objects scanned: %i", k);
            pub_filtered_scan.publish(filtered_dynamic);
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}