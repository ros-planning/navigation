/*
#include <ros/ros.h>
#include <math.h>
#include <string.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#define STATIC_AREA "obstacle"     // value of key from vector map for areas where static objects are expected
#define MAX_DIST 61.5                   // TODO: distance output from LiDAR when no object is detected
// #define SCAN_NUM 1440                   // TODO: number of scan points in one scan

sensor_msgs::LaserScan current_scan;
geometry_msgs::PoseWithCovarianceStamped scan_point_pose;
bool is_in_static = false;
std::string check = "obstacle";

void scanCallBack(const sensor_msgs::LaserScan& msg)
{
    // Constant values that do not change. Need to be run only once
    current_scan.angle_min = msg.angle_min;
    current_scan.angle_max = msg.angle_max;
    current_scan.angle_increment = msg.angle_increment;
    current_scan.scan_time = msg.scan_time;
    current_scan.range_max = msg.range_max;
    current_scan.range_min = msg.range_min;

    scan_point_pose.header = msg.header;
    current_scan.ranges = msg.ranges;
}

void areaCallBack(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("%s", msg->data.c_str());
    if(msg->data.find(check) != std::string::npos)
        is_in_static = true;
    else
        is_in_static = false;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "object_classifier");
    ros::NodeHandle n;
    ros::Subscriber scan_data = n.subscribe("scan", 1, scanCallBack);
    ros::Subscriber scanned_area = n.subscribe("scanned_area", 1, areaCallBack);
    ros::Publisher pub_scan_point_pose = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("scan_pose", 1);
    ros::Publisher pub_filtered_scan = n.advertise<sensor_msgs::LaserScan>("scan_dynamic", 1);
    ros::Rate rate(0.1);

    float i;
    int j, k;

    sensor_msgs::LaserScan filtered_scan;
    filtered_scan.intensities = {0};
    
    scan_point_pose.pose.pose.position.z = 0.0; //TODO: check if zero is okay. Need to adjust to the actual height ~= 0.23m?
    // Identity quaternion (1, 0, 0, 0) means no rotation
    scan_point_pose.pose.pose.orientation.x = 1;
    scan_point_pose.pose.pose.orientation.y = 0;
    scan_point_pose.pose.pose.orientation.z = 0;
    scan_point_pose.pose.pose.orientation.w = 0;
    // Make a zero covariance matrix since scan points' positions are independent of other variables
    scan_point_pose.pose.covariance = {0};  // TODO: NOT effective for initializing a zero matrix 

    while(ros::ok())
    {
        j = 0;
        k = 0;      // TODO: delete

        // TODO: run these parts only once?
        filtered_scan.angle_max = current_scan.angle_max;
        filtered_scan.angle_min = current_scan.angle_min;
        filtered_scan.angle_increment = current_scan.angle_increment;
        filtered_scan.range_max = current_scan.range_max;
        filtered_scan.range_min = current_scan.range_min;

        filtered_scan.header = scan_point_pose.header;
        filtered_scan.ranges = current_scan.ranges;

        for(i = current_scan.angle_min; i < current_scan.angle_max; i += current_scan.angle_increment)
        {
            // TODO: transform to robot frame?
            scan_point_pose.pose.pose.position.x = current_scan.ranges[j] * cos(i);
            scan_point_pose.pose.pose.position.y = current_scan.ranges[j] * sin(i);

            pub_scan_point_pose.publish(scan_point_pose);
            ros::spinOnce();

            if(is_in_static)
            {
                filtered_scan.ranges[j] = MAX_DIST;
                k++;                                    // TODO: delete. Counter for number of static objects
            }

            if(j % 100 == 0)
                ROS_INFO("%i", j);

            j++;
        }

        pub_filtered_scan.publish(filtered_scan);
        ROS_INFO("Number of static objects: %i\n", k);

        ros::spinOnce();
        rate.sleep();
    }
}
*/