#include <ros/ros.h>
#include <math.h>
#include <string.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#define STATIC_AREA "static_object"     // value of key from vector map for areas where static objects are expected
#define MAX_DIST 61.5                   // TODO: distance output from LiDAR when no object is detected
#define SCAN_NUM 1440                   // TODO: number of scan points in one scan

sensor_msgs::LaserScan current_scan;
geometry_msgs::PoseWithCovarianceStamped scan_point_pose;
bool is_in_static = false;

void scanCallBack(const sensor_msgs::LaserScan& msg)
{
    // Constant values that do not change. Need to be run only once
    scan_point_pose.header = msg.header;
    current_scan.angle_min = msg.angle_min;
    current_scan.angle_max = msg.angle_max;
    current_scan.angle_increment = msg.angle_increment;
    current_scan.scan_time = msg.scan_time;

    current_scan.ranges = msg.ranges;
}

void areaCallBack(const std_msgs::String& msg)
{
    if (msg.data.find(STATIC_AREA) != std::string::npos)
    {
        ROS_INFO("static area!");   // TODO: delete
        is_in_static = true;
    }
    else
    {
        ROS_INFO("Not static!");    // TODO: delete
        is_in_static = false;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "obstacle_classifer_scan");
    ros::NodeHandle n;
    ros::Subscriber scan_data = n.subscribe("scan", 1, scanCallBack);
    ros::Subscriber scanned_area = n.subscribe("robot_area", 1, areaCallBack);
    ros::Publisher pub_scan_point_pose = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 1);
    ros::Rate rate(1);

    float i;
    int j;

    sensor_msgs::LaserScan filtered_scan;
    
    scan_point_pose.pose.pose.position.z = 0.0; //TODO: check if zero is okay. Need to adjust to the actual height ~= 0.23m?
    // Identity quaternion (1, 0, 0, 0) means no rotation
    scan_point_pose.pose.pose.orientation.x = 1;
    scan_point_pose.pose.pose.orientation.y = 0;
    scan_point_pose.pose.pose.orientation.z = 0;
    scan_point_pose.pose.pose.orientation.w = 0;
    // Make a zero covariance matrix since scan points' positions are independent of other variables
    scan_point_pose.pose.covariance = {0};  // TODO: check if this is effective for initializing a zero matrix

    while(ros::ok())
    {
        j = 0;
        filtered_scan.ranges = current_scan.ranges;

        for(i = current_scan.angle_min; i < current_scan.angle_max; i += current_scan.angle_increment)
        {
            scan_point_pose.pose.pose.position.x = current_scan.ranges[j] * cos(i);
            scan_point_pose.pose.pose.position.y = current_scan.ranges[j] * sin(i);

            pub_scan_point_pose.publish(scan_point_pose);
            ros::spinOnce();

            if(is_in_static)
                filtered_scan.ranges[j] = MAX_DIST;

            j++;
        }

        ros::spinOnce();
        rate.sleep();
    }
}