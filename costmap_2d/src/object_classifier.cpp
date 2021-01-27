/*
#include <ros/ros.h>
#include <math.h>
#include <string>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

sensor_msgs::LaserScan current_scan;
ros::Publisher pub_scan_result;
std::string = scan_point_area;

void sensorCallback(const sensor_msgs::LaserScan& msg)
{
    current_scan = msg;
    pub_scan_result.publish(current_scan);
}

void robotAreaCallback(const std_msgs::String& msg)
{
    scan_point_area.assign(msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "object_classifier");
    ros::NodeHandle n;
    ros::Subscriber sensor_data = n.subscribe("scan", 1, sensorCallback);                       // Subscribe to LiDAR's scan data
    ros::Subscriber from_vectormap = n.subscribe("robot_area", 1, robotAreaCallback);           // Subscribe to robot_area's information. TODO: remap in launch file?
    ros::Publisher dynamic_object = n.advertise<sensor_msgs::LaserScan>("scan_dynamic", 1);     // Publish to a new topic
    ros::Publisher to_vectormap = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 1);   // Publish to /amcl_pose. TODO: remap in launch file?
    pub_scan_result = n.advertise<sensor_msgs::LaserScan>("current_scan", 1);
    ros::Rate rate(5);

    double i;
    int j;

    // For storing filtered scan data (only dynamic obstacles)
    sensor_msgs::LaserScan scan_dynamic;
    scan_dynamic = current_scan;

    // For sending to VectorMap
    geometry_msgs::PoseWithCovarianceStamped scan_pose;
    scan_pose.pose.position.z = 0.0;
    scan_pose.header = current_scan.header;
    tf2:Quaternion q;
    q.setRPY(0, 0, 0);
    q.normalize();
    scan_pose.pose.orientation.x = q[0];
    scan_pose.pose.orientation.y = q[1];
    scan_pose.pose.orientation.z = q[2];
    scan_pose.pose.orientation.w = q[3];
    scan_pose.pose.covariance = ;   //TODO: need to assign

    while(ros::ok())
    {
        j = 0;
        
        for(i = current_scan.angle_min; i <= current_scan.angle_max; i += current_scan.angle_increment)
        {
            scan_pose.pose.position.x = current_scan.ranges[j] * cos(i); //TODO: check if i is in rad or deg
            scan_pose.pose.position.y = current_scan.ranges[j] * sin(i);

            // if scanned point's value of area_info (key) is static_object
            // do nothing or set the distance to max_distance?
            // else
            // register as dynamic_object (simply copy the distance)
            to_vectormap.publish(scan_pose);
            if (scan_point_area.find("static_object") != string::npos)  // TODO: use #define?
            {
                scan_dynamic.ranges[j] = scan_dynamic.range_max;
            }
            else
            {
                scan_dynamic.ranges[j] = current_scan.ranges[j];
            }

            j++;
        }

        dynamic_object.publish(scan_dynamic);

        ros::spinOnce();
        rate.sleep();
    }
}
*/