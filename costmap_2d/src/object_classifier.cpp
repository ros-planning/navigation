#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>

sensor_msgs::LaserScan current_scan;
ros::Publisher pub_scan_result;

void sensorCallback(const sensor_msgs::LaserScan& msg)
{
    current_scan = msg;
    pub_scan_result.publish(current_scan);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "object_classifier");
    ros::NodeHandle n;
    ros::Subscriber sensor_data = n.subscribe("scan", 1, sensorCallback);
    ros::Publisher dynamic_object = n.advertise<sensor_msgs::LaserScan>("scan_dynamic", 1);
    pub_scan_result = n.advertise<sensor_msgs::LaserScan>("current_scan", 1);
    ros::Rate rate(15);

    while(ros::ok())
    {
        // if scanned point's value of area_info (key) is static_object
        // do nothing
        // else
        // publish as dynamic_object

        ros::spinOnce();
        rate.sleep();
    }
}
