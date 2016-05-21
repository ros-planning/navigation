#include "costmap_2d/depth_3d_integrator.h"

#include <rgbd/Image.h>
#include <rgbd/View.h>

#include <opencv2/highgui/highgui.hpp>

#include <geolib/ros/tf_conversions.h>

// ----------------------------------------------------------------------------------------------------

// Decomposes 'pose' into a (X, Y, YAW) and (Z, ROLL, PITCH) component
void decomposePose(const geo::Pose3D& pose, geo::Pose3D& pose_xya, geo::Pose3D& pose_zrp)
{
    tf::Matrix3x3 m;
    geo::convert(pose.R, m);

    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    pose_xya.R.setRPY(0, 0, yaw);
    pose_xya.t = geo::Vec3(pose.t.x, pose.t.y, 0);

    pose_zrp = pose_xya.inverse() * pose;
}

// ----------------------------------------------------------------------------------------------------

Depth3DIntegrator::Depth3DIntegrator() : initialized_(false)
{
}

// ----------------------------------------------------------------------------------------------------

Depth3DIntegrator::~Depth3DIntegrator()
{
}

// ----------------------------------------------------------------------------------------------------

bool Depth3DIntegrator::initialize(const std::string& rgbd_topic, const std::string& map_frame)
{
    image_buffer_.initialize(rgbd_topic);

    num_samples_ = 640;
    slope_threshold_ = 1;
    slope_window_size_ = 30;
    min_distance_ = 0.4;
    max_distance_ = 2.0;

    map_frame_ = map_frame;

    initialized_ = true;

    return true;
}

// ----------------------------------------------------------------------------------------------------

bool Depth3DIntegrator::update(pcl::PointCloud<pcl::PointXYZ>& cloud)
{
    if (!isInitialized())
        return false;

    cloud.clear();

    bool visualize = false;

    rgbd::ImageConstPtr image;

    geo::Pose3D sensor_pose;
    if (!image_buffer_.nextImage(map_frame_, image, sensor_pose))
        return false;

    // - - - - - - - - - - - - - - - - - - - - - - -

    geo::Pose3D sensor_pose_xya;
    geo::Pose3D sensor_pose_zrp;
    decomposePose(sensor_pose, sensor_pose_xya, sensor_pose_zrp);

    // - - - - - - - - - - - - - - - - - - - - - - -

    cv::Mat depth = image->getDepthImage();

    int width = depth.cols;
    int height = depth.rows;

    rgbd::View view(*image, width);
    const geo::DepthCamera& rasterizer = view.getRasterizer();

    cv::Mat obstacle_map, bla;
    if (visualize)
    {
        obstacle_map = cv::Mat(500, 500, CV_32FC1, 0.0);
        bla = depth.clone();
    }

    cv::Mat buffer(height, 1, CV_64FC4, 0.0);
    std::vector<geo::Vector3> p_floors(height);

    double x_step = 1;
    if (num_samples_ > 0 && num_samples_ < depth.cols)
        x_step = (double)depth.cols / num_samples_;

    for(double x = 0; x < width; x += x_step)
    {
        geo::Vector3 p_floor_closest(1e6, 1e6, 0);

        buffer.at<cv::Vec4d>(0, 0) = cv::Vec4d(0, 0, 0, 0);

        for(int y = 1; y < height; ++y)
        {
            float d = depth.at<float>(y, x);
            if (d == 0 || d != d)
            {
                buffer.at<cv::Vec4d>(y, 0) = buffer.at<cv::Vec4d>(y - 1, 0);
                continue;
            }

            geo::Vector3 p_sensor = rasterizer.project2Dto3D(x, y) * d;
            p_floors[y] = sensor_pose_zrp * p_sensor;
            const geo::Vector3& p_floor = p_floors[y];

            if (p_floor.z > 0.2)
            {
                if (p_floor.y < p_floor_closest.y)
                    p_floor_closest = p_floor;

                if (visualize)
                {
                    cv::Point2i p(p_floor.x * 50 + obstacle_map.rows / 2,
                                  obstacle_map.cols / 2 - p_floor.y * 50);
                    if (p.x >= 0 && p.x < obstacle_map.cols && p.y >= 0 && p.y < obstacle_map.rows)
                        obstacle_map.at<float>(p) = 0.5;
                }
            }

            //            x_sum_ += x;
            //            y_sum_ += y;
            //            xy_sum_ += x * y;
            //            x2_sum_ += x * x;

            buffer.at<cv::Vec4d>(y, 0) = buffer.at<cv::Vec4d>(y - 1, 0)
                        + cv::Vec4d(p_floor.y, p_floor.z, p_floor.y * p_floor.z, p_floor.y * p_floor.y);
        }

        for(int y = slope_window_size_; y < height - slope_window_size_; ++y)
        {
            float d = depth.at<float>(y, x);
            if (d == 0 || d != d)
                continue;

            const geo::Vector3& p_floor = p_floors[y];

            if (p_floor.z < 0.2)
            {
//                const cv::Vec4d& bm = buffer.at<cv::Vec4d>(y, 0);

//                double c1, s1;
//                cv::Vec4d b1 = bm - buffer.at<cv::Vec4d>(y - slope_window_size_, 0);
//                s1 = (slope_window_size_ * b1[2] - b1[0] * b1[1]) / (slope_window_size_ * b1[3] - b1[0] * b1[0]);
                //            c1 = b1[1] / slope_window_size_ - s1 * (b1[0] / slope_window_size_);

                double c2, s2;
                cv::Vec4d b2 = buffer.at<cv::Vec4d>(y + slope_window_size_ / 2, 0)
                        - buffer.at<cv::Vec4d>(y - slope_window_size_ / 2, 0);
                s2 = (slope_window_size_ * b2[2] - b2[0] * b2[1]) / (slope_window_size_ * b2[3] - b2[0] * b2[0]);
                //            c2 = b2[1] / slope_window_size_ - s1 * (b2[0] / slope_window_size_);

//                if (std::abs(s1) > 1 && std::abs(s2) < 1)
                if (std::abs(s2) > slope_threshold_)
                {
                    if (p_floor.y < p_floor_closest.y)
                        p_floor_closest = p_floor;

                    if (visualize)
                    {
                        bla.at<float>(y, x) = 10;
                        cv::Point2i p(p_floor.x * 50 + obstacle_map.rows / 2,
                                      obstacle_map.cols / 2 - p_floor.y * 50);
                        if (p.x >= 0 && p.x < obstacle_map.cols && p.y >= 0 && p.y < obstacle_map.rows)
                            obstacle_map.at<float>(p) = 1;
                    }
                }
            }
        }

        if (p_floor_closest.y < max_distance_ && p_floor_closest.y > min_distance_)
        {
            geo::Vec3 p_map = sensor_pose_xya * p_floor_closest;
            cloud.push_back(pcl::PointXYZ(p_map.x, p_map.y, p_map.z));
        }
    }

    if (visualize)
    {
        cv::imshow("bla", bla / 10);
        cv::imshow("obstacle map", obstacle_map);
        cv::waitKey(3);
    }

    return true;
}

// ----------------------------------------------------------------------------------------------------

