#ifndef _DEPTH_3D_INTEGRATION_H_
#define _DEPTH_3D_INTEGRATION_H_

#include <ed/kinect/image_buffer.h>

// Point cloud
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class Depth3DIntegrator
{

public:

    Depth3DIntegrator();

    ~Depth3DIntegrator();

    bool initialize(const std::string& rgbd_topic, const std::string& map_frame);

    bool isInitialized() const { return initialized_; }

    bool update(pcl::PointCloud<pcl::PointXYZ>& cloud);

private:

    ImageBuffer image_buffer_;

    bool initialized_;

    // Params

    std::string map_frame_;

    double slope_threshold_;
    double min_distance_;
    double max_distance_;

    int num_samples_;

    int slope_window_size_;
};

#endif
