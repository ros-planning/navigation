#include <costmap_2d/inflation_layer_full.h>

#include <costmap_2d/costmap_math.h>
#include <costmap_2d/footprint.h>
#include <pluginlib/class_list_macros.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

PLUGINLIB_EXPORT_CLASS(costmap_2d::InflationLayerFull, costmap_2d::Layer)

namespace costmap_2d
{

InflationLayerFull::InflationLayerFull() :
    dsrv_(NULL),
    cell_dilation_radius_(0)
{
}

void InflationLayerFull::onInitialize()
{
    dynamic_reconfigure::Server<InflationPluginFullConfig>::CallbackType cb = boost::bind(&InflationLayerFull::reconfigureCB, this, _1, _2);

    if(dsrv_ != NULL){
        dsrv_->clearCallback();
        dsrv_->setCallback(cb);
    }
    else
    {
        dsrv_ = new dynamic_reconfigure::Server<InflationPluginFullConfig>(ros::NodeHandle("~/" + name_));
        dsrv_->setCallback(cb);
    }
}

void InflationLayerFull::reconfigureCB(InflationPluginFullConfig& cfg, uint32_t level)
{
    dilation_radius_ = cfg.dilation_radius;
    enabled_ = cfg.enabled;
}

void InflationLayerFull::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y)
{
    *min_x-=dilation_radius_;
    *min_y-=dilation_radius_;
    *max_x+=dilation_radius_;
    *max_y+=dilation_radius_;
}

void InflationLayerFull::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
    if (!enabled_)
        return;

    if (cell_dilation_radius_ == 0)
    {
        cell_dilation_radius_ = dilation_radius_ / layered_costmap_->getCostmap()->getResolution();
        return;
    }

    ros::Time t_start = ros::Time::now();
    cv::Mat obstacle_map(max_j - min_j, max_i - min_i, CV_8U);
    for (unsigned int i = 0; i < obstacle_map.cols; ++i)
    {
        for (unsigned int j = 0; j < obstacle_map.rows; ++j)
        {
            if (master_grid.getCost(min_i + i, min_j + j) < costmap_2d::NO_INFORMATION)
                obstacle_map.at<unsigned char>(j, i) = master_grid.getCost(min_i + i, min_j + j);
            else
                obstacle_map.at<unsigned char>(j, i) = 0;
        }
    }

    //! - - - - - - OpenCV Filter 2D - - - - - - -

    //! Dilation
    int dilate_size = cell_dilation_radius_ * 2 + 1;

    cv::Mat dilated_obstacle_map;
    cv::Mat dilation_element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(dilate_size, dilate_size));
    cv::dilate(obstacle_map, dilated_obstacle_map, dilation_element);

    //! - - - - - END OpenCV Filter 2D - - - - - -

    for (unsigned int i = 0; i < obstacle_map.cols; ++i)
    {
        for (unsigned int j = 0; j < obstacle_map.rows; ++j)
        {
            unsigned char original_map_cost = master_grid.getCost(min_i + i, min_j + j);
            unsigned char dilated_obstacle_map_cost = dilated_obstacle_map.at<unsigned char>(j, i);

            //! -1 (ROS CONVENTION INSCRIBED CELL)
            if (dilated_obstacle_map_cost > 0)
                dilated_obstacle_map_cost-=1;

            unsigned char cost = 0;

            if (dilated_obstacle_map_cost > 0)
                cost = dilated_obstacle_map_cost;
            else
                cost = std::max(original_map_cost, dilated_obstacle_map_cost);

            master_grid.setCost(min_i + i, min_j + j, cost);
        }
    }
}

} // end namespace costmap_2d
