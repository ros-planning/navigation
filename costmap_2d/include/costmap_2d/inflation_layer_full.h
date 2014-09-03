#ifndef inflation_layer_full_h_
#define inflation_layer_full_h_

#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <dynamic_reconfigure/server.h>
#include <queue>

#include "costmap_2d/InflationPluginFullConfig.h"

namespace costmap_2d
{

class InflationLayerFull : public Layer
{
public:
    InflationLayerFull();

    virtual ~InflationLayerFull()
    {
        if(dsrv_)
            delete dsrv_;
    }

    virtual void onInitialize();
    virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y);
    virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
    virtual void reset() { onInitialize(); }

protected:
    dynamic_reconfigure::Server<InflationPluginFullConfig>* dsrv_;
    void reconfigureCB(InflationPluginFullConfig& cfg, uint32_t level);

    double dilation_radius_;
    unsigned int cell_dilation_radius_;
};
}
#endif
