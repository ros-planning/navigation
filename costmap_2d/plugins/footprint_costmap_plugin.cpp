#include<costmap_2d/footprint_costmap_plugin.h>
#include<string>
#include<sstream>

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(common_costmap_plugins::FootprintCostmapPlugin, costmap_2d::CostmapPluginROS)

namespace common_costmap_plugins
{
    void FootprintCostmapPlugin::initialize(costmap_2d::LayeredCostmap* costmap, std::string name)
    {
        ros::NodeHandle nh("~/" + name);
        ros::NodeHandle g_nh;
        layered_costmap_ = costmap;
        name_ = name;
        footprint_.header.frame_id = layered_costmap_->getGlobalFrameID();
        current_ = false;
        got_footprint_ = false;
        
        std::string topic_param, topic;
        if(!nh.searchParam("footprint_topic", topic_param)){
            topic_param = "footprint_topic";
        }
        
        nh.param(topic_param, topic, std::string("footprint"));
        footprint_sub_ = nh.subscribe(topic, 1, &FootprintCostmapPlugin::footprint_cb, this);
        
        ros::Rate r(10);
        while(!got_footprint_ && g_nh.ok()){
            ros::spinOnce();
            r.sleep();
            ROS_INFO_THROTTLE(5.0, "Waiting for footprint.");
        }
        
        current_ = true;
    }
    
    void FootprintCostmapPlugin::footprint_cb(const geometry_msgs::Polygon& footprint) {
        footprint_spec_ = footprint;
        got_footprint_ = true;
    }

    void FootprintCostmapPlugin::update_bounds(double origin_x, double origin_y, double origin_yaw, double* min_x, double* min_y, double* max_x, double* max_y){
        //update transformed polygon 
        footprint_.header.stamp = ros::Time::now();
        footprint_.polygon.points.clear();
        double cos_th = cos(origin_yaw);
        double sin_th = sin(origin_yaw);
        for(unsigned int i = 0; i < footprint_spec_.points.size(); ++i){
          geometry_msgs::Point32 new_pt;
          new_pt.x = origin_x + (footprint_spec_.points[i].x * cos_th - footprint_spec_.points[i].y * sin_th);
          new_pt.y = origin_y + (footprint_spec_.points[i].x * sin_th + footprint_spec_.points[i].y * cos_th);
          footprint_.polygon.points.push_back(new_pt);
        }

        for(unsigned int i=0; i < footprint_.polygon.points.size(); i++){
            double px = footprint_.polygon.points[i].x, py = footprint_.polygon.points[i].y;
            *min_x = std::min(px, *min_x);
            *min_y = std::min(py, *min_y);
            *max_x = std::max(px, *max_x);
            *max_y = std::max(py, *max_y);
        }
    }
    
    void FootprintCostmapPlugin::update_costs(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j){
        master_grid.setConvexPolygonCost(footprint_.polygon, costmap_2d::FREE_SPACE);
    }
}

