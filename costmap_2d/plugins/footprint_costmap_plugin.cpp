#include<costmap_2d/footprint_costmap_plugin.h>
#include<string>
#include<sstream>
#include<costmap_2d/footprint.h>

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(common_costmap_plugins::FootprintCostmapPlugin, costmap_2d::CostmapPluginROS)

namespace common_costmap_plugins
{
    void FootprintCostmapPlugin::initialize(costmap_2d::LayeredCostmap* costmap, std::string name)
    {
        ros::NodeHandle nh("~/" + name);
        ros::NodeHandle g_nh;
        layered_costmap_ = costmap;
        footprint_.header.frame_id = layered_costmap_->getGlobalFrameID();
        current_ = true;
        footprint_pub_ = nh.advertise<geometry_msgs::PolygonStamped>("robot_footprint", 1);

        //load the robot footprint from the parameter server if its available in the global namespace
        footprint_spec_ = loadRobotFootprint(nh);

        if(footprint_spec_.points.size() > 2){
            //now we need to compute the inscribed/circumscribed radius of the robot from the footprint specification
            inscribed_radius_ = std::numeric_limits<double>::max();
            circumscribed_radius_ = 0.0;
            circular_ = false;

            for(unsigned int i = 0; i < footprint_spec_.points.size() - 1; ++i){
                //check the distance from the robot center point to the first vertex
                double vertex_dist = distance(0.0, 0.0, footprint_spec_.points[i].x, footprint_spec_.points[i].y);
                double edge_dist = distanceToLine(0.0, 0.0, footprint_spec_.points[i].x, footprint_spec_.points[i].y, footprint_spec_.points[i+1].x, footprint_spec_.points[i+1].y);
                inscribed_radius_ = std::min(inscribed_radius_, std::min(vertex_dist, edge_dist));
                circumscribed_radius_ = std::max(circumscribed_radius_, std::max(vertex_dist, edge_dist));
            }

            //we also need to do the last vertex and the first vertex
            double vertex_dist = distance(0.0, 0.0, footprint_spec_.points.back().x, footprint_spec_.points.back().y);
            double edge_dist = distanceToLine(0.0, 0.0, footprint_spec_.points.back().x, footprint_spec_.points.back().y, footprint_spec_.points.front().x, footprint_spec_.points.front().y);
            inscribed_radius_ = std::min(inscribed_radius_, std::min(vertex_dist, edge_dist));
            circumscribed_radius_ = std::max(circumscribed_radius_, std::max(vertex_dist, edge_dist));
        }else{
            nh.param("robot_radius", inscribed_radius_, 0.46);
            circumscribed_radius_ = inscribed_radius_;
            circular_ = true;
        }
    }


    void FootprintCostmapPlugin::update_bounds(double origin_x, double origin_y, double origin_yaw, double* min_x, double* min_y, double* max_x, double* max_y){
        //update transformed polygon 
        footprint_.header.stamp = ros::Time::now();
        footprint_.polygon.points.clear();
        if(circular_){
          double angle = 0;
          double step = 2 * M_PI / 72;
          while(angle < 2 * M_PI){
            geometry_msgs::Point32 pt;
            pt.x = inscribed_radius_ * cos(angle) + origin_x;
            pt.y = inscribed_radius_ * sin(angle) + origin_y;
            pt.z = 0.0;
            footprint_.polygon.points.push_back(pt);
            angle += step;
          }
        }else{
            double cos_th = cos(origin_yaw);
            double sin_th = sin(origin_yaw);
            for(unsigned int i = 0; i < footprint_spec_.points.size(); ++i){
              geometry_msgs::Point32 new_pt;
              new_pt.x = origin_x + (footprint_spec_.points[i].x * cos_th - footprint_spec_.points[i].y * sin_th);
              new_pt.y = origin_y + (footprint_spec_.points[i].x * sin_th + footprint_spec_.points[i].y * cos_th);
              footprint_.polygon.points.push_back(new_pt);
            }
        }
        for(unsigned int i=0; i < footprint_.polygon.points.size(); i++){
            double px = footprint_.polygon.points[i].x, py = footprint_.polygon.points[i].y;
            min_x_ = std::min(px, min_x_);
            min_y_ = std::min(py, min_y_);
            max_x_ = std::max(px, max_x_);
            max_y_ = std::max(py, max_y_);
        }
        
        *min_x = std::min(min_x_, *min_x);
        *min_y = std::min(min_y_, *min_y);
        *max_x = std::max(max_x_, *max_x);
        *max_y = std::max(max_y_, *max_y);
        ROS_DEBUG("Publishing footprint");
        footprint_pub_.publish(footprint_);
    }
    
    void FootprintCostmapPlugin::update_costs(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j){
        master_grid.setConvexPolygonCost(footprint_.polygon, costmap_2d::FREE_SPACE);
    }
}

