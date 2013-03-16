#include<costmap_2d/inflation_costmap_plugin.h>
#include<costmap_2d/costmap_math.h>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(common_costmap_plugins::InflationCostmapPlugin, costmap_2d::CostmapPluginROS)
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using costmap_2d::NO_INFORMATION;

namespace common_costmap_plugins
{

    void InflationCostmapPlugin::initialize(costmap_2d::LayeredCostmap* costmap, std::string name)
    {
        ros::NodeHandle nh("~/" + name), g_nh;
        layered_costmap_ = costmap;
        name_ = name;
        current_ = true;
        seen_ = NULL;
        matchSize();
        
        got_footprint_ = false;
        
        dsrv_ = new dynamic_reconfigure::Server<costmap_2d::InflationPluginConfig>(ros::NodeHandle("~/"+name));
        dynamic_reconfigure::Server<costmap_2d::InflationPluginConfig>::CallbackType cb = boost::bind(&InflationCostmapPlugin::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);
        
        footprint_sub_ = g_nh.subscribe("footprint", 1, &InflationCostmapPlugin::footprint_cb, this);
        
        ros::Rate r(10);
        while(!got_footprint_ && g_nh.ok()){
            ros::spinOnce();
            r.sleep();
        }
    }
    
    void InflationCostmapPlugin::footprint_cb(const geometry_msgs::Polygon& footprint) {
        if(footprint.points.size() > 2){
          //now we need to compute the inscribed/circumscribed radius of the robot from the footprint specification
          double min_dist = std::numeric_limits<double>::max();
          double max_dist = 0.0;

          for(unsigned int i = 0; i < footprint.points.size() - 1; ++i){
            //check the distance from the robot center point to the first vertex
            double vertex_dist = distance(0.0, 0.0, footprint.points[i].x, footprint.points[i].y);
            double edge_dist = distanceToLine(0.0, 0.0, footprint.points[i].x, footprint.points[i].y, footprint.points[i+1].x, footprint.points[i+1].y);
            min_dist = std::min(min_dist, std::min(vertex_dist, edge_dist));
            max_dist = std::max(max_dist, std::max(vertex_dist, edge_dist));
          }

          //we also need to do the last vertex and the first vertex
          double vertex_dist = distance(0.0, 0.0, footprint.points.back().x, footprint.points.back().y);
          double edge_dist = distanceToLine(0.0, 0.0, footprint.points.back().x, footprint.points.back().y, footprint.points.front().x, footprint.points.front().y);
          min_dist = std::min(min_dist, std::min(vertex_dist, edge_dist));
          max_dist = std::max(max_dist, std::max(vertex_dist, edge_dist));

          inscribed_radius_ = min_dist;
          circumscribed_radius_ = max_dist;
          // TODO: Set circumscribed_cost
        }
        cell_inflation_radius_ = cellDistance(inflation_radius_);
        computeCaches();
        got_footprint_ = true;
    }
    
    void InflationCostmapPlugin::reconfigureCB(costmap_2d::InflationPluginConfig &config, uint32_t level){
        if(weight_ != config.cost_scaling_factor || inflation_radius_ != config.inflation_radius)
        {
            inflation_radius_ = config.inflation_radius;
            weight_ = config.cost_scaling_factor;
            computeCaches();
        }
    }

    void InflationCostmapPlugin::matchSize(){
        costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();
        resolution_ = costmap->getResolution();
        cell_inflation_radius_ = cellDistance(inflation_radius_);
        computeCaches();
                
        unsigned int size_x = costmap->getSizeInCellsX(), size_y = costmap->getSizeInCellsY();
        if(seen_) delete seen_;
        seen_ = new bool[size_x*size_y];
    }


    void InflationCostmapPlugin::update_bounds(double origin_x, double origin_y, double origin_yaw, double* min_x, double* min_y, double* max_x, double* max_y){
        //make sure the inflation queue is empty at the beginning of the cycle (should always be true)
        ROS_ASSERT_MSG(inflation_queue_.empty(), "The inflation queue must be empty at the beginning of inflation");
        
        double margin = 2 * inflation_radius_;
        *min_x -= margin;
        *min_y -= margin;
        *max_x += margin;
        *max_y += margin;
    }
    
    void InflationCostmapPlugin::update_costs(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j){
        unsigned char* master_array = master_grid.getCharMap();
        unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();

        memset(seen_, false, size_x*size_y*sizeof(bool));

        for(int j=min_j; j<max_j; j++){
            for(int i=min_i; i<max_i; i++){
                int index = master_grid.getIndex(i, j);
                unsigned char cost = master_array[index];
                if(cost == LETHAL_OBSTACLE){
                    enqueue(master_array, index, i, j, i, j); 
                }
            }
        }

        while(!inflation_queue_.empty()){
          //get the highest priority cell and pop it off the priority queue
          const CellData& current_cell = inflation_queue_.top();

          unsigned int index = current_cell.index_;
          unsigned int mx = current_cell.x_;
          unsigned int my = current_cell.y_;
          unsigned int sx = current_cell.src_x_;
          unsigned int sy = current_cell.src_y_;

          //attempt to put the neighbors of the current cell onto the queue
          if(mx > 0)
            enqueue(master_array, index - 1, mx - 1, my, sx, sy); 
          if(my > 0)
            enqueue(master_array, index - size_x, mx, my - 1, sx, sy);
          if(mx < size_x - 1)
            enqueue(master_array, index + 1, mx + 1, my, sx, sy);
          if(my < size_y - 1)
            enqueue(master_array, index + size_x, mx, my + 1, sx, sy);

          //remove the current cell from the priority queue
          inflation_queue_.pop();
        }
    }

      /**
       * @brief  Given an index of a cell in the costmap, place it into a priority queue for obstacle inflation
       * @param  index The index of the cell
       * @param  mx The x coordinate of the cell (can be computed from the index, but saves time to store it)
       * @param  my The y coordinate of the cell (can be computed from the index, but saves time to store it)
       * @param  src_x The x index of the obstacle point inflation started at
       * @param  src_y The y index of the obstacle point inflation started at
       */
      inline void InflationCostmapPlugin::enqueue(unsigned char* grid, unsigned int index, unsigned int mx, unsigned int my, 
          unsigned int src_x, unsigned int src_y){
        
        //set the cost of the cell being inserted
        if(!seen_[index]){
          //we compute our distance table one cell further than the inflation radius dictates so we can make the check below
          double distance = distanceLookup(mx, my, src_x, src_y);

          //we only want to put the cell in the queue if it is within the inflation radius of the obstacle point
          if(distance > cell_inflation_radius_)
            return;

          //assign the cost associated with the distance from an obstacle to the cell
          unsigned char cost = costLookup(mx, my, src_x, src_y);
          unsigned char old_cost = grid[index];
          
          if(old_cost == NO_INFORMATION && cost >= INSCRIBED_INFLATED_OBSTACLE)
              grid[index] = cost;
          else
              grid[index] = std::max(old_cost, cost);
          //push the cell data onto the queue and mark
          seen_[index] = true;
          CellData data(distance, index, mx, my, src_x, src_y);
          inflation_queue_.push(data);
        }
      }

  void InflationCostmapPlugin::computeCaches() {
    //based on the inflation radius... compute distance and cost caches
    cached_costs_ = new unsigned char*[cell_inflation_radius_ + 2];
    cached_distances_ = new double*[cell_inflation_radius_ + 2];
    for(unsigned int i = 0; i <= cell_inflation_radius_ + 1; ++i){
      cached_costs_[i] = new unsigned char[cell_inflation_radius_ + 2];
      cached_distances_[i] = new double[cell_inflation_radius_ + 2];
      for(unsigned int j = 0; j <= cell_inflation_radius_ + 1; ++j){
        cached_distances_[i][j] = sqrt(i*i + j*j);
        cached_costs_[i][j] = computeCost(cached_distances_[i][j]);
      }
    }
  }
  
  void InflationCostmapPlugin::deleteKernels(){
    if(cached_distances_ != NULL){
      for(unsigned int i = 0; i <= cell_inflation_radius_ + 1; ++i){
        delete[] cached_distances_[i];
      }
      delete[] cached_distances_;
    }

    if(cached_costs_ != NULL){
      for(unsigned int i = 0; i <= cell_inflation_radius_ + 1; ++i){
        delete[] cached_costs_[i];
      }
      delete[] cached_costs_;
    }
  }



      /**
       * @brief  Given a distance... compute a cost
       * @param  distance The distance from an obstacle in cells
       * @return A cost value for the distance
       */
      inline unsigned char InflationCostmapPlugin::computeCost(double distance) const {
        unsigned char cost = 0;
        if(distance == 0)
          cost = LETHAL_OBSTACLE;
        else if(distance*resolution_ <= inscribed_radius_)
          cost = INSCRIBED_INFLATED_OBSTACLE;
        else {
          //make sure cost falls off by Euclidean distance
          double euclidean_distance = distance * resolution_;
          double factor = exp(-1.0 * weight_ * (euclidean_distance - inscribed_radius_));
          cost = (unsigned char) ((INSCRIBED_INFLATED_OBSTACLE - 1) * factor);
        }
        return cost;
      }
}
