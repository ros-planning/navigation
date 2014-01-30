#include<costmap_2d/inflation_layer.h>
#include<costmap_2d/costmap_math.h>
#include<costmap_2d/footprint.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(costmap_2d::InflationLayer, costmap_2d::Layer)
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using costmap_2d::NO_INFORMATION;

namespace costmap_2d
{

InflationLayer::InflationLayer()
  : inflation_radius_( 0 )
  , weight_( 0 )
  , cell_inflation_radius_(0)
  , cached_cell_inflation_radius_(0)
  , dsrv_(NULL)
{
  access_ = new boost::shared_mutex();
}

void InflationLayer::onInitialize()
{
  {
    boost::unique_lock < boost::shared_mutex > lock(*access_);
    ros::NodeHandle nh("~/" + name_), g_nh;
    current_ = true;
    seen_ = NULL;
    need_reinflation_ = false;

    dynamic_reconfigure::Server<costmap_2d::InflationPluginConfig>::CallbackType cb = boost::bind(
        &InflationLayer::reconfigureCB, this, _1, _2);
  
    if(dsrv_ != NULL){
      dsrv_->clearCallback();
      dsrv_->setCallback(cb);
    }
    else
    {
      dsrv_ = new dynamic_reconfigure::Server<costmap_2d::InflationPluginConfig>(ros::NodeHandle("~/" + name_));
      dsrv_->setCallback(cb);
    }

  }

  matchSize();
}

void InflationLayer::reconfigureCB(costmap_2d::InflationPluginConfig &config, uint32_t level)
{
  if (weight_ != config.cost_scaling_factor || inflation_radius_ != config.inflation_radius)
  {
    inflation_radius_ = config.inflation_radius;
    cell_inflation_radius_ = cellDistance(inflation_radius_);
    weight_ = config.cost_scaling_factor;
    need_reinflation_ = true;
    computeCaches();
  }

  if (enabled_ != config.enabled) {
    enabled_ = config.enabled;
    need_reinflation_ = true;
  }
}

void InflationLayer::matchSize()
{
  boost::unique_lock < boost::shared_mutex > lock(*access_);
  costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();
  resolution_ = costmap->getResolution();
  cell_inflation_radius_ = cellDistance(inflation_radius_);
  computeCaches();

  unsigned int size_x = costmap->getSizeInCellsX(), size_y = costmap->getSizeInCellsY();
  if (seen_)
    delete seen_;
  seen_ = new bool[size_x * size_y];
}

void InflationLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
  if( need_reinflation_ )
  {
    // For some reason when I make these -<double>::max() it does not
    // work with Costmap2D::worldToMapEnforceBounds(), so I'm using
    // -<float>::max() instead.
    *min_x = -std::numeric_limits<float>::max();
    *min_y = -std::numeric_limits<float>::max();
    *max_x = std::numeric_limits<float>::max();
    *max_y = std::numeric_limits<float>::max();
    need_reinflation_ = false;
  }
}

void InflationLayer::onFootprintChanged()
{
  inscribed_radius_ = layered_costmap_->getInscribedRadius();
  cell_inflation_radius_ = cellDistance( inflation_radius_ );
  computeCaches();
  need_reinflation_ = true;

  ROS_DEBUG( "InflationLayer::onFootprintChanged(): num footprint points: %lu, inscribed_radius_ = %.3f, inflation_radius_ = %.3f",
             layered_costmap_->getFootprint().size(), inscribed_radius_, inflation_radius_ );
}

void InflationLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
  boost::unique_lock < boost::shared_mutex > lock(*access_);
  if (!enabled_)
    return;

  //make sure the inflation queue is empty at the beginning of the cycle (should always be true)
  ROS_ASSERT_MSG(inflation_queue_.empty(), "The inflation queue must be empty at the beginning of inflation");

  unsigned char* master_array = master_grid.getCharMap();
  unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();

  memset(seen_, false, size_x * size_y * sizeof(bool));

  // We need to include in the inflation cells outside the bounding
  // box min_i...max_j, by the amount cell_inflation_radius_.  Cells
  // up to that distance outside the box can still influence the costs
  // stored in cells inside the box.
  min_i -= cell_inflation_radius_;
  min_j -= cell_inflation_radius_;
  max_i += cell_inflation_radius_;
  max_j += cell_inflation_radius_;

  min_i = std::max( 0, min_i );
  min_j = std::max( 0, min_j );
  max_i = std::min( int( size_x  ), max_i );
  max_j = std::min( int( size_y  ), max_j );

  for (int j = min_j; j < max_j; j++)
  {
    for (int i = min_i; i < max_i; i++)
    {
      int index = master_grid.getIndex(i, j);
      unsigned char cost = master_array[index];
      if (cost == LETHAL_OBSTACLE)
      {
        enqueue(master_array, index, i, j, i, j);
      }
    }
  }

  while (!inflation_queue_.empty())
  {
    //get the highest priority cell and pop it off the priority queue
    const CellData& current_cell = inflation_queue_.top();

    unsigned int index = current_cell.index_;
    unsigned int mx = current_cell.x_;
    unsigned int my = current_cell.y_;
    unsigned int sx = current_cell.src_x_;
    unsigned int sy = current_cell.src_y_;

    //attempt to put the neighbors of the current cell onto the queue
    if (mx > 0)
      enqueue(master_array, index - 1, mx - 1, my, sx, sy);
    if (my > 0)
      enqueue(master_array, index - size_x, mx, my - 1, sx, sy);
    if (mx < size_x - 1)
      enqueue(master_array, index + 1, mx + 1, my, sx, sy);
    if (my < size_y - 1)
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
inline void InflationLayer::enqueue(unsigned char* grid, unsigned int index, unsigned int mx, unsigned int my,
                                            unsigned int src_x, unsigned int src_y)
{

  //set the cost of the cell being inserted
  if (!seen_[index])
  {
    //we compute our distance table one cell further than the inflation radius dictates so we can make the check below
    double distance = distanceLookup(mx, my, src_x, src_y);

    //we only want to put the cell in the queue if it is within the inflation radius of the obstacle point
    if (distance > cell_inflation_radius_)
      return;

    //assign the cost associated with the distance from an obstacle to the cell
    unsigned char cost = costLookup(mx, my, src_x, src_y);
    unsigned char old_cost = grid[index];

    if (old_cost == NO_INFORMATION && cost >= INSCRIBED_INFLATED_OBSTACLE)
      grid[index] = cost;
    else
      grid[index] = std::max(old_cost, cost);
    //push the cell data onto the queue and mark
    seen_[index] = true;
    CellData data(distance, index, mx, my, src_x, src_y);
    inflation_queue_.push(data);
  }
}

void InflationLayer::computeCaches()
{
  if(cell_inflation_radius_ == 0)
    return;

  //based on the inflation radius... compute distance and cost caches
  if(cell_inflation_radius_ != cached_cell_inflation_radius_)
  {
    if(cached_cell_inflation_radius_ > 0)
      deleteKernels();

    cached_costs_ = new unsigned char*[cell_inflation_radius_ + 2];
    cached_distances_ = new double*[cell_inflation_radius_ + 2];

    for (unsigned int i = 0; i <= cell_inflation_radius_ + 1; ++i)
    {
      cached_costs_[i] = new unsigned char[cell_inflation_radius_ + 2];
      cached_distances_[i] = new double[cell_inflation_radius_ + 2];
      for (unsigned int j = 0; j <= cell_inflation_radius_ + 1; ++j)
      {
        cached_distances_[i][j] = sqrt(i * i + j * j);
      }
    }

    cached_cell_inflation_radius_ = cell_inflation_radius_;
  }

  for (unsigned int i = 0; i <= cell_inflation_radius_ + 1; ++i)
  {
    for (unsigned int j = 0; j <= cell_inflation_radius_ + 1; ++j)
    {
      cached_costs_[i][j] = computeCost(cached_distances_[i][j]);
    }
  }
}

void InflationLayer::deleteKernels()
{
  if (cached_distances_ != NULL)
  {
    for (unsigned int i = 0; i <= cached_cell_inflation_radius_ + 1; ++i)
    {
      delete[] cached_distances_[i];
    }
    delete[] cached_distances_;
  }

  if (cached_costs_ != NULL)
  {
    for (unsigned int i = 0; i <= cached_cell_inflation_radius_ + 1; ++i)
    {
      delete[] cached_costs_[i];
    }
    delete[] cached_costs_;
  }
}

} // end namespace costmap_2d
