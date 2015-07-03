/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#include <costmap_2d/inflation_layer.h>
#include <costmap_2d/axis_aligned_bounding_box.h>
#include <costmap_2d/layer_actions.h>
#include <costmap_2d/costmap_math.h>
#include <costmap_2d/footprint.h>
#include <pluginlib/class_list_macros.h>
#include <limits>
#include <algorithm>
#include <vector>

PLUGINLIB_EXPORT_CLASS(costmap_2d::InflationLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using costmap_2d::NO_INFORMATION;

// algorithm "names"
#define ALG_PRIORITY_QUEUE  0
#define ALG_LAYER_ACTIONS   1
#define ALG_TOTAL_AVAILABLE 2

namespace costmap_2d
{

InflationLayer::InflationLayer()
  : inflation_radius_( 0 )
  , weight_( 0 )
  , cell_inflation_radius_(0)
  , cached_cell_inflation_radius_(0)
  , dsrv_(NULL)
  , seen_(NULL)
  , cached_costs_(NULL)
  , cached_distances_(NULL)
{
  access_ = new boost::shared_mutex();
  
  algorithmSelect_.setMaxAlgorithmTypes(ALG_TOTAL_AVAILABLE);
}

void InflationLayer::onInitialize()
{
  {
    boost::unique_lock < boost::shared_mutex > lock(*access_);
    ros::NodeHandle nh("~/" + name_), g_nh;
    current_ = false; // This is new, blocks planners until all the calculations are complete
    if (seen_)
      delete[] seen_;
    seen_ = NULL;
    seen_size_ = 0;
    need_reinflation_ = false;

    // This value is read from dynamic reconfigure. To change the default
    // value (-1, automatic) you can edit costmap_2d/cfg/InflationPlugin.cfg
    inflation_method_ = 0;

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

  inflation_method_ = config.method;
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
    delete[] seen_;
  seen_ = new bool[size_x * size_y];
  seen_size_ = size_x * size_y;
}

void InflationLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
  // 1 inflation radius to inflate the data
  // an extra radius to allow the clearing of previous
  // inflated data
  *min_x = *min_x - 2*inflation_radius_;
  *min_y = *min_y - 2*inflation_radius_;
  *max_x = *max_x + 2*inflation_radius_;
  *max_y = *max_y + 2*inflation_radius_;
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

void InflationLayer::updateCosts(Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j)
{
  boost::unique_lock < boost::shared_mutex > lock(*access_);

  updateCostsPQ(master_grid, min_i, min_j, max_i, max_j);

  current_ = true; // update complete - stop blocking processes that are waiting on this data
}

void InflationLayer::updateCostsPQ(Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_)
    return;

  //make sure the inflation queue is empty at the beginning of the cycle (should always be true)
  ROS_ASSERT_MSG(inflation_queue_.empty(), "The inflation queue must be empty at the beginning of inflation");

  unsigned char* master_array = master_grid.getCharMap();
  unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();

  if (seen_ == NULL) {
    ROS_ERROR("seen is NULL");
    seen_ = new bool[size_x * size_y];
    seen_size_ = size_x * size_y;
  }
  else if (seen_size_ != size_x * size_y) 
  {
    ROS_ERROR("seen size is wrong");
    delete[] seen_; 
    seen_ = new bool[size_x * size_y];
    seen_size_ = size_x * size_y;
  }
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

    //pop once we have our cell info
    inflation_queue_.pop();

    //attempt to put the neighbors of the current cell onto the queue
    if (mx > 0)
      enqueue(master_array, index - 1, mx - 1, my, sx, sy);
    if (my > 0)
      enqueue(master_array, index - size_x, mx, my - 1, sx, sy);
    if (mx < size_x - 1)
      enqueue(master_array, index + 1, mx + 1, my, sx, sy);
    if (my < size_y - 1)
      enqueue(master_array, index + size_x, mx, my + 1, sx, sy);
  }
}


void InflationLayer::updateCostsLayerActions(LayerActions *layer_actions, Costmap2D &master_grid,
                           int min_i, int min_j, int max_i, int max_j)
{
    // We have a list of all the actions that got us to this point.
    // Hopefully some actions will have associated, pre-calculated inflations data
    // and other layers will mark areas where we need to re-inflate.
    
    Costmap2DPtr workspace = master_grid.getNamedCostmap2D("Workspace");
    if (workspace.get() == 0)
      workspace = master_grid.addNamedCostmap2D("Workspace", master_grid.createReducedResolutionMap(1));
 
    // build an AxisAlignedBoundingBox of the updateCostsBounds so we can clamp large ranges
    // against it's small window
    AxisAlignedBoundingBox updateCostsBounds(min_i, min_j, max_i, max_j);

    // start with a clean workspace
    workspace->setMapCost(min_i, min_j, max_i, max_j, FREE_SPACE);

    // Find all actions that made modifications to the master_grid
    LayerActions l_acts;
    layer_actions->copyToWithMatchingDest(l_acts, &master_grid);

    // track how much data is pre-inflated
    std::vector<AxisAlignedBoundingBox> inflatedData;

    // track how much data has pending inflation required
    std::vector<AxisAlignedBoundingBox> uninflatedData;

    // now we can act on each action
    for (int i = 0; i < l_acts.size(); i++)
    {
      if (l_acts.actionAt(i) == LayerActions::TRUEOVERWRITE)
      {
        // if it's a true overwrite then we can lay down a copy of the cached inflated data
        Costmap2D* src = l_acts.sourceCostmapAt(i);
        if (src)
        {
          // need to check if the stored inflation radius is appropriate for this pass
          if (src->namedFlag("InflationRadius") != cell_inflation_radius_)
          {
            // if not then we remove any existing inflated data
            src->removeNamedCostmap2D("Inflated");
          }
          
          unsigned int origin_x, origin_y;
          src->worldToMap(0, 0, origin_x, origin_y);
          // make sure the origin isn't moving or else the cached data isn't correct
          if (src->namedFlag("origin_x") != origin_x || src->namedFlag("origin_y") != origin_y)
          {
            src->removeNamedCostmap2D("Inflated");
          }
          
          Costmap2DPtr inflated = src->getNamedCostmap2D("Inflated");
          
          if (inflated.get() == 0)  // then we don't have a cache of the inflated data
          {
            inflated = src->createReducedResolutionMap(1);
            src->copyCellsTo(inflated, Costmap2D::TrueOverwrite);

            int src_nx = src->getSizeInCellsX();
            int src_ny = src->getSizeInCellsY();

            // inflate cached data
            updateCostsPQ(*inflated.get(), 0, 0, src_nx, src_ny);

            // add inflated data to the source map so we have it next time
            src->addNamedCostmap2D("Inflated", inflated);
            
            // store the inflation radius used to generate this data
            src->namedFlag("InflationRadius") = cell_inflation_radius_;
            
            // store the origin for this cache
            src->namedFlag("origin_x") = origin_x;
            src->namedFlag("origin_y") = origin_y;
          }

          // only copying the region that we are interested in
          inflated->copyCellsTo(workspace,
                                updateCostsBounds.x0(), updateCostsBounds.y0(),
                                updateCostsBounds.num_x(), updateCostsBounds.num_y(), Costmap2D::TrueOverwrite);

          // remembering that we have good data
          inflatedData.push_back(AxisAlignedBoundingBox(updateCostsBounds));
        }
        else  // no defined source, need to work with master_grid
        {
          const AxisAlignedBoundingBox& dst = l_acts.destinationAxisAlignedBoundingBoxAt(i);
          if (dst.initialized())
          {
            master_grid.copyCellsTo(workspace,
                                    dst.x0(), dst.y0(),
                                    dst.num_x(), dst.num_y());

            // mark that we need to inflate this data
            uninflatedData.push_back(AxisAlignedBoundingBox(dst));
          }
          else
          {
            // This is a problem that shouldn't be encountered if everything has been done properly.
            ROS_DEBUG("(inflation_layer.cpp:%d) Destination bounding box uninitialized, "
                      "falling back to global inflation.", __LINE__);

            // Falling back to old method: not fancy but it works
            return updateCostsPQ(master_grid, min_i, min_j, max_i, max_j);
          }
        }
      }
      else  // l_acts.actionAt(i) != LayerActions::TRUEOVERWRITE
      {
        if (l_acts.actionAt(i) != LayerActions::NONE)
        {
          // copy from the master to the workspace and mark to inflate the window
          const AxisAlignedBoundingBox& dst = l_acts.destinationAxisAlignedBoundingBoxAt(i);
          if (dst.initialized())
          {
            master_grid.copyCellsTo(workspace,
                                    dst.x0(), dst.y0(),
                                    dst.num_x(), dst.num_y(), Costmap2D::Max);

            // mark that we need to inflate this data
            uninflatedData.push_back(AxisAlignedBoundingBox(dst));
          }
          else
          {
            // This is a problem that shouldn't be encountered if everything has been done properly.
            ROS_DEBUG("(inflation_layer.cpp:%d) Destination bounding box uninitialized, "
                      "falling back to global inflation.", __LINE__);

            // Falling back to old method: not fancy but it works
            return updateCostsPQ(master_grid, min_i, min_j, max_i, max_j);
          }
        }
      }
    }

    // At this point we have data on the workspace and lists of
    // uninflated regions. We will work through those regions
    // and inflate them
    for (int i = 0; i < uninflatedData.size(); i++)
    {
      AxisAlignedBoundingBox& to_inflate = uninflatedData[i];

      // and here it is: We are inflating only the required sections
      updateCostsPQ(*workspace.get(), to_inflate.min_x_, to_inflate.min_y_, to_inflate.max_x_, to_inflate.max_y_);
    }

    layer_actions->clear();

    // Now we will copy from the workspace to the master_grid
    workspace->copyCellsTo(master_grid, min_i, min_j, max_i-min_i, max_j-min_j, Costmap2D::TrueOverwrite);

}

void InflationLayer::updateCosts(LayerActions* layer_actions, costmap_2d::Costmap2D& master_grid,
                                 int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_)
  {
    current_ = true; // don't block a waiting process
    return;
  }

  boost::unique_lock < boost::shared_mutex > lock(*access_);

  int problem_size = (max_j - min_j) * (max_i - min_i);

  int algorithm;
  bool report_statistics = false;

  switch(inflation_method_)
  {
  case ALG_PRIORITY_QUEUE:
    algorithm = ALG_PRIORITY_QUEUE;
    break;
  case ALG_LAYER_ACTIONS:
    algorithm = ALG_LAYER_ACTIONS;
    break;
  default:
    report_statistics = true;
    algorithm =  algorithmSelect_.selectAlgorithm(problem_size);
  }

  DynamicAlgorithmSelect::Timer timer;
  timer.start();

  switch(algorithm)
  {
    case ALG_PRIORITY_QUEUE:
      updateCostsPQ(master_grid, min_i, min_j, max_i, max_j);
    break;

    case ALG_LAYER_ACTIONS:
      updateCostsLayerActions(layer_actions, master_grid, min_i, min_j, max_i, max_j);
    break;
    default:
      ROS_ERROR("(inflation_layer.cpp:%d Unhandled algorithm type: %d", __LINE__, algorithm);
  }

  timer.stop();

  if (report_statistics)
  {
    // Add data to profiler for smarter future choices
    algorithmSelect_.addProfilingData(problem_size, algorithm, timer.elapsed());
  }

  current_ = true; // allow consumers to use this data
}

/**
 * @brief  Given an index of a cell in the costmap, place it into a priority queue for obstacle inflation
 * @param  grid The costmap
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
    deleteKernels();

    cached_costs_ = new unsigned char*[cell_inflation_radius_ + 2];
    cached_distances_ = new double*[cell_inflation_radius_ + 2];

    for (unsigned int i = 0; i <= cell_inflation_radius_ + 1; ++i)
    {
      cached_costs_[i] = new unsigned char[cell_inflation_radius_ + 2];
      cached_distances_[i] = new double[cell_inflation_radius_ + 2];
      for (unsigned int j = 0; j <= cell_inflation_radius_ + 1; ++j)
      {
        cached_distances_[i][j] = hypot(i, j);
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
      if (cached_distances_[i])
        delete[] cached_distances_[i];
    }
    if (cached_distances_)
      delete[] cached_distances_;
    cached_distances_ = NULL;
  }

  if (cached_costs_ != NULL)
  {
    for (unsigned int i = 0; i <= cached_cell_inflation_radius_ + 1; ++i)
    {
      if (cached_costs_[i])
        delete[] cached_costs_[i];
    }
    if (cached_costs_)
      delete[] cached_costs_;
    cached_costs_ = NULL;
  }
}

}  // namespace costmap_2d
