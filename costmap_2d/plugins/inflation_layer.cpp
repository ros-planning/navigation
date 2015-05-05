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
#include <costmap_2d/dynamic_algorithm_select.h>
#include <limits>
#include <algorithm>
#include <vector>
#include <map>

PLUGINLIB_EXPORT_CLASS(costmap_2d::InflationLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using costmap_2d::NO_INFORMATION;


// algorithm "names"
#define ALG_PRIORITY_QUEUE  0
#define ALG_LAYER_ACTIONS   1
#define ALG_TOTAL_AVAILABLE 2

// Global data tracking algorithm runtimes
costmap_2d::DynamicAlgorithmSelect algorithmSelect(ALG_TOTAL_AVAILABLE);

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
  , cached_kernel_inflated_(false)
{
  access_ = new boost::shared_mutex();
}

void InflationLayer::onInitialize()
{
  {
    boost::unique_lock < boost::shared_mutex > lock(*access_);
    ros::NodeHandle nh("~/" + name_), g_nh;
    current_ = true;
    if (seen_)
      delete[] seen_;
    seen_ = NULL;
    seen_size_ = 0;
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
    delete[] seen_;
  seen_ = new bool[size_x * size_y];
  seen_size_ = size_x * size_y;
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

void InflationLayer::updateCosts(Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j)
{
  return updateCostsPQ(master_grid, min_i, min_j, max_i, max_j);
}

// Apply inflation update rule on a contiguous row of memory
static void updateCostsOverlay_row_helper(unsigned char* dest, const unsigned char* overlay, unsigned int n)
{
  for (int i = 0; i < n; i++)
  {
    unsigned char& old_cost = dest[i];
    const unsigned char& cost = overlay[i];

    if (old_cost == NO_INFORMATION && cost >= INSCRIBED_INFLATED_OBSTACLE)
      old_cost = cost;
    else
      old_cost = std::max(old_cost, cost);
  }
}

// This helper function will overlay a source costmap onto a destination costmap with
// logic to keep thing in bounds. The overlay is governed by the inflation update rules
static void updateCostsOverlay_block_helper(Costmap2D* dest, Costmap2D* src, int _start_dest_i, int _start_dest_j)
{
  const int nsi = src->getSizeInCellsX();
  const int nsj = src->getSizeInCellsY();

  const int ndi = dest->getSizeInCellsX();
  const int ndj = dest->getSizeInCellsX();

  int start_dest_i = std::max(0, _start_dest_i);
  int start_dest_j = std::max(0, _start_dest_j);

  int end_dest_i = std::min(ndi, _start_dest_i + nsi);
  int end_dest_j = std::min(ndj, _start_dest_j + nsj);

  int start_src_i = start_dest_i - _start_dest_i;
  int start_src_j = start_dest_j - _start_dest_j;

  int range_i = end_dest_i - start_dest_i;
  int range_j = end_dest_j - start_dest_j;

  unsigned char* dest_array = dest->getCharMap();
  unsigned char*  src_array =  src->getCharMap();

  for (int j = 0; j < range_j; j++)
  {
    int dest_index = dest->getIndex(start_dest_i, start_dest_j + j);
    int  src_index =  src->getIndex(start_src_i,  start_src_j  + j);

    updateCostsOverlay_row_helper(dest_array + dest_index, src_array + src_index, range_i);
  }
}


void InflationLayer::updateCostsOverlay(Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!cached_kernel_inflated_)
  {
    updateCostsPQ(* cached_kernel_.get(), 0, 0, cached_kernel_->getSizeInCellsX(), cached_kernel_->getSizeInCellsX());
    cached_kernel_inflated_ = true;
  }

  boost::unique_lock < boost::shared_mutex > lock(*access_);
  if (!enabled_)
    return;

  unsigned char* master_array = master_grid.getCharMap();
  unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();

  // We need to include in the inflation cells outside the bounding
  // box min_i...max_j, by the amount cell_inflation_radius_.  Cells
  // up to that distance outside the box can still influence the costs
  // stored in cells inside the box.
  min_i -= cell_inflation_radius_;
  min_j -= cell_inflation_radius_;
  max_i += cell_inflation_radius_;
  max_j += cell_inflation_radius_;

  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  max_i = std::min(static_cast<int>(size_x), max_i);
  max_j = std::min(static_cast<int>(size_y), max_j);

  for (int j = min_j; j < max_j; j++)
  {
    for (int i = min_i; i < max_i; i++)
    {
      if (master_grid.getCost(i, j) == LETHAL_OBSTACLE)
      {
        updateCostsOverlay_block_helper(&master_grid, cached_kernel_.get(),
                                        i - cached_kernel_cnx_, j - cached_kernel_cny_);
      }
    }
  }
}



void InflationLayer::updateCostsPQ(Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j)
{
  boost::unique_lock < boost::shared_mutex > lock(*access_);
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

void InflationLayer::updateCosts(LayerActions* layer_actions, costmap_2d::Costmap2D& master_grid,
                                 int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_)
    return;

  int problem_size = master_grid.getSizeInCellsX() * master_grid.getSizeInCellsY();
  int best_algorithm = algorithmSelect.selectAlgorithm(problem_size);

  if(best_algorithm == ALG_PRIORITY_QUEUE)
  {
    DynamicAlgorithmSelect::Timer timer;
    timer.start();
    updateCostsPQ(master_grid, min_i, min_j, max_i, max_j);
    timer.stop();

    // Add data to profiler for smarter future choices
    algorithmSelect.addProfilingData(problem_size, ALG_PRIORITY_QUEUE, timer.elapsed());
    return;
  }

  if(best_algorithm == ALG_LAYER_ACTIONS)
  {
    // We now have a list of all the actions that got us to this point.
    // Hopefully some actions will have associated, pre-calculated inflations data
    // and other layers will mark areas where we need to re-inflate.

    DynamicAlgorithmSelect::Timer timer;
    timer.start();

    Costmap2DPtr workspace = master_grid.getNamedCostmap2D("Workspace");
    if (workspace.get() == 0)
      workspace = master_grid.addNamedCostmap2D("Workspace", master_grid.createReducedResolutionMap(1));

    // build an AxisAlignedBoundingBox of the updateCostsBounds so we can clamp large ranges
    // against it's small window
    AxisAlignedBoundingBox updateCostsBounds(min_i, min_j, max_i, max_j);

    // First we need to look at all the old actions that made modifications
    // to the given master_grid and optimize them, essentially we don't want
    // to inflate the same area twice
    LayerActions l_acts;
    layer_actions->copyToWithMatchingDest(l_acts, &master_grid);
    l_acts.optimize(); // merge similar actions on same regions


    // track how much data is pre-inflated
    std::vector<AxisAlignedBoundingBox> inflatedData;

    // track how much data has pending inflation required
    std::vector<AxisAlignedBoundingBox> uninflatedData;

    // now we can start marching up the actions
    for (int i = 0; i < l_acts.size(); i++)
    {
      if (l_acts.actionAt(i) == LayerActions::TRUEOVERWRITE)
      {
        // if it's a true overwrite then we can lay down a copy of the cached inflated data
        Costmap2D* src = l_acts.sourceCostmapAt(i);
        if (src)
        {
          int src_nx = src->getSizeInCellsX();
          int src_ny = src->getSizeInCellsY();
          Costmap2DPtr inflated = src->getNamedCostmap2D("Inflated");

          if (inflated.get() == 0)  // then we don't have a cache of the inflated data
          {
            inflated = src->createReducedResolutionMap(1);
            src->copyCellsTo(inflated);

            // inflate cached data
            updateCosts(*inflated.get(), 0, 0, src_nx, src_ny);

            // add inflated data to the source map so we have it next time
            src->addNamedCostmap2D("Inflated", inflated);
          }

          // only copying the region that we are interested in
          inflated->copyCellsTo(workspace,
                                updateCostsBounds.x0(), updateCostsBounds.y0(),
                                updateCostsBounds.xn(), updateCostsBounds.yn());

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
                                    dst.xn(), dst.yn());

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
                                    dst.xn(), dst.yn());

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
    // inflated and uninflated regions.
    // The uninflated trumps inflated so we need to merge the
    // uninflated list and apply inflation piecemeal only where it is needed

    int inflation_margin = cellDistance(inflation_radius_);
    AxisAlignedBoundingBox::mergeIntersecting(uninflatedData, inflation_margin);

    for (int i = 0; i < uninflatedData.size(); i++)
    {
      AxisAlignedBoundingBox& to_inflate = uninflatedData[i];

      // and here it is: We are inflating only the required sections
      updateCosts(*workspace.get(), to_inflate.min_x_, to_inflate.min_y_, to_inflate.max_x_, to_inflate.max_y_);
    }

    layer_actions->clear();

    // Now we will copy from the workspace to the master_grid
    workspace->copyCellsTo(master_grid, min_i, min_j, max_i, max_j);

    timer.stop();

    // Add data to profiler for smarter future choices
    algorithmSelect.addProfilingData(problem_size, ALG_LAYER_ACTIONS, timer.elapsed());
    return;
  }

  ROS_ERROR("(inflation_layer.cpp:%d Unhandled dynamic algorithm type: %d", __LINE__, best_algorithm);
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

  // Build costmap representing the inflation about a lethal point
  // This is used in the updateCost with Overlays method
  {
    const unsigned int n = cell_inflation_radius_;
    const unsigned int cnx = 2 * n + 1;
    const unsigned int cny = 2 * n + 1;
    cached_kernel_.reset(new Costmap2D(cnx, cny, getResolution(), 0, 0, FREE_SPACE));

    cached_kernel_->setCost(n, n, LETHAL_OBSTACLE);
    cached_kernel_cnx_ = n;
    cached_kernel_cny_ = n;

    cached_kernel_inflated_ = false;
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
