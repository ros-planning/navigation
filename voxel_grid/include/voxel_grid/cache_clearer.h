#ifndef VOXEL_GRID_CACHE_CLEARER_H_
#define VOXEL_GRID_CACHE_CLEARER_H_

#include <list>
#include <utility>
#include <boost/shared_ptr.hpp>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>
#include <ros/console.h>
#include <voxel_grid/abstract_grid_updater.h>
#include <bitset>

namespace voxel_grid
{

class CacheClearer : public AbstractGridUpdater
{
public:
  CacheClearer(uint32_t* voxel_grid_data, unsigned char* costmap, unsigned int costmap_size_x,
	       unsigned int costmap_size_y, unsigned int cache_width,
	       unsigned int unknown_clear_threshold,
	       unsigned int marked_clear_threshold,
	       bool clear_corners = false,
	       unsigned char free_cost = 0, unsigned char unknown_cost = 255) :
     AbstractGridUpdater(voxel_grid_data, costmap, unknown_clear_threshold,
			 marked_clear_threshold, clear_corners, free_cost, unknown_cost)
  {
    costmap_size_x_ = costmap_size_x;
    costmap_size_y_ = costmap_size_y;

    current_offset_from_costmap_x_ = 0;
    current_offset_from_costmap_y_ = 0;

    cache_width_ = cache_width;
    z_is_offset_ = update_corners_;

    updated_cells_indices_ = boost::shared_ptr<std::list<CostmapXY> > (new std::list<CostmapXY>);
  }

  virtual uint32_t* clearCache(unsigned int cache_size, uint32_t* cache)
  {
    uint32_t empty_mask = (uint32_t)0;
    for (int i = 0; i < cache_size; ++i)
      cache[i] = empty_mask;
    return cache;
  }

  virtual uint32_t* getNewCache(unsigned int cache_width)
  {
    unsigned int cache_size = cache_width * cache_width;
    uint32_t* cache = new uint32_t[cache_size];
    return clearCache(cache_size, cache);
  }

  virtual void setSensorCache(int offset_x, int offset_y)
  {
    CostmapOffsetXY sensor_offset(offset_x, offset_y);
    if (sensor_clearing_caches_[sensor_offset] == NULL ||
	sensor_clearing_caches_.find(sensor_offset) == sensor_clearing_caches_.end()) {
      sensor_clearing_caches_[sensor_offset] = CacheData(this->getNewCache(cache_width_));
    }
  }

  virtual void setCostmapOffsets(int offset_from_costmap_x, int offset_from_costmap_y)
  {
    setSensorCache(offset_from_costmap_x, offset_from_costmap_y);
    current_offset_from_costmap_x_ = offset_from_costmap_x;
    current_offset_from_costmap_y_ = offset_from_costmap_y;
  }

  inline virtual void operator()(unsigned int offset, uint32_t z_mask)
  {
    setSensorCache(current_offset_from_costmap_x_, current_offset_from_costmap_y_);
    CostmapOffsetXY sensor_offset(current_offset_from_costmap_x_, current_offset_from_costmap_y_);
    CacheData cache = sensor_clearing_caches_[sensor_offset];
    cache.get()[offset] |= z_mask;
  }

  virtual void update()
  {
    updated_cells_indices_->clear();
    cleared_voxels_ = sensor_msgs::PointCloud();
    SensorCaches::iterator it;
    for (it=sensor_clearing_caches_.begin(); it!=sensor_clearing_caches_.end(); ++it)
    {
      unsigned int cache_size_x = cache_width_;
      unsigned int cache_size_y = cache_width_;
      unsigned int linear_cache_index = 0;
      unsigned int linear_costmap_index = 0;
      int costmap_x = 0;
      int costmap_y = 0;
      int sensor_offset_x = it->first.first;
      int sensor_offset_y = it->first.second;
      for (unsigned int y_index = 0; y_index < cache_size_y; ++y_index)
      {
        linear_cache_index = y_index * cache_size_x;
        costmap_y = y_index - sensor_offset_y;

        if (costmap_y < 0) continue;

        if (costmap_y >= costmap_size_y_) break;

        costmap_x = -sensor_offset_x;

        linear_costmap_index = costmap_y * costmap_size_x_ + costmap_x;

        for (unsigned int x_index = 0; x_index < cache_size_x; ++x_index)
        {
          uint32_t clearing_mask = it->second[linear_cache_index];
          if (clearing_mask == (uint32_t)0)
          { //not updated
            costmap_x++;
            linear_costmap_index++;
            linear_cache_index++;
            continue;
          }

          if (z_is_offset_)
            clearing_mask = undoClearingMaskOffset(clearing_mask);
	  
          if (costmap_x < 0)
          { //In case of underflow because of corner cases
            costmap_x++;
            linear_costmap_index++;
            linear_cache_index++;
            continue;
          }

          if (costmap_x >= costmap_size_x_)
          { //In case of overflow because of corner cases
            break;
          }

          uint32_t* voxel_column = &voxel_grid_data_[linear_costmap_index];

          *voxel_column &= ~(clearing_mask); //clear unknown and clear cell

          updateCostmap(*voxel_column, linear_costmap_index);
          updated_cells_indices_->push_back(std::make_pair(costmap_x, costmap_y));

          updateClearedVoxels(costmap_x, costmap_y, clearing_mask);

          costmap_x++;
          linear_costmap_index++;
          linear_cache_index++;
        }
      }
    }
  }

  inline virtual uint32_t undoClearingMaskOffset(uint32_t clearing_mask)
  {
    clearing_mask >>= 1;
    uint32_t lower_bits_mask = ~((uint32_t)0) >> 16;

    return (clearing_mask << 16) | (clearing_mask & lower_bits_mask);
  }

  boost::shared_ptr<std::list<std::pair<unsigned int, unsigned int> > > getClearedCellsIndices()
  {
    return updated_cells_indices_;
  }

  sensor_msgs::PointCloud getClearedVoxels()
  {
    return cleared_voxels_;
  }

protected:
  virtual void updateCostmap(uint32_t voxel_column, unsigned int costmap_index)
  {
    unsigned int unknown_bits = uint16_t(voxel_column >> 16) ^ uint16_t(voxel_column);
    unsigned int marked_bits = voxel_column >> 16;

    if (bitsBelowThreshold(marked_bits, marked_clear_threshold_))
    {
      if (bitsBelowThreshold(unknown_bits, unknown_clear_threshold_))
      {
        costmap_[costmap_index] = unknown_cost_;
      }
      else
      {
        costmap_[costmap_index] = free_cost_;
      }
    }
  }

  virtual void updateClearedVoxels(unsigned int x, unsigned int y, uint32_t clearing_mask)
  {
    for (int z_index = 0; z_index < 16; ++z_index)
    {
      if ((clearing_mask & 1) != 0)
      {
        geometry_msgs::Point32 point;
        point.x = x;
        point.y = y;
        point.z = z_index;
        cleared_voxels_.points.push_back(point);
      }
      clearing_mask >>= 1;
    }
  }

private:
  unsigned int costmap_size_x_;
  unsigned int costmap_size_y_;

  int current_offset_from_costmap_x_;
  int current_offset_from_costmap_y_;
  unsigned int cache_width_;
  bool z_is_offset_;


  typedef boost::shared_ptr<uint32_t[]> CacheData;
  typedef std::pair<int, int> CostmapOffsetXY;
  typedef std::map<CostmapOffsetXY, CacheData> SensorCaches;
  typedef std::pair<unsigned int, unsigned int> CostmapXY;
  typedef boost::shared_ptr<std::list<CostmapXY> > UpdateIndices;
  SensorCaches sensor_clearing_caches_;
  UpdateIndices updated_cells_indices_;
  sensor_msgs::PointCloud cleared_voxels_;
};

} //end namespace

#endif /* VOXEL_GRID_CACHE_CLEARER_H_ */
