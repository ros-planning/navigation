#ifndef VOXEL_GRID_PLAIN_CLEARER_H_
#define VOXEL_GRID_PLAIN_CLEARER_H_

#include <voxel_grid/abstract_grid_updater.h>

namespace voxel_grid
{

class PlainClearer : public AbstractGridUpdater
{
public:
  PlainClearer(uint32_t* voxel_grid_data, unsigned char* costmap,
	       unsigned int unknown_clear_threshold, unsigned int marked_clear_threshold,
	       unsigned char free_cost = 0, unsigned char unknown_cost = 255) :
      AbstractGridUpdater(voxel_grid_data, costmap, unknown_clear_threshold,
			  marked_clear_threshold, free_cost, unknown_cost)
  {
  }

  inline virtual void operator()(unsigned int offset, uint32_t z_mask)
  {
    uint32_t* col = &voxel_grid_data_[offset];
    *col &= ~(z_mask); //clear unknown and clear cell

    unsigned int unknown_bits = uint16_t(*col >> 16) ^ uint16_t(*col);
    unsigned int marked_bits = *col >> 16;

    if (bitsBelowThreshold(marked_bits, marked_clear_threshold_))
    {
      if (bitsBelowThreshold(unknown_bits, unknown_clear_threshold_))
      {
        costmap_[offset] = free_cost_;
      }
      else
      {
        costmap_[offset] = unknown_cost_;
      }
    }
  }
};

} //end namespace

#endif /* VOXEL_GRID_PLAIN_CLEARER_H_ */
