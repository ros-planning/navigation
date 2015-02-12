#ifndef VOXEL_GRID_PLAIN_CLEARER_H_
#define VOXEL_GRID_PLAIN_CLEARER_H_

#include <voxel_grid/abstract_grid_updater.h>

namespace voxel_grid
{

class PlainClearer : public AbstractGridUpdater
{
public:
  PlainClearer(uint32_t* voxel_grid_data) :
      AbstractGridUpdater(voxel_grid_data, NULL, 0, 0, 0, 0)
  {
  }

  inline virtual void operator()(unsigned int offset, uint32_t z_mask)
  {
    voxel_grid_data_[offset] &= ~(z_mask);
  }

};

} //end namespace

#endif /* VOXEL_GRID_PLAIN_CLEARER_H_ */
