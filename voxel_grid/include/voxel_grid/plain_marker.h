#ifndef VOXEL_GRID_PLAIN_MARKER_H_
#define VOXEL_GRID_PLAIN_MARKER_H_

#include <voxel_grid/abstract_grid_updater.h>

namespace voxel_grid
{

class PlainMarker : public AbstractGridUpdater
{
public:
  PlainMarker(uint32_t* voxel_grid_data) :
      AbstractGridUpdater(voxel_grid_data, NULL, 0, 0, 0, 0)
  {
  }

  inline virtual void operator()(unsigned int offset, uint32_t z_mask)
  {
    voxel_grid_data_[offset] |= ~(z_mask);
  }
};

} //end namespace

#endif /* VOXEL_GRID_PLAIN_MARKER_H_ */
