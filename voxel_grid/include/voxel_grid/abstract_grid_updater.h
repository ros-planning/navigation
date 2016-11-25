#ifndef VOXEL_GRID_ABSTRACT_GRID_UPDATER_H_
#define VOXEL_GRID_ABSTRACT_GRID_UPDATER_H_

namespace voxel_grid
{

class AbstractGridUpdater
{
public:
  AbstractGridUpdater(uint32_t* voxel_grid_data, unsigned char* costmap, unsigned int unknown_clear_threshold,
                  unsigned int marked_clear_threshold, unsigned char free_cost = 0, unsigned char unknown_cost = 255) :
      voxel_grid_data_(voxel_grid_data), costmap_(costmap)
  {
    unknown_clear_threshold_ = unknown_clear_threshold;
    marked_clear_threshold_ = marked_clear_threshold;
    free_cost_ = free_cost;
    unknown_cost_ = unknown_cost;
  }

  virtual ~AbstractGridUpdater() {};

  virtual void operator()(unsigned int offset, uint32_t z_mask) =0;

protected:
  static inline bool bitsBelowThreshold(unsigned int bits, unsigned int bit_threshold)
  {
    unsigned int bit_count;
    for (bit_count = 0; bits;)
    {
      ++bit_count;
      if (bit_count > bit_threshold)
      {
        return false;
      }
      bits &= bits - 1; //clear the least significant bit set
    }
    return true;
  }

  uint32_t* voxel_grid_data_;
  unsigned char* costmap_;
  unsigned int unknown_clear_threshold_, marked_clear_threshold_;
  unsigned char free_cost_, unknown_cost_;
};

} //end namespace

#endif /* VOXEL_GRID_ABSTRACT_GRID_UPDATER_H_ */
