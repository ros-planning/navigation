#ifndef COSTMAP_2D_COSTMAP_2D_AxisAlignedBoundingBox_H_
#define COSTMAP_2D_COSTMAP_2D_AxisAlignedBoundingBox_H_

#include <vector>

namespace costmap_2d
{

/**
 * @class AxisAlignedBoundingBox
 * @brief Integer based Bounding Box with methods to expand, clamp and test against points or regions
 */
class AxisAlignedBoundingBox
{
public:
  /**
   * @brief Create an empty, uninitialized Axis Aligned Bounding Box with both points at the origin
   */
  AxisAlignedBoundingBox();

  /**
   * @brief Copy constructor
   */
  AxisAlignedBoundingBox(const AxisAlignedBoundingBox& r);

  /**
   * @brief Create an empty Axis Aligned Bounding Box with both points at the given location
   */
  AxisAlignedBoundingBox(int px, int py);

  /**
   * @brief Create an Axis Aligned Bounding Box with corners at the given points
   */
  AxisAlignedBoundingBox(int px1, int py1, int px2, int py2);

  /**
   * @brief Determine if this AxisAlignedBoundingBox is the same as another with possible tolerance
   * @param r AxisAlignedBoundingBox to compare against
   * @param tol Tolerance of compairson (default 0)
   */
  bool same(const AxisAlignedBoundingBox& r, int tol=0);


  /**
   * @brief Copy bounding coordinates to a destination Bounding Box
   * @param dst AxisAlignedBoundingBox to receive coodinates
   */
  void copyTo(AxisAlignedBoundingBox& dst) const;

  /**
   * @brief Expand the bounding box so that it contains the given bounding box
   * @param r AxisAlignedBoundingBox to compare against
   * @param tol Tolerance of compairson (default 0)
   */
  void expandBoundingBox(const AxisAlignedBoundingBox& bb);

  /**
   * @brief Expand the bounding box by a constant value
   * @param r expansion margin
   */
  void expandBoundingBox(int r);


  /**
   * @brief Test if a point is inside a Bounding Box
   * @param px x coordinate of test point
   * @param py y coordinate of test point
   * @param margin extra, optional margin to be added around bounding box in test
   */
  bool inside(int px, int py, int margin = 0) const;
  
  
  /**
   * @brief Test if a bounding box is completely inside the calling bounding box
   * @param bb test bounding box
   * @param margin extra, optional margin to be added around containing bounding box in test
   */
  bool inside(const AxisAlignedBoundingBox& bb, int margin = 0) const;

  /**
   * @brief Clip the passed AxisAlignedBoundingBox against the calling AxisAlignedBoundingBox
   * @param bb AxisAlignedBoundingBox to be clipped
   * @param clipped destination AxisAlignedBoundingBox representing the clipped bounding box
   */
  void clip(const AxisAlignedBoundingBox& bb, AxisAlignedBoundingBox& clipped) const;

  /**
   * @brief Make sure max coordinates are not smaller than min coordinates.
   */
  void clampBounds();

  /**
   * @brief Compute how much of the given AxisAlignedBoundingBox is inside the calling AxisAlignedBoundingBox as a ratio of areas
   * @param bb The AxisAlignedBoundingBox to use as a test
   */
  double ratioInside(const AxisAlignedBoundingBox& bb) const;

  
  /**
   * @brief Calculate the area of the bounding box
   * @return Area of the bounding box
   */
  int area() const;

  /**
   * @brief Number of grid points in the X direction
   */
  int xn() const;
  /**
   * @brief Number of grid points in the Y direction
   */
  int yn() const;

  
  /**
   * @brief X coordinate of minimum point
   */
  int x0() const;

  /**
   * @brief Y coordinate of minimum point
   */
  int y0() const;

  /**
   * @brief Determine if this AxisAlignedBoundingBox intersects another AxisAlignedBoundingBox
   * @param bb AxisAlignedBoundingBox to compare against
   * @param margin extra margin applied to the AxisAlignedBoundingBoxs. A positive margin will make side by side AxisAlignedBoundingBoxs appear to intersect. Default 0.
   */
  bool intersect(const AxisAlignedBoundingBox& bb, int margin = 0) const;

  /** 
   * @brief Determine if bounding box has been initialized
   */
  bool initialized() const {return initialized_;}
  
  /** 
   * @brief Set the initialization value of the bounding box
   */
  void setInitialized(bool b) {initialized_ = b;}

  /** 
   * @brief Expand bounding box to make sure the given points pass the inside test
   * @param x  X coordinate
   * @param y  Y coordinate
   */
  void ensureInside(int x, int y); 

  /**
   * @brief Static method that will take a vector of bounding boxes and modify it so that overlapping boxes are merged
   * @param vec_aabb the vector of boxes that will be merged. This vector is modified to contain the solution
   * @param margin extra margin applied to the AxisAlignedBoundingBoxs. A positive margin will make side by side AxisAlignedBoundingBoxs appear to intersect. Default 0.
   */
  static void mergeIntersecting(std::vector<AxisAlignedBoundingBox>& vec_aabb, int margin=0);

  int min_x_, min_y_;
  int max_x_, max_y_;
  bool initialized_;
};

}  // namespace costmap_2d

#endif  // COSTMAP_2D_COSTMAP_2D_AxisAlignedBoundingBox_H_
