/*
 * Copyright (c) 2013, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef RANGE_BASED_POINT_FILTER_H
#define RANGE_BASED_POINT_FILTER_H

#include <vector>
#include <pcl/point_types.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_datatypes.h>

namespace costmap_2d
{

/** @brief Class for deciding whether points are above or below a
 * height threshold which varies with distance from a point.
 *
 * The threshold is set by adding "control points" which are endpoints
 * of line segments in the range/Z plane.  The "range" coordinate
 * being the distance from the filter pose in the X/Y dimensions.
 *
 * 
 */
class RangeBasedPointFilter
{
public:
  RangeBasedPointFilter();

  /** @brief Add a control point.
   *
   * Control points can be added in any order. */
  void addControlPoint( float range, float z );

  /** @brief Return true if a given point relative to the world frame
   * is above the filter threshold.
   *
   * This uses the same frame of reference as that in
   * setFilterPose(). */
  bool checkPoint( const tf::Point& point_rel_world );

  bool checkPoint( const geometry_msgs::Point& point_msg )
    {
      tf::Point point;
      tf::pointMsgToTF( point_msg, point );
      return checkPoint( point );
    }

  bool checkPoint( const pcl::PointXYZ& pcl_point )
    {
      return checkPoint( tf::Point( pcl_point.x, pcl_point.y, pcl_point.z ));
    }

  /** @brief Set the pose of this filter relative to the world frame.
   *
   * The "world frame" is the same frame in which points should be
   * checked with checkPoint(). */
  void setFilterPose( tf::Pose filter_pose );

  /** @brief Clears the filter back to original state, as if newly constructed. */
  void clear();

  /** @brief Return a string describing the control points of the filter. */
  std::string describe() const;

  /** @brief Initialize this filter based on the given description. */
  void readDescription( const std::string& description );

  /** @brief Return the number of internal control points.  This may
   * not be the same as the number of times addControlPoint() was
   * called. */
  int size() const { return control_points_.size(); }

  /** @brief Return the range value of the ith control point. */
  float getRange( int i ) const { return control_points_[ i ].range_; }

  /** @brief Return the Z value of the ith control point. */
  float getZ( int i ) const { return control_points_[ i ].z_; }

private:
  struct ControlPoint
  {
    float range_;
    float z_;
    float slope_; ///< The slope of the line segment away from here towards the next control point.
    ControlPoint( float range, float z ): range_( range ), z_( z ), slope_( 0 ) {}
  };

  std::vector<ControlPoint> control_points_; ///< Array of control points, sorted by range.
  tf::Pose point_reference_rel_filter_;
};

} // end namespace costmap_2d

#endif // RANGE_BASED_POINT_FILTER_H
