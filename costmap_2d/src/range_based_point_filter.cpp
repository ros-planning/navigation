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

#include <sstream>

#include "costmap_2d/range_based_point_filter.h"

namespace costmap_2d
{

RangeBasedPointFilter::RangeBasedPointFilter()
{
  clear();
}

void RangeBasedPointFilter::clear()
{
  control_points_.clear();
  control_points_.push_back( ControlPoint( 0, 0 ));
  point_reference_rel_filter_.setIdentity();
}

/** @brief Return a string describing the control points of the filter. */
std::string RangeBasedPointFilter::describe() const
{
  std::stringstream out;
  out.precision( 4 );
  for( unsigned int i = 1; i < control_points_.size(); i++ )
  {
    out << control_points_[ i ].range_ << " ";
    out << control_points_[ i ].z_ << "  ";
  }
  return out.str();
}

void RangeBasedPointFilter::readDescription( const std::string& description )
{
  std::stringstream in( description );
  float range, z;
  in >> range >> z;
  while( in )
  {
    addControlPoint( range, z );
    in >> range >> z;
  }
}

void RangeBasedPointFilter::addControlPoint( float range, float z )
{
  // If speed becomes important here, convert this to a binary search.
  unsigned int i;
  for( i = 0; i < control_points_.size(); i++ )
  {
    if( range < control_points_[ i ].range_ )
    {
      break;
    }
  }
  control_points_.insert( control_points_.begin() + i, ControlPoint( range, z ));

  // If we are inserting the second point, set the Z of the first
  // point to match it.
  if( i == 1 )
  {
    control_points_[ 0 ].z_ = z;
  }

  // Update the slope entry for the previous point.
  control_points_[ i - 1 ].slope_ =
    (control_points_[ i ].z_ - control_points_[ i - 1 ].z_ ) /
    (control_points_[ i ].range_ - control_points_[ i - 1 ].range_);

  // If the new point is not the last, update the new point's slope entry.
  if( i < control_points_.size() - 1 )
  {
    control_points_[ i ].slope_ =
      (control_points_[ i + 1 ].z_ - control_points_[ i ].z_ ) /
      (control_points_[ i + 1 ].range_ - control_points_[ i ].range_);
  }
  else // else we are the last point, so copy the slope from the prior point.
  {
    control_points_[ i ].slope_ = control_points_[ i - 1 ].slope_;
  }
}

bool RangeBasedPointFilter::checkPoint( const tf::Point& point_rel_world )
{
  // First transform point into filter frame
  tf::Point point = point_reference_rel_filter_ * point_rel_world;

  // Find range
  float range = hypot( point[ 0 ], point[ 1 ]);

  // The remainder is checking the range and z against the filter.
  // addControlPoint() ensures that there is always a control point
  // before and after any plausible input point, so we don't have to
  // check for that.
  
  // First binary search to see where the point fits in.
  int lower_bound = 0;
  int upper_bound = control_points_.size() - 1;
  while( upper_bound - lower_bound > 1 )
  {
    int test_index = (lower_bound + upper_bound) / 2;
    if( range < control_points_[ test_index ].range_ )
    {
      upper_bound = test_index;
    }
    else
    {
      lower_bound = test_index;
    }
  }

  // Now find the Z value of the threshold line
  const ControlPoint& control = control_points_[ lower_bound ];
  float z_threshold = control.z_ + control.slope_ * (range - control.range_);

  // Return true if the input Z value is above the threshold.
  return point[ 2 ] > z_threshold;
}

void RangeBasedPointFilter::setFilterPose( tf::Pose filter_pose )
{
  point_reference_rel_filter_ = filter_pose.inverse();
}

} // end namespace costmap_2d
