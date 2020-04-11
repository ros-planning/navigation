/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Badger Technologies LLC
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
 * Author: C. Andy Martin
 *********************************************************************/
#ifndef COSTMAP_3D_FCL_HELPER_H_
#define COSTMAP_3D_FCL_HELPER_H_

#include <fcl/geometry/shape/utility.h>
#include <pcl/point_types.h>
#include <geometry_msgs/Pose.h>

namespace costmap_3d
{

template <typename S>
inline fcl::Transform3<S> poseToFCLTransform(const geometry_msgs::Pose& pose)
{
  return fcl::Transform3<S>(
      fcl::Translation3<S>(pose.position.x, pose.position.y, pose.position.z) *
      fcl::Quaternion<S>(pose.orientation.w,
                         pose.orientation.x,
                         pose.orientation.y,
                         pose.orientation.z));
}

template <typename S>
inline fcl::Halfspace<S> convertTriangleToHalfspace(const fcl::TriangleP<S>& triangle)
{
  // Find the normal for the triangle
  fcl::Vector3<S> vec_a_b = triangle.a - triangle.b;
  fcl::Vector3<S> vec_a_c = triangle.a - triangle.c;
  fcl::Vector3<S> normal = vec_a_b.cross(vec_a_c).normalized();
  S plane_distance = triangle.a.dot(normal);
  return fcl::Halfspace<S>(normal, plane_distance);
}

// FCL does not have a box to halfspace distance function.
// It does have a collision function, which does much of the same work, but
// only returns a binary result.
// The code assumes the box is not rotated, which is true of octomap boxes in
// the fixed frame.
template <typename S>
inline S boxHalfspaceSignedDistance(const fcl::Box<S>& box,
                                    const fcl::Transform3<S>& box_tf,
                                    const fcl::Halfspace<S>& halfspace)
{
  fcl::Vector3<S> normal = halfspace.n;
  fcl::Vector3<S> n_dot_d_components(
      normal[0] * box.side[0],
      normal[1] * box.side[1],
      normal[2] * box.side[2]);
  fcl::Vector3<S> n_dot_d_abs = n_dot_d_components.cwiseAbs();
  S box_extent = 0.5 * (n_dot_d_abs[0] + n_dot_d_abs[1] + n_dot_d_abs[2]);
  S center_distance = halfspace.signedDistance(box_tf.translation());
  return center_distance - box_extent;
}

template <typename S>
inline fcl::Vector3<S> convertPCLPointToFCL(const pcl::PointXYZ& p)
{
  return fcl::Vector3<S>(p.x, p.y, p.z);
}

template <typename S>
inline pcl::PointXYZ convertFCLPointToPCL(const fcl::Vector3<S>& p)
{
  return pcl::PointXYZ(p.x(), p.y(), p.z());
}

}  // namespace costmap_3d

#endif  // COSTMAP_3D_FCL_HELPER_H_
