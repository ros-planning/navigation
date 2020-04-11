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
#ifndef COSTMAP_3D_INTERIOR_COLLISON_LUT_H_
#define COSTMAP_3D_INTERIOR_COLLISON_LUT_H_

#include <memory>
#include <vector>

#include <fcl/geometry/bvh/BVH_model.h>
#include <fcl/geometry/octree/octree.h>
#include <fcl/narrowphase/distance.h>
#include <pcl/point_types.h>
#include <pcl/io/vtk_lib_io.h>
#include <costmap_3d/fcl_helper.h>
#include <costmap_3d/crop_hull.h>
#include <costmap_3d/octree_solver.h>

namespace costmap_3d
{

template <typename S>
class InteriorCollisionLUT
{
  using Mesh = fcl::BVHModel<fcl::OBBRSS<S>>;
  using GridPoint = Eigen::Matrix<unsigned, 3, 1>;
  using Halfspaces = std::vector<fcl::Halfspace<S>>;
  using HalfspacesPtr = std::shared_ptr<const Halfspaces>;
  using FCLSolver = fcl::detail::GJKSolver_libccd<S>;

public:
  // If setup ever takes too long to run on startup, its work could be
  // deferred, or its construction could be saved to disk.
  void setup(
      S box_size,
      S resolution,
      const Mesh& mesh,
      const CropHull<pcl::PointXYZ>& crop_hull,
      const HalfspacesPtr& halfspaces);

  S distance(
      const fcl::Box<S>& box,
      const fcl::Transform3<S>& box_tf,
      const fcl::Transform3<S>& mesh_tf,
      const fcl::Transform3<S>& inverse_mesh_tf,
      int* mesh_triangle_id_ptr = nullptr);

private:
  inline GridPoint meshToGrid(const fcl::Vector3<S>& mesh_pt) const
  {
    GridPoint rv;
    for (unsigned int i = 0; i < 3; ++i)
    {
      rv[i] = std::floor((mesh_pt[i] - origin_[i]) / resolution_);
    }
    return rv;
  }
  inline fcl::Vector3<S> gridToMesh(const GridPoint& grid_pt) const
  {
    fcl::Vector3<S> rv;
    for (unsigned int i = 0; i < 3; ++i)
    {
      rv[i] = grid_pt[i] * resolution_ + origin_[i] + resolution_ / 2.0;
    }
    return rv;
  }
  inline bool gridPointInBounds(const GridPoint& grid_pt) const
  {
    for (unsigned int i = 0; i < 3; ++i)
    {
      if (grid_pt[i] < 0 || grid_pt[i] >= dimensions_[i])
      {
        return false;
      }
    }
    return true;
  }
  inline size_t lutIndex(const GridPoint& grid_pt) const
  {
    return grid_pt[2] * dimensions_[0] * dimensions_[1] +
        grid_pt[1] * dimensions_[0] +
        grid_pt[0];
  }
  HalfspacesPtr halfspaces_;
  fcl::Vector3<S> origin_;
  unsigned dimensions_[3];
  S resolution_;
  std::vector<int> lut_;
};

template <typename S>
void InteriorCollisionLUT<S>::setup(
    S box_size,
    S resolution,
    const Mesh& mesh,
    const CropHull<pcl::PointXYZ>& crop_hull,
    const HalfspacesPtr& halfspaces)
{
  resolution_ = resolution;
  halfspaces_ = halfspaces;
  const fcl::Vector3<S>& min = mesh.aabb_local.min_;
  const fcl::Vector3<S>& max = mesh.aabb_local.max_;
  for (unsigned int i=0; i<3; ++i)
  {
    origin_[i] = std::floor(min[i] / resolution_) * resolution_;
    dimensions_[i] = static_cast<unsigned>(std::floor((max[i] - origin_[i]) / resolution_)) + 1;
  }
  lut_.resize(dimensions_[0] * dimensions_[1] * dimensions_[2]);
  assert(lut_.size() > 0);

  FCLSolver solver;
  OcTreeMeshSolver<FCLSolver> octree_solver(&solver);
  fcl::DistanceRequest<S> request;
  request.enable_nearest_points = true;

  std::shared_ptr<octomap::OcTree> singleton_map(new octomap::OcTree(box_size));
  octomap::point3d box_center(box_size/2.0, box_size/2.0, box_size/2.0);
  singleton_map->updateNode(box_center, true);
  fcl::Transform3<S> tf = fcl::Transform3<S>::Identity();
  fcl::OcTree<S> fcl_singleton_map(singleton_map);
  pcl::PointCloud<pcl::PointXYZ> singleton_cloud;
  singleton_cloud.resize(1);
  singleton_cloud.is_dense = true;
  singleton_cloud.width = 1;
  singleton_cloud.height = 1;
  std::vector<int> indices;

  GridPoint grid_pt;
  for (grid_pt[2] = 0; grid_pt[2] < dimensions_[2]; ++grid_pt[2])
  {
    for (grid_pt[1] = 0; grid_pt[1] < dimensions_[1]; ++grid_pt[1])
    {
      for (grid_pt[0] = 0; grid_pt[0] < dimensions_[0]; ++grid_pt[0])
      {
        // Get the octomap occupied box centered on the origin
        // and move the centered octomap to the given point
        tf.translation() =
            fcl::Vector3<S>(-box_size/2.0, -box_size/2.0, -box_size/2.0) +
            gridToMesh(grid_pt);
        // Check if this point is an interior point.
        singleton_cloud.points[0] = convertFCLPointToPCL<S>(tf.translation());
        indices.clear();
        crop_hull.applyFilter(singleton_cloud, indices);
        int triangle_id = -1;
        if (indices.size() > 0)
        {
          // This is an interior point
          // Now find the closest triangle ...
          fcl::DistanceResult<S> result;
          octree_solver.distance(
              &fcl_singleton_map,
              &mesh,
              tf,
              fcl::Transform3<S>::Identity(),
              request,
              &result);
          // Save the mesh triangle index in the robot model
          triangle_id = result.b2;
        }
        lut_[lutIndex(grid_pt)] = triangle_id;
      }
    }
  }
}

template <typename S>
S InteriorCollisionLUT<S>::distance(
    const fcl::Box<S>& box,
    const fcl::Transform3<S>& box_tf,
    const fcl::Transform3<S>& mesh_tf,
    const fcl::Transform3<S>& inverse_mesh_tf,
    int* mesh_triangle_id_ptr)
{
  GridPoint grid_pt = meshToGrid(inverse_mesh_tf * box_tf.translation());
  if (gridPointInBounds(grid_pt))
  {
    int mesh_triangle_id = lut_[lutIndex(grid_pt)];
    if (mesh_triangle_id >= 0)
    {
      if (mesh_triangle_id_ptr != nullptr)
      {
        *mesh_triangle_id_ptr = mesh_triangle_id;
      }
      fcl::Halfspace<S> halfspace(fcl::transform(
              (*halfspaces_)[mesh_triangle_id], mesh_tf));
      return boxHalfspaceSignedDistance<S>(box, box_tf, halfspace);
    }
  }
  return 0.0;
}

}  // namespace costmap_3d

#endif  // COSTMAP_3D_INTERIOR_COLLISON_LUT_H_
