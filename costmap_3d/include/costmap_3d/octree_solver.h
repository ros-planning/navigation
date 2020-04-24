/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2016, Open Source Robotics Foundation
 *  Copyright (c) 2019-2020, Badger Technologies, LLC
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
 *   * Neither the name of Open Source Robotics Foundation nor the names of its
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
 */

#ifndef COSTMAP_3D_COSTMAP_MESH_DISTANCE_H
#define COSTMAP_3D_COSTMAP_MESH_DISTANCE_H

#include <vector>
#include <limits>
#include <functional>

#include <fcl/math/bv/utility.h>
#include <fcl/geometry/octree/octree.h>
#include <fcl/geometry/bvh/BVH_model.h>
#include <fcl/geometry/shape/utility.h>
#include <fcl/geometry/shape/box.h>
#include <fcl/narrowphase/distance_request.h>
#include <fcl/narrowphase/distance_result.h>

namespace costmap_3d
{

/** Find distance between an FCL OcTree and a mesh representing a closed volume.
 *
 * FCL does not currently have any API support for treating a mesh as a closed
 * volume. For Costmap purposes, the mesh is always treated as a closed (but
 * not necessarily convex) volume. Because of this, FCL's octree distance
 * function will not find interior collisions and is therefore inadequate for
 * costmap queries.
 */
template <typename NarrowPhaseSolver>
class OcTreeMeshSolver
{
  using S = typename NarrowPhaseSolver::S;
public:
  // Function returning the negative penetration distance of the box into the
  // mesh. If the box is not entirely within the mesh, this function may simply
  // return any non-negative number. This function must always return the
  // negative penetration distance when the entire box is inside the mesh. It
  // is OK for it to miss casses when the box and the mesh touch, as those are
  // handled directly by the solver traversal.  The solver needs to know when
  // an interior collision happens (and how deep it is), but knowing that is
  // outside of its scope. It uses this function to find out if there is an
  // interior collision and the penetration depth.
  //
  // The box_to_mesh_tf must put the box into the *mesh* frame.
  using InteriorCollisionFunction = std::function<S(const fcl::Box<S>& box,
                                                    const fcl::Transform3<S>& box_tf,
                                                    const fcl::Transform3<S>& mesh_tf,
                                                    const fcl::Transform3<S>& inverse_mesh_tf,
                                                    int* mesh_trinagle_id_ptr)>;
  using SignedDistanceFunction = std::function<S(const fcl::Box<S>& box,
                                                 const fcl::Transform3<S>& box_tf,
                                                 int mesh_triangle_id,
                                                 const fcl::Transform3<S>& mesh_tf)>;

  // If interior_collision_function is given, that function is used to detect
  // interior collisions, and to directly estimate the distance for an interior
  // collision. This makes searching all interior collisions fast enough to
  // find the deepest penetrating collision for calculating signed distance,
  // and also correctly models the mesh as a closed volume.
  // If no interior_collision_function is given, the nearest triangle to the
  // mesh will be used.
  // If signed_distance_function is given and doing signed distance queries,
  // the returned distance for a collision between the mesh and the octomap
  // will use this function to modify the distance. This can be used to model
  // the signed distance for the boundary as a halfspace-box penetration, for
  // instance.
  // When doing signed distance queries, all octomap boxes that touch the mesh,
  // or are in the volume (when interior_collision_function is given) are
  // queried to find the deepest penetration to return as the signed distance.
  // When doing non-signed distance, once it is known the mesh and map collide,
  // -1.0 is immediately returned.
  OcTreeMeshSolver(
      const NarrowPhaseSolver* solver,
      InteriorCollisionFunction interior_collision_function = InteriorCollisionFunction(),
      SignedDistanceFunction signed_distance_function = SignedDistanceFunction())
      : solver_(solver),
        interior_collision_function_(interior_collision_function),
        signed_distance_function_(signed_distance_function)
  {
  }

  /** Distance between fcl OcTree and fcl BVHModel (mesh)
   *
   * Note: request and result must not be shared between threads.
   */
  template <typename BV>
  void distance(const fcl::OcTree<S>* tree1,
                const fcl::BVHModel<BV>* tree2,
                const fcl::Transform3<S>& tf1,
                const fcl::Transform3<S>& tf2,
                const fcl::DistanceRequest<S>& request,
                fcl::DistanceResult<S>* result) const;

private:
  const NarrowPhaseSolver* solver_;
  InteriorCollisionFunction interior_collision_function_;
  SignedDistanceFunction signed_distance_function_;
  // This is only mutated when OcTreeMeshDistance is called and is used to
  // keep from having to put the request and result on the stack for every
  // recursive call.
  mutable const fcl::DistanceRequest<S>* drequest_ = nullptr;
  mutable fcl::DistanceResult<S>* dresult_ = nullptr;
  mutable fcl::Transform3<S> mesh_tf_inverse_;
  mutable double rel_err_factor_;

  template <typename BV>
  bool OcTreeMeshDistanceRecurse(const fcl::OcTree<S>* tree1,
                                 const typename fcl::OcTree<S>::OcTreeNode* root1,
                                 const fcl::AABB<S>& bv1,
                                 const fcl::BVHModel<BV>* tree2,
                                 int root2,
                                 const fcl::Transform3<S>& tf2,
                                 const std::vector<fcl::Halfspace<S>>* roi) const;
};

template <typename NarrowPhaseSolver>
template <typename BV>
void OcTreeMeshSolver<NarrowPhaseSolver>::distance(
    const fcl::OcTree<S>* tree1,
    const fcl::BVHModel<BV>* tree2,
    const fcl::Transform3<S>& tf1,
    const fcl::Transform3<S>& tf2,
    const fcl::DistanceRequest<S>& request,
    fcl::DistanceResult<S>* result) const
{
  drequest_ = &request;
  dresult_ = result;

  fcl::Transform3<S> mesh_tf = tf1.inverse() * tf2;
  mesh_tf_inverse_ = mesh_tf.inverse();
  rel_err_factor_ = 1.0 - drequest_->rel_err;

  OcTreeMeshDistanceRecurse(tree1,
                            tree1->getRoot(),
                            tree1->getRootBV(),
                            tree2,
                            0,
                            mesh_tf,
                            &tree1->getRegionOfInterest());
}

// Convenience function to find the distance from a cube (represented in aabb1) in I configuration
// to some other bounding volume
template <typename S, typename BV2>
inline S distanceOctomapRSS(const fcl::AABB<S>& aabb1,
                            const fcl::Vector3<S>& bv1_center,
                            const BV2& bv2,
                            const fcl::Transform3<S>& tf2,
                            const fcl::Vector3<S>& bv2_center)
{
  static fcl::Matrix3<S> axis_yzx = (fcl::Matrix3<S>() << 0, 0, 1,
                                                          1, 0, 0,
                                                          0, 1, 0).finished();
  static fcl::Matrix3<S> axis_zxy = (fcl::Matrix3<S>() << 0, 1, 0,
                                                          0, 0, 1,
                                                          1, 0, 0).finished();
  static fcl::Matrix3<S> axis_xyz = (fcl::Matrix3<S>() << 1, 0, 0,
                                                          0, 1, 0,
                                                          0, 0, 1).finished();
  // Leverage the fact that octomap cells are cubes and
  // orient the flat face of the RSS at the current other BV
  fcl::RSS<S> rss;
  fcl::Vector3<S> dir(bv1_center-bv2_center);
  dir[0] = std::abs(dir[0]);
  dir[1] = std::abs(dir[1]);
  dir[2] = std::abs(dir[2]);
  const S x = aabb1.width();
  rss.r = x / 2;
  rss.l[0] = x;
  rss.l[1] = x;
  const bool x_greater_than_y = (dir[0] > dir[1]);
  const bool y_greater_than_z = (dir[1] > dir[2]);
  if (x_greater_than_y && y_greater_than_z)
  {
    rss.axis = axis_yzx;
  }
  else if (!(x_greater_than_y) && (y_greater_than_z))
  {
    rss.axis = axis_zxy;
  }
  else
  {
    rss.axis = axis_xyz;
  }
  rss.setToFromCenter(bv1_center);
  return fcl::distanceBV(rss, bv2, tf2);
}

// return -1 if bv1 out of roi, 1 if in, and 0 if on.
template <typename S>
inline int checkROI(const fcl::AABB<S>& bv1, const std::vector<fcl::Halfspace<S>>* roi)
{
  fcl::Vector3<S> bv1_center = bv1.center();
  fcl::Vector3<S> bv1_diag = bv1.max_ - bv1.min_;
  bool all_in = true;

  for (unsigned int i=0; i<roi->size(); ++i)
  {
    // This is performance critical code.
    // So do not call boxHalfSpaceSignedDistance, but repeat the work here, as
    // we do not want to spend the time to create a Box from an AABB.
    // Also, we know that the AABB is axis-aligned with the world frame and
    // skip rotating the halfspace normal into the boxes frame.
    const fcl::Halfspace<S>& region((*roi)[i]);
    fcl::Vector3<S> normal = region.n;
    fcl::Vector3<S> n_dot_d(normal[0] * bv1_diag[0], normal[1] * bv1_diag[1], normal[2] * bv1_diag[2]);
    fcl::Vector3<S> n_dot_d_abs = n_dot_d.cwiseAbs();
    S bv1_extent = 0.5 * (n_dot_d_abs[0] + n_dot_d_abs[1] + n_dot_d_abs[2]);
    S center_dist = region.signedDistance(bv1_center);
    // If the distance from the center of the AABB in bv1 to the halfspace is
    // bigger than the maximum extent of the AABB, the AABB is outside the
    // boundary of the halfspace. If the negative of the center distance is
    // bigger than the extent, the whole AABB is inside the halfspace.
    // Otherwise, its on the boundary.
    bool out = (center_dist > bv1_extent);
    bool in = (-center_dist > bv1_extent);
    if (out)
    {
      return -1;
    }
    all_in = (all_in && in);
  }
  if (all_in)
  {
    return 1;
  }
  return 0;
}

template <typename NarrowPhaseSolver>
template <typename BV>
bool OcTreeMeshSolver<NarrowPhaseSolver>::OcTreeMeshDistanceRecurse(
    const fcl::OcTree<S>* tree1,
    const typename fcl::OcTree<S>::OcTreeNode* root1,
    const fcl::AABB<S>& bv1,
    const fcl::BVHModel<BV>* tree2,
    int root2,
    const fcl::Transform3<S>& tf2,
    const std::vector<fcl::Halfspace<S>>* roi) const
{
  // First check region of interest.
  if (roi)
  {
    int rv = checkROI<S>(bv1, roi);
    if (rv == -1)
    {
      // this octomap region is entirely out of the region of interest
      return false;
    }
    if (rv == 1)
    {
      // this octomap region is entirely inside the region of interest.
      // There is no need to check any sub-region. Null out the roi for
      // subsequent recursive calls.
      roi = nullptr;
    }
  }

  if(!tree1->nodeHasChildren(root1) && tree2->getBV(root2).isLeaf())
  {
    if(tree1->isNodeOccupied(root1))
    {
      fcl::Box<S> box;
      fcl::Transform3<S> box_tf;
      fcl::constructBox(bv1, fcl::Transform3<S>::Identity(), box, box_tf);

      int primitive_id = tree2->getBV(root2).primitiveId();
      const fcl::Triangle& tri_id = tree2->tri_indices[primitive_id];
      const fcl::Vector3<S>& p1 = tree2->vertices[tri_id[0]];
      const fcl::Vector3<S>& p2 = tree2->vertices[tri_id[1]];
      const fcl::Vector3<S>& p3 = tree2->vertices[tri_id[2]];

      S dist;
      fcl::Vector3<S> closest_p1, closest_p2;
      solver_->shapeTriangleDistance(box, box_tf, p1, p2, p3, tf2, &dist, &closest_p1, &closest_p2);
      dresult_->primative_distance_calculations++;

      if (dist < 0.0 && drequest_->enable_signed_distance && signed_distance_function_)
      {
        dist = signed_distance_function_(
            box,
            box_tf,
            primitive_id,
            tf2);
      }

      if (dist < dresult_->min_distance)
      {
        // only allocate dynamic memory in the case where a new min was found
        std::shared_ptr<fcl::Box<S>> box_ptr(new fcl::Box<S>(box));
        std::shared_ptr<fcl::TriangleP<S>> triangle(new fcl::TriangleP<S>(p1, p2, p3));
        dresult_->update(dist, tree1, tree2, root1 - tree1->getRoot(), primitive_id,
                        closest_p1, closest_p2, box_ptr, box_tf, triangle, tf2);
      }

      return drequest_->isSatisfied(*dresult_);
    }
    else
      return false;
  }

  if(!tree1->isNodeOccupied(root1)) return false;

  if(tree1->nodeHasChildren(root1))
  {
    unsigned int nchildren = 0;
    const typename fcl::OcTree<S>::OcTreeNode* children[8];
    fcl::AABB<S> child_bvs[8];
    S distances[8];
    S min_distance = std::numeric_limits<S>::max();
    S next_min;
    const BV& bv2 = tree2->getBV(root2).bv;
    const fcl::Vector3<S> bv2_center(tf2 * bv2.center());
    for(unsigned int i = 0; i < 8; ++i)
    {
      if(tree1->nodeChildExists(root1, i))
      {
        const typename fcl::OcTree<S>::OcTreeNode* child = tree1->getNodeChild(root1, i);
        if(tree1->isNodeOccupied(child))
        {
          children[nchildren] = child;
          computeChildBV(bv1, i, child_bvs[nchildren]);
          distances[nchildren] = distanceOctomapRSS(child_bvs[nchildren], child_bvs[nchildren].center(),
                                                    bv2, tf2, bv2_center);
          dresult_->bv_distance_calculations++;
          if (distances[nchildren] < min_distance)
          {
            min_distance = distances[nchildren];
          }
          nchildren++;
        }
      }
    }
    // Visit the octree from closest to furthest and quit early when we have
    // crossed the result min distance
    while(min_distance < rel_err_factor_ * dresult_->min_distance)
    {
      next_min = std::numeric_limits<S>::max();
      for(unsigned int i = 0; i < nchildren; ++i)
      {
        if(distances[i] == min_distance)
        {
          if(distances[i] < rel_err_factor_ * dresult_->min_distance)
          {
            // Possible a better result is below, descend
            if(OcTreeMeshDistanceRecurse(tree1, children[i], child_bvs[i], tree2, root2, tf2, roi))
              return true;
          }
          else
          {
            break;
          }
        }
        else if(distances[i] > min_distance)
        {
          if(distances[i] < next_min)
          {
            next_min = distances[i];
          }
        }
        else
        {
          // an already visited spot on a previous iteration
        }
      }
      min_distance = next_min;
    }
  }
  else
  {
    const fcl::Vector3<S> bv1_center(bv1.center());

    // There is no way (that I can think of) to correctly direct the search to
    // avoid having to test each box that touches the bounding volume of the
    // whole mesh. So, to make it take a reasonable amount of time to check each
    // box that is tangential to the bounding volume of the mesh (because its
    // min_distance lower bound would be zero), create a LUT to use that takes
    // as input the centroid of the octomap box, rounded to a grid, and looks up
    // the mesh triangle that is closest to a box modeling that grid cell. This
    // will give close enough results to work well for costmap queries in a
    // reasonable amount of time. After looking up the mesh (instead of having
    // to search for it through the BVH), the mesh triangle is modeled as a
    // halfspace (all halfspace models can also be created at construction time)
    // and the box to halfspace distance can be calculated (this is a fast
    // calculation compared to a GJK run). This distance will then be used as
    // the penetration distance.
    //
    // The bound distance will never be negative, clip all negatives to zero, as
    // the overlap of bounding volumes does not give an accurate penetration
    // depth. This forces the search to try all boxes within the mesh.
    //
    // The LUT and halfspaces will be pre-calculated and passed to the solver.
    // If the solver does not have them, it will proceed normally. This way,
    // the solver can be used to create the LUT by finding the distance
    // between an octomap with only one cell occupied to identify the closest
    // mesh triangle to that spot. It is outside the scope of this class how
    // to determine if the octomap box is inside the mesh, but a method like
    // PCL's crop hull could be used on the centroid of the mesh. This LUT
    // mechanism does introduce some inaccuracies when the rounded centroid
    // appears in the mesh, but is actually outside the mesh. For this reason,
    // the LUT resolution should be more than double the octomap resolution.
    // This can easily be achieved by using the transform argument to the
    // distance function. Create an octomap with a single cell at the octomap
    // index corresponding to the cell near the origin. Then provide different
    // arguments for the transform to find the nearest mesh polygon for each
    // entry.
    //
    // The LUT could also be made more accurate by taking not only position but
    // orientation as an input. This can be abstracted at the LUT level by
    // passing the transform to put the box into the mesh frame (this
    // transform can be pre-calculated once for the whole distance operation).
    // Then the LUT can decide how to find the appropriate entry.
    if (interior_collision_function_ && root2 == 0)
    {
      // There is an interior collision function and we are at the root of the BVH.
      fcl::Box<S> box;
      fcl::Transform3<S> box_tf;
      fcl::constructBox(bv1, fcl::Transform3<S>::Identity(), box, box_tf);
      int primitive_id;
      S collision_distance = interior_collision_function_(
          box,
          box_tf,
          tf2,
          mesh_tf_inverse_,
          &primitive_id);
      if (collision_distance < 0.0)
      {
        // A known interior collision.
        const fcl::Triangle& tri_id = tree2->tri_indices[primitive_id];
        const fcl::Vector3<S>& p1 = tree2->vertices[tri_id[0]];
        const fcl::Vector3<S>& p2 = tree2->vertices[tri_id[1]];
        const fcl::Vector3<S>& p3 = tree2->vertices[tri_id[2]];
        std::shared_ptr<fcl::Box<S>> box_ptr(new fcl::Box<S>(box));
        std::shared_ptr<fcl::TriangleP<S>> triangle(new fcl::TriangleP<S>(p1, p2, p3));
        // Closest point doesn't make much sense on an interior collision of two volumes.
        // Just return the center point of the box.
        dresult_->update(collision_distance, tree1, tree2, root1 - tree1->getRoot(), primitive_id,
                         box_tf.translation(), box_tf.translation(),
                         box_ptr, box_tf, triangle, tf2);
        return true;
      }
    }
    int children[2] = {
      tree2->getBV(root2).leftChild(),
      tree2->getBV(root2).rightChild()};
    const BV* bv2[2] = {
      &tree2->getBV(children[0]).bv,
      &tree2->getBV(children[1]).bv};
    S d[2] = {
      distanceOctomapRSS(bv1, bv1_center, *bv2[0], tf2, tf2 * bv2[0]->center()),
      distanceOctomapRSS(bv1, bv1_center, *bv2[1], tf2, tf2 * bv2[1]->center())};
    dresult_->bv_distance_calculations+=2;
    // Go left first if it is closer, otherwise go right first
    if (d[0] < d[1])
    {
      for (int i=0; i<2; ++i)
      {
        if(d[i] < rel_err_factor_ * dresult_->min_distance)
        {
          // Because we descend the octree first, there is no need to check the
          // ROI when descending the mesh.
          if(OcTreeMeshDistanceRecurse(tree1, root1, bv1, tree2, children[i], tf2, nullptr))
            return true;
        }
      }
    }
    else
    {
      for (int i=1; i>-1; --i)
      {
        if(d[i] < rel_err_factor_ * dresult_->min_distance)
        {
          // Because we descend the octree first, there is no need to check the
          // ROI when descending the mesh.
          if(OcTreeMeshDistanceRecurse(tree1, root1, bv1, tree2, children[i], tf2, nullptr))
            return true;
        }
      }
    }
  }

  return false;
}

}  // namespace costmap_3d

#endif  // COSTMAP_3D_COSTMAP_MESH_DISTANCE_H
