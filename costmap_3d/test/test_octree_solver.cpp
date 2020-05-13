/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Badger Technologies LLC
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
 *
 * Author: C. Andy Martin
 *********************************************************************/

#include <gtest/gtest.h>

#include <fcl/config.h>
#include <fcl/geometry/octree/octree.h>
#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include <costmap_3d/costmap_3d_query.h>
#include <costmap_3d/octree_solver.h>

static const std::string PACKAGE_URL("package://costmap_3d/");

void octree_solver_test(std::size_t n, double resolution = 0.1, bool negative_x_roi = false, bool non_negative_x_roi = false);

TEST(test_octree_solver, test_against_fcl)
{
  octree_solver_test(15, 0.1, false, false);
  octree_solver_test(15, 0.1, true, false);
  octree_solver_test(15, 0.1, false, true);
}

template <typename S>
S rand_interval(S rmin, S rmax)
{
  S t = rand() / ((S)RAND_MAX + 1);
  return (t * (rmax - rmin) + rmin);
}

template <typename S>
void eulerToMatrix(S a, S b, S c, fcl::Matrix3<S>& R)
{
  auto c1 = std::cos(a);
  auto c2 = std::cos(b);
  auto c3 = std::cos(c);
  auto s1 = std::sin(a);
  auto s2 = std::sin(b);
  auto s3 = std::sin(c);

  R << c1 * c2, - c2 * s1, s2,
      c3 * s1 + c1 * s2 * s3, c1 * c3 - s1 * s2 * s3, - c2 * s3,
      s1 * s3 - c1 * c3 * s2, c3 * s1 * s2 + c1 * s3, c2 * c3;
}

template <typename S>
void generateRandomTransforms(S extents[6], fcl::aligned_vector<fcl::Transform3<S>>& transforms, std::size_t n)
{
  transforms.resize(n);
  for(std::size_t i = 0; i < n; ++i)
  {
    auto x = rand_interval(extents[0], extents[3]);
    auto y = rand_interval(extents[1], extents[4]);
    auto z = rand_interval(extents[2], extents[5]);

    const auto pi = fcl::constants<S>::pi();
    auto a = rand_interval((S)0, 2 * pi);
    auto b = rand_interval((S)0, 2 * pi);
    auto c = rand_interval((S)0, 2 * pi);

    {
      fcl::Matrix3<S> R;
      eulerToMatrix(a, b, c, R);
      fcl::Vector3<S> T(x, y, z);
      transforms[i].setIdentity();
      transforms[i].linear() = R;
      transforms[i].translation() = T;
    }
  }
}

template <typename S>
void generateBoxesFromOctomap(
    const fcl::OcTree<S>& tree,
    std::vector<std::shared_ptr<fcl::CollisionObject<S>>>* boxes)
{
  std::vector<std::array<S, 6>> tree_boxes = tree.toBoxes();

  for(std::size_t i = 0; i < tree_boxes.size(); ++i)
  {
    S x = tree_boxes[i][0];
    S y = tree_boxes[i][1];
    S z = tree_boxes[i][2];
    S size = tree_boxes[i][3];
    S cost = tree_boxes[i][4];
    S threshold = tree_boxes[i][5];

    std::shared_ptr<fcl::CollisionGeometry<S>> box(new fcl::Box<S>(size, size, size));
    box->cost_density = cost;
    box->threshold_occupied = threshold;
    std::shared_ptr<fcl::CollisionObject<S>> obj(
        new fcl::CollisionObject<S>(
            box,
            fcl::Transform3<S>(fcl::Translation3<S>(fcl::Vector3<S>(x, y, z)))));
    boxes->push_back(obj);
  }
}

template <typename S>
struct DistanceData
{
  fcl::DistanceRequest<S> request;
  fcl::DistanceResult<S> result;
  bool done = false;
};

template <typename S>
bool defaultDistanceFunction(fcl::CollisionObject<S>* o1, fcl::CollisionObject<S>* o2, void* cdata_, S& dist)
{
  auto* cdata = static_cast<DistanceData<S>*>(cdata_);
  const fcl::DistanceRequest<S>& request = cdata->request;
  fcl::DistanceResult<S>& result = cdata->result;

  if(cdata->done) { dist = result.min_distance; return true; }

  fcl::distance(o1, o2, request, result);

  dist = result.min_distance;

  if(dist <= 0) return true; // in collision or in touch

  return cdata->done;
}

void octree_solver_test(std::size_t n, double resolution, bool negative_x_roi, bool non_negative_x_roi)
{
  using S = costmap_3d::Costmap3DQuery::FCLFloat;
  costmap_3d::Costmap3DPtr octree(new costmap_3d::Costmap3D(
          costmap_3d::Costmap3DQuery::getFileNameFromPackageURL(PACKAGE_URL + "test/aisles.bt")));
  std::shared_ptr<fcl::OcTree<S>> tree_ptr(new fcl::OcTree<S>(octree));

  // Use Costmap3DQuery to get BVH for test mesh
  costmap_3d::Costmap3DQuery query(octree, PACKAGE_URL + "test/test_robot.stl");
  costmap_3d::Costmap3DQuery::FCLRobotModelConstPtr m1 = query.getFCLRobotModel();
  std::shared_ptr<const fcl::CollisionGeometry<S>> m1_ptr(m1);

  if (negative_x_roi)
  {
    fcl::Vector3<S> normal(1.0, 0.0, 0.0);
    fcl::Halfspace<S> negative_x(normal, -resolution/2.0);
    tree_ptr->addToRegionOfInterest(negative_x);
  }
  if (non_negative_x_roi)
  {
    fcl::Vector3<S> normal(-1.0, 0.0, 0.0);
    fcl::Halfspace<S> non_negative_x(normal, -resolution/2.0);
    tree_ptr->addToRegionOfInterest(non_negative_x);
  }

  fcl::aligned_vector<fcl::Transform3<S>> transforms;
  S extents[] = {-2, -2, -2, 2, 2, 2};

  generateRandomTransforms(extents, transforms, n);
  if (n > 1)
  {
    // Be sure to test identity
    transforms[n - 1] = fcl::Transform3<S>::Identity();
    transforms[n / 2 - 1] = fcl::Transform3<S>::Identity();
    transforms[n / 2] = fcl::Transform3<S>::Identity();
  }

  for(std::size_t i = 0; i < n; ++i)
  {
    fcl::Transform3<S> tf1(transforms[i]);
    fcl::Transform3<S> tf2(transforms[n-1-i]);

    fcl::detail::GJKSolver_libccd<S> solver;
    costmap_3d::OcTreeMeshSolver<fcl::detail::GJKSolver_libccd<S>> octree_solver(&solver);
    fcl::DistanceRequest<S> request;
    fcl::DistanceResult<S> result;
    request.rel_err = 0.0;
    request.enable_nearest_points = true;
    request.enable_signed_distance = true;
    result.min_distance = std::numeric_limits<S>::max();
    octree_solver.distance(
        tree_ptr.get(),
        m1.get(),
        tf1,
        tf2,
        request,
        &result);
    S dist1 = result.min_distance;

    // Check the result against FCL's broadphase distance
    std::vector<std::shared_ptr<fcl::CollisionObject<S>>> boxes;
    generateBoxesFromOctomap<S>(*tree_ptr, &boxes);
    for(std::size_t j = 0; j < boxes.size(); ++j)
      boxes[j]->setTransform(tf2 * boxes[j]->getTransform());

    fcl::DynamicAABBTreeCollisionManager<S> manager;
    for (auto box : boxes)
    {
      manager.registerObject(box.get());
    }
    manager.setup();

    DistanceData<S> cdata2;
    fcl::CollisionObject<S> obj1(std::const_pointer_cast<fcl::CollisionGeometry<S>>(m1_ptr), tf1);
    manager.distance(&obj1, &cdata2, defaultDistanceFunction);
    S dist2 = cdata2.result.min_distance;

    if (dist1 > 1e-6 && dist2 > 1e-6)
    {
      EXPECT_NEAR(dist1, dist2, 1e-6);
    }
    else
    {
      // Signed distance is allowed to not be equivalent between FCL broadphase
      // and the costmap 3D octree solver
      EXPECT_TRUE(dist1 < 1e-6 && dist2 < 1e-6);
    }
  }
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
