/*
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

#include <gtest/gtest.h>
#include <costmap_2d/observation_buffer.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/Point32.h>

#include <algorithm>
#include <set>
#include <utility>

/**
 * @brief Compares the points giving x precedence over y and y precedence over z.
 */
struct compare_points
{
  bool operator()(const geometry_msgs::Point32 &l, const geometry_msgs::Point32 &r) const
  {
    if (l.x == r.x)
    {
      if (l.y == r.y)
        return l.z < r.z;
      return l.y < r.y;
    }
    return l.x < r.x;
  }
};

/**
 * @brief Sorts the points using the compare_points functor.
 */
std::set<geometry_msgs::Point32, compare_points>
sort_points(const sensor_msgs::PointCloud &cloud)
{
  std::set<geometry_msgs::Point32, compare_points> sorted_points;
  for (const auto &p : cloud.points)
    sorted_points.insert(p);

  return sorted_points;
}

bool equal(const sensor_msgs::PointCloud &l, const sensor_msgs::PointCloud &r)
{
  // Sort the points.
  const auto l_points = sort_points(l);
  const auto r_points = sort_points(r);

  // Compare the results.
  return l_points == r_points;
}

TEST(heightFilterPointCloud, empty)
{
  sensor_msgs::PointCloud2 input;
  ASSERT_NO_THROW(costmap_2d::heightFilterPointCloud(input, 0, 1));
}

TEST(heightFilterPointCloud, no_z)
{
  sensor_msgs::PointCloud2 input;
  input.point_step = 1;
  ASSERT_NO_THROW(costmap_2d::heightFilterPointCloud(input, 0, 1));
}

/**
 * @brief A copying version of the box-filter.
 * 
 * @param _cloud2 The point-cloud to copy.
 * @param _min_x The lower bound.
 * @param _max_x The upper bound.
 */
void copyingHeightFilterPointCloud(sensor_msgs::PointCloud &_cloud,
                                   double _min_z, double _max_z)
{
  sensor_msgs::PointCloud out = _cloud;
  auto dd = out.points.begin();

  for (auto ss = _cloud.points.begin(); ss != _cloud.points.end(); ++ss)
  {
    if (ss->z <= _max_z && ss->z >= _min_z)
    {
      *dd = *ss;
      ++dd;
    }
  }

  out.points.erase(dd, out.points.end());
  _cloud = out;
}

TEST(heightFilterPointCloud, no_change)
{
  // Create a simple point cloud
  sensor_msgs::PointCloud points, output;
  points.points.resize(100);

  double x = 0;
  for (auto &p : points.points)
  {
    p.x = ++x;
    p.z = 1;
  }

  sensor_msgs::PointCloud2 input;
  sensor_msgs::convertPointCloudToPointCloud2(points, input);
  costmap_2d::heightFilterPointCloud(input, 0, 2);

  ASSERT_TRUE(sensor_msgs::convertPointCloud2ToPointCloud(input, output));

  // We should keep everything.
  ASSERT_EQ(points, output);
}

/// The base fixture defining a pair (min_z, max_z) as parameter.
struct FilterFixture : public testing::TestWithParam<std::pair<double, double>>
{
};

using RemoveAllFixture = FilterFixture;
INSTANTIATE_TEST_CASE_P(/**/, RemoveAllFixture,
                        testing::Values(
                            std::make_pair(0., 1.9), // all values above
                            std::make_pair(2.1, 3.)  // all values below
                            ));

TEST_P(RemoveAllFixture, regression)
{
  // Create a point cloud with uniform points.
  sensor_msgs::PointCloud points, output;
  geometry_msgs::Point32 point;
  point.z = 2;
  points.points.resize(100, point);

  // Apply the filter.
  sensor_msgs::PointCloud2 input;
  sensor_msgs::convertPointCloudToPointCloud2(points, input);
  const auto param = GetParam();
  costmap_2d::heightFilterPointCloud(input, param.first, param.second);

  ASSERT_TRUE(sensor_msgs::convertPointCloud2ToPointCloud(input, output));

  // We should remove everything.
  ASSERT_TRUE(output.points.empty());
}

using KeepSomeFixture = FilterFixture;
INSTANTIATE_TEST_CASE_P(/**/, KeepSomeFixture,
                        testing::Values(
                            std::make_pair(0., 1.),  // keep lower first half
                            std::make_pair(1., 3.),  // keep second half
                            std::make_pair(0.5, 1.5) // keep middle
                            ));

TEST_P(KeepSomeFixture, regression)
{
  // Create a point cloud with different x and z values.
  sensor_msgs::PointCloud points, output;
  points.points.resize(100);

  double x = 0;
  for (auto &p : points.points)
  {
    p.x = ++x;
    p.z = x * 0.02;
  }

  // Shuffle the points
  std::random_shuffle(points.points.begin(), points.points.end());

  // Apply the filter.
  sensor_msgs::PointCloud2 input;
  sensor_msgs::convertPointCloudToPointCloud2(points, input);
  const auto param = GetParam();
  costmap_2d::heightFilterPointCloud(input, param.first, param.second);

  ASSERT_TRUE(sensor_msgs::convertPointCloud2ToPointCloud(input, output));

  // Apply the copying filter
  copyingHeightFilterPointCloud(points, param.first, param.second);

  ASSERT_TRUE(equal(points, output));
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
