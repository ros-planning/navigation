#include <costmap_2d/costmap_2d.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <tuple>

/// @brief Helper class which contains the legacy implementation of updateOrigin.
struct Costmap2DLegacy : public costmap_2d::Costmap2D
{

  Costmap2DLegacy(unsigned int cells_size_x, unsigned int cells_size_y, double resolution,
                  double origin_x, double origin_y) : costmap_2d::Costmap2D(cells_size_x, cells_size_y, resolution, origin_x, origin_y) {}

  // This code is the legacy version of the updateOrigin method. It's well proven
  // and requires a temporal buffer for updating the costmap. Our tests will use
  // this method as ground-truth.
  void updateOrigin(double new_origin_x, double new_origin_y) override
  {
    int cell_ox, cell_oy;
    cell_ox = int((new_origin_x - origin_x_) / resolution_);
    cell_oy = int((new_origin_y - origin_y_) / resolution_);

    if (cell_ox == 0 && cell_oy == 0)
      return;

    double new_grid_ox, new_grid_oy;
    new_grid_ox = origin_x_ + cell_ox * resolution_;
    new_grid_oy = origin_y_ + cell_oy * resolution_;

    int size_x = size_x_;
    int size_y = size_y_;

    int lower_left_x, lower_left_y, upper_right_x, upper_right_y;
    lower_left_x = std::min(std::max(cell_ox, 0), size_x);
    lower_left_y = std::min(std::max(cell_oy, 0), size_y);
    upper_right_x = std::min(std::max(cell_ox + size_x, 0), size_x);
    upper_right_y = std::min(std::max(cell_oy + size_y, 0), size_y);

    unsigned int cell_size_x = upper_right_x - lower_left_x;
    unsigned int cell_size_y = upper_right_y - lower_left_y;
    unsigned char *local_map = new unsigned char[cell_size_x * cell_size_y];

    copyMapRegion(costmap_, lower_left_x, lower_left_y, size_x_, local_map, 0, 0, cell_size_x, cell_size_x, cell_size_y);

    resetMaps();

    origin_x_ = new_grid_ox;
    origin_y_ = new_grid_oy;

    int start_x = lower_left_x - cell_ox;
    int start_y = lower_left_y - cell_oy;

    copyMapRegion(local_map, 0, 0, cell_size_x, costmap_, start_x, start_y, size_x_, cell_size_x, cell_size_y);
    delete[] local_map;
  }

  bool operator==(const costmap_2d::Costmap2D &other) const
  {
    if (size_x_ != other.getSizeInCellsX() || size_y_ != other.getSizeInCellsY())
      return false;
    const auto size = size_x_ * size_y_;
    return std::equal(costmap_, costmap_ + size, other.getCharMap());
  }
};

/// @brief Fixture which will provide a costmap to test.
struct Costmap2DFixture : public testing::Test
{
  Costmap2DLegacy map_;
  Costmap2DFixture() : map_(10, 20, 0.1, 1, 2)
  {
    // Fill the map with some data.
    const size_t size = map_.getSizeInCellsX() * map_.getSizeInCellsY();
    auto data = map_.getCharMap();
    for (size_t ii = 0; ii != size; ++ii, ++data)
    {
      // this will overflow - but its fine.
      *data = ii;
    }
  }
};

TEST_F(Costmap2DFixture, no_update)
{
  // Verify that if there is no update, the data does not change.
  costmap_2d::Costmap2D copy_map(map_);

  map_.updateOrigin(map_.getOriginX(), map_.getOriginY());
  ASSERT_EQ(map_, copy_map);
}

using coordinate = std::tuple<double, double>;
struct ParamCostmap2DFixture : public Costmap2DFixture,
                               public testing::WithParamInterface<coordinate>
{
};

INSTANTIATE_TEST_SUITE_P(/**/,
                         ParamCostmap2DFixture,
                         testing::Combine(testing::Range(-0.1, 2.1, 0.55),
                                          testing::Range(-0.1, 4.1, 0.44)));

TEST_P(ParamCostmap2DFixture, regression)
{
  // Verfies that the updateOrigin produces the same result as the legacy version.
  const auto param = GetParam();
  const auto x = std::get<0>(param);
  const auto y = std::get<1>(param);
  costmap_2d::Costmap2D copy_map(map_);
  map_.updateOrigin(x, y);
  copy_map.updateOrigin(x, y);

  ASSERT_EQ(map_, copy_map);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
