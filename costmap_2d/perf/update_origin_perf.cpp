#include <benchmark/benchmark.h>
#include <costmap_2d/costmap_2d.h>

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
};

static void
perf_legacy(benchmark::State &_state)
{
  // Benchmarks for the current updateOrigin implementation
  Costmap2DLegacy map(1000, 1000, 0.01, 0, 0);
  for (auto _ : _state)
  {
    for (int ii = 0; ii != 10; ++ii)
    {
      map.updateOrigin(map.getOriginX() + ii * 0.2, ii * 0.05);
    }
  }
}

static void
perf_current(benchmark::State &_state)
{
  // Benchmarks for the current updateOrigin implementation
  costmap_2d::Costmap2D map(1000, 1000, 0.01, 0, 0);
  for (auto _ : _state)
  {
    for (int ii = 0; ii != 10; ++ii)
    {
      map.updateOrigin(map.getOriginX() + ii * 0.2, ii * 0.1);
    }
  }
}

// benchmarks comparing the tf product of transform with the eigen-based.
BENCHMARK(perf_legacy);
BENCHMARK(perf_current);

BENCHMARK_MAIN();
