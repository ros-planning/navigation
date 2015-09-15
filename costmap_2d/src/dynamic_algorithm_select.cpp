#include <costmap_2d/dynamic_algorithm_select.h>
#include <stdio.h>
#include <time.h>
#include <ros/ros.h>


namespace costmap_2d
{


DynamicAlgorithmSelect::DynamicAlgorithmSelect(int max_alg_type, int max_data_per_type)
  : max_algorithm_types_(max_alg_type), max_data_per_type_(max_data_per_type)
{

}

/// @brief Determine if a given key is within the bounds of a map (or compatible container)
template <typename T>
inline bool key_in_bounds(const T& Map, const typename T::key_type& key)
{
  const typename T::key_type first = Map.begin()->first;
  const typename T::key_type last  = Map.rbegin()->first;
  return key >= first && key <= last;
}

void DynamicAlgorithmSelect::addProfilingData(int problem_size, int algorithm_number, double delta_t)
{
  size_time_map& stm = profiling_data_[algorithm_number];

  if(stm.size() > max_data_per_type_)
  {
    // only adding data if it extends range
    if(!key_in_bounds(stm, problem_size))
    {
      stm[problem_size] = delta_t;
    }
  }
  else
  {
    if(stm.find(problem_size) != stm.end()) // then we have existing data
    {
      // averaging in the new data with a 10% weight
      stm[problem_size] = 0.9 * stm[problem_size] + 0.1 * delta_t;
    }
    else
    {
      stm[problem_size] = delta_t;
    }
  }
}

double DynamicAlgorithmSelect::getAlgorithmTime(const size_time_map& data, int problem_size)
{
  typedef size_time_map::const_iterator it;

  it exact = data.find(problem_size);
  if(exact != data.end()) // if we have the exact data point, use it
    return exact->second;

  it i=data.upper_bound(problem_size);
  if (i==data.end())
    return 0;

  if (i==data.begin())
    return 0;

  it l=i; --l;

  // linear interpolation between bounding points
  const double delta= static_cast<double>(problem_size - l->first) / static_cast<double>(i->first - l->first);
  return delta*i->second + (1.0-delta)*l->second;
}

int DynamicAlgorithmSelect::selectAlgorithm(int problem_size)
{
  int min_alg = 0;
  double min_time = getAlgorithmTime(profiling_data_[0], problem_size);
  for(int i=1; i<max_algorithm_types_; i++)
  {
    double alg_i_time = getAlgorithmTime(profiling_data_[i], problem_size);
    if(alg_i_time < min_time)
    {
      min_time = alg_i_time;
      min_alg = i;
    }
  }

  return min_alg;
}

void DynamicAlgorithmSelect::writeData(const char *filename)
{
  FILE* f = fopen(filename, "w");

  if(!f)
  {
    ROS_ERROR("(dynamic_algorithm_select.cpp:%d) Failed to open file `%s' for writing.", __LINE__, filename);
    return;
  }

  fprintf(f, "DynamicAlgorithmSelect (%p)\n", static_cast<void*>(this));
  fprintf(f, "  max_algorithm_types_(%d)\n", max_algorithm_types_);
  fprintf(f, "  max_data_per_type_(%d)\n", max_data_per_type_);
  fprintf(f, "\n");

  fprintf(f, "  Algorithm Type    Problem Size     Run Time\n");
  for (int i=0; i<max_algorithm_types_; i++)
  {
    size_time_map& stm = profiling_data_[i];

    for (size_time_map::const_iterator it=stm.begin(); it != stm.end(); ++it)
    {
      const int ps = it->first;
      const double t = it->second;

      fprintf(f, "  %14d    %12d     %e\n", i, ps, t);
    }
    fprintf(f, "\n");
  }
  fclose(f);
}


// We are using an opaque class with a d-pointer
// so we can modify the implementation without messing with the
// header file. Should reduce compile times when this is
// used more widely.
DynamicAlgorithmSelect::Timer::Timer()
{

}

DynamicAlgorithmSelect::Timer::~Timer()
{
}

void DynamicAlgorithmSelect::Timer::start()
{
  start_ = ros::Time::now();
}

void DynamicAlgorithmSelect::Timer::stop()
{
  end_ = ros::Time::now();
}

double DynamicAlgorithmSelect::Timer::elapsed()
{
  return (end_-start_).toNSec();
}


}  // costmap_2d
