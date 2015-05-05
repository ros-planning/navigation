#ifndef COSTMAP_DYNAMIC_ALGORITHM_SELECT_H_
#define COSTMAP_DYNAMIC_ALGORITHM_SELECT_H_

#include <map>

namespace costmap_2d
{
typedef std::map<int,double> size_time_map;

/**
 * @class DynamicAlgorithmSelect
 * @brief Build a small local dataset dealing with problem size, algorithm type and runtime to make informed
 *        choices on algorithm types. This class will help choose the fastest algorithm for the situation.
 */
class DynamicAlgorithmSelect
{
public:
  /**
   * @brief  Constructor for a DynamicAlgorithmSelect
   * @param  max_alg_type Maximum number of algorithms to consider
   * @param  max_data_per_type Maximum number of data points per algorithm type
   */
  DynamicAlgorithmSelect(int max_alg_type, int max_data_oer_type = 8);

  /**
   * @brief Add data related to problem size, algorithm type and run time to the internal state.
   * @param problem_size  quantification of problem size
   * @param algorithm_number  class of the algorithm
   * @param delta_t time used by this algooeithm
   */
  void addProfilingData(int problem_size, int algorithm_number, double delta_t);

  /**
   * @brief Based on the internal state, choose the best estimated algorithm type for the given problem size
   * @param problem_size  quantification of problem size
   * @param algorithm_number  class of the algorithm
   * @return Best estimated algorithm type
   */
  int selectAlgorithm(int problem_size);

  /**
   * @brief Write internal data to a file
   * @param filename Name of the data file
   */
  void writeData(const char* filename);

  /**
   * @class Opaque class used to track elapsed time.
   */
  class Timer
  {
  public:
    Timer();
    ~Timer();

    /**
     * @brief Start the timer
     */
    void start();
    /**
     * @brief Stop the timer
     */
    void stop();
    /**
     * @brief Determine the elapsed time in nanoseconds
     */
    double elapsed();
  private:
    struct TimerStruct;
    TimerStruct* d;
  };

private:
  int max_algorithm_types_;
  int max_data_per_type_;
  std::map<int, size_time_map> profiling_data_;

  /**
   * @brief Given a size_time_map and a key, interpolate the time. Values outside the data range will be given
   *        zeros so the code will pick them as "best" to try to get valid data.
   * @param data  map of problem sizes and associated times
   * @param problem_size  quantification of problem size
   * @return Estimated time to execute the algorithm on a given problem size or zero if out of bounds.
   */
  double getAlgorithmTime(const size_time_map& data, int problem_size);
};

}  // namespace costmap_2d

#endif  // COSTMAP_COSTMAP_2D_H_
