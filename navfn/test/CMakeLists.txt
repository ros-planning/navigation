catkin_add_gtest(path_calc_test path_calc_test.cpp ../src/read_pgm_costmap.cpp)
target_link_libraries(path_calc_test navfn netpbm)
