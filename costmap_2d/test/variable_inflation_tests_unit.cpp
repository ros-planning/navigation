/*
 * Copyright (c) 2020, LexxPluss, Inc.
 * All rights reserved.
 * 
 */

#include <gtest/gtest.h>
#include <boost/make_shared.hpp>

#define protected public
#define private public
#include <costmap_2d/variable_inflation_layer.h>
#undef private
#undef protected

using namespace costmap_2d;

TEST(costmap, testCalculateVariableInflationRadius){
  boost::shared_ptr<VariableInflationLayer> ilayer = boost::make_shared<VariableInflationLayer>();
  double radius = 0.0;

  ilayer->min_inflation_radius_ = 0.1;
  ilayer->max_inflation_radius_ = 1.0;
  ilayer->min_inflation_vel_ = 0.1;
  ilayer->max_inflation_vel_ = 1.0;

  ilayer->diff_drive_debug_info_msg.measured_twist_filtered.linear.x = 0.05;
  radius = ilayer->calculate_variable_inflation_radius();
  EXPECT_EQ(radius, ilayer->min_inflation_radius_);

  ilayer->diff_drive_debug_info_msg.measured_twist_filtered.linear.x = 0.5;
  radius = ilayer->calculate_variable_inflation_radius();
  EXPECT_EQ(radius, 0.5);

  ilayer->diff_drive_debug_info_msg.measured_twist_filtered.linear.x = 1.1;
  radius = ilayer->calculate_variable_inflation_radius();
  EXPECT_EQ(radius, ilayer->max_inflation_radius_);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "inflation_tests_unit");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
