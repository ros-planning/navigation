/*
 * gtest_main.cpp
 *
 *  Created on: Apr 6, 2012
 *      Author: tkruse
 */

#include <iostream>

#include <ros/ros.h>
#include <gtest/gtest.h>

int main(int argc, char **argv) {
  std::cout << "Running main() from gtest_main.cc\n";

  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  return RUN_ALL_TESTS();
}

