#include <ros/ros.h>
#include <gtest/gtest.h>


TEST(ClearTester, basicTest){
  EXPECT_TRUE(true);
}


int main(int argc, char** argv){
  ros::init(argc, argv, "clear_tests");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
