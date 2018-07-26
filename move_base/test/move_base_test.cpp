#include <gtest/gtest.h>
#include <vector>
#include <move_base/move_base.h>

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "control_loop_test");

  return RUN_ALL_TESTS();
}

TEST(ControlLoopMissTest1, normalCaseTest)
{
  srs::ControlLoopAnalyzer analyzer;

  float test_array[] = {0.0408, 0.0415, 0.0489, 0.0501, 0.0577, 0.0462, 0.0412};
  std::vector<float> test_vec(test_array, test_array + 7);
  float maximun_miss = 0.0;
  size_t count = 7;

  analyzer.compute(test_vec, maximun_miss);

  EXPECT_EQ(test_vec.size(), 7);
  EXPECT_FLOAT_EQ(maximun_miss, 0.0577);
}

TEST(ControlLoopMissTest2, emptyVectorTest)
{
  srs::ControlLoopAnalyzer analyzer;

  std::vector<float> test_vec;
  float maximun_miss = 0.0;
  size_t count = 0;

  analyzer.compute(test_vec, maximun_miss);

  EXPECT_EQ(test_vec.size(), 0);
  EXPECT_FLOAT_EQ(maximun_miss, 0.0);
}

TEST(ControlLoopMissTest3, zeroVectorTest)
{
  srs::ControlLoopAnalyzer analyzer;

  float test_array[] = {0.0, 0.0, 0.0, 0.0, 0.0};
  std::vector<float> test_vec(test_array, test_array + 5);
  float maximun_miss = 0.0;
  size_t count = 5;

  analyzer.compute(test_vec, maximun_miss);

  EXPECT_EQ(test_vec.size(), 5);
  EXPECT_FLOAT_EQ(maximun_miss, 0.0);
}

TEST(ControlLoopMissTest4, minusValueVectorTest)
{
  srs::ControlLoopAnalyzer analyzer;

  float test_array[] = {-0.0123, 0.0, -0.0452, -100.0, 0.0};
  std::vector<float> test_vec(test_array, test_array + 5);
  float maximun_miss = 0.0;
  size_t count = 5;

  analyzer.compute(test_vec, maximun_miss);

  EXPECT_EQ(test_vec.size(), 5);
  EXPECT_FLOAT_EQ(maximun_miss, 0.0);
}

TEST(ControlLoopMissTest5, singleValueVectorTest)
{
  srs::ControlLoopAnalyzer analyzer;

  float test_array[] = {0.0123};
  std::vector<float> test_vec(test_array, test_array + 1);
  float maximun_miss = 0.0;
  size_t count = 1;

  analyzer.compute(test_vec, maximun_miss);

  EXPECT_EQ(test_vec.size(), 1);
  EXPECT_FLOAT_EQ(maximun_miss, 0.0123);
}
