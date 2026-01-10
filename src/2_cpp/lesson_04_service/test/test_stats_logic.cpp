#include <gtest/gtest.h>
#include <vector>
#include <string>

#include "lesson_04_service/stats_logic.hpp"

TEST(StatsLogicTest, ComputeNormalValues) {
  std::vector<double> data = {10.0, 20.0, 30.0};
  
  auto result = stats_logic::Logic::compute(data);
  
  EXPECT_DOUBLE_EQ(result.sum, 60.0);
  EXPECT_DOUBLE_EQ(result.average, 20.0);
  EXPECT_EQ(result.status, "Success");
}

TEST(StatsLogicTest, ComputeEmptyList) {
  std::vector<double> data = {};
  
  auto result = stats_logic::Logic::compute(data);
  
  EXPECT_DOUBLE_EQ(result.sum, 0.0);
  EXPECT_DOUBLE_EQ(result.average, 0.0);
  // Check that the status contains "Warning"
  EXPECT_NE(result.status.find("Warning"), std::string::npos);
}

TEST(StatsLogicTest, ComputeFloats) {
  std::vector<double> data = {1.5, 2.5};
  
  auto result = stats_logic::Logic::compute(data);
  
  EXPECT_DOUBLE_EQ(result.sum, 4.0);
  EXPECT_DOUBLE_EQ(result.average, 2.0);
}

TEST(StatsLogicTest, ComputeSingleElement) {
  std::vector<double> data = {42.0};
  
  auto result = stats_logic::Logic::compute(data);
  
  EXPECT_DOUBLE_EQ(result.sum, 42.0);
  EXPECT_DOUBLE_EQ(result.average, 42.0);
  EXPECT_EQ(result.status, "Success");
}