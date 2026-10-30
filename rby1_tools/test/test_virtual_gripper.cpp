// The virtual gripper's step and command check; no ROS graph.
#include <gtest/gtest.h>

#include <limits>

#include "rby1_tools/virtual_gripper.hpp"

using namespace rby1_tools;

TEST(Step, MovesAtSpeedAndStopsAtTheGoal) {
  EXPECT_NEAR(step_toward(0.05, 0.0, 0.05, 0.02), 0.049, 1e-12);  // closing, 1 mm per 20 ms
  EXPECT_NEAR(step_toward(0.0, 0.05, 0.05, 0.02), 0.001, 1e-12);  // opening
  EXPECT_EQ(step_toward(0.0005, 0.0, 0.05, 0.02), 0.0);           // does not pass the goal
  EXPECT_EQ(step_toward(0.03, 0.03, 0.05, 0.02), 0.03);
  double opening = 0.05;
  int steps = 0;
  while (opening > 0.0 && steps < 1000) {
    opening = step_toward(opening, 0.0, 0.05, 0.02);
    ++steps;
  }
  EXPECT_NEAR(steps, 50, 1);  // fully open to closed in 1 s (one more for rounding)
}

TEST(Command, OnlyOpeningsTheGripperHas) {
  EXPECT_FALSE(refused(0.0, 0.05));
  EXPECT_FALSE(refused(0.05, 0.05));
  EXPECT_TRUE(refused(-0.001, 0.05));
  EXPECT_TRUE(refused(0.051, 0.05));
  EXPECT_TRUE(refused(std::numeric_limits<double>::quiet_NaN(), 0.05));
}
