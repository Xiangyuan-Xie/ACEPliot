#include <gtest/gtest.h>

#include <cmath>

#include <figure8_trajectory_mode/figure8_generator.hpp>
#include <trajectory_generator_utils/generator.hpp>

TEST(Figure8TrajectoryMode, ProducesTrajectorySamplesAfterReset)
{
  Figure8Generator generator;
  TrajectoryGeneratorState state;
  state.root_pos_w = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
  state.heading_w = 0.0f;

  generator.reset(state);
  const TrajectorySample sample = generator.step(0.02f, state);

  ASSERT_TRUE(sample.position.has_value());
  ASSERT_TRUE(sample.velocity.has_value());
  EXPECT_TRUE(std::isfinite(sample.position->x()));
  EXPECT_TRUE(std::isfinite(sample.position->y()));
  EXPECT_TRUE(std::isfinite(sample.position->z()));
}

TEST(Figure8TrajectoryMode, BuildsVisualizationPath)
{
  Figure8Generator generator;
  TrajectoryGeneratorState state;
  generator.reset(state);

  const nav_msgs::msg::Path path = generator.getFullPathForVisualization(rclcpp::Time(0), 16);

  EXPECT_EQ(path.poses.size(), 17u);
  EXPECT_EQ(path.header.frame_id, "world");
}
