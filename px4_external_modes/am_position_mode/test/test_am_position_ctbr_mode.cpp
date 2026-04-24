#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

#include <am_position_ctbr_mode.hpp>

TEST(AmPositionCtbrMode, ZeroRawActionMapsToHoverCollective)
{
  const CtbrSetpointFrd setpoint = mapRawCtbrActionToPx4Setpoint(
    {0.0f, 0.0f, 0.0f, 0.0f}, 6.0f,
    1.0f);

  EXPECT_FLOAT_EQ(setpoint.body_rate_rad_s.x(), 0.0f);
  EXPECT_FLOAT_EQ(setpoint.body_rate_rad_s.y(), 0.0f);
  EXPECT_FLOAT_EQ(setpoint.body_rate_rad_s.z(), 0.0f);
  EXPECT_NEAR(setpoint.thrust.z(), -0.5f, 1.0e-6f);
}

TEST(AmPositionCtbrMode, PositivePitchAndYawFlipIntoFrd)
{
  const CtbrSetpointFrd setpoint = mapRawCtbrActionToPx4Setpoint(
    {0.0f, 0.4f, 0.3f, 0.0f}, 6.0f,
    1.0f);

  EXPECT_LT(setpoint.body_rate_rad_s.y(), 0.0f);
  EXPECT_LT(setpoint.body_rate_rad_s.z(), 0.0f);
}

TEST(AmPositionCtbrMode, MetadataLoaderParsesCtbrFields)
{
  const std::filesystem::path metadata_path =
    std::filesystem::temp_directory_path() / "am_position_ctbr_metadata_test.json";
  std::ofstream output(metadata_path);
  output <<
    R"({
  "action_semantics": "ctbr_raw",
  "body_frame": "FLU",
  "publish_frame": "FRD",
  "max_body_rate_rad_s": 6.0,
  "collective_preprocess": "sigmoid_2x"
})";
  output.close();

  const CtbrDeploymentMetadata metadata = loadCtbrDeploymentMetadata(metadata_path.string());

  EXPECT_EQ(metadata.action_semantics, "ctbr_raw");
  EXPECT_EQ(metadata.body_frame, "FLU");
  EXPECT_EQ(metadata.publish_frame, "FRD");
  EXPECT_EQ(metadata.collective_preprocess, "sigmoid_2x");
  EXPECT_FLOAT_EQ(metadata.max_body_rate_rad_s, 6.0f);

  std::filesystem::remove(metadata_path);
}
