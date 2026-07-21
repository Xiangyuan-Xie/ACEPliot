#include <am_position_mode.hpp>

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <limits>

namespace
{
std::filesystem::path writeMetadata(const std::string & action_semantics)
{
  const std::filesystem::path path =
    std::filesystem::temp_directory_path() /
    ("am_position_" + action_semantics + "_metadata_test.json");
  std::ofstream output(path);
  output << "{\n"
         << "  \"action_semantics\": \"" << action_semantics << "\",\n"
         << "  \"body_frame\": \"FLU\",\n"
         << "  \"publish_frame\": \"FRD\",\n"
         << "  \"max_body_rate_rad_s\": 6.0,\n"
         << "  \"collective_preprocess\": \"sigmoid_2x\"\n"
         << "}\n";
  return path;
}
}  // namespace

TEST(AmPositionMode, ZeroRawActionMapsToHoverCollective)
{
  const auto setpoint = am_position_mode::mapRawActionToSetpoint(
    {0.0F, 0.0F, 0.0F, 0.0F}, 6.0F, 1.0F);

  EXPECT_FLOAT_EQ(setpoint.body_rate_rad_s.x(), 0.0F);
  EXPECT_FLOAT_EQ(setpoint.body_rate_rad_s.y(), 0.0F);
  EXPECT_FLOAT_EQ(setpoint.body_rate_rad_s.z(), 0.0F);
  EXPECT_NEAR(setpoint.thrust.z(), -0.5F, 1.0e-6F);
}

TEST(AmPositionMode, ConvertsFluPitchAndYawRatesToFrd)
{
  const auto setpoint = am_position_mode::mapRawActionToSetpoint(
    {0.2F, 0.4F, 0.3F, 0.0F}, 6.0F, 1.0F);

  EXPECT_GT(setpoint.body_rate_rad_s.x(), 0.0F);
  EXPECT_LT(setpoint.body_rate_rad_s.y(), 0.0F);
  EXPECT_LT(setpoint.body_rate_rad_s.z(), 0.0F);
}

TEST(AmPositionMode, RejectsMalformedRawAction)
{
  EXPECT_THROW(
    am_position_mode::mapRawActionToSetpoint({0.0F, 0.0F, 0.0F}, 6.0F, 1.0F),
    std::invalid_argument);
  EXPECT_THROW(
    am_position_mode::mapRawActionToSetpoint(
      {0.0F, 0.0F, 0.0F, std::numeric_limits<float>::infinity()}, 6.0F, 1.0F),
    std::invalid_argument);
}

TEST(AmPositionMode, LoadsBodyRateThrustMetadata)
{
  const auto path = writeMetadata("body_rate_thrust_raw");
  const auto metadata = am_position_mode::loadDeploymentMetadata(path.string());

  EXPECT_EQ(metadata.action_semantics, "body_rate_thrust_raw");
  EXPECT_EQ(metadata.body_frame, "FLU");
  EXPECT_EQ(metadata.publish_frame, "FRD");
  EXPECT_EQ(metadata.collective_preprocess, "sigmoid_2x");
  EXPECT_FLOAT_EQ(metadata.max_body_rate_rad_s, 6.0F);
  std::filesystem::remove(path);
}

TEST(AmPositionMode, RejectsLegacyCtbrMetadataToken)
{
  const auto path = writeMetadata("ctbr_raw");
  EXPECT_THROW(
    am_position_mode::loadDeploymentMetadata(path.string()),
    std::runtime_error);
  std::filesystem::remove(path);
}
