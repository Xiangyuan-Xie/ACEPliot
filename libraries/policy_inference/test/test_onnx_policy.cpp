#include <policy_inference/onnx_policy.hpp>

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace policy_inference
{
namespace
{

constexpr std::string_view kOrderedModelBase64 =
  "CAkSFXBvbGljeV9pbmZlcmVuY2VfdGVzdDp0ChwKBWZpcnN0CgZzZWNvbmQSBnJlc3VsdCIDU3ViEgdv"
  "cmRlcmVkWhgKBnNlY29uZBIOCgwIARIICgIIAQoCCAJaFwoFZmlyc3QSDgoMCAESCAoCCAEKAggCYhgK"
  "BnJlc3VsdBIOCgwIARIICgIIAQoCCAJCBAoAEA0=";
constexpr std::string_view kDynamicModelBase64 =
  "CAkSFXBvbGljeV9pbmZlcmVuY2VfdGVzdDphCiAKB2R5bmFtaWMSC2R5bmFtaWNfb3V0IghJZGVudGl0"
  "eRIHZHluYW1pY1oXCgdkeW5hbWljEgwKCggBEgYKAggBCgBiGwoLZHluYW1pY19vdXQSDAoKCAESBgoC"
  "CAEKAEIECgAQDQ==";
constexpr std::string_view kAmbiguousModelBase64 =
  "CAkSFXBvbGljeV9pbmZlcmVuY2VfdGVzdDpnCiQKCWFtYmlndW91cxINYW1iaWd1b3VzX291dCIISWRl"
  "bnRpdHkSCWFtYmlndW91c1oXCglhbWJpZ3VvdXMSCgoICAESBAoACgBiGwoNYW1iaWd1b3VzX291dBIK"
  "CggIARIECgAKAEIECgAQDQ==";
constexpr std::string_view kExternalDataModelBase64 =
  "CAkSFXBvbGljeV9pbmZlcmVuY2VfdGVzdDqSAQoTCgF4CgZ3ZWlnaHQSAXkiA0FkZBIIZXh0ZXJuYWwq"
  "TwgBEAFCBndlaWdodGolCghsb2NhdGlvbhIZbWlzc2luZ19leHRlcm5hbF9kYXRhLmJpbmoLCgZvZmZz"
  "ZXQSATBqCwoGbGVuZ3RoEgE0cAFaDwoBeBIKCggIARIECgIIAWIPCgF5EgoKCAgBEgQKAggBQgQKABAN";

std::vector<std::uint8_t> decodeBase64(std::string_view encoded)
{
  std::array<int, 256> values{};
  values.fill(-1);
  constexpr std::string_view alphabet =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  for (std::size_t index = 0; index < alphabet.size(); ++index) {
    values[static_cast<unsigned char>(alphabet[index])] = static_cast<int>(index);
  }

  std::vector<std::uint8_t> decoded;
  std::uint32_t accumulator = 0;
  int available_bits = -8;
  for (const unsigned char character : encoded) {
    if (character == '=') {
      break;
    }
    const int value = values[character];
    if (value < 0) {
      throw std::invalid_argument("Invalid base64 ONNX test fixture");
    }
    accumulator = (accumulator << 6) | static_cast<std::uint32_t>(value);
    available_bits += 6;
    if (available_bits >= 0) {
      decoded.push_back(static_cast<std::uint8_t>((accumulator >> available_bits) & 0xFF));
      available_bits -= 8;
    }
  }
  return decoded;
}

class TemporaryModel final
{
public:
  TemporaryModel(const std::string & filename, std::string_view encoded)
  : directory_(std::filesystem::temp_directory_path() / "policy_inference_onnx_tests"),
    path_(directory_ / filename)
  {
    std::filesystem::create_directories(directory_);
    std::filesystem::remove(path_);
    const std::vector<std::uint8_t> bytes = decodeBase64(encoded);
    std::ofstream stream(path_, std::ios::binary | std::ios::trunc);
    stream.write(
      reinterpret_cast<const char *>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
    if (!stream) {
      throw std::runtime_error("Failed to write ONNX test fixture");
    }
  }

  ~TemporaryModel()
  {
    std::error_code error;
    std::filesystem::remove(path_, error);
  }

  const std::filesystem::path & path() const noexcept
  {
    return path_;
  }

  const std::filesystem::path & directory() const noexcept
  {
    return directory_;
  }

private:
  std::filesystem::path directory_;
  std::filesystem::path path_;
};

TEST(OnnxPolicy, RejectsMissingModel)
{
  const auto missing =
    std::filesystem::temp_directory_path() / "acepliot_missing_policy_model.onnx";
  std::filesystem::remove(missing);
  EXPECT_THROW(OnnxPolicy policy(missing), std::invalid_argument);
}

TEST(OnnxPolicy, RejectsEmptyModelPath)
{
  EXPECT_THROW(OnnxPolicy policy(std::filesystem::path{}), std::invalid_argument);
}

TEST(OnnxPolicy, PreservesModelInputOrderAndRunsNamedInputs)
{
  const TemporaryModel model("ordered.onnx", kOrderedModelBase64);
  const OnnxPolicy policy(model.path());

  EXPECT_EQ(policy.modelPath(), model.path());
  EXPECT_EQ(policy.inputNames(), (std::vector<std::string>{"second", "first"}));
  EXPECT_EQ(policy.outputNames(), (std::vector<std::string>{"result"}));
  EXPECT_EQ(policy.inputShape("first"), (std::vector<int64_t>{1, 2}));
  EXPECT_EQ(policy.outputShape("result"), (std::vector<int64_t>{1, 2}));
  EXPECT_TRUE(policy.hasInput("second"));
  EXPECT_TRUE(policy.hasOutput("result"));
  EXPECT_FALSE(policy.hasInput("result"));
  EXPECT_FALSE(policy.hasOutput("first"));

  const TensorMap outputs = policy.infer(
      {
        {"first", {5.0F, 7.0F}},
        {"second", {2.0F, 3.0F}},
      });
  ASSERT_EQ(outputs.at("result"), (Tensor{3.0F, 4.0F}));
}

TEST(OnnxPolicy, RejectsMissingUnexpectedAndWrongSizedInputs)
{
  const TemporaryModel model("ordered_contract.onnx", kOrderedModelBase64);
  const OnnxPolicy policy(model.path());

  EXPECT_THROW(policy.infer({{"first", {1.0F, 2.0F}}}), std::invalid_argument);
  EXPECT_THROW(
    policy.infer(
      {
        {"first", {1.0F, 2.0F}}, {"second", {3.0F, 4.0F}}, {"typo", {0.0F}}}),
    std::invalid_argument);
  EXPECT_THROW(
    policy.infer({{"first", {1.0F}}, {"second", {3.0F, 4.0F}}}),
    std::invalid_argument);
  EXPECT_THROW(policy.inputShape("missing"), std::out_of_range);
  EXPECT_THROW(policy.outputShape("missing"), std::out_of_range);
}

TEST(OnnxPolicy, ResolvesOneDynamicDimensionFromTensorSize)
{
  const TemporaryModel model("dynamic.onnx", kDynamicModelBase64);
  const OnnxPolicy policy(model.path());

  EXPECT_EQ(policy.inputShape("dynamic"), (std::vector<int64_t>{1, -1}));
  const TensorMap output = policy.infer({{"dynamic", {1.0F, 2.0F, 3.0F}}});
  EXPECT_EQ(output.at("dynamic_out"), (Tensor{1.0F, 2.0F, 3.0F}));
}

TEST(OnnxPolicy, RejectsAmbiguousDynamicInputShape)
{
  const TemporaryModel model("ambiguous.onnx", kAmbiguousModelBase64);
  const OnnxPolicy policy(model.path());

  EXPECT_THROW(
    policy.infer({{"ambiguous", {1.0F, 2.0F, 3.0F, 4.0F}}}),
    std::invalid_argument);
}

TEST(OnnxPolicy, PreservesExternalDataLoadError)
{
  const TemporaryModel model("external.onnx", kExternalDataModelBase64);
  const auto missing_sidecar = model.directory() / "missing_external_data.bin";
  std::filesystem::remove(missing_sidecar);

  std::string message;
  testing::internal::CaptureStderr();
  try {
    const OnnxPolicy policy(model.path());
    static_cast<void>(policy);
  } catch (const std::runtime_error & error) {
    message = error.what();
  }
  static_cast<void>(testing::internal::GetCapturedStderr());

  ASSERT_FALSE(message.empty()) << "Expected the missing external-data sidecar to reject loading";
  EXPECT_NE(message.find("Failed to load ONNX model"), std::string::npos);
  EXPECT_NE(message.find("external.onnx"), std::string::npos);
  EXPECT_NE(message.find("missing_external_data.bin"), std::string::npos);
}

}  // namespace
}  // namespace policy_inference
