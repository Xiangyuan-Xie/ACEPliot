#pragma once

#include <policy_inference/tensor.hpp>

#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace policy_inference
{

class OnnxPolicy final
{
public:
  explicit OnnxPolicy(const std::filesystem::path & model_path);
  ~OnnxPolicy();

  OnnxPolicy(const OnnxPolicy &) = delete;
  OnnxPolicy & operator=(const OnnxPolicy &) = delete;
  OnnxPolicy(OnnxPolicy &&) = delete;
  OnnxPolicy & operator=(OnnxPolicy &&) = delete;

  TensorMap infer(const TensorMap & inputs) const;
  bool hasInput(const std::string & name) const noexcept;
  bool hasOutput(const std::string & name) const noexcept;
  const std::vector<int64_t> & inputShape(const std::string & name) const;
  const std::vector<int64_t> & outputShape(const std::string & name) const;
  const std::vector<std::string> & inputNames() const noexcept;
  const std::vector<std::string> & outputNames() const noexcept;
  const std::filesystem::path & modelPath() const noexcept;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace policy_inference
