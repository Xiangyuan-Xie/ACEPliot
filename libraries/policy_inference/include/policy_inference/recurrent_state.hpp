#pragma once

#include <policy_inference/tensor.hpp>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace policy_inference
{

class RecurrentState final
{
public:
  void configure(const std::vector<int64_t> & input_shape);
  void disable() noexcept;
  void reset() noexcept;
  void appendInput(TensorMap & inputs, const std::string & input_name = "h_in") const;
  void updateFromOutput(
    const TensorMap & outputs, const std::string & output_name = "h_out");

  bool enabled() const noexcept;
  std::size_t size() const noexcept;
  const std::vector<int64_t> & shape() const noexcept;
  const Tensor & values() const noexcept;

private:
  bool enabled_{false};
  std::vector<int64_t> shape_;
  Tensor values_;
};

}  // namespace policy_inference
