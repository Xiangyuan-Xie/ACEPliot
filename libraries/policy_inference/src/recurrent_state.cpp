#include <policy_inference/recurrent_state.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace policy_inference
{

void RecurrentState::configure(const std::vector<int64_t> & input_shape)
{
  if (input_shape.empty()) {
    throw std::invalid_argument("Recurrent input shape must not be empty");
  }

  std::size_t element_count = 1;
  for (const int64_t dimension : input_shape) {
    if (dimension <= 0) {
      throw std::invalid_argument("Recurrent input shape must be fully static and positive");
    }
    const auto concrete_dimension = static_cast<std::size_t>(dimension);
    if (concrete_dimension > std::numeric_limits<std::size_t>::max() / element_count) {
      throw std::overflow_error("Recurrent input shape is too large");
    }
    element_count *= concrete_dimension;
  }
  if (element_count == 0) {
    throw std::invalid_argument("Recurrent input shape must contain elements");
  }

  shape_ = input_shape;
  values_.assign(element_count, 0.0F);
  enabled_ = true;
}

void RecurrentState::disable() noexcept
{
  enabled_ = false;
  shape_.clear();
  values_.clear();
}

void RecurrentState::reset() noexcept
{
  std::fill(values_.begin(), values_.end(), 0.0F);
}

void RecurrentState::appendInput(TensorMap & inputs, const std::string & input_name) const
{
  if (!enabled_) {
    return;
  }
  if (input_name.empty()) {
    throw std::invalid_argument("Recurrent input name must not be empty");
  }
  inputs[input_name] = values_;
}

void RecurrentState::updateFromOutput(
  const TensorMap & outputs, const std::string & output_name)
{
  if (!enabled_) {
    return;
  }
  if (output_name.empty()) {
    throw std::invalid_argument("Recurrent output name must not be empty");
  }
  const auto output = outputs.find(output_name);
  if (output == outputs.end()) {
    throw std::invalid_argument("Missing recurrent policy output: " + output_name);
  }
  if (output->second.size() != values_.size()) {
    throw std::invalid_argument(
            "Recurrent output '" + output_name + "' expects " +
            std::to_string(values_.size()) + " elements, received " +
            std::to_string(output->second.size()));
  }
  if (!std::all_of(
      output->second.begin(), output->second.end(),
      [](float value) {return std::isfinite(value);}))
  {
    throw std::invalid_argument(
            "Recurrent output '" + output_name +
            "' contains non-finite values");
  }
  values_ = output->second;
}

bool RecurrentState::enabled() const noexcept
{
  return enabled_;
}

std::size_t RecurrentState::size() const noexcept
{
  return values_.size();
}

const std::vector<int64_t> & RecurrentState::shape() const noexcept
{
  return shape_;
}

const Tensor & RecurrentState::values() const noexcept
{
  return values_;
}

}  // namespace policy_inference
