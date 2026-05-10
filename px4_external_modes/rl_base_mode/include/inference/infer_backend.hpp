#pragma once

#include <string>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

using GenericTensor = std::variant<std::vector<float>, cv::Mat>;
using TensorMap = std::unordered_map<std::string, GenericTensor>;

/**
 * @class InferBackend
 * @brief Abstract inference backend interface for model I/O and execution.
 */
class InferBackend
{
public:
  /// @brief Virtual destructor for safe polymorphic cleanup.
  virtual ~InferBackend() = default;

  /**
   * @brief Checks whether the model exposes an input tensor by name.
   * @param name Input tensor name.
   * @return True when the input exists; otherwise false.
   */
  virtual bool hasInput(const std::string & name) const = 0;

  /**
   * @brief Returns the expected shape of a named input tensor.
   * @param name Input tensor name.
   * @return Tensor shape vector.
   */
  virtual std::vector<int64_t> inputShape(const std::string & name) const = 0;

  /**
   * @brief Executes one forward inference pass.
   * @param inputs Input tensor map.
   * @param dt_s Control period in seconds for optional stateful backend logic.
   * @return Output tensor map.
   */
  virtual TensorMap forward(const TensorMap & inputs, float dt_s) = 0;
};
