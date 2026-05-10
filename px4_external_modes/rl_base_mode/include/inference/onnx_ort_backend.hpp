#pragma once

#include <onnxruntime_cxx_api.h>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include <inference/infer_backend.hpp>

/**
 * @class OnnxOrtBackend
 * @brief An inference backend that uses the ONNX Runtime C++ API.
 * This class is responsible for loading an ONNX model and running inference.
 */
class OnnxOrtBackend final : public InferBackend
{
public:
  /**
   * @brief Constructor for OnnxOrtBackend.
   * @param node The ROS 2 node, used for getting parameters (e.g., model path).
   */
  explicit OnnxOrtBackend(rclcpp::Node * node);

  /// @brief Default destructor.
  ~OnnxOrtBackend() override = default;

  /**
   * @brief Performs a forward pass using the loaded ONNX model.
   * @param inputs The input tensor map for the model.
   * @param dt_s The time delta in seconds (may not be used by this backend).
   * @return A map of output tensors from the model.
   */
  TensorMap forward(const TensorMap & inputs, float dt_s) override;

  /// @brief Returns true if the model expects an input tensor with the given name.
  bool hasInput(const std::string & name) const override;

  /// @brief Returns the expected input shape for the specified input tensor.
  std::vector<int64_t> inputShape(const std::string & name) const override;

private:
  /** @name Private Helper Functions */
  ///@{

  /**
   * @brief Overloads to check if the input tensor dimensions match the model's expectations.
   * @param vec The input tensor represented as a std::vector<float>.
   * @param name The name of the input tensor.
   */
  void check_input_dimensions(const std::vector<float> & vec, const std::string & name) const;

  /**
   * @brief Overloads to check if the input tensor dimensions match the model's expectations.
   * @param mat The input tensor represented as a cv::Mat.
   * @param name The name of the input tensor.
   */
  void check_input_dimensions(const cv::Mat & mat, const std::string & name) const;

  /**
   * @brief Creates an ONNX Runtime Value object from a GenericTensor variant.
   * @param tensor The input GenericTensor.
   * @param name The name of the input, used for dimension checking.
   * @return An Ort::Value ready for inference.
   */
  Ort::Value create_ort_value(const GenericTensor & tensor, const std::string & name);
  ///@}

  // --- Member Variables ---
  rclcpp::Node * node_{nullptr};                 ///< ROS 2 node used for parameters and logging.
  std::string model_path_;                        ///< Path to the .onnx model file.

  // ONNX Runtime members
  Ort::Env env_;                                  ///< The ONNX Runtime environment.
  Ort::MemoryInfo mem_info_;                      ///< Memory info for tensor allocation (CPU).
  std::unique_ptr<Ort::Session> session_;         ///< The ONNX Runtime inference session.

  // Model properties
  size_t num_inputs_{0}, num_outputs_{0};         ///< Number of model inputs and outputs.
  std::vector<std::string> input_names_, output_names_;  ///< Names of model inputs/outputs.
  std::vector<const char *> input_names_cstr_,
    output_names_cstr_;  ///< C-string versions of names for the ORT API.
  std::unordered_map<std::string,
    std::vector<int64_t>> input_shapes_;  ///< Expected shapes for each model input.
};
