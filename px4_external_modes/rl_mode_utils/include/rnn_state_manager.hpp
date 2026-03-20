#pragma once

#include <Eigen/Core>
#include <inference/infer_backend.hpp>
#include <rclcpp/rclcpp.hpp>

/**
 * @class RnnStateManager
 * @brief Manages optional recurrent hidden-state tensors for policy inference.
 *
 * This helper detects whether the loaded inference backend expects an input
 * tensor named `h_in` and, if present, maintains the recurrent state across
 * inference steps. It also consumes `h_out` from model outputs to update the
 * internal state for the next step.
 */
class RnnStateManager
{
public:
  /**
   * @brief Constructs the manager with a logger used for diagnostics.
   * @param logger ROS logger associated with the owning mode/node.
   */
  explicit RnnStateManager(rclcpp::Logger logger);

  /**
   * @brief Initializes RNN-state handling from backend tensor metadata.
   * @param backend Inference backend; may be null.
   *
   * If the backend exposes a valid `h_in` shape, recurrent mode is enabled and
   * the hidden state is allocated and zero-initialized.
   */
  void initialize(const InferBackend * backend);

  /**
   * @brief Appends the current hidden state to model input tensor map.
   * @param inputs Mutable tensor map passed to backend inference.
   *
   * No-op when recurrent mode is not enabled.
   */
  void appendInput(TensorMap & inputs) const;

  /**
   * @brief Updates internal hidden state from model output tensor map.
   * @param outputs Immutable tensor map returned by backend inference.
   *
   * Reads `h_out` when available and shape-compatible; otherwise keeps the
   * previous state unchanged.
   */
  void updateFromOutput(const TensorMap & outputs);

  /**
   * @brief Returns whether recurrent-state handling is active.
   * @return True when `h_in`/`h_out` management is enabled.
   */
  bool enabled() const;

private:
  rclcpp::Logger logger_;
  bool enabled_{false};
  int num_layers_{0};
  int hidden_dim_{0};
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> h_in_;
};
