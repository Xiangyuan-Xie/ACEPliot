#include <rnn_state_manager.hpp>

RnnStateManager::RnnStateManager(rclcpp::Logger logger)
: logger_(std::move(logger))
{
}

void RnnStateManager::initialize(const InferBackend * backend)
{
  enabled_ = false;
  num_layers_ = 0;
  hidden_dim_ = 0;
  h_in_.resize(0, 0);

  if (!backend) {
    return;
  }

  if (!backend->hasInput("h_in")) {
    return;
  }

  const auto shape = backend->inputShape("h_in");
  if (shape.size() >= 2) {
    hidden_dim_ = static_cast<int>(shape.back());
    num_layers_ = static_cast<int>(shape[shape.size() - 2]);
  } else if (shape.size() == 1) {
    num_layers_ = 1;
    hidden_dim_ = static_cast<int>(shape[0]);
  }

  if (num_layers_ <= 0 || hidden_dim_ <= 0) {
    RCLCPP_WARN(logger_, "Invalid RNN shape for h_in. Disabling RNN handling.");
    return;
  }

  enabled_ = true;
  h_in_.resize(num_layers_, hidden_dim_);
  h_in_.setZero();
  RCLCPP_INFO(logger_, "Detected RNN state: layers=%d hidden=%d", num_layers_, hidden_dim_);
}

void RnnStateManager::appendInput(TensorMap & inputs) const
{
  if (!enabled_) {
    return;
  }

  std::vector<float> h_in_vec(h_in_.data(), h_in_.data() + h_in_.size());
  inputs["h_in"] = std::move(h_in_vec);
}

void RnnStateManager::updateFromOutput(const TensorMap & outputs)
{
  if (!enabled_) {
    return;
  }

  const auto it = outputs.find("h_out");
  if (it == outputs.end() || !std::holds_alternative<std::vector<float>>(it->second)) {
    return;
  }

  const auto & h_out_vec = std::get<std::vector<float>>(it->second);
  const int expected_size = num_layers_ * hidden_dim_;
  if (static_cast<int>(h_out_vec.size()) != expected_size) {
    RCLCPP_WARN(
      logger_,
      "h_out size mismatch. Expected %d elements, got %zu. Skipping state update.",
      expected_size,
      h_out_vec.size());
    return;
  }

  Eigen::Map<const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>> h_out(
    h_out_vec.data(), num_layers_, hidden_dim_);
  h_in_ = h_out;
}

bool RnnStateManager::enabled() const
{
  return enabled_;
}
