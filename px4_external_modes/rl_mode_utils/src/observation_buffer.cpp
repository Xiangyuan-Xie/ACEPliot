#include <observation_buffer.hpp>

ObservationBuffer::ObservationBuffer()
: obs_dim_(0), history_length_(1) {}

ObservationBuffer::ObservationBuffer(int obs_dim, int history_length)
: obs_dim_(obs_dim), history_length_(history_length)
{
  if (obs_dim <= 0) {
    throw std::invalid_argument("obs_dim must be positive");
  }
  if (history_length <= 0) {
    throw std::invalid_argument("history_length must be positive");
  }
}

void ObservationBuffer::reset()
{
  obs_buffer_.clear();
}

void ObservationBuffer::insert(const std::vector<float> & new_obs)
{
  if (obs_dim_ > 0 && static_cast<int>(new_obs.size()) != obs_dim_) {
    throw std::invalid_argument("Observation dimension mismatch");
  }
  if (obs_dim_ == 0) {
    obs_dim_ = static_cast<int>(new_obs.size());
  }
  obs_buffer_.push_front(new_obs);
  while (static_cast<int>(obs_buffer_.size()) > history_length_) {
    obs_buffer_.pop_back();
  }
}

std::vector<float> ObservationBuffer::get_obs_vec(const std::vector<int> & obs_ids) const
{
  std::vector<float> result;
  if (obs_ids.empty()) {
    for (const auto & obs : obs_buffer_) {
      result.insert(result.end(), obs.begin(), obs.end());
    }
    return result;
  }
  for (int id : obs_ids) {
    if (id < 0 || id >= static_cast<int>(obs_buffer_.size())) {
      throw std::out_of_range("obs_id out of range");
    }
    const auto & obs = obs_buffer_[id];
    result.insert(result.end(), obs.begin(), obs.end());
  }
  return result;
}

std::vector<float> ObservationBuffer::get_latest_obs() const
{
  if (obs_buffer_.empty()) {
    return std::vector<float>();
  }
  return obs_buffer_.front();
}

std::vector<std::vector<float>> ObservationBuffer::get_all_history() const
{
  std::vector<std::vector<float>> result;
  for (const auto & obs : obs_buffer_) {
    result.push_back(obs);
  }
  return result;
}

std::vector<float> ObservationBuffer::get_flattened_history() const
{
  std::vector<float> result;
  result.reserve(obs_buffer_.size() * obs_dim_);
  for (auto it = obs_buffer_.rbegin(); it != obs_buffer_.rend(); ++it) {
    result.insert(result.end(), it->begin(), it->end());
  }
  return result;
}

bool ObservationBuffer::is_full() const
{
  return static_cast<int>(obs_buffer_.size()) >= history_length_;
}

size_t ObservationBuffer::size() const
{
  return obs_buffer_.size();
}

int ObservationBuffer::get_obs_dim() const
{
  return obs_dim_;
}

int ObservationBuffer::get_history_length() const
{
  return history_length_;
}
