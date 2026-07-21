#pragma once

#include <vector>
#include <deque>
#include <stdexcept>

/**
 * @class ObservationBuffer
 * @brief Fixed-length observation history buffer with flexible retrieval APIs.
 */
class ObservationBuffer
{
public:
  /**
   * @brief Constructs a buffer with explicit observation dimension and history length.
   * @param obs_dim Observation dimension per frame.
   * @param history_length Number of history frames to keep.
   */
  explicit ObservationBuffer(int obs_dim, int history_length = 1);

  /**
   * @brief Default constructor; dimension is inferred on first insertion.
   */
  ObservationBuffer();

  /// @brief Clears all cached observations.
  void reset();

  /**
   * @brief Inserts a new observation frame.
   * @param new_obs New observation vector.
   */
  void insert(const std::vector<float> & new_obs);

  /**
   * @brief Collects and concatenates observations by selected history indices.
   * @param obs_ids History index list; when empty, concatenates all cached frames.
   * @return Concatenated observation vector.
   */
  std::vector<float> get_obs_vec(const std::vector<int> & obs_ids = {}) const;

  /**
   * @brief Returns the latest observation frame.
   * @return Latest observation; returns an empty vector when cache is empty.
   */
  std::vector<float> get_latest_obs() const;

  /**
   * @brief Returns all cached history frames.
   * @return History frames in buffer order.
   */
  std::vector<std::vector<float>> get_all_history() const;

  /**
   * @brief Flattens all cached history frames in temporal order.
   * @return Flattened observation history vector.
   */
  std::vector<float> get_flattened_history() const;

  /// @brief Returns true when the buffer reaches configured history length.
  bool is_full() const;

  /// @brief Returns the current number of cached history frames.
  size_t size() const;

  /// @brief Returns observation dimension per frame.
  int get_obs_dim() const;

  /// @brief Returns configured history length.
  int get_history_length() const;

private:
  int obs_dim_;                                ///< Observation dimension per frame.
  int history_length_;                         ///< Configured history length.
  std::deque<std::vector<float>> obs_buffer_;  ///< Observation buffer (front is newest).
};
