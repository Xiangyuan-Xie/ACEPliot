#pragma once

#include <flying_hand_mode/runtime/controller_types.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>

namespace flying_hand_mode::runtime
{

struct ControllerWorkerResult
{
  ControllerOutput output{};
  double elapsed_s{0.0};
  std::uint64_t generation{0};
};

class ControllerWorker
{
public:
  explicit ControllerWorker(ControllerCallbacks callbacks);
  ~ControllerWorker();

  ControllerWorker(const ControllerWorker &) = delete;
  ControllerWorker & operator=(const ControllerWorker &) = delete;

  bool valid() const noexcept;
  bool submit(const ControllerInput & input) noexcept;
  std::optional<ControllerWorkerResult> result() const noexcept;
  bool acceptResult() noexcept;
  bool rejectResult() noexcept;
  bool recoverResult() noexcept;
  void requestRecovery() noexcept;
  void requestReset() noexcept;
  bool busy() const noexcept;
  double busyElapsedS() const noexcept;

private:
  struct State;
  std::shared_ptr<State> state_;
};

}  // namespace flying_hand_mode::runtime
