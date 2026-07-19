#include <flying_hand_control_common/controller_worker.hpp>

#include <condition_variable>
#include <mutex>
#include <thread>
#include <utility>

namespace flying_hand_control_common
{

struct ControllerWorker::State
{
  enum class Decision
  {
    kNone,
    kAccept,
    kReject,
  };

  explicit State(ControllerCallbacks controller_callbacks)
  : callbacks(std::move(controller_callbacks))
  {
  }

  ControllerCallbacks callbacks;
  mutable std::mutex mutex;
  std::condition_variable condition;
  std::optional<ControllerInput> task;
  std::optional<ControllerWorkerResult> result;
  std::chrono::steady_clock::time_point solve_start{};
  std::uint64_t next_generation{1};
  Decision decision{Decision::kNone};
  bool solving{false};
  bool recovery_requested{false};
  bool reset_requested{false};
  bool stop_requested{false};
  bool exited{false};
  std::thread thread;
};

ControllerWorker::ControllerWorker(ControllerCallbacks callbacks)
: state_(std::make_shared<State>(std::move(callbacks)))
{
  if (!state_->callbacks.valid()) {
    return;
  }

  const std::shared_ptr<State> state = state_;
  state_->thread = std::thread(
    [state]() {
      std::unique_lock<std::mutex> lock(state->mutex);
      while (!state->stop_requested) {
        state->condition.wait(
          lock, [state]() {
            return state->stop_requested || state->reset_requested ||
            state->recovery_requested || state->task.has_value();
          });
        if (state->stop_requested) {
          break;
        }
        if ((state->reset_requested || state->recovery_requested) &&
        !state->task.has_value())
        {
          const bool hard_reset = state->reset_requested;
          state->reset_requested = false;
          state->recovery_requested = false;
          lock.unlock();
          if (hard_reset) {
            state->callbacks.reset();
          } else {
            state->callbacks.recover();
          }
          lock.lock();
          state->condition.notify_all();
          continue;
        }

        ControllerInput input = *state->task;
        state->task.reset();
        state->solving = true;
        state->solve_start = std::chrono::steady_clock::now();
        const std::uint64_t generation = state->next_generation++;
        lock.unlock();
        const auto start = std::chrono::steady_clock::now();
        ControllerOutput output;
        try {
          output = state->callbacks.update(input);
        } catch (...) {
          output = ControllerOutput{};
        }
        const auto end = std::chrono::steady_clock::now();
        lock.lock();
        state->solving = false;
        state->result = ControllerWorkerResult{
          output, std::chrono::duration<double>(end - start).count(), generation};
        if (state->reset_requested || state->recovery_requested) {
          const bool hard_reset = state->reset_requested;
          state->reset_requested = false;
          state->recovery_requested = false;
          state->result.reset();
          lock.unlock();
          state->callbacks.reject();
          if (hard_reset) {
            state->callbacks.reset();
          } else {
            state->callbacks.recover();
          }
          lock.lock();
          state->condition.notify_all();
          continue;
        }
        state->condition.notify_all();

        state->condition.wait(
          lock, [state]() {
            return state->stop_requested || state->decision != State::Decision::kNone;
          });
        state->decision = State::Decision::kNone;
        state->condition.notify_all();
      }

      if (state->result.has_value() || state->solving) {
        lock.unlock();
        state->callbacks.reject();
        lock.lock();
      }
      state->exited = true;
      state->condition.notify_all();
    });
}

ControllerWorker::~ControllerWorker()
{
  if (!state_ || !state_->thread.joinable()) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(state_->mutex);
    state_->stop_requested = true;
    state_->condition.notify_all();
  }

  std::unique_lock<std::mutex> lock(state_->mutex);
  const bool exited = state_->condition.wait_for(
    lock, std::chrono::milliseconds(100), [this]() {return state_->exited;});
  lock.unlock();
  if (exited) {
    state_->thread.join();
  } else {
    state_->thread.detach();
  }
}

bool ControllerWorker::valid() const noexcept
{
  return state_ && state_->callbacks.valid();
}

bool ControllerWorker::submit(const ControllerInput & input) noexcept
{
  if (!valid()) {
    return false;
  }
  std::lock_guard<std::mutex> lock(state_->mutex);
  if (state_->stop_requested || state_->solving || state_->task.has_value() ||
    state_->result.has_value() || state_->recovery_requested || state_->reset_requested)
  {
    return false;
  }
  state_->task = input;
  state_->condition.notify_all();
  return true;
}

std::optional<ControllerWorkerResult> ControllerWorker::result() const noexcept
{
  if (!valid()) {
    return std::nullopt;
  }
  std::lock_guard<std::mutex> lock(state_->mutex);
  if (state_->decision != State::Decision::kNone) {
    return std::nullopt;
  }
  return state_->result;
}

bool ControllerWorker::acceptResult() noexcept
{
  if (!valid()) {
    return false;
  }
  std::lock_guard<std::mutex> lock(state_->mutex);
  if (!state_->result.has_value() || state_->decision != State::Decision::kNone) {
    return false;
  }
  const bool accepted = state_->callbacks.accept();
  if (!accepted) {
    state_->callbacks.reject();
  }
  state_->result.reset();
  state_->decision = accepted ? State::Decision::kAccept : State::Decision::kReject;
  state_->condition.notify_all();
  return accepted;
}

bool ControllerWorker::rejectResult() noexcept
{
  if (!valid()) {
    return false;
  }
  std::lock_guard<std::mutex> lock(state_->mutex);
  if (!state_->result.has_value() || state_->decision != State::Decision::kNone) {
    return false;
  }
  state_->callbacks.reject();
  state_->result.reset();
  state_->decision = State::Decision::kReject;
  state_->condition.notify_all();
  return true;
}

bool ControllerWorker::recoverResult() noexcept
{
  if (!valid()) {
    return false;
  }
  std::lock_guard<std::mutex> lock(state_->mutex);
  if (!state_->result.has_value() || state_->decision != State::Decision::kNone) {
    return false;
  }
  state_->callbacks.reject();
  state_->callbacks.recover();
  state_->result.reset();
  state_->decision = State::Decision::kReject;
  state_->condition.notify_all();
  return true;
}

void ControllerWorker::requestRecovery() noexcept
{
  if (!valid()) {
    return;
  }
  std::lock_guard<std::mutex> lock(state_->mutex);
  if (!state_->reset_requested) {
    state_->recovery_requested = true;
  }
  if (state_->result.has_value() && state_->decision == State::Decision::kNone) {
    state_->callbacks.reject();
    state_->result.reset();
    state_->decision = State::Decision::kReject;
  }
  state_->condition.notify_all();
}

void ControllerWorker::requestReset() noexcept
{
  if (!valid()) {
    return;
  }
  std::lock_guard<std::mutex> lock(state_->mutex);
  state_->reset_requested = true;
  state_->recovery_requested = false;
  if (state_->result.has_value() && state_->decision == State::Decision::kNone) {
    state_->callbacks.reject();
    state_->result.reset();
    state_->decision = State::Decision::kReject;
  }
  state_->condition.notify_all();
}

bool ControllerWorker::busy() const noexcept
{
  if (!valid()) {
    return false;
  }
  std::lock_guard<std::mutex> lock(state_->mutex);
  return state_->solving;
}

double ControllerWorker::busyElapsedS() const noexcept
{
  if (!valid()) {
    return 0.0;
  }
  std::lock_guard<std::mutex> lock(state_->mutex);
  if (!state_->solving) {
    return 0.0;
  }
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now() - state_->solve_start).count();
}

}  // namespace flying_hand_control_common
