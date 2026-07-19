#include <gtest/gtest.h>

#include <flying_hand_control_common/controller_worker.hpp>

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

namespace flying_hand_control_common
{
namespace
{

using namespace std::chrono_literals;

bool waitUntil(const std::function<bool()> & condition, std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (condition()) {
      return true;
    }
    std::this_thread::sleep_for(1ms);
  }
  return condition();
}

TEST(ControllerWorker, SlowSolveDoesNotBlockCallingThread)
{
  auto accepted = std::make_shared<std::atomic<int>>(0);
  ControllerCallbacks callbacks;
  callbacks.update = [](const ControllerInput &) {
      std::this_thread::sleep_for(40ms);
      ControllerOutput output;
      output.feasible = true;
      output.normalized_thrust_frd.z() = -0.3F;
      return output;
    };
  callbacks.accept = [accepted]() {
      ++*accepted;
      return true;
    };
  callbacks.reject = []() {};
  callbacks.recover = []() {};
  callbacks.reset = []() {};
  ControllerWorker worker(std::move(callbacks));

  const auto start = std::chrono::steady_clock::now();
  ASSERT_TRUE(worker.submit(ControllerInput{}));
  EXPECT_LT(std::chrono::steady_clock::now() - start, 5ms);
  ASSERT_TRUE(waitUntil([&worker]() {return worker.busy();}, 20ms));
  EXPECT_GT(worker.busyElapsedS(), 0.0);
  ASSERT_TRUE(waitUntil([&worker]() {return worker.result().has_value();}, 100ms));
  const auto result = worker.result();
  ASSERT_TRUE(result.has_value());
  EXPECT_GE(result->elapsed_s, 0.03);
  EXPECT_TRUE(worker.acceptResult());
  EXPECT_TRUE(waitUntil([accepted]() {return accepted->load() == 1;}, 20ms));
}

TEST(ControllerWorker, ResetDuringSolveRejectsPendingTransaction)
{
  auto rejected = std::make_shared<std::atomic<int>>(0);
  auto reset = std::make_shared<std::atomic<int>>(0);
  ControllerCallbacks callbacks;
  callbacks.update = [](const ControllerInput &) {
      std::this_thread::sleep_for(20ms);
      return ControllerOutput{};
    };
  callbacks.accept = []() {return true;};
  callbacks.reject = [rejected]() {++*rejected;};
  callbacks.recover = []() {};
  callbacks.reset = [reset]() {++*reset;};
  ControllerWorker worker(std::move(callbacks));

  ASSERT_TRUE(worker.submit(ControllerInput{}));
  ASSERT_TRUE(waitUntil([&worker]() {return worker.busy();}, 20ms));
  worker.requestReset();

  EXPECT_TRUE(waitUntil([rejected]() {return rejected->load() == 1;}, 100ms));
  EXPECT_TRUE(waitUntil([reset]() {return reset->load() == 1;}, 100ms));
  EXPECT_FALSE(worker.result().has_value());
}

TEST(ControllerWorker, ReportsFailedStateCommitAndRejectsTransaction)
{
  auto rejected = std::make_shared<std::atomic<int>>(0);
  ControllerCallbacks callbacks;
  callbacks.update = [](const ControllerInput &) {
      return ControllerOutput{};
    };
  callbacks.accept = []() {return false;};
  callbacks.reject = [rejected]() {++*rejected;};
  callbacks.recover = []() {};
  callbacks.reset = []() {};
  ControllerWorker worker(std::move(callbacks));

  ASSERT_TRUE(worker.submit(ControllerInput{}));
  ASSERT_TRUE(waitUntil([&worker]() {return worker.result().has_value();}, 20ms));
  EXPECT_FALSE(worker.acceptResult());
  EXPECT_TRUE(waitUntil([rejected]() {return rejected->load() == 1;}, 20ms));
}

TEST(ControllerWorker, AcceptAllowsNextControlSampleWithoutAnIdleCycle)
{
  auto updates = std::make_shared<std::atomic<int>>(0);
  auto accepted = std::make_shared<std::atomic<int>>(0);
  ControllerCallbacks callbacks;
  callbacks.update = [updates](const ControllerInput &) {
      ++*updates;
      ControllerOutput output;
      output.feasible = true;
      output.normalized_thrust_frd.z() = -0.3F;
      return output;
    };
  callbacks.accept = [accepted]() {
      ++*accepted;
      return true;
    };
  callbacks.reject = []() {};
  callbacks.recover = []() {};
  callbacks.reset = []() {};
  ControllerWorker worker(std::move(callbacks));

  ASSERT_TRUE(worker.submit(ControllerInput{}));
  ASSERT_TRUE(waitUntil([&worker]() {return worker.result().has_value();}, 20ms));
  const auto first = worker.result();
  ASSERT_TRUE(first.has_value());
  EXPECT_TRUE(worker.acceptResult());

  EXPECT_TRUE(worker.submit(ControllerInput{}));
  ASSERT_TRUE(
    waitUntil(
      [&worker, &first]() {
        const auto next = worker.result();
        return next.has_value() && next->generation > first->generation;
      },
      20ms));
  EXPECT_TRUE(worker.acceptResult());
  EXPECT_EQ(updates->load(), 2);
  EXPECT_EQ(accepted->load(), 2);
}

TEST(ControllerWorker, RecoveryRejectsCandidateAndAllowsNextSample)
{
  auto rejected = std::make_shared<std::atomic<int>>(0);
  auto recovered = std::make_shared<std::atomic<int>>(0);
  ControllerCallbacks callbacks;
  callbacks.update = [](const ControllerInput &) {
      ControllerOutput output;
      output.feasible = true;
      output.normalized_thrust_frd.z() = -0.3F;
      return output;
    };
  callbacks.accept = []() {return true;};
  callbacks.reject = [rejected]() {++*rejected;};
  callbacks.recover = [recovered]() {++*recovered;};
  callbacks.reset = []() {};
  ControllerWorker worker(std::move(callbacks));

  ASSERT_TRUE(worker.submit(ControllerInput{}));
  ASSERT_TRUE(waitUntil([&worker]() {return worker.result().has_value();}, 20ms));
  EXPECT_TRUE(worker.recoverResult());
  EXPECT_EQ(rejected->load(), 1);
  EXPECT_EQ(recovered->load(), 1);

  EXPECT_TRUE(worker.submit(ControllerInput{}));
  ASSERT_TRUE(waitUntil([&worker]() {return worker.result().has_value();}, 20ms));
  EXPECT_TRUE(worker.acceptResult());
}

}  // namespace
}  // namespace flying_hand_control_common
