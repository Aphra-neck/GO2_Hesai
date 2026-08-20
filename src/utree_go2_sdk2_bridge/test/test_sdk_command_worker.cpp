#include "utree_go2_sdk2_bridge/sdk_command_worker.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

namespace utree_go2_sdk2_bridge
{
namespace
{
using namespace std::chrono_literals;

class BlockingBackend final : public SdkCommandBackend
{
public:
  std::int32_t move(const SdkVelocityCommand & command) override
  {
    std::unique_lock<std::mutex> lock(mutex_);
    ++active_calls_;
    max_active_calls_ = std::max(max_active_calls_, active_calls_);
    moves_.push_back(command);
    condition_.notify_all();
    const bool released = condition_.wait_for(
      lock, 500ms, [this]() {return moves_released_;});
    --active_calls_;
    condition_.notify_all();
    if (!released) {
      return -99;
    }
    if (throw_on_move_) {
      throw std::runtime_error("injected Move exception");
    }
    return move_status_;
  }

  std::int32_t stop() override
  {
    std::unique_lock<std::mutex> lock(mutex_);
    ++active_calls_;
    max_active_calls_ = std::max(max_active_calls_, active_calls_);
    ++stop_calls_;
    condition_.notify_all();
    const bool released = condition_.wait_for(
      lock, 500ms, [this]() {return stops_released_;});
    --active_calls_;
    condition_.notify_all();
    if (!released) {
      return -99;
    }
    if (throw_on_stop_) {
      throw std::runtime_error("injected StopMove exception");
    }
    if (stop_failures_before_success_ > 0U) {
      --stop_failures_before_success_;
      return 23;
    }
    return stop_status_;
  }

  bool waitForMoveCount(std::size_t count)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, 2s, [this, count]() {return moves_.size() >= count;});
  }

  void releaseMoves()
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    moves_released_ = true;
    condition_.notify_all();
  }

  std::vector<SdkVelocityCommand> moves() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return moves_;
  }

  bool waitForStopCount(std::size_t count)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, 2s, [this, count]() {return stop_calls_ >= count;});
  }

  std::size_t maxActiveCalls() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return max_active_calls_;
  }

  void setMoveStatus(std::int32_t status)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    move_status_ = status;
  }

  void setThrowOnMove(bool enabled)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    throw_on_move_ = enabled;
  }

  void setStopFailuresBeforeSuccess(std::size_t failures)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    stop_failures_before_success_ = failures;
  }

  void setStopStatus(std::int32_t status)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    stop_status_ = status;
  }

  void setThrowOnStop(bool enabled)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    throw_on_stop_ = enabled;
  }

  std::size_t stopCount() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return stop_calls_;
  }

  void holdStops()
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    stops_released_ = false;
  }

  void releaseStops()
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    stops_released_ = true;
    condition_.notify_all();
  }

private:
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::vector<SdkVelocityCommand> moves_;
  std::size_t stop_calls_{0U};
  std::size_t active_calls_{0U};
  std::size_t max_active_calls_{0U};
  std::int32_t move_status_{0};
  std::int32_t stop_status_{0};
  std::size_t stop_failures_before_success_{0U};
  bool throw_on_move_{false};
  bool throw_on_stop_{false};
  bool moves_released_{false};
  bool stops_released_{true};
};

std::vector<SdkCommandCompletion> waitForCompletions(
  SdkCommandWorker & worker, std::size_t count)
{
  const auto deadline = std::chrono::steady_clock::now() + 2s;
  std::vector<SdkCommandCompletion> completions;
  while (completions.size() < count && std::chrono::steady_clock::now() < deadline) {
    SdkCommandCompletion completion;
    while (worker.tryPopCompletion(completion)) {
      completions.push_back(completion);
    }
    std::this_thread::sleep_for(1ms);
  }
  return completions;
}

TEST(SdkCommandWorker, LatestMoveCoalescesPendingBacklog)
{
  auto backend = std::make_shared<BlockingBackend>();
  SdkCommandWorker worker(backend);

  const auto first = worker.submitMove({0.1F, 0.0F, 0.0F});
  ASSERT_TRUE(first.has_value());
  ASSERT_TRUE(backend->waitForMoveCount(1U));

  const auto stale = worker.submitMove({0.2F, 0.0F, 0.0F});
  const auto latest = worker.submitMove({0.3F, 0.0F, 0.0F});
  ASSERT_TRUE(stale.has_value());
  ASSERT_TRUE(latest.has_value());

  backend->releaseMoves();
  ASSERT_TRUE(backend->waitForMoveCount(2U));
  const auto completions = waitForCompletions(worker, 3U);

  ASSERT_EQ(completions.size(), 3U);
  const auto moves = backend->moves();
  ASSERT_EQ(moves.size(), 2U);
  EXPECT_FLOAT_EQ(moves[0].vx, 0.1F);
  EXPECT_FLOAT_EQ(moves[1].vx, 0.3F);
  EXPECT_EQ(completions[0].sequence, *stale);
  EXPECT_EQ(completions[0].outcome, SdkCommandOutcome::kSuperseded);

  worker.shutdown();
}

TEST(SdkCommandWorker, DiscardPendingMoveCannotExecuteAfterTheInFlightRpc)
{
  auto backend = std::make_shared<BlockingBackend>();
  SdkCommandWorker worker(backend);

  const auto in_flight = worker.submitMove({0.1F, 0.0F, 0.0F});
  ASSERT_TRUE(in_flight.has_value());
  ASSERT_TRUE(backend->waitForMoveCount(1U));

  const auto stale = worker.submitMove({0.2F, 0.0F, 0.0F});
  ASSERT_TRUE(stale.has_value());
  ASSERT_TRUE(worker.status().move_pending);
  EXPECT_TRUE(worker.discardPendingMove());
  EXPECT_FALSE(worker.discardPendingMove());
  EXPECT_FALSE(worker.status().move_pending);

  backend->releaseMoves();
  const auto completions = waitForCompletions(worker, 2U);
  ASSERT_EQ(completions.size(), 2U);
  ASSERT_EQ(backend->moves().size(), 1U);

  const auto discarded = std::find_if(
    completions.begin(), completions.end(),
    [stale](const SdkCommandCompletion & completion) {
      return completion.sequence == *stale;
    });
  ASSERT_NE(discarded, completions.end());
  EXPECT_EQ(discarded->outcome, SdkCommandOutcome::kDiscarded);

  worker.shutdown();
}

TEST(SdkCommandWorker, StopPreemptsPendingMoveAndBlocksMovesUntilConfirmed)
{
  auto backend = std::make_shared<BlockingBackend>();
  SdkCommandWorker worker(backend);

  ASSERT_TRUE(worker.submitMove({0.1F, 0.0F, 0.0F}));
  ASSERT_TRUE(backend->waitForMoveCount(1U));
  const auto preempted = worker.submitMove({0.2F, 0.0F, 0.0F});
  ASSERT_TRUE(preempted.has_value());
  const auto stop = worker.submitStop();
  ASSERT_TRUE(stop.has_value());

  EXPECT_FALSE(worker.submitMove({0.3F, 0.0F, 0.0F}).has_value());
  EXPECT_EQ(worker.status().stop_state, SdkStopState::kPending);
  backend->releaseMoves();
  ASSERT_TRUE(backend->waitForStopCount(1U));
  const auto completions = waitForCompletions(worker, 3U);

  ASSERT_EQ(completions.size(), 3U);
  const auto moves = backend->moves();
  ASSERT_EQ(moves.size(), 1U);
  EXPECT_EQ(completions[0].sequence, *preempted);
  EXPECT_EQ(completions[0].outcome, SdkCommandOutcome::kPreemptedByStop);
  EXPECT_EQ(worker.status().stop_state, SdkStopState::kConfirmed);
  EXPECT_EQ(backend->maxActiveCalls(), 1U);

  worker.shutdown();
}

TEST(SdkCommandWorker, MoveErrorAtomicallyLatchesFaultAndQueuesStop)
{
  auto backend = std::make_shared<BlockingBackend>();
  backend->setMoveStatus(17);
  backend->holdStops();
  SdkCommandWorker worker(backend);

  const auto failed = worker.submitMove({0.1F, 0.0F, 0.0F});
  ASSERT_TRUE(failed.has_value());
  ASSERT_TRUE(backend->waitForMoveCount(1U));
  const auto preempted = worker.submitMove({0.2F, 0.0F, 0.0F});
  ASSERT_TRUE(preempted.has_value());
  backend->releaseMoves();
  ASSERT_TRUE(backend->waitForStopCount(1U));

  const auto pending_status = worker.status();
  EXPECT_TRUE(pending_status.move_fault_latched);
  EXPECT_EQ(pending_status.stop_state, SdkStopState::kPending);
  EXPECT_FALSE(worker.submitMove({0.3F, 0.0F, 0.0F}).has_value());
  backend->releaseStops();
  const auto completions = waitForCompletions(worker, 3U);

  ASSERT_EQ(completions.size(), 3U);
  const auto failed_completion = std::find_if(
    completions.begin(), completions.end(),
    [failed](const SdkCommandCompletion & completion) {
      return completion.sequence == *failed;
    });
  const auto preempted_completion = std::find_if(
    completions.begin(), completions.end(),
    [preempted](const SdkCommandCompletion & completion) {
      return completion.sequence == *preempted;
    });
  const auto stop_completion = std::find_if(
    completions.begin(), completions.end(),
    [](const SdkCommandCompletion & completion) {
      return completion.kind == SdkCommandKind::kStop;
    });
  ASSERT_NE(failed_completion, completions.end());
  ASSERT_NE(preempted_completion, completions.end());
  ASSERT_NE(stop_completion, completions.end());
  EXPECT_EQ(failed_completion->outcome, SdkCommandOutcome::kSdkError);
  EXPECT_EQ(failed_completion->sdk_status, 17);
  EXPECT_EQ(preempted_completion->outcome, SdkCommandOutcome::kPreemptedByStop);
  EXPECT_EQ(stop_completion->outcome, SdkCommandOutcome::kSucceeded);
  EXPECT_EQ(worker.status().stop_state, SdkStopState::kConfirmed);
  EXPECT_FALSE(worker.submitMove({0.4F, 0.0F, 0.0F}).has_value());
  EXPECT_TRUE(worker.resetFaultAfterConfirmedStop());
  EXPECT_TRUE(worker.submitMove({0.5F, 0.0F, 0.0F}).has_value());

  worker.shutdown();
}

TEST(SdkCommandWorker, MoveExceptionAlsoLatchesFaultAndQueuesStop)
{
  auto backend = std::make_shared<BlockingBackend>();
  backend->setThrowOnMove(true);
  backend->releaseMoves();
  SdkCommandWorker worker(backend);

  const auto failed = worker.submitMove({0.1F, 0.0F, 0.0F});
  ASSERT_TRUE(failed.has_value());
  ASSERT_TRUE(backend->waitForStopCount(1U));
  const auto completions = waitForCompletions(worker, 2U);

  ASSERT_EQ(completions.size(), 2U);
  EXPECT_EQ(completions[0].sequence, *failed);
  EXPECT_EQ(completions[0].outcome, SdkCommandOutcome::kSdkException);
  EXPECT_EQ(completions[1].kind, SdkCommandKind::kStop);
  EXPECT_EQ(completions[1].outcome, SdkCommandOutcome::kSucceeded);
  EXPECT_TRUE(worker.status().move_fault_latched);
  EXPECT_EQ(worker.status().stop_state, SdkStopState::kConfirmed);

  worker.shutdown();
}

TEST(SdkCommandWorker, ShutdownWaitsForFinalStopAndRejectsFurtherCommands)
{
  auto backend = std::make_shared<BlockingBackend>();
  backend->releaseMoves();
  SdkCommandWorker worker(backend);
  ASSERT_TRUE(worker.submitMove({0.1F, 0.0F, 0.0F}));
  ASSERT_EQ(waitForCompletions(worker, 1U).size(), 1U);
  backend->holdStops();

  std::atomic<bool> shutdown_finished{false};
  std::thread shutdown_thread([&worker, &shutdown_finished]() {
      worker.shutdown();
      shutdown_finished.store(true);
    });
  const bool stop_started = backend->waitForStopCount(1U);
  EXPECT_TRUE(stop_started);
  if (!stop_started) {
    backend->releaseStops();
    shutdown_thread.join();
    return;
  }
  EXPECT_FALSE(shutdown_finished.load());
  EXPECT_FALSE(worker.submitMove({0.2F, 0.0F, 0.0F}).has_value());
  backend->releaseStops();
  shutdown_thread.join();

  EXPECT_TRUE(shutdown_finished.load());
  EXPECT_EQ(worker.status().stop_state, SdkStopState::kConfirmed);
  EXPECT_FALSE(worker.status().accepting);
  EXPECT_EQ(backend->maxActiveCalls(), 1U);
}

TEST(SdkCommandWorker, ShutdownRetriesStopUntilThirdAttemptSucceeds)
{
  auto backend = std::make_shared<BlockingBackend>();
  backend->setStopFailuresBeforeSuccess(2U);
  SdkCommandWorker worker(backend);

  worker.shutdown();

  EXPECT_EQ(backend->stopCount(), 3U);
  EXPECT_EQ(worker.status().stop_state, SdkStopState::kConfirmed);
}

TEST(SdkCommandWorker, ShutdownStopsAfterThreeSdkErrors)
{
  auto backend = std::make_shared<BlockingBackend>();
  backend->setStopStatus(23);
  SdkCommandWorker worker(backend);

  worker.shutdown();

  EXPECT_EQ(backend->stopCount(), 3U);
  EXPECT_EQ(worker.status().stop_state, SdkStopState::kFailed);
}

TEST(SdkCommandWorker, ShutdownStopsAfterThreeExceptions)
{
  auto backend = std::make_shared<BlockingBackend>();
  backend->setThrowOnStop(true);
  SdkCommandWorker worker(backend);

  worker.shutdown();

  EXPECT_EQ(backend->stopCount(), 3U);
  EXPECT_EQ(worker.status().stop_state, SdkStopState::kFailed);
}

}  // namespace
}  // namespace utree_go2_sdk2_bridge
