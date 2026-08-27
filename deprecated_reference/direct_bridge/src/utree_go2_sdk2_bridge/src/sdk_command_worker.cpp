#include "utree_go2_sdk2_bridge/sdk_command_worker.hpp"

#include <condition_variable>
#include <deque>
#include <exception>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>

namespace utree_go2_sdk2_bridge
{
namespace
{

constexpr std::size_t kShutdownStopAttempts = 3U;

struct PendingMove
{
  SdkCommandSequence sequence{0U};
  SdkVelocityCommand command{};
};

SdkCommandCompletion executeStop(
  SdkCommandBackend & backend, SdkCommandSequence sequence)
{
  SdkCommandCompletion completion;
  completion.sequence = sequence;
  completion.kind = SdkCommandKind::kStop;
  try {
    completion.sdk_status = backend.stop();
    completion.outcome = completion.sdk_status == 0 ?
      SdkCommandOutcome::kSucceeded : SdkCommandOutcome::kSdkError;
  } catch (...) {
    completion.sdk_status = -1;
    completion.outcome = SdkCommandOutcome::kSdkException;
  }
  return completion;
}

SdkCommandCompletion executeMove(
  SdkCommandBackend & backend, const PendingMove & pending)
{
  SdkCommandCompletion completion;
  completion.sequence = pending.sequence;
  completion.kind = SdkCommandKind::kMove;
  completion.command = pending.command;
  try {
    completion.sdk_status = backend.move(pending.command);
    completion.outcome = completion.sdk_status == 0 ?
      SdkCommandOutcome::kSucceeded : SdkCommandOutcome::kSdkError;
  } catch (...) {
    completion.sdk_status = -1;
    completion.outcome = SdkCommandOutcome::kSdkException;
  }
  return completion;
}

}  // namespace

class SdkCommandWorker::Impl
{
public:
  explicit Impl(std::shared_ptr<SdkCommandBackend> backend)
  : backend_(std::move(backend)), thread_(&Impl::run, this)
  {
  }

  std::optional<SdkCommandSequence> submitMove(const SdkVelocityCommand & command)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    if (!accepting_ || move_fault_latched_ ||
      stop_state_ != SdkStopState::kConfirmed)
    {
      return std::nullopt;
    }

    const SdkCommandSequence sequence = next_sequence_++;
    discardPendingMoveLocked(SdkCommandOutcome::kSuperseded);
    pending_move_ = PendingMove{sequence, command};
    condition_.notify_one();
    return sequence;
  }

  std::optional<SdkCommandSequence> submitStop()
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    if (!accepting_) {
      return std::nullopt;
    }
    if (pending_stop_) {
      return pending_stop_;
    }
    if (in_flight_stop_) {
      return in_flight_stop_;
    }

    discardPendingMoveLocked(SdkCommandOutcome::kPreemptedByStop);
    const SdkCommandSequence sequence = next_sequence_++;
    pending_stop_ = sequence;
    stop_state_ = SdkStopState::kPending;
    condition_.notify_one();
    return sequence;
  }

  bool discardPendingMove()
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return discardPendingMoveLocked(SdkCommandOutcome::kDiscarded);
  }

  bool resetFaultAfterConfirmedStop()
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    if (!accepting_ || stop_state_ != SdkStopState::kConfirmed) {
      return false;
    }
    move_fault_latched_ = false;
    return true;
  }

  bool tryPopCompletion(SdkCommandCompletion & completion)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    if (completions_.empty()) {
      return false;
    }
    completion = completions_.front();
    completions_.pop_front();
    return true;
  }

  SdkCommandWorkerStatus status() const noexcept
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return SdkCommandWorkerStatus{
      accepting_, rpc_in_flight_, pending_move_.has_value(),
      move_fault_latched_, stop_state_};
  }

  void shutdown() noexcept
  {
    const std::lock_guard<std::mutex> shutdown_lock(shutdown_mutex_);
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      if (!shutdown_requested_) {
        accepting_ = false;
        shutdown_requested_ = true;
        discardPendingMoveLocked(SdkCommandOutcome::kPreemptedByStop);
        if (pending_stop_) {
          shutdown_stop_sequence_ = pending_stop_;
        } else if (in_flight_stop_) {
          shutdown_stop_sequence_ = in_flight_stop_;
        } else {
          const SdkCommandSequence sequence = next_sequence_++;
          pending_stop_ = sequence;
          shutdown_stop_sequence_ = sequence;
          stop_state_ = SdkStopState::kPending;
        }
        condition_.notify_one();
      }
    }
    try {
      if (thread_.joinable()) {
        thread_.join();
      }
    } catch (...) {
      // Destruction cannot report a thread-library exception to ROS shutdown.
    }
  }

private:
  void run() noexcept
  {
    while (true) {
      std::optional<PendingMove> move;
      std::optional<SdkCommandSequence> stop;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait(lock, [this]() {return pending_stop_ || pending_move_;});
        if (pending_stop_) {
          stop = pending_stop_;
          pending_stop_.reset();
          in_flight_stop_ = stop;
          rpc_in_flight_ = true;
        } else {
          move = pending_move_;
          pending_move_.reset();
          rpc_in_flight_ = true;
        }
      }

      if (stop) {
        const auto completion = executeStop(*backend_, *stop);
        bool shutdown_complete = false;
        {
          const std::lock_guard<std::mutex> lock(mutex_);
          rpc_in_flight_ = false;
          in_flight_stop_.reset();
          stop_state_ = completion.outcome == SdkCommandOutcome::kSucceeded ?
            SdkStopState::kConfirmed : SdkStopState::kFailed;
          completions_.push_back(completion);
          if (shutdown_requested_ && shutdown_stop_sequence_ == stop) {
            ++shutdown_stop_attempts_;
            shutdown_complete =
              completion.outcome == SdkCommandOutcome::kSucceeded ||
              shutdown_stop_attempts_ >= kShutdownStopAttempts;
            if (!shutdown_complete) {
              const SdkCommandSequence retry_sequence = next_sequence_++;
              pending_stop_ = retry_sequence;
              shutdown_stop_sequence_ = retry_sequence;
              stop_state_ = SdkStopState::kPending;
              condition_.notify_one();
            }
          }
        }
        if (shutdown_complete) {
          return;
        }
        continue;
      }

      const auto completion = executeMove(*backend_, *move);
      {
        const std::lock_guard<std::mutex> lock(mutex_);
        rpc_in_flight_ = false;
        completions_.push_back(completion);
        if (completion.outcome == SdkCommandOutcome::kSdkError ||
          completion.outcome == SdkCommandOutcome::kSdkException)
        {
          move_fault_latched_ = true;
          discardPendingMoveLocked(SdkCommandOutcome::kPreemptedByStop);
          if (!pending_stop_ && !in_flight_stop_) {
            pending_stop_ = next_sequence_++;
          }
          stop_state_ = SdkStopState::kPending;
          condition_.notify_one();
        }
      }
    }
  }

  bool discardPendingMoveLocked(SdkCommandOutcome outcome)
  {
    if (!pending_move_) {
      return false;
    }
    completions_.push_back(
      SdkCommandCompletion{
        pending_move_->sequence, SdkCommandKind::kMove,
        outcome, 0, pending_move_->command});
    pending_move_.reset();
    return true;
  }

  std::shared_ptr<SdkCommandBackend> backend_;
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::deque<SdkCommandCompletion> completions_;
  std::optional<PendingMove> pending_move_;
  std::optional<SdkCommandSequence> pending_stop_;
  std::optional<SdkCommandSequence> in_flight_stop_;
  std::optional<SdkCommandSequence> shutdown_stop_sequence_;
  std::size_t shutdown_stop_attempts_{0U};
  SdkCommandSequence next_sequence_{1U};
  bool accepting_{true};
  bool shutdown_requested_{false};
  bool rpc_in_flight_{false};
  bool move_fault_latched_{false};
  SdkStopState stop_state_{SdkStopState::kConfirmed};
  std::mutex shutdown_mutex_;
  std::thread thread_;
};

SdkCommandWorker::SdkCommandWorker(std::shared_ptr<SdkCommandBackend> backend)
{
  if (!backend) {
    throw std::invalid_argument("SDK command backend must not be null");
  }
  impl_ = std::make_unique<Impl>(std::move(backend));
}

SdkCommandWorker::~SdkCommandWorker() noexcept
{
  shutdown();
}

std::optional<SdkCommandSequence> SdkCommandWorker::submitMove(
  const SdkVelocityCommand & command)
{
  return impl_->submitMove(command);
}

std::optional<SdkCommandSequence> SdkCommandWorker::submitStop()
{
  return impl_->submitStop();
}

bool SdkCommandWorker::discardPendingMove()
{
  return impl_->discardPendingMove();
}

bool SdkCommandWorker::resetFaultAfterConfirmedStop()
{
  return impl_->resetFaultAfterConfirmedStop();
}

bool SdkCommandWorker::tryPopCompletion(SdkCommandCompletion & completion)
{
  return impl_->tryPopCompletion(completion);
}

SdkCommandWorkerStatus SdkCommandWorker::status() const noexcept
{
  return impl_->status();
}

void SdkCommandWorker::shutdown() noexcept
{
  if (impl_) {
    impl_->shutdown();
  }
}

}  // namespace utree_go2_sdk2_bridge
