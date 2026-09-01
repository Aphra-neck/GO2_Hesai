#pragma once

#include <cstdint>
#include <memory>
#include <optional>

namespace utree_go2_sdk2_bridge
{

using SdkCommandSequence = std::uint64_t;

struct SdkVelocityCommand
{
  float vx{0.0F};
  float vy{0.0F};
  float yaw_rate{0.0F};
};

enum class SdkCommandKind
{
  kMove,
  kStop
};

enum class SdkCommandOutcome
{
  kSucceeded,
  kSdkError,
  kSdkException,
  kSuperseded,
  kDiscarded,
  kPreemptedByStop
};

struct SdkCommandCompletion
{
  SdkCommandSequence sequence{0U};
  SdkCommandKind kind{SdkCommandKind::kMove};
  SdkCommandOutcome outcome{SdkCommandOutcome::kSdkException};
  std::int32_t sdk_status{-1};
  SdkVelocityCommand command{};
};

enum class SdkStopState
{
  kConfirmed,
  kPending,
  kFailed
};

struct SdkCommandWorkerStatus
{
  bool accepting{false};
  bool rpc_in_flight{false};
  bool move_pending{false};
  bool move_fault_latched{false};
  SdkStopState stop_state{SdkStopState::kConfirmed};
};

class SdkCommandBackend
{
public:
  virtual ~SdkCommandBackend() = default;

  virtual std::int32_t move(const SdkVelocityCommand & command) = 0;
  virtual std::int32_t stop() = 0;
};

class SdkCommandWorker final
{
public:
  explicit SdkCommandWorker(std::shared_ptr<SdkCommandBackend> backend);
  ~SdkCommandWorker() noexcept;

  SdkCommandWorker(const SdkCommandWorker &) = delete;
  SdkCommandWorker & operator=(const SdkCommandWorker &) = delete;
  SdkCommandWorker(SdkCommandWorker &&) = delete;
  SdkCommandWorker & operator=(SdkCommandWorker &&) = delete;

  std::optional<SdkCommandSequence> submitMove(const SdkVelocityCommand & command);
  std::optional<SdkCommandSequence> submitStop();
  bool discardPendingMove();
  bool resetFaultAfterConfirmedStop();
  bool tryPopCompletion(SdkCommandCompletion & completion);
  SdkCommandWorkerStatus status() const noexcept;
  void shutdown() noexcept;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace utree_go2_sdk2_bridge
