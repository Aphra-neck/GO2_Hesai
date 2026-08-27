#include <unitree/dds_wrapper/common/unitree_joystick.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <exception>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

namespace
{
using LowState = unitree_go::msg::dds_::LowState_;
using SportState = unitree_go::msg::dds_::SportModeState_;

volatile std::sig_atomic_t keep_running = 1;
std::atomic<std::uint64_t> sport_frames{0};
std::atomic<std::uint64_t> lowstate_frames{0};
std::atomic<std::uint64_t> remote_frames{0};
std::atomic<std::uint64_t> invalid_remote_frames{0};
std::atomic<std::int64_t> last_invalid_remote_diagnostic_ns{0};
std::mutex output_mutex;

constexpr std::int64_t kInvalidRemoteDiagnosticIntervalNs = 1000000000LL;

std::int64_t monotonicNanoseconds()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

void stopSignalHandler(int)
{
  keep_running = 0;
}

template<typename Value>
void appendNumber(std::ostringstream & output, const Value value)
{
  if (std::isfinite(static_cast<double>(value))) {
    output << value;
  } else {
    output << "null";
  }
}

void emitLine(const std::string & line)
{
  std::lock_guard<std::mutex> lock(output_mutex);
  std::cout << line << '\n';
}

void emitLifecycle(const char * event)
{
  std::ostringstream output;
  output << "{\"schema\":1,\"source\":\"reader\",\"event\":\""
         << event << "\",\"mono_ns\":" << monotonicNanoseconds() << '}';
  emitLine(output.str());
}

bool claimInvalidRemoteDiagnostic(const std::int64_t now_ns)
{
  auto previous_ns = last_invalid_remote_diagnostic_ns.load(std::memory_order_relaxed);
  while (previous_ns == 0 ||
    now_ns - previous_ns >= kInvalidRemoteDiagnosticIntervalNs)
  {
    if (last_invalid_remote_diagnostic_ns.compare_exchange_weak(
        previous_ns, now_ns, std::memory_order_relaxed))
    {
      return true;
    }
  }
  return false;
}

void sportCallback(const void * data)
{
  if (data == nullptr) {
    return;
  }
  const auto & message = *static_cast<const SportState *>(data);
  const auto sequence = sport_frames.fetch_add(1, std::memory_order_relaxed) + 1;
  const auto & velocity = message.velocity();

  std::ostringstream output;
  output << std::setprecision(9)
         << "{\"schema\":1,\"source\":\"sport\""
         << ",\"topic\":\"rt/sportmodestate\""
         << ",\"mono_ns\":" << monotonicNanoseconds()
         << ",\"sequence\":" << sequence
         << ",\"error_code\":" << message.error_code()
         << ",\"mode\":" << static_cast<unsigned>(message.mode())
         << ",\"gait_type\":" << static_cast<unsigned>(message.gait_type())
         << ",\"vx\":";
  appendNumber(output, velocity[0]);
  output << ",\"vy\":";
  appendNumber(output, velocity[1]);
  output << ",\"vz\":";
  appendNumber(output, velocity[2]);
  output << ",\"yaw_rate\":";
  appendNumber(output, message.yaw_speed());
  output << ",\"body_height\":";
  appendNumber(output, message.body_height());
  output << '}';
  emitLine(output.str());
}

void lowStateCallback(const void * data)
{
  if (data == nullptr) {
    return;
  }
  const auto & message = *static_cast<const LowState *>(data);
  const auto lowstate_sequence =
    lowstate_frames.fetch_add(1, std::memory_order_relaxed) + 1;
  const auto callback_time_ns = monotonicNanoseconds();
  const std::array<std::uint8_t, 40> bytes = message.wireless_remote();
  const std::uint16_t packet_head =
    (static_cast<std::uint16_t>(bytes[0]) << 8U) |
    static_cast<std::uint16_t>(bytes[1]);
  if (packet_head != 0x5551U) {
    const auto invalid_sequence =
      invalid_remote_frames.fetch_add(1, std::memory_order_relaxed) + 1;
    if (claimInvalidRemoteDiagnostic(callback_time_ns)) {
      std::size_t nonzero_bytes = 0;
      for (const auto byte : bytes) {
        if (byte != 0U) {
          ++nonzero_bytes;
        }
      }
      std::ostringstream output;
      output << "{\"schema\":1,\"source\":\"lowstate\""
             << ",\"topic\":\"rt/lowstate\""
             << ",\"field\":\"wireless_remote\""
             << ",\"event\":\"invalid_wireless_remote_packet\""
             << ",\"mono_ns\":" << callback_time_ns
             << ",\"lowstate_frames\":" << lowstate_sequence
             << ",\"invalid_remote_frames\":" << invalid_sequence
             << ",\"valid_remote_frames\":"
             << remote_frames.load(std::memory_order_relaxed)
             << ",\"packet_head\":" << packet_head
             << ",\"nonzero_bytes\":" << nonzero_bytes
             << '}';
      emitLine(output.str());
    }
    return;
  }
  const auto sequence = remote_frames.fetch_add(1, std::memory_order_relaxed) + 1;
  unitree::common::REMOTE_DATA_RX remote{};
  static_assert(sizeof(remote.buff) == 40U, "Unexpected Unitree remote packet size");
  std::memcpy(remote.buff, bytes.data(), bytes.size());

  std::ostringstream output;
  output << std::setprecision(9)
         << "{\"schema\":1,\"source\":\"remote\""
         << ",\"topic\":\"rt/lowstate\""
         << ",\"field\":\"wireless_remote\""
         << ",\"mono_ns\":" << callback_time_ns
         << ",\"sequence\":" << sequence
         << ",\"packet_head\":" << packet_head
         << ",\"buttons\":" << remote.RF_RX.btn.value
         << ",\"lx\":";
  appendNumber(output, remote.RF_RX.lx);
  output << ",\"ly\":";
  appendNumber(output, remote.RF_RX.ly);
  output << ",\"rx\":";
  appendNumber(output, remote.RF_RX.rx);
  output << ",\"ry\":";
  appendNumber(output, remote.RF_RX.ry);
  output << ",\"l2\":";
  appendNumber(output, remote.RF_RX.L2);
  output << '}';
  emitLine(output.str());
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::cerr << "usage: " << argv[0] << " NETWORK_INTERFACE\n";
    return 2;
  }

  std::signal(SIGINT, stopSignalHandler);
  std::signal(SIGTERM, stopSignalHandler);

  try {
    unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);

    unitree::robot::ChannelSubscriber<SportState> sport_subscriber(
      "rt/sportmodestate");
    sport_subscriber.InitChannel(sportCallback, 1);

    unitree::robot::ChannelSubscriber<LowState> lowstate_subscriber("rt/lowstate");
    lowstate_subscriber.InitChannel(lowStateCallback, 1);

    emitLifecycle("started");
    while (keep_running != 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    emitLifecycle("stopped");
  } catch (const std::exception & exception) {
    std::cerr << "reader_exception=" << exception.what() << '\n';
    return 4;
  } catch (...) {
    std::cerr << "reader_exception=unknown\n";
    return 4;
  }

  if (sport_frames.load(std::memory_order_relaxed) == 0U ||
    lowstate_frames.load(std::memory_order_relaxed) == 0U ||
    remote_frames.load(std::memory_order_relaxed) == 0U)
  {
    std::cerr << "reader_missing_stream sport_frames="
              << sport_frames.load(std::memory_order_relaxed)
              << " lowstate_frames="
              << lowstate_frames.load(std::memory_order_relaxed)
              << " remote_frames="
              << remote_frames.load(std::memory_order_relaxed)
              << " invalid_remote_frames="
              << invalid_remote_frames.load(std::memory_order_relaxed) << '\n';
    return 3;
  }
  return 0;
}
