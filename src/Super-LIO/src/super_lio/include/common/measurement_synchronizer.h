#ifndef SUPER_LIO_MEASUREMENT_SYNCHRONIZER_H_
#define SUPER_LIO_MEASUREMENT_SYNCHRONIZER_H_

#include <cmath>
#include <cstddef>
#include <optional>

namespace LI2Sup {

enum class MeasurementSyncAction {
  WaitForImu,
  DropNonmonotonicLidar,
  DropWithoutStateAdvance,
  Ready,
};

struct MeasurementSyncResult {
  MeasurementSyncAction action = MeasurementSyncAction::WaitForImu;
  std::size_t consume_imu_count = 0;
  std::size_t first_measurement_imu_index = 0;
};

class MeasurementSynchronizer {
 public:
  template<typename ImuRange, typename TimestampAccessor>
  MeasurementSyncResult evaluate(
    double lidar_end_time,
    const ImuRange & imu_buffer,
    TimestampAccessor timestamp)
  {
    if (!std::isfinite(lidar_end_time) ||
      (last_seen_lidar_end_time_ && lidar_end_time <= *last_seen_lidar_end_time_))
    {
      return {MeasurementSyncAction::DropNonmonotonicLidar, 0, 0};
    }

    if (imu_buffer.empty() || timestamp(imu_buffer.back()) < lidar_end_time) {
      return {MeasurementSyncAction::WaitForImu, 0, 0};
    }

    // A future IMU closes the interval but cannot advance this scan. Ready
    // requires a sample in (last ready lidar end, current lidar end].
    std::size_t consume_count = 0;
    std::size_t first_measurement_index = 0;
    for (const auto & imu : imu_buffer) {
      const double imu_time = timestamp(imu);
      if (imu_time > lidar_end_time) {
        break;
      }
      if (last_ready_lidar_end_time_ &&
        imu_time <= *last_ready_lidar_end_time_)
      {
        ++first_measurement_index;
      }
      ++consume_count;
    }

    last_seen_lidar_end_time_ = lidar_end_time;
    if (first_measurement_index == consume_count) {
      return {
        MeasurementSyncAction::DropWithoutStateAdvance,
        consume_count,
        first_measurement_index};
    }

    last_ready_lidar_end_time_ = lidar_end_time;
    return {
      MeasurementSyncAction::Ready,
      consume_count,
      first_measurement_index};
  }

 private:
  std::optional<double> last_seen_lidar_end_time_;
  std::optional<double> last_ready_lidar_end_time_;
};

}  // namespace LI2Sup

#endif  // SUPER_LIO_MEASUREMENT_SYNCHRONIZER_H_
