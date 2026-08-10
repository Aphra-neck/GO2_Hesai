#ifndef SUPER_LIO_RUNTIME_TIMING_H_
#define SUPER_LIO_RUNTIME_TIMING_H_

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string_view>

namespace LI2Sup {

enum class SyncWaitReason {
  Ready,
  NoLidar,
  NoImu,
  ImuBehind,
  NonmonotonicLidar,
};

inline constexpr std::string_view syncWaitReasonName(SyncWaitReason reason)
{
  switch (reason) {
    case SyncWaitReason::Ready:
      return "ready";
    case SyncWaitReason::NoLidar:
      return "no_lidar";
    case SyncWaitReason::NoImu:
      return "no_imu";
    case SyncWaitReason::ImuBehind:
      return "imu_behind";
    case SyncWaitReason::NonmonotonicLidar:
      return "nonmonotonic_lidar";
  }
  return "unknown";
}

struct RuntimeTimingSample {
  std::uint64_t frame_sequence = 0;
  double lidar_source_stamp_sec = -1.0;
  double lidar_source_age_ms = -1.0;
  std::size_t raw_point_count = 0;
  std::size_t downsampled_point_count = 0;
  std::size_t imu_measurement_count = 0;
  std::size_t lidar_queue_depth = 0;
  std::size_t imu_queue_depth = 0;

  double process_timer_gap_ms = -1.0;
  double lidar_rmw_gap_ms = -1.0;
  double lidar_callback_gap_ms = -1.0;
  double lidar_dispatch_delay_ms = -1.0;
  double lidar_callback_ms = -1.0;
  double imu_rmw_gap_ms = -1.0;
  double imu_callback_gap_ms = -1.0;
  double imu_dispatch_delay_ms = -1.0;
  double imu_callback_ms = -1.0;

  SyncWaitReason sync_reason = SyncWaitReason::Ready;
  double sync_wait_ms = -1.0;

  double undistort_ms = -1.0;
  double downsample_ms = -1.0;
  double observe_ms = -1.0;
  double update_map_ms = -1.0;
  double odom_publish_ms = -1.0;
  double cloud_transform_ms = -1.0;
  double cloud_to_ros_ms = -1.0;
  double cloud_publish_ms = -1.0;
  double frame_total_ms = -1.0;
};

struct RuntimeTimingAttribution {
  std::string_view cause = "none";
  double duration_ms = -1.0;
};

struct RuntimeTimingReport {
  std::uint64_t sample_count = 0;
  std::uint64_t anomaly_count = 0;
  std::string_view dominant_cause = "none";
  double dominant_ms = -1.0;
  RuntimeTimingSample worst_sample;
};

class BoundedRuntimeTiming {
 public:
  static constexpr double kWarningThresholdMs = 250.0;
  static constexpr std::int64_t kReportPeriodNs = 5'000'000'000LL;

  explicit BoundedRuntimeTiming(
    double warning_threshold_ms = kWarningThresholdMs,
    std::int64_t report_period_ns = kReportPeriodNs)
  : warning_threshold_ms_(warning_threshold_ms),
    report_period_ns_(report_period_ns)
  {
  }

  std::optional<RuntimeTimingReport> observe(
    const RuntimeTimingSample & sample,
    std::int64_t steady_now_ns)
  {
    if (!window_started_) {
      window_started_ = true;
      window_start_ns_ = steady_now_ns;
    }

    ++sample_count_;
    const RuntimeTimingAttribution attribution = attribute(sample);
    const bool anomalous = attribution.duration_ms > warning_threshold_ms_;
    if (anomalous) {
      ++anomaly_count_;
      if (attribution.duration_ms > worst_attribution_.duration_ms) {
        worst_attribution_ = attribution;
        worst_sample_ = sample;
      }
    }

    const bool first_anomaly = anomalous && !has_reported_;
    const bool report_due = has_reported_ && anomaly_count_ > 0 &&
      elapsed(steady_now_ns, last_report_ns_) >= report_period_ns_;
    if (first_anomaly || report_due) {
      return takeReport(steady_now_ns);
    }

    if (anomaly_count_ == 0 &&
      elapsed(steady_now_ns, window_start_ns_) >= report_period_ns_)
    {
      resetWindow(steady_now_ns);
    }
    return std::nullopt;
  }

  std::uint64_t pending_anomaly_count() const
  {
    return anomaly_count_;
  }

 private:
  static std::int64_t elapsed(std::int64_t now, std::int64_t then)
  {
    return now >= then ? now - then : 0;
  }

  RuntimeTimingAttribution attribute(const RuntimeTimingSample & sample) const
  {
    RuntimeTimingAttribution dominant;
    const auto consider = [&dominant](std::string_view cause, double duration_ms) {
        if (std::isfinite(duration_ms) && duration_ms > dominant.duration_ms) {
          dominant = {cause, duration_ms};
        }
      };

    if (sample.lidar_rmw_gap_ms > warning_threshold_ms_) {
      consider("lidar_delivery_gap", sample.lidar_rmw_gap_ms);
    }
    if (sample.lidar_dispatch_delay_ms > warning_threshold_ms_) {
      consider("lidar_executor_backlog", sample.lidar_dispatch_delay_ms);
    }
    if (sample.imu_rmw_gap_ms > warning_threshold_ms_) {
      consider("imu_delivery_gap", sample.imu_rmw_gap_ms);
    }
    if (sample.imu_dispatch_delay_ms > warning_threshold_ms_) {
      consider("imu_executor_backlog", sample.imu_dispatch_delay_ms);
    }

    consider("lidar_callback", sample.lidar_callback_ms);
    consider("imu_callback", sample.imu_callback_ms);
    if (sample.sync_wait_ms > warning_threshold_ms_) {
      switch (sample.sync_reason) {
        case SyncWaitReason::NoLidar:
          consider("sync_wait_no_lidar", sample.sync_wait_ms);
          break;
        case SyncWaitReason::NoImu:
          consider("sync_wait_no_imu", sample.sync_wait_ms);
          break;
        case SyncWaitReason::ImuBehind:
          consider("sync_wait_imu_behind", sample.sync_wait_ms);
          break;
        case SyncWaitReason::NonmonotonicLidar:
          consider("sync_wait_nonmonotonic_lidar", sample.sync_wait_ms);
          break;
        case SyncWaitReason::Ready:
          break;
      }
    }

    consider("undistort", sample.undistort_ms);
    consider("downsample", sample.downsample_ms);
    consider("observe", sample.observe_ms);
    consider("update_map", sample.update_map_ms);
    consider("odom_publish", sample.odom_publish_ms);
    consider("cloud_transform", sample.cloud_transform_ms);
    consider("cloud_to_ros", sample.cloud_to_ros_ms);
    consider("cloud_publish", sample.cloud_publish_ms);

    if (dominant.duration_ms <= warning_threshold_ms_) {
      consider("lidar_source_backlog", sample.lidar_source_age_ms);
      consider("process_timer_gap", sample.process_timer_gap_ms);
      consider("frame_total", sample.frame_total_ms);
    }
    return dominant;
  }

  std::optional<RuntimeTimingReport> takeReport(std::int64_t steady_now_ns)
  {
    RuntimeTimingReport report;
    report.sample_count = sample_count_;
    report.anomaly_count = anomaly_count_;
    report.dominant_cause = worst_attribution_.cause;
    report.dominant_ms = worst_attribution_.duration_ms;
    report.worst_sample = worst_sample_;

    has_reported_ = true;
    last_report_ns_ = steady_now_ns;
    resetWindow(steady_now_ns);
    return report;
  }

  void resetWindow(std::int64_t steady_now_ns)
  {
    window_start_ns_ = steady_now_ns;
    sample_count_ = 0;
    anomaly_count_ = 0;
    worst_attribution_ = {};
    worst_sample_ = {};
  }

  double warning_threshold_ms_;
  std::int64_t report_period_ns_;
  bool window_started_ = false;
  bool has_reported_ = false;
  std::int64_t window_start_ns_ = 0;
  std::int64_t last_report_ns_ = 0;
  std::uint64_t sample_count_ = 0;
  std::uint64_t anomaly_count_ = 0;
  RuntimeTimingAttribution worst_attribution_;
  RuntimeTimingSample worst_sample_;
};

}  // namespace LI2Sup

#endif
