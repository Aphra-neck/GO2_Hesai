#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>

namespace hesai_frame_timing
{

constexpr double kUnsetMilliseconds = -1.0;
constexpr double kAnomalyThresholdMilliseconds = 250.0;
constexpr std::chrono::seconds kReportInterval{5};

enum class Cause : uint8_t
{
  kNone,
  kFrameIndexDiscontinuity,
  kSourceTimestampNonmonotonic,
  kSdkReadyGap,
  kSourceFrameGap,
  kSourceBacklog,
  kToRosStage,
  kPublishCall,
  kMessageCleanup,
  kCallbackTotal,
  kCallbackPeriod,
};

inline const char* CauseName(Cause cause)
{
  switch (cause) {
    case Cause::kFrameIndexDiscontinuity:
      return "frame_index_gap";
    case Cause::kSourceTimestampNonmonotonic:
      return "source_nonmonotonic";
    case Cause::kSdkReadyGap:
      return "sdk_ready_gap";
    case Cause::kSourceFrameGap:
      return "source_frame_gap";
    case Cause::kSourceBacklog:
      return "source_backlog";
    case Cause::kToRosStage:
      return "to_ros_stage";
    case Cause::kPublishCall:
      return "publish_call";
    case Cause::kMessageCleanup:
      return "message_cleanup";
    case Cause::kCallbackTotal:
      return "callback_total";
    case Cause::kCallbackPeriod:
      return "callback_period";
    case Cause::kNone:
    default:
      return "none";
  }
}

struct Sample
{
  using SteadyClock = std::chrono::steady_clock;

  SteadyClock::time_point callback_start{};
  bool is_multi_frame = false;
  bool source_age_valid = false;
  bool origin_packet_buffer_full = false;
  bool has_frame_index_delta = false;
  bool has_source_start_delta = false;
  int64_t output_frame_index = 0;
  int64_t output_frame_index_delta = -1;
  int raw_frame_index = 0;
  int multi_rate = 1;
  uint32_t points = 0;
  uint32_t packets = 0;
  double output_gap_threshold_ms = kAnomalyThresholdMilliseconds;
  double callback_period_ms = kUnsetMilliseconds;
  double previous_callback_total_ms = kUnsetMilliseconds;
  double sdk_ready_gap_ms = kUnsetMilliseconds;
  double source_start_delta_ms = kUnsetMilliseconds;
  double source_span_ms = kUnsetMilliseconds;
  double source_end_age_ms = kUnsetMilliseconds;
  double to_ros_ms = 0.0;
  double publish_ms = 0.0;
  double cleanup_ms = 0.0;
  double callback_total_ms = 0.0;
};

struct Report
{
  struct Maxima
  {
    double source_start_delta_ms = kUnsetMilliseconds;
    double source_span_ms = kUnsetMilliseconds;
    double source_end_age_ms = kUnsetMilliseconds;
    double callback_period_ms = kUnsetMilliseconds;
    double previous_callback_total_ms = kUnsetMilliseconds;
    double sdk_ready_gap_ms = kUnsetMilliseconds;
    double to_ros_ms = kUnsetMilliseconds;
    double publish_ms = kUnsetMilliseconds;
    double cleanup_ms = kUnsetMilliseconds;
    double callback_total_ms = kUnsetMilliseconds;
  };

  Cause cause = Cause::kNone;
  double max_ms = 0.0;
  uint64_t samples = 0;
  uint64_t anomalies = 0;
  Sample worst{};
  Maxima maxima{};
};

class Window
{
public:
  using SteadyClock = std::chrono::steady_clock;
  using SystemClock = std::chrono::system_clock;

  Sample Begin(
      bool is_multi_frame,
      int output_frame_index,
      int raw_frame_index,
      int multi_rate,
      uint32_t points,
      uint32_t packets,
      double output_frequency_hz,
      double source_start_timestamp,
      double source_end_timestamp,
      bool source_age_valid,
      bool origin_packet_buffer_full,
      SteadyClock::time_point steady_now,
      SystemClock::time_point system_now)
  {
    Sample sample;
    sample.callback_start = steady_now;
    sample.is_multi_frame = is_multi_frame;
    sample.source_age_valid = source_age_valid;
    sample.origin_packet_buffer_full = origin_packet_buffer_full;
    sample.output_frame_index = static_cast<int64_t>(output_frame_index);
    sample.raw_frame_index = raw_frame_index;
    sample.multi_rate = multi_rate;
    sample.points = points;
    sample.packets = packets;
    if (std::isfinite(output_frequency_hz) && output_frequency_hz > 0.0) {
      const double expected_output_period_ms = 1000.0 / output_frequency_hz;
      const double frequency_scaled_threshold_ms =
          expected_output_period_ms * 2.5;
      if (frequency_scaled_threshold_ms > sample.output_gap_threshold_ms) {
        sample.output_gap_threshold_ms = frequency_scaled_threshold_ms;
      }
    }

    if (have_last_callback_start_) {
      sample.callback_period_ms = Milliseconds(steady_now - last_callback_start_);
    }
    if (have_last_callback_end_) {
      sample.sdk_ready_gap_ms = Milliseconds(steady_now - last_callback_end_);
    }
    if (have_last_callback_total_) {
      sample.previous_callback_total_ms = last_callback_total_ms_;
    }
    if (have_last_output_frame_index_) {
      sample.has_frame_index_delta = true;
      sample.output_frame_index_delta =
          sample.output_frame_index - last_output_frame_index_;
    }

    const bool source_timestamps_valid =
        std::isfinite(source_start_timestamp) && source_start_timestamp > 0.0 &&
        std::isfinite(source_end_timestamp) && source_end_timestamp > 0.0;
    if (source_timestamps_valid) {
      sample.source_span_ms =
          (source_end_timestamp - source_start_timestamp) * 1000.0;
      if (have_last_source_start_timestamp_) {
        sample.has_source_start_delta = true;
        sample.source_start_delta_ms =
            (source_start_timestamp - last_source_start_timestamp_) * 1000.0;
      }
      if (source_age_valid) {
        const double system_now_seconds =
            std::chrono::duration<double>(system_now.time_since_epoch()).count();
        sample.source_end_age_ms =
            (system_now_seconds - source_end_timestamp) * 1000.0;
      }
      last_source_start_timestamp_ = source_start_timestamp;
      have_last_source_start_timestamp_ = true;
    }

    last_callback_start_ = steady_now;
    have_last_callback_start_ = true;
    last_output_frame_index_ = sample.output_frame_index;
    have_last_output_frame_index_ = true;
    return sample;
  }

  bool Complete(
      Sample* sample,
      SteadyClock::time_point to_ros_start,
      SteadyClock::time_point after_to_ros,
      SteadyClock::time_point after_publish,
      SteadyClock::time_point after_cleanup,
      Report* report)
  {
    sample->to_ros_ms = Milliseconds(after_to_ros - to_ros_start);
    sample->publish_ms = Milliseconds(after_publish - after_to_ros);
    sample->cleanup_ms = Milliseconds(after_cleanup - after_publish);
    sample->callback_total_ms =
        Milliseconds(after_cleanup - sample->callback_start);
    last_callback_total_ms_ = sample->callback_total_ms;
    have_last_callback_total_ = true;

    if (!window_started_) {
      window_started_ = true;
      window_start_time_ = after_cleanup;
    }

    ObserveMaxima(*sample);
    ++samples_;
    const Attribution attribution = Attribute(*sample);
    const Cause cause = attribution.cause;
    if (cause != Cause::kNone) {
      ++anomalies_;
      if (!have_worst_ || attribution.duration_ms > worst_metric_ms_) {
        worst_ = *sample;
        worst_cause_ = cause;
        worst_metric_ms_ = attribution.duration_ms;
        have_worst_ = true;
      }
    }

    const bool first_anomaly =
        cause != Cause::kNone && !have_reported_anomaly_;
    const bool report_interval_elapsed =
        have_reported_anomaly_ && anomalies_ > 0 &&
        after_cleanup - last_report_time_ >= kReportInterval;
    if (!first_anomaly && !report_interval_elapsed) {
      if (anomalies_ == 0 &&
          after_cleanup - window_start_time_ >= kReportInterval) {
        ResetWindow(after_cleanup);
      }
      return false;
    }

    report->cause = worst_cause_;
    report->max_ms = worst_metric_ms_;
    report->samples = samples_;
    report->anomalies = anomalies_;
    report->worst = worst_;
    report->maxima = maxima_;

    have_reported_anomaly_ = true;
    last_report_time_ = after_cleanup;
    ResetWindow(after_cleanup);
    return true;
  }

  void MarkCallbackReturned(SteadyClock::time_point steady_now)
  {
    last_callback_end_ = steady_now;
    have_last_callback_end_ = true;
  }

private:
  struct Attribution
  {
    Cause cause = Cause::kNone;
    double duration_ms = 0.0;
  };

  void ResetWindow(SteadyClock::time_point steady_now)
  {
    window_start_time_ = steady_now;
    samples_ = 0;
    anomalies_ = 0;
    have_worst_ = false;
    worst_cause_ = Cause::kNone;
    worst_metric_ms_ = 0.0;
    maxima_ = {};
  }

  template<typename DurationT>
  static double Milliseconds(DurationT duration)
  {
    return std::chrono::duration<double, std::milli>(duration).count();
  }

  static Attribution Attribute(const Sample& sample)
  {
    if (sample.has_frame_index_delta && sample.output_frame_index_delta != 1) {
      return {
          Cause::kFrameIndexDiscontinuity,
          ContextDurationMilliseconds(sample)};
    }
    if (sample.has_source_start_delta && sample.source_start_delta_ms <= 0.0) {
      return {
          Cause::kSourceTimestampNonmonotonic,
          ContextDurationMilliseconds(sample)};
    }

    Attribution dominant;
    const auto consider = [&dominant](
        Cause cause, double duration_ms, double threshold_ms) {
      if (std::isfinite(duration_ms) &&
          duration_ms > threshold_ms &&
          duration_ms > dominant.duration_ms) {
        dominant = {cause, duration_ms};
      }
    };

    if (sample.has_source_start_delta &&
        sample.source_start_delta_ms > sample.output_gap_threshold_ms) {
      consider(
          Cause::kSourceFrameGap,
          sample.source_start_delta_ms,
          sample.output_gap_threshold_ms);
    } else {
      consider(
          Cause::kSdkReadyGap,
          sample.sdk_ready_gap_ms,
          sample.output_gap_threshold_ms);
    }
    if (sample.source_age_valid) {
      consider(
          Cause::kSourceBacklog,
          sample.source_end_age_ms,
          kAnomalyThresholdMilliseconds);
    }
    consider(Cause::kToRosStage, sample.to_ros_ms, kAnomalyThresholdMilliseconds);
    consider(Cause::kPublishCall, sample.publish_ms, kAnomalyThresholdMilliseconds);
    consider(Cause::kMessageCleanup, sample.cleanup_ms, kAnomalyThresholdMilliseconds);

    if (dominant.cause == Cause::kNone) {
      consider(
          Cause::kCallbackTotal,
          sample.callback_total_ms,
          kAnomalyThresholdMilliseconds);
    }
    if (dominant.cause == Cause::kNone) {
      consider(
          Cause::kCallbackPeriod,
          sample.callback_period_ms,
          sample.output_gap_threshold_ms);
    }
    return dominant;
  }

  void ObserveMaxima(const Sample& sample)
  {
    UpdateMaximum(sample.source_start_delta_ms, &maxima_.source_start_delta_ms);
    UpdateMaximum(sample.source_span_ms, &maxima_.source_span_ms);
    UpdateMaximum(sample.source_end_age_ms, &maxima_.source_end_age_ms);
    UpdateMaximum(sample.callback_period_ms, &maxima_.callback_period_ms);
    UpdateMaximum(
        sample.previous_callback_total_ms,
        &maxima_.previous_callback_total_ms);
    UpdateMaximum(sample.sdk_ready_gap_ms, &maxima_.sdk_ready_gap_ms);
    UpdateMaximum(sample.to_ros_ms, &maxima_.to_ros_ms);
    UpdateMaximum(sample.publish_ms, &maxima_.publish_ms);
    UpdateMaximum(sample.cleanup_ms, &maxima_.cleanup_ms);
    UpdateMaximum(sample.callback_total_ms, &maxima_.callback_total_ms);
  }

  static void UpdateMaximum(double value, double* maximum)
  {
    if (std::isfinite(value) && value > *maximum) {
      *maximum = value;
    }
  }

  static double ContextDurationMilliseconds(const Sample& sample)
  {
    double duration_ms = 0.0;
    const auto consider = [&duration_ms](double candidate_ms) {
      if (std::isfinite(candidate_ms) && candidate_ms > duration_ms) {
        duration_ms = candidate_ms;
      }
    };
    consider(sample.callback_period_ms);
    consider(std::abs(sample.source_start_delta_ms));
    consider(sample.sdk_ready_gap_ms);
    consider(sample.callback_total_ms);
    return duration_ms;
  }

  SteadyClock::time_point last_callback_start_{};
  SteadyClock::time_point last_callback_end_{};
  SteadyClock::time_point last_report_time_{};
  SteadyClock::time_point window_start_time_{};
  bool have_last_callback_start_ = false;
  bool have_last_callback_end_ = false;
  bool have_last_callback_total_ = false;
  bool have_last_output_frame_index_ = false;
  bool have_last_source_start_timestamp_ = false;
  bool window_started_ = false;
  bool have_reported_anomaly_ = false;
  bool have_worst_ = false;
  int64_t last_output_frame_index_ = 0;
  double last_source_start_timestamp_ = 0.0;
  double last_callback_total_ms_ = 0.0;
  uint64_t samples_ = 0;
  uint64_t anomalies_ = 0;
  Cause worst_cause_ = Cause::kNone;
  double worst_metric_ms_ = 0.0;
  Sample worst_{};
  Report::Maxima maxima_{};
};

}  // namespace hesai_frame_timing
