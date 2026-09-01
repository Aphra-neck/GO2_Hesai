#include <gtest/gtest.h>

#include <cstdint>
#include <string_view>

#include "common/runtime_timing.h"

namespace LI2Sup {
namespace {

RuntimeTimingSample slowObserveSample()
{
  RuntimeTimingSample sample;
  sample.frame_sequence = 42;
  sample.observe_ms = 620.0;
  sample.frame_total_ms = 700.0;
  return sample;
}

TEST(BoundedRuntimeTimingTest, AttributesEstimatorStage)
{
  BoundedRuntimeTiming timing;

  const auto report = timing.observe(slowObserveSample(), 1'000'000'000LL);

  ASSERT_TRUE(report.has_value());
  EXPECT_EQ(report->dominant_cause, std::string_view("observe"));
  EXPECT_DOUBLE_EQ(report->dominant_ms, 620.0);
  EXPECT_EQ(report->worst_sample.frame_sequence, 42U);
}

TEST(BoundedRuntimeTimingTest, AttributesExecutorBacklogWhenMiddlewareArrivalContinues)
{
  BoundedRuntimeTiming timing;
  RuntimeTimingSample sample;
  sample.lidar_rmw_gap_ms = 100.0;
  sample.lidar_dispatch_delay_ms = 2'400.0;

  const auto report = timing.observe(sample, 1'000'000'000LL);

  ASSERT_TRUE(report.has_value());
  EXPECT_EQ(report->dominant_cause, std::string_view("lidar_executor_backlog"));
  EXPECT_DOUBLE_EQ(report->dominant_ms, 2'400.0);
}

TEST(BoundedRuntimeTimingTest, AttributesLargestInputDelayWhenGapAndBacklogAreBothSlow)
{
  BoundedRuntimeTiming timing;
  RuntimeTimingSample sample;
  sample.lidar_rmw_gap_ms = 600.0;
  sample.lidar_dispatch_delay_ms = 2'400.0;

  const auto report = timing.observe(sample, 1'000'000'000LL);

  ASSERT_TRUE(report.has_value());
  EXPECT_EQ(report->dominant_cause, std::string_view("lidar_executor_backlog"));
  EXPECT_DOUBLE_EQ(report->dominant_ms, 2'400.0);
}

TEST(BoundedRuntimeTimingTest, AttributesPublisherStage)
{
  BoundedRuntimeTiming timing;
  RuntimeTimingSample sample;
  sample.cloud_to_ros_ms = 510.0;
  sample.cloud_publish_ms = 12.0;
  sample.frame_total_ms = 560.0;

  const auto report = timing.observe(sample, 1'000'000'000LL);

  ASSERT_TRUE(report.has_value());
  EXPECT_EQ(report->dominant_cause, std::string_view("cloud_to_ros"));
  EXPECT_DOUBLE_EQ(report->dominant_ms, 510.0);
}

TEST(BoundedRuntimeTimingTest, ReportsGrowingSourceBacklogWithoutASlowStage)
{
  BoundedRuntimeTiming timing;
  RuntimeTimingSample sample;
  sample.lidar_source_age_ms = 8'000.0;
  sample.lidar_queue_depth = 18;
  sample.frame_total_ms = 120.0;

  const auto report = timing.observe(sample, 1'000'000'000LL);

  ASSERT_TRUE(report.has_value());
  EXPECT_EQ(report->dominant_cause, std::string_view("lidar_source_backlog"));
  EXPECT_DOUBLE_EQ(report->dominant_ms, 8'000.0);
  EXPECT_EQ(report->worst_sample.lidar_queue_depth, 18U);
}

TEST(BoundedRuntimeTimingTest, ReportsAContinuousSyncWait)
{
  BoundedRuntimeTiming timing;
  RuntimeTimingSample sample;
  sample.sync_reason = SyncWaitReason::ImuBehind;
  sample.sync_wait_ms = 300.0;

  const auto report = timing.observe(sample, 1'000'000'000LL);

  ASSERT_TRUE(report.has_value());
  EXPECT_EQ(report->dominant_cause, std::string_view("sync_wait_imu_behind"));
  EXPECT_DOUBLE_EQ(report->dominant_ms, 300.0);
}

TEST(BoundedRuntimeTimingTest, ThrottlesAndAggregatesForFiveSeconds)
{
  BoundedRuntimeTiming timing;
  RuntimeTimingSample first = slowObserveSample();
  RuntimeTimingSample worse = first;
  worse.frame_sequence = 43;
  worse.observe_ms = 900.0;

  ASSERT_TRUE(timing.observe(first, 1'000'000'000LL).has_value());
  EXPECT_FALSE(timing.observe(worse, 2'000'000'000LL).has_value());
  EXPECT_EQ(timing.pending_anomaly_count(), 1U);

  RuntimeTimingSample normal;
  const auto report = timing.observe(normal, 6'000'000'000LL);

  ASSERT_TRUE(report.has_value());
  EXPECT_EQ(report->anomaly_count, 1U);
  EXPECT_EQ(report->worst_sample.frame_sequence, 43U);
  EXPECT_DOUBLE_EQ(report->dominant_ms, 900.0);
  EXPECT_EQ(timing.pending_anomaly_count(), 0U);
}

TEST(BoundedRuntimeTimingTest, IgnoresUnavailableMessageInfoTimestamps)
{
  BoundedRuntimeTiming timing;
  RuntimeTimingSample sample;
  sample.lidar_rmw_gap_ms = -1.0;
  sample.lidar_dispatch_delay_ms = -1.0;
  sample.frame_total_ms = 25.0;

  EXPECT_FALSE(timing.observe(sample, 1'000'000'000LL).has_value());
}

TEST(BoundedRuntimeTimingTest, KeepsConstantStorageUnderSustainedAnomalies)
{
  static_assert(sizeof(BoundedRuntimeTiming) < 2048);

  BoundedRuntimeTiming timing;
  const RuntimeTimingSample sample = slowObserveSample();
  ASSERT_TRUE(timing.observe(sample, 1'000'000'000LL).has_value());

  for (std::uint64_t i = 0; i < 1'000'000; ++i) {
    EXPECT_FALSE(timing.observe(sample, 1'000'000'001LL).has_value());
  }

  EXPECT_EQ(timing.pending_anomaly_count(), 1'000'000U);
}

}  // namespace
}  // namespace LI2Sup
