#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <functional>
#include <limits>
#include <vector>

#include "common/measurement_synchronizer.h"

namespace LI2Sup {
namespace {

MeasurementSyncResult evaluate(
  MeasurementSynchronizer & synchronizer,
  double lidar_end_time,
  const std::deque<double> & imu_timestamps)
{
  return synchronizer.evaluate(
    lidar_end_time, imu_timestamps,
    [](double timestamp) {return timestamp;});
}

TEST(MeasurementSynchronizerTest, WaitsUntilImuReachesScanEnd)
{
  MeasurementSynchronizer synchronizer;

  const auto result = evaluate(synchronizer, 10.0, {9.98, 9.99});

  EXPECT_EQ(result.action, MeasurementSyncAction::WaitForImu);
  EXPECT_EQ(result.consume_imu_count, 0U);
}

TEST(MeasurementSynchronizerTest, WaitsWhenNoImuHasArrived)
{
  MeasurementSynchronizer synchronizer;

  const auto result = evaluate(synchronizer, 10.0, {});

  EXPECT_EQ(result.action, MeasurementSyncAction::WaitForImu);
  EXPECT_EQ(result.consume_imu_count, 0U);
}

TEST(MeasurementSynchronizerTest, RejectsEqualAndBackwardLidarEndTimes)
{
  MeasurementSynchronizer synchronizer;
  ASSERT_EQ(
    evaluate(synchronizer, 10.0, {9.98, 9.99, 10.01}).action,
    MeasurementSyncAction::Ready);

  EXPECT_EQ(
    evaluate(synchronizer, 10.0, {10.01}).action,
    MeasurementSyncAction::DropNonmonotonicLidar);
  EXPECT_EQ(
    evaluate(synchronizer, 9.9, {10.01}).action,
    MeasurementSyncAction::DropNonmonotonicLidar);
}

TEST(MeasurementSynchronizerTest, DropsBackloggedScansWithoutNewImuEvidence)
{
  MeasurementSynchronizer synchronizer;
  ASSERT_EQ(
    evaluate(synchronizer, 10.0, {9.98, 9.99, 10.03}).action,
    MeasurementSyncAction::Ready);

  EXPECT_EQ(
    evaluate(synchronizer, 10.1, {10.35}).action,
    MeasurementSyncAction::DropWithoutStateAdvance);
  EXPECT_EQ(
    evaluate(synchronizer, 10.2, {10.35}).action,
    MeasurementSyncAction::DropWithoutStateAdvance);
  EXPECT_EQ(
    evaluate(synchronizer, 10.3, {10.35}).action,
    MeasurementSyncAction::DropWithoutStateAdvance);

  const auto recovered = evaluate(synchronizer, 10.4, {10.35, 10.41});
  EXPECT_EQ(recovered.action, MeasurementSyncAction::Ready);
  EXPECT_EQ(recovered.consume_imu_count, 1U);
  EXPECT_EQ(recovered.first_measurement_imu_index, 0U);
}

TEST(MeasurementSynchronizerTest, ConsumesExactScanEndImu)
{
  MeasurementSynchronizer synchronizer;

  const auto result = evaluate(synchronizer, 10.0, {10.0});

  EXPECT_EQ(result.action, MeasurementSyncAction::Ready);
  EXPECT_EQ(result.consume_imu_count, 1U);
  EXPECT_EQ(result.first_measurement_imu_index, 0U);
}

TEST(MeasurementSynchronizerTest, DropsClosedScanWithOnlyFutureImu)
{
  MeasurementSynchronizer synchronizer;
  ASSERT_EQ(
    evaluate(synchronizer, 10.0, {9.98, 9.99, 10.03}).action,
    MeasurementSyncAction::Ready);

  const auto result = evaluate(synchronizer, 10.01, {10.03});

  EXPECT_EQ(result.action, MeasurementSyncAction::DropWithoutStateAdvance);
  EXPECT_EQ(result.consume_imu_count, 0U);
}

TEST(MeasurementSynchronizerTest, DropsStaleImuButRetainsFutureImu)
{
  MeasurementSynchronizer synchronizer;
  ASSERT_EQ(
    evaluate(synchronizer, 10.0, {9.98, 9.99, 10.03}).action,
    MeasurementSyncAction::Ready);

  const auto result = evaluate(synchronizer, 10.1, {9.99, 10.0, 10.2});

  EXPECT_EQ(result.action, MeasurementSyncAction::DropWithoutStateAdvance);
  EXPECT_EQ(result.consume_imu_count, 2U);
  EXPECT_EQ(result.first_measurement_imu_index, 2U);
}

TEST(MeasurementSynchronizerTest, WaitDoesNotConsumeOrMarkLidarSeen)
{
  MeasurementSynchronizer synchronizer;

  EXPECT_EQ(
    evaluate(synchronizer, 10.0, {9.98, 9.99}).action,
    MeasurementSyncAction::WaitForImu);
  EXPECT_EQ(
    evaluate(synchronizer, 10.0, {9.98, 9.99}).action,
    MeasurementSyncAction::WaitForImu);
  EXPECT_EQ(
    evaluate(synchronizer, 10.0, {9.98, 9.99, 10.0}).action,
    MeasurementSyncAction::Ready);
}

TEST(MeasurementSynchronizerTest, DropMarksLidarSeen)
{
  MeasurementSynchronizer synchronizer;
  ASSERT_EQ(
    evaluate(synchronizer, 10.0, {9.98, 9.99, 10.03}).action,
    MeasurementSyncAction::Ready);
  ASSERT_EQ(
    evaluate(synchronizer, 10.01, {10.03}).action,
    MeasurementSyncAction::DropWithoutStateAdvance);

  EXPECT_EQ(
    evaluate(synchronizer, 10.01, {10.03}).action,
    MeasurementSyncAction::DropNonmonotonicLidar);
}

TEST(MeasurementSynchronizerTest, RejectsNonfiniteLidarEndTime)
{
  MeasurementSynchronizer synchronizer;

  EXPECT_EQ(
    evaluate(
      synchronizer, std::numeric_limits<double>::quiet_NaN(), {10.0}).action,
    MeasurementSyncAction::DropNonmonotonicLidar);
}

TEST(MeasurementSynchronizerTest, QueueRecoveryPublishesStrictlyNewStateTimes)
{
  MeasurementSynchronizer synchronizer;
  std::deque<double> imu_queue{9.98, 9.99, 10.03};
  std::vector<double> published_state_times;

  const auto apply_scan = [&] (double lidar_end_time) {
      const auto result = evaluate(synchronizer, lidar_end_time, imu_queue);
      if (result.action == MeasurementSyncAction::Ready) {
        EXPECT_LT(result.first_measurement_imu_index, result.consume_imu_count);
        if (result.first_measurement_imu_index < result.consume_imu_count) {
          published_state_times.push_back(
            imu_queue[result.consume_imu_count - 1]);
        }
      }
      for (std::size_t index = 0;
        index < result.consume_imu_count; ++index)
      {
        imu_queue.pop_front();
      }
      return result.action;
    };

  EXPECT_EQ(apply_scan(10.0), MeasurementSyncAction::Ready);
  ASSERT_EQ(imu_queue, std::deque<double>({10.03}));

  imu_queue.push_back(10.35);
  EXPECT_EQ(apply_scan(10.1), MeasurementSyncAction::Ready);
  EXPECT_EQ(apply_scan(10.2), MeasurementSyncAction::DropWithoutStateAdvance);
  EXPECT_EQ(apply_scan(10.3), MeasurementSyncAction::DropWithoutStateAdvance);

  imu_queue.push_back(10.41);
  EXPECT_EQ(apply_scan(10.4), MeasurementSyncAction::Ready);
  ASSERT_EQ(imu_queue, std::deque<double>({10.41}));

  ASSERT_EQ(published_state_times.size(), 3U);
  EXPECT_TRUE(std::is_sorted(
    published_state_times.begin(), published_state_times.end(),
    std::less<double>()));
  EXPECT_EQ(published_state_times, std::vector<double>({9.99, 10.03, 10.35}));
}

TEST(MeasurementSynchronizerTest, ExcludesImuAlreadyCoveredByPreviousReadyScan)
{
  MeasurementSynchronizer synchronizer;
  ASSERT_EQ(
    evaluate(synchronizer, 10.0, {9.98, 9.99, 10.03}).action,
    MeasurementSyncAction::Ready);

  const auto result = evaluate(synchronizer, 10.1, {9.99, 10.05, 10.11});

  EXPECT_EQ(result.action, MeasurementSyncAction::Ready);
  EXPECT_EQ(result.consume_imu_count, 2U);
  EXPECT_EQ(result.first_measurement_imu_index, 1U);
}

}  // namespace
}  // namespace LI2Sup
