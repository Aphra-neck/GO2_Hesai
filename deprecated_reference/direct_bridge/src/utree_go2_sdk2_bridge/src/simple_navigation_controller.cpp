#include "utree_go2_sdk2_bridge/simple_navigation_controller.hpp"

#include <algorithm>
#include <cmath>

namespace utree_go2_sdk2_bridge
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

bool finite(double value)
{
  return std::isfinite(value);
}

bool finitePose(const SimpleNavigationPose & pose)
{
  return finite(pose.x) && finite(pose.y) && finite(pose.yaw);
}

bool finiteGoal(const SimpleNavigationGoal & goal)
{
  return finite(goal.x) && finite(goal.y) && finite(goal.yaw);
}

bool validConfig(const SimpleNavigationConfig & config)
{
  return finite(config.position_tolerance) && config.position_tolerance >= 0.0 &&
         finite(config.yaw_tolerance) && config.yaw_tolerance >= 0.0 &&
         finite(config.align_tolerance) && config.align_tolerance >= 0.0 &&
         finite(config.waypoint_cross_track_tolerance) &&
         config.waypoint_cross_track_tolerance >= 0.0 &&
         finite(config.linear_gain) && config.linear_gain > 0.0 &&
         finite(config.lateral_gain) && config.lateral_gain > 0.0 &&
         finite(config.yaw_gain) && config.yaw_gain > 0.0 &&
         finite(config.max_vx) && config.max_vx > 0.0 &&
         finite(config.max_vy) && config.max_vy > 0.0 &&
         finite(config.max_yaw_rate) && config.max_yaw_rate > 0.0;
}

}  // namespace

void SimpleNavigationController::setConfig(const SimpleNavigationConfig & config)
{
  config_ = config;
}

void SimpleNavigationController::setGoal(const SimpleNavigationGoal & goal)
{
  goal_ = goal;
  route_.clear();
  route_index_ = 0;
  segment_start_ = {};
  has_segment_start_ = false;
  phase_ = SimpleNavigationPhase::kAlignSegment;
}

void SimpleNavigationController::clearGoal()
{
  goal_.reset();
  route_.clear();
  route_index_ = 0;
  segment_start_ = {};
  has_segment_start_ = false;
  phase_ = SimpleNavigationPhase::kIdle;
}

void SimpleNavigationController::prepareRoute(const SimpleNavigationPose & pose)
{
  if (goal_ && route_.empty() && route_index_ == 0) {
    rebuildRoute(pose);
  }
}

bool SimpleNavigationController::hasGoal() const
{
  return goal_.has_value();
}

bool SimpleNavigationController::hasRoute() const
{
  return !route_.empty();
}

std::size_t SimpleNavigationController::routeIndex() const
{
  return route_index_;
}

SimpleNavigationPhase SimpleNavigationController::phase() const
{
  return phase_;
}

const std::vector<SimpleWaypoint> & SimpleNavigationController::route() const
{
  return route_;
}

SimpleNavigationCommand SimpleNavigationController::update(const SimpleNavigationPose & pose)
{
  SimpleNavigationCommand output;
  output.route_index = route_index_;
  output.phase = phase_;
  if (!goal_) {
    return output;
  }

  // The ROS node validates these inputs before reaching the controller, but
  // keeping the pure state machine fail-closed prevents NaNs from turning
  // into a seemingly valid SDK command in tests or future callers.
  if (!validConfig(config_) || !finitePose(pose) || !finiteGoal(*goal_)) {
    return output;
  }

  prepareRoute(pose);

  // The loop lets a close/overshot waypoint and the following alignment be
  // handled in one tick, avoiding a zero-command gap at a corner.
  const std::size_t max_iterations = route_.size() + 2;
  for (std::size_t iteration = 0; iteration < max_iterations; ++iteration) {
    output.waypoints_reached += advanceReachedSegments(pose);
    output.route_index = route_index_;

    if (route_index_ >= route_.size()) {
      const double yaw_error = normalizeAngle(goal_->yaw - pose.yaw);
      if (std::abs(yaw_error) <= config_.yaw_tolerance) {
        output.valid = true;
        output.goal_reached = true;
        output.vx = 0.0;
        output.vy = 0.0;
        output.yaw_rate = 0.0;
        output.phase = SimpleNavigationPhase::kIdle;
        clearGoal();
        return output;
      }
      output.valid = true;
      output.vx = 0.0;
      output.vy = 0.0;
      output.yaw_rate = clamp(config_.yaw_gain * yaw_error, config_.max_yaw_rate);
      output.phase = SimpleNavigationPhase::kAlignSegment;
      return output;
    }

    if (phase_ == SimpleNavigationPhase::kAlignSegment) {
      const auto desired = desiredSegmentYaw();
      if (!desired || !finite(pose.yaw)) {
        return output;
      }
      const double yaw_error = normalizeAngle(*desired - pose.yaw);
      if (std::abs(yaw_error) > config_.align_tolerance) {
        output.valid = true;
        output.vx = 0.0;
        output.vy = 0.0;
        output.yaw_rate = clamp(config_.yaw_gain * yaw_error, config_.max_yaw_rate);
        output.phase = phase_;
        return output;
      }
      phase_ = SimpleNavigationPhase::kTranslateSegment;
      output.segment_aligned = true;
    }

    const auto & target = route_[route_index_];
    const double world_dx = target.x - pose.x;
    const double world_dy = target.y - pose.y;
    const double distance = std::hypot(world_dx, world_dy);
    if (!finite(distance) || distance <= config_.position_tolerance) {
      continue;
    }

    const auto body_delta = targetDeltaInBody(pose.x, pose.y, pose.yaw, target.x, target.y);
    const auto desired = desiredSegmentYaw();
    const double yaw_error = desired ? normalizeAngle(*desired - pose.yaw) : 0.0;
    output.valid = true;
    output.vx = clamp(config_.linear_gain * body_delta.x, config_.max_vx);
    output.vy = clamp(config_.lateral_gain * body_delta.y, config_.max_vy);
    output.yaw_rate = clamp(config_.yaw_gain * yaw_error, config_.max_yaw_rate);
    output.phase = phase_;
    return output;
  }

  return output;
}

void SimpleNavigationController::rebuildRoute(const SimpleNavigationPose & pose)
{
  if (!goal_) {
    return;
  }
  route_ = makeRightAngleRoute(
    pose.x, pose.y, pose.yaw, goal_->x, goal_->y, config_.position_tolerance);
  route_index_ = 0;
  segment_start_ = {pose.x, pose.y};
  has_segment_start_ = true;
  phase_ = SimpleNavigationPhase::kAlignSegment;
}

std::size_t SimpleNavigationController::advanceReachedSegments(const SimpleNavigationPose & pose)
{
  std::size_t count = 0;
  while (route_index_ < route_.size() && currentTargetReached(pose)) {
    segment_start_ = route_[route_index_];
    has_segment_start_ = true;
    ++route_index_;
    phase_ = SimpleNavigationPhase::kAlignSegment;
    ++count;
  }
  return count;
}

bool SimpleNavigationController::currentTargetReached(const SimpleNavigationPose & pose) const
{
  if (route_index_ >= route_.size() || !has_segment_start_) {
    return false;
  }
  const auto & target = route_[route_index_];
  const double distance = std::hypot(target.x - pose.x, target.y - pose.y);
  if (finite(distance) && distance <= config_.position_tolerance) {
    return true;
  }

  const double segment_dx = target.x - segment_start_.x;
  const double segment_dy = target.y - segment_start_.y;
  const double segment_length = std::hypot(segment_dx, segment_dy);
  if (!finite(segment_length) || segment_length <= config_.position_tolerance) {
    return false;
  }
  const double direction_x = segment_dx / segment_length;
  const double direction_y = segment_dy / segment_length;
  const double from_start_x = pose.x - segment_start_.x;
  const double from_start_y = pose.y - segment_start_.y;
  const double progress = from_start_x * direction_x + from_start_y * direction_y;
  const double lateral = std::abs(from_start_x * (-direction_y) + from_start_y * direction_x);
  const bool is_final_target = route_index_ + 1 == route_.size();
  return !is_final_target && finite(progress) && finite(lateral) && progress >= segment_length &&
         lateral <= std::max(
    config_.waypoint_cross_track_tolerance, 2.0 * config_.position_tolerance);
}

std::optional<double> SimpleNavigationController::desiredSegmentYaw() const
{
  if (!has_segment_start_ || route_index_ >= route_.size()) {
    return std::nullopt;
  }
  const auto & target = route_[route_index_];
  const double dx = target.x - segment_start_.x;
  const double dy = target.y - segment_start_.y;
  if (std::hypot(dx, dy) <= config_.position_tolerance) {
    return std::nullopt;
  }
  return std::atan2(dy, dx);
}

double SimpleNavigationController::normalizeAngle(double angle)
{
  return std::remainder(angle, 2.0 * kPi);
}

double SimpleNavigationController::clamp(double value, double limit)
{
  return std::clamp(value, -limit, limit);
}

}  // namespace utree_go2_sdk2_bridge
