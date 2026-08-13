#include "utree_dog_navigation/flat_obstacle_layer.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace utree_dog_navigation
{
namespace
{
constexpr double kSolveEpsilon = 1.0e-10;
constexpr double kRayEpsilon = 1.0e-10;
constexpr int kRobustGroundFitIterations = 4;
constexpr int kHardGroundFitIterations = 8;

bool finitePoint(const TerrainPoint & point) noexcept
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

double median(std::vector<double> values)
{
  if (values.empty()) {
    return 0.0;
  }
  const auto middle = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2U);
  std::nth_element(values.begin(), middle, values.end());
  const double upper = *middle;
  if (values.size() % 2U != 0U) {
    return upper;
  }
  const auto lower = std::max_element(values.begin(), middle);
  return 0.5 * (*lower + upper);
}

bool solvePlane(
  const std::vector<TerrainPoint> & points,
  const std::vector<std::size_t> & indices,
  double & slope_x, double & slope_y, double & intercept)
{
  if (indices.size() < 3U) {
    return false;
  }
  double xx = 0.0;
  double xy = 0.0;
  double x = 0.0;
  double yy = 0.0;
  double y = 0.0;
  double xz = 0.0;
  double yz = 0.0;
  double z = 0.0;
  for (const std::size_t index : indices) {
    const auto & point = points[index];
    xx += point.x * point.x;
    xy += point.x * point.y;
    x += point.x;
    yy += point.y * point.y;
    y += point.y;
    xz += point.x * point.z;
    yz += point.y * point.z;
    z += point.z;
  }
  std::array<std::array<double, 4>, 3> matrix{{
    {{xx, xy, x, xz}},
    {{xy, yy, y, yz}},
    {{x, y, static_cast<double>(indices.size()), z}},
  }};
  for (std::size_t column = 0U; column < 3U; ++column) {
    std::size_t pivot = column;
    for (std::size_t row = column + 1U; row < 3U; ++row) {
      if (std::abs(matrix[row][column]) > std::abs(matrix[pivot][column])) {
        pivot = row;
      }
    }
    if (std::abs(matrix[pivot][column]) <= kSolveEpsilon) {
      return false;
    }
    if (pivot != column) {
      std::swap(matrix[pivot], matrix[column]);
    }
    const double divisor = matrix[column][column];
    for (std::size_t entry = column; entry < 4U; ++entry) {
      matrix[column][entry] /= divisor;
    }
    for (std::size_t row = 0U; row < 3U; ++row) {
      if (row == column) {
        continue;
      }
      const double factor = matrix[row][column];
      for (std::size_t entry = column; entry < 4U; ++entry) {
        matrix[row][entry] -= factor * matrix[column][entry];
      }
    }
  }
  slope_x = matrix[0][3];
  slope_y = matrix[1][3];
  intercept = matrix[2][3];
  return std::isfinite(slope_x) && std::isfinite(slope_y) && std::isfinite(intercept);
}

std::vector<std::size_t> classifyPlaneInliers(
  const std::vector<TerrainPoint> & points, const FlatGroundPlane & plane,
  double distance)
{
  std::vector<std::size_t> inliers;
  inliers.reserve(points.size());
  for (std::size_t index = 0U; index < points.size(); ++index) {
    if (std::abs(points[index].z - plane.heightAt(points[index].x, points[index].y)) <=
      distance)
    {
      inliers.push_back(index);
    }
  }
  return inliers;
}

bool hasMinimumPlanarSpan(
  const std::vector<TerrainPoint> & points,
  const std::vector<std::size_t> & indices, double minimum_span)
{
  if (indices.empty()) {
    return false;
  }
  double minimum_x = std::numeric_limits<double>::infinity();
  double maximum_x = -std::numeric_limits<double>::infinity();
  double minimum_y = std::numeric_limits<double>::infinity();
  double maximum_y = -std::numeric_limits<double>::infinity();
  for (const std::size_t index : indices) {
    minimum_x = std::min(minimum_x, points[index].x);
    maximum_x = std::max(maximum_x, points[index].x);
    minimum_y = std::min(minimum_y, points[index].y);
    maximum_y = std::max(maximum_y, points[index].y);
  }
  return maximum_x - minimum_x >= minimum_span &&
         maximum_y - minimum_y >= minimum_span;
}

std::int64_t gridCoordinate(double value, double resolution) noexcept
{
  return static_cast<std::int64_t>(std::floor(value / resolution));
}

std::uint64_t packedCell(std::int64_t x, std::int64_t y) noexcept
{
  const auto ux = static_cast<std::uint32_t>(x);
  const auto uy = static_cast<std::uint32_t>(y);
  return (static_cast<std::uint64_t>(ux) << 32U) | static_cast<std::uint64_t>(uy);
}

bool clipAxis(
  double origin, double direction, double minimum, double maximum,
  double & enter, double & exit) noexcept
{
  if (std::abs(direction) <= kRayEpsilon) {
    return origin >= minimum && origin < maximum;
  }
  double first = (minimum - origin) / direction;
  double second = (maximum - origin) / direction;
  if (first > second) {
    std::swap(first, second);
  }
  enter = std::max(enter, first);
  exit = std::min(exit, second);
  return enter <= exit;
}

double rollingOrigin(double center, double size, double resolution) noexcept
{
  return std::floor((center - 0.5 * size) / resolution) * resolution;
}
}  // namespace

double FlatGroundPlane::heightAt(double x, double y) const noexcept
{
  return slope_x * x + slope_y * y + intercept;
}

std::size_t FlatObstacleLayer::VoxelKeyHash::operator()(const VoxelKey & key) const noexcept
{
  std::size_t seed = std::hash<std::int64_t>{}(key.x);
  const auto combine = [&seed](std::int64_t value) {
      const std::size_t hashed = std::hash<std::int64_t>{}(value);
      seed ^= hashed + static_cast<std::size_t>(0x9e3779b9U) +
        (seed << 6U) + (seed >> 2U);
    };
  combine(key.y);
  combine(key.z);
  return seed;
}

const char * toString(FlatObstacleLayerStatus status) noexcept
{
  switch (status) {
    case FlatObstacleLayerStatus::kUninitialized: return "uninitialized";
    case FlatObstacleLayerStatus::kReady: return "ready";
    case FlatObstacleLayerStatus::kWarmingUp: return "warming_up";
    case FlatObstacleLayerStatus::kInvalidInput: return "invalid_input";
    case FlatObstacleLayerStatus::kDuplicateTimestamp: return "duplicate_timestamp";
    case FlatObstacleLayerStatus::kGroundFitFailed: return "ground_fit_failed";
  }
  return "unknown";
}

FlatObstacleLayer::FlatObstacleLayer(FlatObstacleLayerConfig config)
: config_(std::move(config))
{
  const auto & fit = config_.ground_fit;
  if (!std::isfinite(config_.resolution) || config_.resolution <= 0.0 ||
    !std::isfinite(config_.voxel_resolution_z) || config_.voxel_resolution_z <= 0.0 ||
    !std::isfinite(config_.size_x) || config_.size_x <= 0.0 ||
    !std::isfinite(config_.size_y) || config_.size_y <= 0.0 ||
    !std::isfinite(config_.origin_x) || !std::isfinite(config_.origin_y) ||
    !std::isfinite(config_.min_range) || config_.min_range < 0.0 ||
    !std::isfinite(config_.max_range) || config_.max_range <= config_.min_range ||
    !std::isfinite(config_.min_height) || config_.min_height < 0.0 ||
    !std::isfinite(config_.max_height) || config_.max_height <= config_.min_height ||
    !std::isfinite(config_.self_length) || config_.self_length <= 0.0 ||
    !std::isfinite(config_.self_width) || config_.self_width <= 0.0 ||
    !std::isfinite(config_.self_height) || config_.self_height <= 0.0 ||
    !std::isfinite(config_.nominal_body_height) || config_.nominal_body_height <= 0.0 ||
    !std::isfinite(config_.obstacle_clearance) || config_.obstacle_clearance < 0.0 ||
    config_.hit_confirmation_frames < 2U ||
    !std::isfinite(config_.hit_confirmation_window) ||
    config_.hit_confirmation_window <= 0.0 ||
    config_.clear_confirmation_frames == 0U ||
    !std::isfinite(config_.clear_confirmation_window) ||
    config_.clear_confirmation_window <= 0.0 ||
    !std::isfinite(fit.max_range) || fit.max_range <= config_.min_range ||
    !std::isfinite(fit.seed_height_tolerance) || fit.seed_height_tolerance <= 0.0 ||
    !std::isfinite(fit.cell_size) || fit.cell_size <= 0.0 || fit.min_points < 3U ||
    !std::isfinite(fit.min_span) || fit.min_span <= 0.0 ||
    !std::isfinite(fit.inlier_distance) || fit.inlier_distance <= 0.0 ||
    !std::isfinite(fit.min_inlier_ratio) || fit.min_inlier_ratio <= 0.0 ||
    fit.min_inlier_ratio > 1.0 || !std::isfinite(fit.max_rmse) || fit.max_rmse <= 0.0 ||
    !std::isfinite(fit.max_tilt) || fit.max_tilt <= 0.0 ||
    !std::isfinite(fit.max_anchor_error) || fit.max_anchor_error <= 0.0 ||
    fit.max_anchor_error >= config_.min_height)
  {
    throw std::invalid_argument("flat obstacle layer configuration is invalid");
  }
  width_ = static_cast<std::size_t>(std::ceil(config_.size_x / config_.resolution));
  height_ = static_cast<std::size_t>(std::ceil(config_.size_y / config_.resolution));
  snapshot_.resolution = config_.resolution;
  snapshot_.origin_x = config_.origin_x;
  snapshot_.origin_y = config_.origin_y;
  snapshot_.width = width_;
  snapshot_.height = height_;
  snapshot_.raw_obstacles.assign(width_ * height_, 0U);
  snapshot_.inflated_obstacles.assign(width_ * height_, 0U);
}

FlatObstacleLayerUpdate FlatObstacleLayer::update(const FlatObstacleFrame & frame)
{
  FlatObstacleLayerUpdate result;
  if (!finitePoint(frame.body_position) || !finitePoint(frame.sensor_origin) ||
    !std::isfinite(frame.body_yaw) || !std::isfinite(frame.stamp_seconds))
  {
    clearState(FlatObstacleLayerStatus::kInvalidInput, "nonfinite_frame_metadata");
    result.status = snapshot_.status;
    result.reason = snapshot_.reason;
    return result;
  }
  if (have_stamp_ && frame.stamp_seconds == latest_stamp_seconds_) {
    result.status = FlatObstacleLayerStatus::kDuplicateTimestamp;
    result.reason = "duplicate_source_timestamp";
    result.confirmed_voxels = static_cast<std::size_t>(std::count_if(
        evidence_.begin(), evidence_.end(),
        [](const auto & entry) {return entry.second.occupied;}));
    return result;
  }
  const bool forward_gap = have_stamp_ && frame.stamp_seconds > latest_stamp_seconds_ &&
    frame.stamp_seconds - latest_stamp_seconds_ > config_.hit_confirmation_window;
  if (have_stamp_ &&
    (frame.stamp_seconds < latest_stamp_seconds_ || !frame.timing_continuous || forward_gap))
  {
    resetEpoch();
    result.epoch_reset = true;
  }

  snapshot_.origin_x = rollingOrigin(
    frame.body_position.x, config_.size_x, config_.resolution);
  snapshot_.origin_y = rollingOrigin(
    frame.body_position.y, config_.size_y, config_.resolution);
  snapshot_.stamp_seconds = frame.stamp_seconds;

  const double cos_yaw = std::cos(frame.body_yaw);
  const double sin_yaw = std::sin(frame.body_yaw);
  FlatGroundPlane plane;
  std::string fit_reason;
  if (!fitGroundPlane(frame, cos_yaw, sin_yaw, plane, fit_reason)) {
    clearState(FlatObstacleLayerStatus::kGroundFitFailed, std::move(fit_reason));
    latest_stamp_seconds_ = frame.stamp_seconds;
    have_stamp_ = true;
    result.accepted = true;
    result.status = snapshot_.status;
    result.reason = snapshot_.reason;
    result.ground_plane = plane;
    return result;
  }

  std::unordered_map<VoxelKey, TerrainPoint, VoxelKeyHash> frame_obstacles;
  std::unordered_map<VoxelKey, TerrainPoint, VoxelKeyHash> ray_endpoints;
  const std::size_t expected_endpoints = std::min(frame.points.size(), width_ * height_);
  frame_obstacles.reserve(expected_endpoints);
  ray_endpoints.reserve(expected_endpoints);
  snapshot_.filtered_points.clear();
  snapshot_.filtered_points.reserve(expected_endpoints);
  for (const auto & point : frame.points) {
    if (!pointPassesCommonFilters(point, frame, cos_yaw, sin_yaw)) {
      continue;
    }
    const double height = point.z - plane.heightAt(point.x, point.y);
    const VoxelKey key = voxelKey(point.x, point.y, point.z);
    ray_endpoints.try_emplace(key, point);
    if (height < config_.min_height || height > config_.max_height) {
      continue;
    }
    const int grid_x = static_cast<int>(std::floor(
        (point.x - snapshot_.origin_x) / config_.resolution));
    const int grid_y = static_cast<int>(std::floor(
        (point.y - snapshot_.origin_y) / config_.resolution));
    if (grid_x < 0 || grid_y < 0 || grid_x >= static_cast<int>(width_) ||
      grid_y >= static_cast<int>(height_))
    {
      continue;
    }
    const auto inserted = frame_obstacles.try_emplace(key, point);
    if (inserted.second) {
      snapshot_.filtered_points.push_back(point);
    }
  }
  result.filtered_voxels = frame_obstacles.size();

  const bool evidence_continuous = have_stamp_ && frame.timing_continuous;
  for (const auto & entry : frame_obstacles) {
    auto & evidence = evidence_[entry.first];
    evidence.representative_point = entry.second;
    if (evidence.occupied) {
      continue;
    }
    const bool extends_candidate = evidence_continuous && evidence.candidate_frames != 0U &&
      frame.stamp_seconds - evidence.candidate_stamp <= config_.hit_confirmation_window;
    evidence.candidate_frames = extends_candidate ? evidence.candidate_frames + 1U : 1U;
    evidence.candidate_stamp = frame.stamp_seconds;
    if (evidence.candidate_frames >= config_.hit_confirmation_frames) {
      evidence.occupied = true;
      evidence.candidate_frames = 0U;
      ++result.newly_confirmed_voxels;
    }
  }

  std::unordered_set<VoxelKey, VoxelKeyHash> free_voxels;
  free_voxels.reserve(evidence_.size());
  for (const auto & ray_endpoint : ray_endpoints) {
    traceFreeVoxels(frame.sensor_origin, ray_endpoint.second, plane, free_voxels);
  }
  for (const auto & obstacle : frame_obstacles) {
    free_voxels.erase(obstacle.first);
  }

  for (auto entry = evidence_.begin(); entry != evidence_.end(); ) {
    if (cellAddress(entry->second.representative_point) >= width_ * height_) {
      if (entry->second.occupied) {
        ++result.cleared_voxels;
      }
      entry = evidence_.erase(entry);
      continue;
    }
    const bool hit_this_frame = frame_obstacles.find(entry->first) != frame_obstacles.end();
    if (!entry->second.occupied && !hit_this_frame && evidence_continuous &&
      free_voxels.find(entry->first) != free_voxels.end())
    {
      entry = evidence_.erase(entry);
      continue;
    }
    if (entry->second.occupied && !hit_this_frame && evidence_continuous &&
      free_voxels.find(entry->first) != free_voxels.end())
    {
      const bool extends_clear = entry->second.clear_frames != 0U &&
        frame.stamp_seconds - entry->second.clear_stamp <=
        config_.clear_confirmation_window;
      entry->second.clear_frames = extends_clear ? entry->second.clear_frames + 1U : 1U;
      entry->second.clear_stamp = frame.stamp_seconds;
      if (entry->second.clear_frames >= config_.clear_confirmation_frames) {
        entry = evidence_.erase(entry);
        ++result.cleared_voxels;
        continue;
      }
    } else if (!hit_this_frame) {
      entry->second.clear_frames = 0U;
      entry->second.clear_stamp = 0.0;
    }
    if (hit_this_frame) {
      entry->second.clear_frames = 0U;
      entry->second.clear_stamp = 0.0;
    }
    if (!entry->second.occupied && !hit_this_frame &&
      frame.stamp_seconds - entry->second.candidate_stamp >
      config_.hit_confirmation_window)
    {
      entry = evidence_.erase(entry);
      continue;
    }
    ++entry;
  }

  ++accepted_epoch_frames_;
  latest_stamp_seconds_ = frame.stamp_seconds;
  have_stamp_ = true;
  snapshot_.stamp_seconds = frame.stamp_seconds;
  snapshot_.ground_plane = plane;
  snapshot_.raw_obstacles.assign(width_ * height_, 0U);
  snapshot_.obstacle_points.clear();
  std::size_t confirmed = 0U;
  for (const auto & entry : evidence_) {
    if (!entry.second.occupied) {
      continue;
    }
    const std::size_t address = cellAddress(entry.second.representative_point);
    if (address >= snapshot_.raw_obstacles.size()) {
      continue;
    }
    snapshot_.raw_obstacles[address] = 1U;
    snapshot_.obstacle_points.push_back(entry.second.representative_point);
    ++confirmed;
  }
  snapshot_.inflated_obstacles = inflate(snapshot_.raw_obstacles);
  snapshot_.usable = accepted_epoch_frames_ >= config_.hit_confirmation_frames;
  snapshot_.status = snapshot_.usable ?
    FlatObstacleLayerStatus::kReady : FlatObstacleLayerStatus::kWarmingUp;
  snapshot_.reason = snapshot_.usable ? "" : "waiting_for_distinct_source_frames";

  result.accepted = true;
  result.usable = snapshot_.usable;
  result.status = snapshot_.status;
  result.reason = snapshot_.reason;
  result.confirmed_voxels = confirmed;
  result.ground_plane = plane;
  return result;
}

FlatObstacleLayerSnapshot FlatObstacleLayer::snapshot() const
{
  return snapshot_;
}

void FlatObstacleLayer::resetEpoch()
{
  evidence_.clear();
  accepted_epoch_frames_ = 0U;
  have_stamp_ = false;
  latest_stamp_seconds_ = 0.0;
  clearState(FlatObstacleLayerStatus::kUninitialized, "waiting_for_first_frame");
}

const FlatObstacleLayerConfig & FlatObstacleLayer::config() const noexcept
{
  return config_;
}

bool FlatObstacleLayer::fitGroundPlane(
  const FlatObstacleFrame & frame, double cos_yaw, double sin_yaw,
  FlatGroundPlane & plane,
  std::string & reason) const
{
  struct Candidate
  {
    TerrainPoint point;
    double seed_error;
  };
  std::unordered_map<std::uint64_t, Candidate> cells;
  const double expected_ground = frame.body_position.z - config_.nominal_body_height;
  for (const auto & point : frame.points) {
    if (!finitePoint(point)) {
      continue;
    }
    const double dx = point.x - frame.body_position.x;
    const double dy = point.y - frame.body_position.y;
    const double range = std::hypot(dx, dy);
    if (range < config_.min_range || range > config_.ground_fit.max_range ||
      std::abs(point.z - expected_ground) > config_.ground_fit.seed_height_tolerance)
    {
      continue;
    }
    const double body_x = cos_yaw * dx + sin_yaw * dy;
    const double body_y = -sin_yaw * dx + cos_yaw * dy;
    if (std::abs(body_x) < 0.5 * config_.self_length &&
      std::abs(body_y) < 0.5 * config_.self_width)
    {
      continue;
    }
    const std::uint64_t key = packedCell(
      gridCoordinate(point.x, config_.ground_fit.cell_size),
      gridCoordinate(point.y, config_.ground_fit.cell_size));
    const double seed_error = std::abs(point.z - expected_ground);
    const auto found = cells.find(key);
    if (found == cells.end() || seed_error < found->second.seed_error) {
      cells[key] = {point, seed_error};
    }
  }

  std::vector<TerrainPoint> candidates;
  candidates.reserve(cells.size());
  for (const auto & entry : cells) {
    candidates.push_back(entry.second.point);
  }
  std::sort(
    candidates.begin(), candidates.end(),
    [](const TerrainPoint & left, const TerrainPoint & right) {
      if (left.x != right.x) {
        return left.x < right.x;
      }
      if (left.y != right.y) {
        return left.y < right.y;
      }
      return left.z < right.z;
    });
  plane.candidate_points = candidates.size();
  if (candidates.size() < config_.ground_fit.min_points) {
    reason = "insufficient_ground_candidates";
    return false;
  }

  std::vector<std::size_t> inliers(candidates.size());
  for (std::size_t index = 0U; index < inliers.size(); ++index) {
    inliers[index] = index;
  }
  if (!hasMinimumPlanarSpan(candidates, inliers, config_.ground_fit.min_span)) {
    reason = "insufficient_ground_span";
    return false;
  }
  for (int iteration = 0; iteration < kRobustGroundFitIterations; ++iteration) {
    if (!solvePlane(
        candidates, inliers, plane.slope_x, plane.slope_y, plane.intercept))
    {
      reason = "degenerate_ground_geometry";
      return false;
    }
    std::vector<double> absolute_residuals;
    absolute_residuals.reserve(candidates.size());
    for (const auto & point : candidates) {
      absolute_residuals.push_back(
        std::abs(
          point.z - plane.heightAt(point.x, point.y)));
    }
    const double robust_sigma = 1.4826 * median(absolute_residuals);
    const double threshold = std::min(
      config_.ground_fit.inlier_distance,
      std::max(0.5 * config_.ground_fit.inlier_distance, 2.5 * robust_sigma));
    auto next = classifyPlaneInliers(candidates, plane, threshold);
    if (next == inliers) {
      break;
    }
    inliers = std::move(next);
    if (inliers.size() < 3U) {
      reason = "insufficient_ground_inliers";
      return false;
    }
  }
  bool hard_fit_converged = false;
  for (int iteration = 0; iteration < kHardGroundFitIterations; ++iteration) {
    if (!solvePlane(candidates, inliers, plane.slope_x, plane.slope_y, plane.intercept)) {
      reason = "degenerate_ground_geometry";
      return false;
    }
    auto next = classifyPlaneInliers(
      candidates, plane, config_.ground_fit.inlier_distance);
    if (next.size() < 3U) {
      plane.inlier_points = next.size();
      reason = "insufficient_ground_inliers";
      return false;
    }
    if (next == inliers) {
      hard_fit_converged = true;
      break;
    }
    inliers = std::move(next);
  }
  if (!hard_fit_converged) {
    plane.inlier_points = inliers.size();
    reason = "ground_inlier_fit_did_not_converge";
    return false;
  }
  plane.inlier_points = inliers.size();
  if (!hasMinimumPlanarSpan(candidates, inliers, config_.ground_fit.min_span)) {
    reason = "insufficient_ground_inlier_span";
    return false;
  }
  double squared_error = 0.0;
  for (const std::size_t index : inliers) {
    const auto & point = candidates[index];
    const double residual = point.z - plane.heightAt(point.x, point.y);
    squared_error += residual * residual;
  }
  plane.rmse = std::sqrt(squared_error / static_cast<double>(inliers.size()));
  if (static_cast<double>(inliers.size()) / static_cast<double>(candidates.size()) <
    config_.ground_fit.min_inlier_ratio)
  {
    reason = "ground_inlier_ratio_below_limit";
    return false;
  }
  if (plane.rmse > config_.ground_fit.max_rmse) {
    reason = "ground_fit_rmse_above_limit";
    return false;
  }
  if (std::atan(std::hypot(plane.slope_x, plane.slope_y)) > config_.ground_fit.max_tilt) {
    reason = "ground_tilt_above_limit";
    return false;
  }
  const double anchor_error = std::abs(
    plane.heightAt(frame.body_position.x, frame.body_position.y) - expected_ground);
  if (anchor_error > config_.ground_fit.max_anchor_error) {
    reason = "ground_anchor_error_above_limit";
    return false;
  }
  return true;
}

bool FlatObstacleLayer::pointPassesCommonFilters(
  const TerrainPoint & point, const FlatObstacleFrame & frame,
  double cos_yaw, double sin_yaw) const noexcept
{
  if (!finitePoint(point)) {
    return false;
  }
  const double dx = point.x - frame.body_position.x;
  const double dy = point.y - frame.body_position.y;
  const double range = std::hypot(dx, dy);
  if (range < config_.min_range || range > config_.max_range) {
    return false;
  }
  const double body_x = cos_yaw * dx + sin_yaw * dy;
  const double body_y = -sin_yaw * dx + cos_yaw * dy;
  return !(std::abs(body_x) < 0.5 * config_.self_length &&
         std::abs(body_y) < 0.5 * config_.self_width &&
         std::abs(point.z - frame.body_position.z) < 0.5 * config_.self_height);
}

FlatObstacleLayer::VoxelKey FlatObstacleLayer::voxelKey(
  double x, double y, double world_z) const noexcept
{
  return {
    gridCoordinate(x, config_.resolution),
    gridCoordinate(y, config_.resolution),
    gridCoordinate(world_z, config_.voxel_resolution_z)};
}

void FlatObstacleLayer::traceFreeVoxels(
  const TerrainPoint & sensor_origin, const TerrainPoint & endpoint,
  const FlatGroundPlane & plane,
  std::unordered_set<VoxelKey, VoxelKeyHash> & free_voxels) const
{
  const double sensor_height = sensor_origin.z -
    plane.heightAt(sensor_origin.x, sensor_origin.y);
  const TerrainPoint direction{
    endpoint.x - sensor_origin.x,
    endpoint.y - sensor_origin.y,
    endpoint.z - sensor_origin.z};
  const double height_direction = direction.z -
    plane.slope_x * direction.x - plane.slope_y * direction.y;
  if (std::abs(direction.x) <= kRayEpsilon &&
    std::abs(direction.y) <= kRayEpsilon &&
    std::abs(direction.z) <= kRayEpsilon)
  {
    return;
  }

  double enter = 0.0;
  double exit = 1.0;
  const double maximum_x = snapshot_.origin_x +
    static_cast<double>(width_) * config_.resolution;
  const double maximum_y = snapshot_.origin_y +
    static_cast<double>(height_) * config_.resolution;
  if (!clipAxis(
      sensor_origin.x, direction.x, snapshot_.origin_x, maximum_x, enter, exit) ||
    !clipAxis(
      sensor_origin.y, direction.y, snapshot_.origin_y, maximum_y, enter, exit) ||
    !clipAxis(
      sensor_height, height_direction, config_.min_height, config_.max_height,
      enter, exit) ||
    exit < 0.0 || enter > 1.0)
  {
    return;
  }
  enter = std::max(0.0, enter);
  exit = std::min(1.0, exit);
  if (enter >= exit) {
    return;
  }

  const double start_t = std::nextafter(enter, exit);
  const double finish_t = std::nextafter(exit, enter);
  const VoxelKey start = voxelKey(
    sensor_origin.x + direction.x * start_t,
    sensor_origin.y + direction.y * start_t,
    sensor_origin.z + direction.z * start_t);
  const VoxelKey finish = voxelKey(
    sensor_origin.x + direction.x * finish_t,
    sensor_origin.y + direction.y * finish_t,
    sensor_origin.z + direction.z * finish_t);

  std::int64_t x = start.x;
  std::int64_t y = start.y;
  std::int64_t z = start.z;
  const int step_x = direction.x > kRayEpsilon ? 1 :
    (direction.x < -kRayEpsilon ? -1 : 0);
  const int step_y = direction.y > kRayEpsilon ? 1 :
    (direction.y < -kRayEpsilon ? -1 : 0);
  const int step_z = direction.z > kRayEpsilon ? 1 :
    (direction.z < -kRayEpsilon ? -1 : 0);
  const double infinity = std::numeric_limits<double>::infinity();
  const auto nextCrossing = [](
    double origin, double direction_value, double grid_origin,
    double cell_size, std::int64_t coordinate, int step) {
      if (step == 0) {
        return std::numeric_limits<double>::infinity();
      }
      const double boundary = grid_origin +
        static_cast<double>(coordinate + (step > 0 ? 1 : 0)) * cell_size;
      return (boundary - origin) / direction_value;
    };
  double next_x = nextCrossing(
    sensor_origin.x, direction.x, 0.0, config_.resolution, x, step_x);
  double next_y = nextCrossing(
    sensor_origin.y, direction.y, 0.0, config_.resolution, y, step_y);
  double next_z = nextCrossing(
    sensor_origin.z, direction.z, 0.0, config_.voxel_resolution_z, z, step_z);
  const double delta_x = step_x == 0 ? infinity :
    config_.resolution / std::abs(direction.x);
  const double delta_y = step_y == 0 ? infinity :
    config_.resolution / std::abs(direction.y);
  const double delta_z = step_z == 0 ? infinity :
    config_.voxel_resolution_z / std::abs(direction.z);
  const auto voxelDistance = [](std::int64_t first, std::int64_t second) {
      return static_cast<std::size_t>(
        first >= second ? first - second : second - first);
    };
  const std::size_t maximum_steps =
    voxelDistance(start.x, finish.x) + voxelDistance(start.y, finish.y) +
    voxelDistance(start.z, finish.z) + 3U;

  for (std::size_t step = 0U; step < maximum_steps; ++step) {
    const VoxelKey current{x, y, z};
    if (evidence_.find(current) != evidence_.end()) {
      free_voxels.insert(current);
    }
    if (x == finish.x && y == finish.y && z == finish.z) {
      break;
    }
    const double next = std::min({next_x, next_y, next_z});
    if (!std::isfinite(next) || next > exit + kRayEpsilon) {
      break;
    }
    if (next_x <= next + kRayEpsilon) {
      x += step_x;
      next_x += delta_x;
    }
    if (next_y <= next + kRayEpsilon) {
      y += step_y;
      next_y += delta_y;
    }
    if (next_z <= next + kRayEpsilon) {
      z += step_z;
      next_z += delta_z;
    }
  }
}

std::size_t FlatObstacleLayer::cellAddress(const TerrainPoint & point) const noexcept
{
  const int x = static_cast<int>(std::floor(
      (point.x - snapshot_.origin_x) / config_.resolution));
  const int y = static_cast<int>(std::floor(
      (point.y - snapshot_.origin_y) / config_.resolution));
  if (x < 0 || y < 0 || x >= static_cast<int>(width_) || y >= static_cast<int>(height_)) {
    return width_ * height_;
  }
  return static_cast<std::size_t>(y) * width_ + static_cast<std::size_t>(x);
}

std::vector<std::uint8_t> FlatObstacleLayer::inflate(
  const std::vector<std::uint8_t> & raw) const
{
  std::vector<std::uint8_t> result = raw;
  if (config_.obstacle_clearance <= 0.0) {
    return result;
  }
  const int radius = static_cast<int>(std::ceil(
      config_.obstacle_clearance / config_.resolution)) + 1;
  constexpr double kDistanceEpsilon = 1.0e-12;
  for (std::size_t index = 0U; index < raw.size(); ++index) {
    if (raw[index] == 0U) {
      continue;
    }
    const int source_x = static_cast<int>(index % width_);
    const int source_y = static_cast<int>(index / width_);
    for (int dy = -radius; dy <= radius; ++dy) {
      for (int dx = -radius; dx <= radius; ++dx) {
        const double gap_x = static_cast<double>(std::max(0, std::abs(dx) - 1)) *
          config_.resolution;
        const double gap_y = static_cast<double>(std::max(0, std::abs(dy) - 1)) *
          config_.resolution;
        if (std::hypot(gap_x, gap_y) >
          config_.obstacle_clearance + kDistanceEpsilon)
        {
          continue;
        }
        const int x = source_x + dx;
        const int y = source_y + dy;
        if (x >= 0 && y >= 0 && x < static_cast<int>(width_) &&
          y < static_cast<int>(height_))
        {
          result[static_cast<std::size_t>(y) * width_ + static_cast<std::size_t>(x)] = 1U;
        }
      }
    }
  }
  return result;
}

void FlatObstacleLayer::clearState(FlatObstacleLayerStatus status, std::string reason)
{
  evidence_.clear();
  accepted_epoch_frames_ = 0U;
  snapshot_.usable = false;
  snapshot_.status = status;
  snapshot_.reason = std::move(reason);
  snapshot_.ground_plane = FlatGroundPlane{};
  snapshot_.raw_obstacles.assign(width_ * height_, 0U);
  snapshot_.inflated_obstacles.assign(width_ * height_, 0U);
  snapshot_.filtered_points.clear();
  snapshot_.obstacle_points.clear();
}

}  // namespace utree_dog_navigation
