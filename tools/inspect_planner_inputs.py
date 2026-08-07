#!/usr/bin/env python3
"""Inspect planner map, odometry, and goal inputs without publishing commands."""

from __future__ import annotations

import argparse
import datetime as dt
import json
import math
import os
import stat
import sys
import tempfile
import time
from collections import deque
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Callable, Mapping, Sequence


SESSION_LOG_MAX_BYTES = 8 * 1024 * 1024
SESSION_RECORD_MAX_BYTES = 1024 * 1024
PARAMETER_SERVICE_TIMEOUT_SEC = 2.0
PARAMETER_RESPONSE_TIMEOUT_SEC = 2.0
PARAMETER_READ_ATTEMPTS = 3


@dataclass(frozen=True)
class GridSnapshot:
    frame_id: str
    stamp_ns: int
    resolution: float
    width: int
    height: int
    origin_x: float
    origin_y: float
    unknown_value: float
    elevation: Sequence[float]
    slope: Sequence[float]
    roughness: Sequence[float]
    traversability: Sequence[float]
    observation_count: Sequence[int]


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float
    frame_id: str
    stamp_ns: int
    child_frame_id: str = ""


@dataclass(frozen=True)
class PlannerThresholds:
    min_traversability: float = 0.18
    max_slope: float = 0.65
    mapper_max_slope: float = 0.65
    max_roughness: float = 0.08
    max_step_height: float = 0.24
    snap_radius: float = 0.5
    min_observed_frames: int = 4
    start_snap_radius: float | None = None


@dataclass(frozen=True)
class PlannerFreshnessSettings:
    max_map_age: float = 1.0
    max_odom_age: float = 0.5
    future_tolerance: float = 0.2
    input_watchdog_rate: float = 10.0


@dataclass(frozen=True)
class InputContract:
    map_frame: str = "world"
    body_frame: str = "base_link"
    max_map_age: float = 1.0
    max_odom_age: float = 0.5
    max_goal_age: float = 2.0
    future_tolerance: float = 0.2


@dataclass(frozen=True)
class SubscriptionSpec:
    key: str
    message_type: object
    topic: str
    qos: object


def _request_parameter_batch(
    node: object,
    get_parameters_type: object,
    spin_until_future_complete: Callable[..., object],
    service_name: str,
    parameter_names: Sequence[str],
    service_timeout: float,
    response_timeout: float,
    max_attempts: int,
) -> Sequence[object]:
    client = node.create_client(get_parameters_type, service_name)
    last_failure = "request was not attempted"
    try:
        for _ in range(max_attempts):
            if not client.wait_for_service(timeout_sec=service_timeout):
                last_failure = (
                    f"service discovery timed out after {service_timeout:.3f} s"
                )
                continue

            request = get_parameters_type.Request()
            request.names = list(parameter_names)
            future = client.call_async(request)
            spin_until_future_complete(
                node,
                future,
                timeout_sec=response_timeout,
            )
            if not future.done():
                future.cancel()
                last_failure = (
                    f"response timed out after {response_timeout:.3f} s"
                )
                continue
            error = future.exception()
            if error is not None:
                last_failure = f"request failed: {error}"
                continue
            response = future.result()
            if response is None:
                last_failure = "request completed without a response"
                continue
            values = response.values
            if len(values) != len(parameter_names):
                raise RuntimeError(
                    f"{service_name} returned {len(values)} values for "
                    f"{len(parameter_names)} requested parameters"
                )
            return values
    finally:
        node.destroy_client(client)

    requested = ", ".join(parameter_names)
    raise RuntimeError(
        f"could not read [{requested}] from {service_name} after "
        f"{max_attempts} attempts: {last_failure}"
    )


def _decode_live_parameter(
    service_name: str,
    parameter_name: str,
    value: object,
    expected_type: int,
    value_field: str,
    expected_type_name: str,
) -> int | float:
    if value.type != expected_type:
        raise RuntimeError(
            f"{service_name} parameter {parameter_name} must be "
            f"{expected_type_name}; received ROS parameter type {value.type}"
        )
    decoded = getattr(value, value_field)
    if isinstance(decoded, float) and not math.isfinite(decoded):
        raise RuntimeError(
            f"{service_name} parameter {parameter_name} must be finite"
        )
    return decoded


def _read_live_planner_thresholds(
    node: object,
    get_parameters_type: object,
    parameter_type: object,
    spin_until_future_complete: Callable[..., object],
    *,
    service_timeout: float,
    response_timeout: float,
    max_attempts: int,
) -> tuple[PlannerThresholds, PlannerFreshnessSettings]:
    """Read mapper and planner settings using two bounded service calls."""
    mapper_service = "/terrain_mapper/get_parameters"
    mapper_names = ("min_observed_frames", "max_slope", "max_roughness")
    mapper_values = _request_parameter_batch(
        node,
        get_parameters_type,
        spin_until_future_complete,
        mapper_service,
        mapper_names,
        service_timeout,
        response_timeout,
        max_attempts,
    )
    planner_service = "/body_lattice_planner/get_parameters"
    planner_names = (
        "min_traversability",
        "max_slope",
        "max_step_height",
        "start_snap_radius",
        "snap_radius",
        "max_map_age",
        "max_odom_age",
        "timestamp_future_tolerance",
        "input_watchdog_rate",
    )
    planner_values = _request_parameter_batch(
        node,
        get_parameters_type,
        spin_until_future_complete,
        planner_service,
        planner_names,
        service_timeout,
        response_timeout,
        max_attempts,
    )
    thresholds = PlannerThresholds(
        min_observed_frames=int(
            _decode_live_parameter(
                mapper_service,
                mapper_names[0],
                mapper_values[0],
                parameter_type.PARAMETER_INTEGER,
                "integer_value",
                "an integer",
            )
        ),
        mapper_max_slope=float(
            _decode_live_parameter(
                mapper_service,
                mapper_names[1],
                mapper_values[1],
                parameter_type.PARAMETER_DOUBLE,
                "double_value",
                "a double",
            )
        ),
        max_roughness=float(
            _decode_live_parameter(
                mapper_service,
                mapper_names[2],
                mapper_values[2],
                parameter_type.PARAMETER_DOUBLE,
                "double_value",
                "a double",
            )
        ),
        min_traversability=float(
            _decode_live_parameter(
                planner_service,
                planner_names[0],
                planner_values[0],
                parameter_type.PARAMETER_DOUBLE,
                "double_value",
                "a double",
            )
        ),
        max_slope=float(
            _decode_live_parameter(
                planner_service,
                planner_names[1],
                planner_values[1],
                parameter_type.PARAMETER_DOUBLE,
                "double_value",
                "a double",
            )
        ),
        max_step_height=float(
            _decode_live_parameter(
                planner_service,
                planner_names[2],
                planner_values[2],
                parameter_type.PARAMETER_DOUBLE,
                "double_value",
                "a double",
            )
        ),
        start_snap_radius=float(
            _decode_live_parameter(
                planner_service,
                planner_names[3],
                planner_values[3],
                parameter_type.PARAMETER_DOUBLE,
                "double_value",
                "a double",
            )
        ),
        snap_radius=float(
            _decode_live_parameter(
                planner_service,
                planner_names[4],
                planner_values[4],
                parameter_type.PARAMETER_DOUBLE,
                "double_value",
                "a double",
            )
        ),
    )
    _validate_thresholds(thresholds)
    freshness = PlannerFreshnessSettings(
        max_map_age=float(
            _decode_live_parameter(
                planner_service,
                planner_names[5],
                planner_values[5],
                parameter_type.PARAMETER_DOUBLE,
                "double_value",
                "a double",
            )
        ),
        max_odom_age=float(
            _decode_live_parameter(
                planner_service,
                planner_names[6],
                planner_values[6],
                parameter_type.PARAMETER_DOUBLE,
                "double_value",
                "a double",
            )
        ),
        future_tolerance=float(
            _decode_live_parameter(
                planner_service,
                planner_names[7],
                planner_values[7],
                parameter_type.PARAMETER_DOUBLE,
                "double_value",
                "a double",
            )
        ),
        input_watchdog_rate=float(
            _decode_live_parameter(
                planner_service,
                planner_names[8],
                planner_values[8],
                parameter_type.PARAMETER_DOUBLE,
                "double_value",
                "a double",
            )
        ),
    )
    _validate_freshness_settings(freshness)
    return thresholds, freshness


def _validate_grid(grid: GridSnapshot) -> None:
    if not math.isfinite(grid.resolution) or grid.resolution <= 0.0:
        raise ValueError("map resolution must be finite and positive")
    if grid.width <= 0 or grid.height <= 0:
        raise ValueError("map dimensions must be positive")
    for name, value in (
        ("origin_x", grid.origin_x),
        ("origin_y", grid.origin_y),
        ("unknown_value", grid.unknown_value),
    ):
        if not math.isfinite(value):
            raise ValueError(f"map {name} must be finite")
    expected = grid.width * grid.height
    for name, values in (
        ("elevation", grid.elevation),
        ("slope", grid.slope),
        ("roughness", grid.roughness),
        ("traversability", grid.traversability),
        ("observation_count", grid.observation_count),
    ):
        if len(values) != expected:
            raise ValueError(
                f"map {name} length {len(values)} does not match {expected} cells"
            )


def _validate_thresholds(thresholds: PlannerThresholds) -> None:
    for name, value in (
        ("min_traversability", thresholds.min_traversability),
        ("max_slope", thresholds.max_slope),
        ("mapper_max_slope", thresholds.mapper_max_slope),
        ("max_roughness", thresholds.max_roughness),
        ("max_step_height", thresholds.max_step_height),
        ("snap_radius", thresholds.snap_radius),
    ):
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(f"{name} must be finite and non-negative")
    if thresholds.start_snap_radius is not None and (
        not math.isfinite(thresholds.start_snap_radius)
        or thresholds.start_snap_radius < 0.0
    ):
        raise ValueError("start_snap_radius must be finite and non-negative")
    if thresholds.min_observed_frames < 1:
        raise ValueError("min_observed_frames must be positive")


def _validate_freshness_settings(settings: PlannerFreshnessSettings) -> None:
    for name, value, minimum, maximum in (
        ("max_map_age", settings.max_map_age, 0.001, 60.0),
        ("max_odom_age", settings.max_odom_age, 0.001, 60.0),
        ("future_tolerance", settings.future_tolerance, 0.0, 5.0),
        ("input_watchdog_rate", settings.input_watchdog_rate, 0.1, 100.0),
    ):
        if not math.isfinite(value) or not minimum <= value <= maximum:
            raise ValueError(
                f"{name} must be finite and in [{minimum}, {maximum}]"
            )


def _validate_pose(name: str, pose: Pose2D) -> None:
    for field, value in (("x", pose.x), ("y", pose.y), ("yaw", pose.yaw)):
        if not math.isfinite(value):
            raise ValueError(f"{name} {field} must be finite")


def _world_to_grid(grid: GridSnapshot, x: float, y: float) -> tuple[int, int]:
    return (
        math.floor((x - grid.origin_x) / grid.resolution),
        math.floor((y - grid.origin_y) / grid.resolution),
    )


def _inside(grid: GridSnapshot, x: int, y: int) -> bool:
    return 0 <= x < grid.width and 0 <= y < grid.height


def _address(grid: GridSnapshot, x: int, y: int) -> int:
    return y * grid.width + x


def _known_layer_value(value: float, unknown_value: float) -> bool:
    return value != unknown_value and math.isfinite(value)


def _terrain_layer_stats(
    grid: GridSnapshot,
    thresholds: PlannerThresholds,
    indices: Sequence[int] | None = None,
) -> dict[str, int | float | None]:
    """Count independent terrain-layer facts over the requested cells."""
    selected = range(grid.width * grid.height) if indices is None else indices
    stats: dict[str, int | float | None] = {
        "cell_count": len(selected),
        "observation_zero": 0,
        "observation_below_min": 0,
        "observation_ready": 0,
        "elevation_known": 0,
        "elevation_unknown_or_nonfinite": 0,
        "elevation_known_below_min_observations": 0,
        "slope_known": 0,
        "slope_unknown_or_nonfinite": 0,
        "roughness_known": 0,
        "roughness_unknown_or_nonfinite": 0,
        "traversability_known": 0,
        "traversability_unknown_or_nonfinite": 0,
        "features_known": 0,
        "slope_over_limit": 0,
        "slope_at_or_above_mapper_limit": 0,
        "roughness_over_limit": 0,
        "traversability_zero": 0,
        "traversability_below_min_nonzero": 0,
        "traversability_at_or_above_min": 0,
        "zero_traversability_with_slope_and_roughness_below_limits": 0,
        "planner_valid": 0,
        "slope_min_rad": None,
        "slope_p50_rad": None,
        "slope_p90_rad": None,
        "slope_p95_rad": None,
        "slope_max_rad": None,
    }
    known_slopes: list[float] = []
    for index in selected:
        observations = int(grid.observation_count[index])
        if observations == 0:
            stats["observation_zero"] += 1
        elif observations < thresholds.min_observed_frames:
            stats["observation_below_min"] += 1
        else:
            stats["observation_ready"] += 1

        elevation = grid.elevation[index]
        slope = grid.slope[index]
        roughness = grid.roughness[index]
        traversability = grid.traversability[index]
        elevation_known = _known_layer_value(elevation, grid.unknown_value)
        slope_known = _known_layer_value(slope, grid.unknown_value)
        roughness_known = _known_layer_value(roughness, grid.unknown_value)
        traversability_known = _known_layer_value(
            traversability, grid.unknown_value
        )

        if elevation_known:
            stats["elevation_known"] += 1
        else:
            stats["elevation_unknown_or_nonfinite"] += 1
        if slope_known:
            stats["slope_known"] += 1
            known_slopes.append(float(slope))
        else:
            stats["slope_unknown_or_nonfinite"] += 1
        if roughness_known:
            stats["roughness_known"] += 1
        else:
            stats["roughness_unknown_or_nonfinite"] += 1
        if traversability_known:
            stats["traversability_known"] += 1
        else:
            stats["traversability_unknown_or_nonfinite"] += 1

        if elevation_known and observations < thresholds.min_observed_frames:
            stats["elevation_known_below_min_observations"] += 1
        if slope_known and roughness_known and traversability_known:
            stats["features_known"] += 1
        if slope_known and slope > thresholds.max_slope:
            stats["slope_over_limit"] += 1
        if slope_known and slope >= thresholds.mapper_max_slope:
            stats["slope_at_or_above_mapper_limit"] += 1
        if roughness_known and roughness > thresholds.max_roughness:
            stats["roughness_over_limit"] += 1
        if traversability_known:
            if traversability == 0.0:
                stats["traversability_zero"] += 1
            elif 0.0 < traversability < thresholds.min_traversability:
                stats["traversability_below_min_nonzero"] += 1
            if traversability >= thresholds.min_traversability:
                stats["traversability_at_or_above_min"] += 1
        if (
            traversability_known
            and traversability == 0.0
            and slope_known
            and slope < thresholds.mapper_max_slope
            and roughness_known
            and roughness < thresholds.max_roughness
        ):
            stats[
                "zero_traversability_with_slope_and_roughness_below_limits"
            ] += 1
        if (
            traversability_known
            and slope_known
            and traversability >= thresholds.min_traversability
            and slope <= thresholds.max_slope
        ):
            stats["planner_valid"] += 1

    if known_slopes:
        known_slopes.sort()

        def percentile(fraction: float) -> float:
            position = fraction * (len(known_slopes) - 1)
            lower = math.floor(position)
            upper = math.ceil(position)
            weight = position - lower
            return (
                known_slopes[lower] * (1.0 - weight)
                + known_slopes[upper] * weight
            )

        stats.update(
            {
                "slope_min_rad": known_slopes[0],
                "slope_p50_rad": percentile(0.50),
                "slope_p90_rad": percentile(0.90),
                "slope_p95_rad": percentile(0.95),
                "slope_max_rad": known_slopes[-1],
            }
        )
    return stats


def _valid_mask(
    grid: GridSnapshot, thresholds: PlannerThresholds
) -> tuple[bytearray, int, int]:
    total = grid.width * grid.height
    mask = bytearray(total)
    known = 0
    valid = 0
    for index in range(total):
        traversability = grid.traversability[index]
        slope = grid.slope[index]
        if not _known_layer_value(
            traversability, grid.unknown_value
        ) or not _known_layer_value(slope, grid.unknown_value):
            continue
        known += 1
        if (
            traversability >= thresholds.min_traversability
            and slope <= thresholds.max_slope
        ):
            mask[index] = 1
            valid += 1
    return mask, known, valid


def _continuous_ground_mask(
    grid: GridSnapshot, planner_valid_mask: bytearray
) -> tuple[bytearray, int]:
    mask = bytearray(len(planner_valid_mask))
    count = 0
    for index, planner_valid in enumerate(planner_valid_mask):
        elevation = grid.elevation[index]
        if (
            planner_valid
            and elevation != grid.unknown_value
            and math.isfinite(elevation)
        ):
            mask[index] = 1
            count += 1
    return mask, count


def _snap_endpoint(
    grid: GridSnapshot,
    mask: bytearray,
    pose: Pose2D,
    thresholds: PlannerThresholds,
    snap_radius: float,
) -> dict[str, object]:
    grid_x, grid_y = _world_to_grid(grid, pose.x, pose.y)
    result: dict[str, object] = {
        "frame_id": pose.frame_id,
        "child_frame_id": pose.child_frame_id,
        "stamp_ns": pose.stamp_ns,
        "world_x": pose.x,
        "world_y": pose.y,
        "yaw_rad": pose.yaw,
        "snap_radius_m": snap_radius,
        "grid_x": grid_x,
        "grid_y": grid_y,
        "inside_map": _inside(grid, grid_x, grid_y),
        "exact_cell_valid": False,
        "valid_cells_in_snap_square": 0,
        "valid_cells_in_snap_radius": 0,
        "snapped": False,
        "snapped_grid_x": None,
        "snapped_grid_y": None,
        "snapped_world_x": None,
        "snapped_world_y": None,
        "snap_grid_distance_m": None,
        "snap_world_to_center_distance_m": None,
        "snap_grid_distance_outside_nominal_radius": False,
        "snap_square_terrain_layers": None,
        "snap_radius_terrain_layers": None,
    }
    if not result["inside_map"]:
        return result

    exact_valid = bool(mask[_address(grid, grid_x, grid_y)])
    result["exact_cell_valid"] = exact_valid
    radius = max(1, math.ceil(snap_radius / grid.resolution))
    candidates: list[tuple[float, int, int]] = []
    snap_square_indices: list[int] = []
    snap_radius_indices: list[int] = []
    for delta_y in range(-radius, radius + 1):
        for delta_x in range(-radius, radius + 1):
            x = grid_x + delta_x
            y = grid_y + delta_y
            if not _inside(grid, x, y):
                continue
            address = _address(grid, x, y)
            snap_square_indices.append(address)
            candidate_world_x = grid.origin_x + (x + 0.5) * grid.resolution
            candidate_world_y = grid.origin_y + (y + 0.5) * grid.resolution
            distance_m = math.hypot(
                candidate_world_x - pose.x,
                candidate_world_y - pose.y,
            )
            exact_cell = delta_x == 0 and delta_y == 0 and exact_valid
            if distance_m > snap_radius + 1.0e-6 and not exact_cell:
                continue
            snap_radius_indices.append(address)
            if mask[address]:
                candidates.append((distance_m, x, y))
    result["snap_square_terrain_layers"] = _terrain_layer_stats(
        grid,
        thresholds,
        snap_square_indices,
    )
    result["snap_radius_terrain_layers"] = _terrain_layer_stats(
        grid,
        thresholds,
        snap_radius_indices,
    )
    result["valid_cells_in_snap_square"] = sum(
        bool(mask[index]) for index in snap_square_indices
    )
    result["valid_cells_in_snap_radius"] = len(candidates)
    if not candidates:
        return result

    if exact_valid:
        snapped_x, snapped_y = grid_x, grid_y
    else:
        _, snapped_x, snapped_y = min(
            candidates,
            key=lambda candidate: candidate[0],
        )
    snapped_world_x = grid.origin_x + (snapped_x + 0.5) * grid.resolution
    snapped_world_y = grid.origin_y + (snapped_y + 0.5) * grid.resolution
    grid_distance = (
        math.hypot(snapped_x - grid_x, snapped_y - grid_y) * grid.resolution
    )
    world_to_center_distance = math.hypot(
        snapped_world_x - pose.x,
        snapped_world_y - pose.y,
    )
    result.update(
        {
            "snapped": True,
            "snapped_grid_x": snapped_x,
            "snapped_grid_y": snapped_y,
            "snapped_world_x": snapped_world_x,
            "snapped_world_y": snapped_world_y,
            "snap_grid_distance_m": grid_distance,
            "snap_world_to_center_distance_m": world_to_center_distance,
            "snap_grid_distance_outside_nominal_radius": (
                grid_distance > snap_radius + 1.0e-6
            ),
        }
    )
    return result


def _component_from(
    grid: GridSnapshot,
    mask: bytearray,
    start_x: int,
    start_y: int,
    thresholds: PlannerThresholds,
) -> tuple[bytearray, int]:
    visited = bytearray(grid.width * grid.height)
    start = _address(grid, start_x, start_y)
    if not mask[start]:
        return visited, 0
    visited[start] = 1
    queue: deque[int] = deque([start])
    count = 0
    while queue:
        current = queue.popleft()
        count += 1
        x = current % grid.width
        y = current // grid.width
        for delta_x, delta_y in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            next_x = x + delta_x
            next_y = y + delta_y
            if not _inside(grid, next_x, next_y):
                continue
            address = _address(grid, next_x, next_y)
            if not mask[address] or visited[address]:
                continue
            current_elevation = grid.elevation[current]
            next_elevation = grid.elevation[address]
            if (
                current_elevation == grid.unknown_value
                or next_elevation == grid.unknown_value
                or not math.isfinite(current_elevation)
                or not math.isfinite(next_elevation)
                or abs(next_elevation - current_elevation)
                > thresholds.max_step_height
            ):
                continue
            visited[address] = 1
            queue.append(address)
    return visited, count


def _input_age(stamp_ns: int, now_ns: int | None) -> float | None:
    if now_ns is None or stamp_ns <= 0:
        return None
    return (now_ns - stamp_ns) / 1_000_000_000.0


def _freshness_diagnosis(
    name: str,
    age: float | None,
    max_age: float,
    future_tolerance: float,
) -> str | None:
    if age is None:
        return f"{name}_stamp_missing"
    if age < -future_tolerance:
        return f"{name}_stamp_from_future"
    if age > max_age:
        return f"{name}_stale"
    return None


def analyze_planner_inputs(
    grid: GridSnapshot,
    start: Pose2D,
    goal: Pose2D | None,
    thresholds: PlannerThresholds = PlannerThresholds(),
    contract: InputContract = InputContract(),
    now_ns: int | None = None,
) -> dict[str, object]:
    """Return input-contract, endpoint, and continuous-ground diagnostics."""
    _validate_grid(grid)
    _validate_thresholds(thresholds)
    _validate_pose("start", start)
    if goal is not None:
        _validate_pose("goal", goal)
    planner_mask, known_cells, valid_cells = _valid_mask(grid, thresholds)
    ground_mask, ground_cells = _continuous_ground_mask(grid, planner_mask)
    total_cells = grid.width * grid.height
    start_snap_radius = (
        thresholds.snap_radius
        if thresholds.start_snap_radius is None
        else thresholds.start_snap_radius
    )
    start_result = _snap_endpoint(
        grid,
        planner_mask,
        start,
        thresholds,
        start_snap_radius,
    )
    goal_result = (
        _snap_endpoint(
            grid,
            planner_mask,
            goal,
            thresholds,
            thresholds.snap_radius,
        )
        if goal
        else None
    )
    map_age = _input_age(grid.stamp_ns, now_ns)
    odom_age = _input_age(start.stamp_ns, now_ns)
    goal_age = _input_age(goal.stamp_ns, now_ns) if goal else None

    result: dict[str, object] = {
        "map": {
            "frame_id": grid.frame_id,
            "stamp_ns": grid.stamp_ns,
            "resolution": grid.resolution,
            "width": grid.width,
            "height": grid.height,
            "origin_x": grid.origin_x,
            "origin_y": grid.origin_y,
            "total_cells": total_cells,
            "known_cells": known_cells,
            "planner_gate_known_cells": known_cells,
            "valid_cells": valid_cells,
            "continuous_ground_cells": ground_cells,
            "known_percent": 100.0 * known_cells / total_cells,
            "valid_percent": 100.0 * valid_cells / total_cells,
            "continuous_ground_percent": 100.0 * ground_cells / total_cells,
            "terrain_layers": _terrain_layer_stats(grid, thresholds),
        },
        "thresholds": {
            **asdict(thresholds),
            "start_snap_radius": start_snap_radius,
        },
        "contract": asdict(contract),
        "freshness": {
            "evaluated": now_ns is not None,
            "map_age_sec": map_age,
            "odom_age_sec": odom_age,
            "goal_age_sec": goal_age,
            "map_odom_stamp_delta_sec": (
                (grid.stamp_ns - start.stamp_ns) / 1_000_000_000.0
                if grid.stamp_ns > 0 and start.stamp_ns > 0
                else None
            ),
        },
        "start": start_result,
        "goal": goal_result,
        "start_component_cells": 0,
        "start_component_percent_of_ground": 0.0,
        "goal_in_start_component": None,
        "diagnosis": "",
        "limitations": (
            "Continuous-ground connectivity uses edge-adjacent planner-valid cells "
            "with finite elevation and step-height limits. It is not LatticePlanner "
            "reachability and does not validate motion primitives, footprint, or "
            "swept collision"
        ),
    }

    if grid.frame_id != contract.map_frame:
        result["diagnosis"] = "map_frame_mismatch"
        return result
    if start.frame_id != grid.frame_id:
        result["diagnosis"] = "start_frame_mismatch"
        return result
    if start.child_frame_id != contract.body_frame:
        result["diagnosis"] = "start_child_frame_mismatch"
        return result
    if now_ns is not None:
        freshness_error = _freshness_diagnosis(
            "map", map_age, contract.max_map_age, contract.future_tolerance
        ) or _freshness_diagnosis(
            "odom", odom_age, contract.max_odom_age, contract.future_tolerance
        )
        if freshness_error:
            result["diagnosis"] = freshness_error
            return result
    if not start_result["inside_map"]:
        result["diagnosis"] = "start_outside_map"
        return result
    if not start_result["snapped"]:
        result["diagnosis"] = "start_has_no_valid_cell_in_snap_radius"
        return result

    start_x = int(start_result["snapped_grid_x"])
    start_y = int(start_result["snapped_grid_y"])
    if not ground_mask[_address(grid, start_x, start_y)]:
        result["diagnosis"] = "start_elevation_invalid_for_ground_topology"
        return result

    component, component_cells = _component_from(
        grid, ground_mask, start_x, start_y, thresholds
    )
    result["start_component_cells"] = component_cells
    result["start_component_percent_of_ground"] = (
        100.0 * component_cells / ground_cells if ground_cells else 0.0
    )

    if goal is None:
        result["diagnosis"] = "start_ready_waiting_for_goal"
        return result
    if goal.frame_id != grid.frame_id:
        result["diagnosis"] = "goal_frame_mismatch"
        return result
    if now_ns is not None:
        freshness_error = _freshness_diagnosis(
            "goal", goal_age, contract.max_goal_age, contract.future_tolerance
        )
        if freshness_error:
            result["diagnosis"] = freshness_error
            return result
    assert goal_result is not None
    if not goal_result["inside_map"]:
        result["diagnosis"] = "goal_outside_map"
        return result
    if not goal_result["snapped"]:
        result["diagnosis"] = "goal_has_no_valid_cell_in_snap_radius"
        return result

    goal_address = _address(
        grid,
        int(goal_result["snapped_grid_x"]),
        int(goal_result["snapped_grid_y"]),
    )
    if not ground_mask[goal_address]:
        result["diagnosis"] = "goal_elevation_invalid_for_ground_topology"
        return result
    same_component = bool(component[goal_address])
    result["goal_in_start_component"] = same_component
    result["diagnosis"] = (
        "same_continuous_ground_component_not_planner_approval"
        if same_component
        else "start_and_goal_continuous_ground_disconnected"
    )
    return result


def _quaternion_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _stamp_ns(header: object) -> int:
    stamp = getattr(header, "stamp")
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


WaitFunction = Callable[[object, Callable[[], bool], float], bool]


def _wait_for(node: object, predicate: Callable[[], bool], timeout: float) -> bool:
    import rclpy

    deadline = time.monotonic() + timeout
    while not predicate() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=min(0.2, max(0.0, deadline - time.monotonic())))
    return bool(predicate())


def _capture_stage(
    node: object,
    specs: Sequence[SubscriptionSpec],
    timeout: float,
    wait_fn: WaitFunction = _wait_for,
    accept: Mapping[str, Callable[[object], bool]] | None = None,
) -> dict[str, object] | None:
    """Capture one stage and always unregister every subscription it creates."""
    received: dict[str, object] = {}
    subscriptions: list[object] = []
    accept = accept or {}
    try:
        for spec in specs:
            def callback(message: object, current: SubscriptionSpec = spec) -> None:
                predicate = accept.get(current.key)
                if predicate is None or predicate(message):
                    received[current.key] = message

            subscriptions.append(
                node.create_subscription(
                    spec.message_type,
                    spec.topic,
                    callback,
                    spec.qos,
                )
            )
        def complete() -> bool:
            return all(spec.key in received for spec in specs)

        return received if wait_fn(node, complete, timeout) else None
    finally:
        for subscription in subscriptions:
            node.destroy_subscription(subscription)


def _capture_goal_stage(
    node: object,
    spec: SubscriptionSpec,
    publisher_timeout: float,
    goal_timeout: float,
    wait_fn: WaitFunction,
) -> tuple[dict[str, object] | None, str | None]:
    received: dict[str, object] = {}

    def callback(message: object) -> None:
        received[spec.key] = message

    subscription = node.create_subscription(
        spec.message_type,
        spec.topic,
        callback,
        spec.qos,
    )
    try:
        publisher_ready = lambda: node.count_publishers(spec.topic) > 0
        if not publisher_ready() and not wait_fn(
            node,
            publisher_ready,
            publisher_timeout,
        ):
            return None, "goal_publisher_discovery_timeout"
        publisher_count = node.count_publishers(spec.topic)
        print(
            f"Goal listener ready with {publisher_count} publisher(s). "
            f"Set one RViz goal within {goal_timeout:.0f} seconds...",
            file=sys.stderr,
            flush=True,
        )
        if not wait_fn(node, lambda: spec.key in received, goal_timeout):
            return None, "goal_wait_timeout"
        return received, None
    finally:
        node.destroy_subscription(subscription)


def _capture_runtime_messages(
    node: object,
    specs: Mapping[str, SubscriptionSpec],
    wait_for_goal: bool,
    input_timeout: float,
    goal_timeout: float,
    goal_discovery_timeout: float = 10.0,
    record_start_on_goal_timeout: bool = False,
    wait_fn: WaitFunction = _wait_for,
) -> dict[str, object]:
    """Capture live inputs without retaining TerrainGrid while waiting for a goal."""
    if not wait_for_goal:
        print(
            f"Waiting for {specs['live_map'].topic} and {specs['odom'].topic}...",
            file=sys.stderr,
            flush=True,
        )
        captured = _capture_stage(
            node,
            (specs["live_map"], specs["odom"]),
            input_timeout,
            wait_fn,
        )
        if captured is None:
            raise RuntimeError("timed out waiting for terrain map and body odometry")
        return captured

    print(
        f"Waiting for {specs['latched_map'].topic} and {specs['odom'].topic}...",
        file=sys.stderr,
        flush=True,
    )
    initial = _capture_stage(
        node,
        (specs["latched_map"], specs["odom"]),
        input_timeout,
        wait_fn,
    )
    if initial is None:
        raise RuntimeError("timed out waiting for terrain map and body odometry")

    initial_map_stamp = _stamp_ns(getattr(initial["map"], "header"))
    initial_odom_stamp = _stamp_ns(getattr(initial["odom"], "header"))
    del initial
    goal, goal_capture_failure = _capture_goal_stage(
        node,
        specs["goal"],
        goal_discovery_timeout,
        goal_timeout,
        wait_fn,
    )
    if goal is None:
        if not record_start_on_goal_timeout:
            if goal_capture_failure == "goal_publisher_discovery_timeout":
                raise RuntimeError("timed out waiting for a /goal_pose publisher")
            raise RuntimeError("timed out waiting for a new goal message")
        print(
            "No goal captured. Recording fresh map and start diagnostics...",
            file=sys.stderr,
            flush=True,
        )
        live_inputs = _capture_stage(
            node,
            (specs["live_map"], specs["odom"]),
            input_timeout,
            wait_fn,
            accept={
                "map": lambda message: _stamp_ns(getattr(message, "header"))
                > initial_map_stamp,
                "odom": lambda message: _stamp_ns(getattr(message, "header"))
                > initial_odom_stamp,
            },
        )
        if live_inputs is None:
            raise RuntimeError(
                "timed out waiting for fresh terrain map and body odometry "
                "after goal timeout"
            )
        live_inputs["goal_capture_failure"] = goal_capture_failure
        return live_inputs

    print(
        "Goal captured. Waiting for independently newer terrain and odometry "
        "messages...",
        file=sys.stderr,
        flush=True,
    )
    live_inputs = _capture_stage(
        node,
        (specs["live_map"], specs["odom"]),
        input_timeout,
        wait_fn,
        accept={
            "map": lambda message: _stamp_ns(getattr(message, "header"))
            > initial_map_stamp,
            "odom": lambda message: _stamp_ns(getattr(message, "header"))
            > initial_odom_stamp,
        },
    )
    if live_inputs is None:
        raise RuntimeError(
            "timed out waiting for terrain map and body odometry newer than the "
            "initial samples"
        )
    return {**live_inputs, **goal}


def _grid_snapshot(message: object) -> GridSnapshot:
    header = getattr(message, "header")
    return GridSnapshot(
        frame_id=header.frame_id,
        stamp_ns=_stamp_ns(header),
        resolution=float(getattr(message, "resolution")),
        width=int(getattr(message, "width")),
        height=int(getattr(message, "height")),
        origin_x=float(getattr(message, "origin_x")),
        origin_y=float(getattr(message, "origin_y")),
        unknown_value=float(getattr(message, "unknown_value")),
        elevation=getattr(message, "elevation"),
        slope=getattr(message, "slope"),
        roughness=getattr(message, "roughness"),
        traversability=getattr(message, "traversability"),
        observation_count=getattr(message, "observation_count"),
    )


def _odom_pose(message: object) -> Pose2D:
    header = getattr(message, "header")
    pose = getattr(getattr(message, "pose"), "pose")
    orientation = pose.orientation
    return Pose2D(
        x=float(pose.position.x),
        y=float(pose.position.y),
        yaw=_quaternion_yaw(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        ),
        frame_id=header.frame_id,
        stamp_ns=_stamp_ns(header),
        child_frame_id=getattr(message, "child_frame_id"),
    )


def _goal_pose(message: object) -> Pose2D:
    header = getattr(message, "header")
    pose = getattr(message, "pose")
    orientation = pose.orientation
    return Pose2D(
        x=float(pose.position.x),
        y=float(pose.position.y),
        yaw=_quaternion_yaw(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        ),
        frame_id=header.frame_id,
        stamp_ns=_stamp_ns(header),
    )


def collect_ros_inputs(
    args: argparse.Namespace,
) -> tuple[
    GridSnapshot,
    Pose2D,
    Pose2D | None,
    int,
    str | None,
    PlannerThresholds | None,
    PlannerFreshnessSettings | None,
]:
    try:
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Odometry
        from rcl_interfaces.msg import ParameterType
        from rcl_interfaces.srv import GetParameters
        from rclpy.node import Node
        from rclpy.qos import (
            DurabilityPolicy,
            QoSProfile,
            ReliabilityPolicy,
            qos_profile_sensor_data,
        )
        from utree_dog_msgs.msg import TerrainGrid
    except ImportError as error:
        raise RuntimeError(
            "ROS 2 Python messages are unavailable; source /opt/ros/humble/setup.bash "
            "and install/setup.bash first"
        ) from error

    rclpy.init()
    node = None
    latched_map_qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    live_map_qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )
    goal_qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )
    try:
        node = Node("inspect_planner_inputs")
        live_thresholds = None
        live_freshness = None
        if args.read_live_parameters:
            print(
                "Reading live terrain mapper and planner parameters...",
                file=sys.stderr,
                flush=True,
            )
            live_thresholds, live_freshness = _read_live_planner_thresholds(
                node,
                GetParameters,
                ParameterType,
                rclpy.spin_until_future_complete,
                service_timeout=PARAMETER_SERVICE_TIMEOUT_SEC,
                response_timeout=PARAMETER_RESPONSE_TIMEOUT_SEC,
                max_attempts=PARAMETER_READ_ATTEMPTS,
            )
        specs = {
            "latched_map": SubscriptionSpec(
                "map", TerrainGrid, args.map_topic, latched_map_qos
            ),
            "live_map": SubscriptionSpec(
                "map", TerrainGrid, args.map_topic, live_map_qos
            ),
            "odom": SubscriptionSpec(
                "odom", Odometry, args.odom_topic, qos_profile_sensor_data
            ),
            "goal": SubscriptionSpec(
                "goal", PoseStamped, args.goal_topic, goal_qos
            ),
        }
        received = _capture_runtime_messages(
            node,
            specs,
            wait_for_goal=not args.no_goal and args.goal_x is None,
            input_timeout=args.input_timeout,
            goal_timeout=args.goal_timeout,
            goal_discovery_timeout=args.goal_discovery_timeout,
            record_start_on_goal_timeout=args.record_start_on_goal_timeout,
        )
        grid = _grid_snapshot(received["map"])
        start = _odom_pose(received["odom"])
        now_ns = int(node.get_clock().now().nanoseconds)

        goal: Pose2D | None = None
        if args.goal_x is not None:
            goal = Pose2D(
                x=args.goal_x,
                y=args.goal_y,
                yaw=args.goal_yaw,
                frame_id=args.goal_frame or grid.frame_id,
                stamp_ns=now_ns,
            )
        elif "goal" in received:
            goal = _goal_pose(received["goal"])
        return (
            grid,
            start,
            goal,
            now_ns,
            received.get("goal_capture_failure"),
            live_thresholds,
            live_freshness,
        )
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _format_terrain_layers(
    name: str,
    stats: dict[str, int],
    indent: str = "",
) -> list[str]:
    hard_reject = stats[
        "zero_traversability_with_slope_and_roughness_below_limits"
    ]
    return [
        f"{indent}{name}: cells={stats['cell_count']} observations="
        f"zero:{stats['observation_zero']} "
        f"below_min:{stats['observation_below_min']} "
        f"ready:{stats['observation_ready']}",
        f"{indent}  elevation known:{stats['elevation_known']} "
        f"unknown/nonfinite:{stats['elevation_unknown_or_nonfinite']} "
        f"known_below_min_observations:"
        f"{stats['elevation_known_below_min_observations']}",
        f"{indent}  features slope_known:{stats['slope_known']} "
        f"roughness_known:{stats['roughness_known']} "
        f"traversability_known:{stats['traversability_known']} "
        f"all_known:{stats['features_known']}",
        f"{indent}  gates slope_over:{stats['slope_over_limit']} "
        f"mapper_slope_at_or_above:"
        f"{stats['slope_at_or_above_mapper_limit']} "
        f"roughness_over:{stats['roughness_over_limit']} "
        f"traversability_zero:{stats['traversability_zero']} "
        f"traversability_low:{stats['traversability_below_min_nonzero']} "
        f"traversability_accepted:"
        f"{stats['traversability_at_or_above_min']} "
        f"hard_reject_candidate:{hard_reject} "
        f"planner_valid:{stats['planner_valid']}",
    ]


def _format_endpoint(
    name: str,
    endpoint: dict[str, object] | None,
) -> list[str]:
    if endpoint is None:
        return [f"{name}: not captured"]
    lines = [
        f"{name}: frame={endpoint['frame_id']} child={endpoint['child_frame_id'] or '-'} "
        f"position=({endpoint['world_x']:.3f}, {endpoint['world_y']:.3f}) "
        f"grid=({endpoint['grid_x']}, {endpoint['grid_y']}) "
        f"inside={endpoint['inside_map']}",
        f"  exact_valid={endpoint['exact_cell_valid']} "
        f"valid_in_snap_square={endpoint['valid_cells_in_snap_square']} "
        f"valid_in_snap_radius={endpoint['valid_cells_in_snap_radius']} "
        f"snapped={endpoint['snapped']} "
        f"grid_distance_m={endpoint['snap_grid_distance_m']} "
        f"world_to_center_m={endpoint['snap_world_to_center_distance_m']}",
    ]
    local_stats = endpoint["snap_radius_terrain_layers"]
    if isinstance(local_stats, dict):
        lines.extend(
            _format_terrain_layers("snap_radius_layers", local_stats, "  ")
        )
    return lines


def _format_age(value: object) -> str:
    return "not available" if value is None else f"{float(value):.3f} s"


def print_human(result: dict[str, object]) -> None:
    map_result = result["map"]
    assert isinstance(map_result, dict)
    print("Planner input inspection")
    print(
        f"map: frame={map_result['frame_id']} size={map_result['width']}x"
        f"{map_result['height']} resolution={map_result['resolution']:.3f} m"
    )
    print(
        "cells: planner_gate_known="
        f"{map_result['planner_gate_known_cells']} "
        f"({map_result['known_percent']:.3f}%) "
        f"planner_valid={map_result['valid_cells']} "
        f"({map_result['valid_percent']:.3f}%) ground="
        f"{map_result['continuous_ground_cells']} "
        f"({map_result['continuous_ground_percent']:.3f}%)"
    )
    thresholds = result["thresholds"]
    assert isinstance(thresholds, dict)
    print(
        "thresholds: "
        f"min_observed_frames={thresholds['min_observed_frames']} "
        f"min_traversability={thresholds['min_traversability']:.3f} "
        f"planner_max_slope={thresholds['max_slope']:.3f} "
        f"mapper_max_slope={thresholds['mapper_max_slope']:.3f} "
        f"max_roughness={thresholds['max_roughness']:.3f}"
    )
    terrain_layers = map_result["terrain_layers"]
    assert isinstance(terrain_layers, dict)
    for line in _format_terrain_layers("all_map_layers", terrain_layers):
        print(line)
    freshness = result["freshness"]
    assert isinstance(freshness, dict)
    print(
        "freshness: "
        f"map_age={_format_age(freshness['map_age_sec'])} "
        f"odom_age={_format_age(freshness['odom_age_sec'])} "
        f"goal_age={_format_age(freshness['goal_age_sec'])}"
    )
    print(
        "timestamps: "
        f"map_minus_odom={_format_age(freshness['map_odom_stamp_delta_sec'])}"
    )
    for line in _format_endpoint("start", result["start"]):
        print(line)
    for line in _format_endpoint("goal", result["goal"]):
        print(line)
    print(
        f"start_component_cells={result['start_component_cells']} "
        f"({result['start_component_percent_of_ground']:.3f}% of ground)"
    )
    print(f"goal_in_start_component={result['goal_in_start_component']}")
    print(f"diagnosis={result['diagnosis']}")
    print(f"NOTE: {result['limitations']}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map-topic", default="/terrain_map")
    parser.add_argument("--odom-topic", default="/lio/body_odom")
    parser.add_argument("--goal-topic", default="/goal_pose")
    parser.add_argument("--input-timeout", type=float, default=10.0)
    parser.add_argument("--goal-timeout", type=float, default=60.0)
    parser.add_argument("--goal-discovery-timeout", type=float, default=10.0)
    parser.add_argument(
        "--record-start-on-goal-timeout",
        action="store_true",
        help=argparse.SUPPRESS,
    )
    parser.add_argument(
        "--read-live-parameters",
        action="store_true",
        help=argparse.SUPPRESS,
    )
    parser.add_argument(
        "--no-goal",
        action="store_true",
        help="inspect only map and start",
    )
    parser.add_argument("--goal-x", type=float)
    parser.add_argument("--goal-y", type=float)
    parser.add_argument("--goal-yaw", type=float, default=0.0)
    parser.add_argument("--goal-frame", default="")
    parser.add_argument("--min-traversability", type=float, default=0.18)
    parser.add_argument("--max-slope", type=float, default=0.65)
    parser.add_argument("--mapper-max-slope", type=float, default=0.65)
    parser.add_argument("--max-roughness", type=float, default=0.08)
    parser.add_argument("--max-step-height", type=float, default=0.24)
    parser.add_argument("--start-snap-radius", type=float)
    parser.add_argument("--snap-radius", type=float, default=0.5)
    parser.add_argument("--min-observed-frames", type=int, default=4)
    parser.add_argument("--expected-map-frame", default="world")
    parser.add_argument("--expected-body-frame", default="base_link")
    parser.add_argument("--max-map-age", type=float, default=1.0)
    parser.add_argument("--max-odom-age", type=float, default=0.5)
    parser.add_argument("--max-goal-age", type=float, default=2.0)
    parser.add_argument("--future-tolerance", type=float, default=0.2)
    parser.add_argument(
        "--json",
        action="store_true",
        help="emit machine-readable JSON",
    )
    args = parser.parse_args()
    if (args.goal_x is None) != (args.goal_y is None):
        parser.error("--goal-x and --goal-y must be provided together")
    if args.no_goal and args.goal_x is not None:
        parser.error(
            "--no-goal cannot be combined with manual goal coordinates"
        )
    for name in (
        "input_timeout",
        "goal_timeout",
        "goal_discovery_timeout",
        "max_slope",
        "mapper_max_slope",
        "max_roughness",
        "max_step_height",
        "start_snap_radius",
        "snap_radius",
        "max_map_age",
        "max_odom_age",
        "max_goal_age",
        "future_tolerance",
    ):
        value = getattr(args, name)
        if value is None:
            continue
        if not math.isfinite(value) or value < 0.0:
            option = name.replace("_", "-")
            parser.error(f"--{option} must be finite and non-negative")
    if args.min_observed_frames < 1:
        parser.error("--min-observed-frames must be positive")
    for name in (
        "goal_x",
        "goal_y",
        "goal_yaw",
        "min_traversability",
    ):
        if getattr(args, name) is None:
            continue
        if not math.isfinite(getattr(args, name)):
            parser.error(f"--{name.replace('_', '-')} must be finite")
    if not args.expected_map_frame or not args.expected_body_frame:
        parser.error("expected frame names must not be empty")
    return args


def _diagnosis_exit_code(diagnosis: object) -> int:
    return (
        0
        if diagnosis
        in {
            "start_ready_waiting_for_goal",
            "same_continuous_ground_component_not_planner_approval",
        }
        else 2
    )


def _mark_goal_capture_failure(
    result: dict[str, object], diagnosis: str
) -> None:
    result["start_map_diagnosis"] = result["diagnosis"]
    result["diagnosis"] = diagnosis
    result["goal_capture_failure"] = diagnosis


def _write_session_record_atomic(
    path: Path,
    temporary_directory: Path,
    exit_code: int,
    inspection: dict[str, object] | None = None,
    error: dict[str, str] | None = None,
    recorded_at: str | None = None,
) -> None:
    """Atomically add one bounded record to a collector-owned JSONL file."""
    if (inspection is None) == (error is None):
        raise ValueError("record must contain exactly one inspection or error")
    if inspection is not None and exit_code not in {0, 2}:
        raise ValueError("inspection exit code must be 0 or 2")
    if error is not None and exit_code != 1:
        raise ValueError("capture error exit code must be 1")
    status = (
        "capture_error"
        if error is not None
        else "ok"
        if exit_code == 0
        else "diagnostic_failure"
    )
    record: dict[str, object] = {
        "schema_version": 1,
        "recorded_at": recorded_at
        or dt.datetime.now(dt.timezone.utc).isoformat(),
        "exit_code": exit_code,
        "status": status,
    }
    if inspection is not None:
        record["inspection"] = inspection
    else:
        record["error"] = error
    payload = (
        json.dumps(
            record,
            ensure_ascii=True,
            allow_nan=False,
            separators=(",", ":"),
        )
        + "\n"
    ).encode("ascii")
    if len(payload) > SESSION_RECORD_MAX_BYTES:
        raise ValueError("planner inspection record exceeds 1 MiB")

    existing = b""
    if path.exists() or path.is_symlink():
        flags = os.O_RDONLY | os.O_CLOEXEC | getattr(os, "O_NOFOLLOW", 0)
        descriptor = os.open(path, flags)
        try:
            file_status = os.fstat(descriptor)
            if not stat.S_ISREG(file_status.st_mode):
                raise OSError("session log is not a regular file")
            if file_status.st_size > SESSION_LOG_MAX_BYTES:
                raise OSError("planner inspection session log exceeds 8 MiB")
            chunks: list[bytes] = []
            while True:
                chunk = os.read(descriptor, 1024 * 1024)
                if not chunk:
                    break
                chunks.append(chunk)
            existing = b"".join(chunks)
        finally:
            os.close(descriptor)

    if b"\0" in existing:
        raise ValueError("existing planner inspection log contains NUL bytes")
    try:
        existing_text = existing.decode("utf-8")
    except UnicodeDecodeError as error_decode:
        raise ValueError(
            "existing planner inspection log is not valid UTF-8"
        ) from error_decode
    if existing and not existing.endswith(b"\n"):
        raise ValueError("existing planner inspection log has a partial final line")

    def reject_constant(value: str) -> None:
        raise ValueError(f"invalid JSON numeric constant: {value}")

    for line in existing_text.splitlines():
        if not line:
            raise ValueError("existing planner inspection log contains a blank line")
        try:
            previous = json.loads(line, parse_constant=reject_constant)
        except json.JSONDecodeError as error_decode:
            raise ValueError(
                "existing planner inspection log contains malformed JSON"
            ) from error_decode
        if not isinstance(previous, dict):
            raise ValueError("existing planner inspection record is not an object")

    if len(existing) + len(payload) > SESSION_LOG_MAX_BYTES:
        raise OSError("planner inspection session log exceeds 8 MiB")
    if not temporary_directory.is_dir():
        raise OSError("planner inspection temporary directory does not exist")

    temporary_descriptor, temporary_name = tempfile.mkstemp(
        prefix=".planner-input-",
        dir=temporary_directory,
    )
    try:
        os.fchmod(temporary_descriptor, 0o640)
        remaining = memoryview(existing + payload)
        while remaining:
            written = os.write(temporary_descriptor, remaining)
            if written <= 0:
                raise OSError("short write while recording planner inspection")
            remaining = remaining[written:]
        os.fsync(temporary_descriptor)
        os.close(temporary_descriptor)
        temporary_descriptor = -1
        os.replace(temporary_name, path)
        directory_flags = os.O_RDONLY | getattr(os, "O_DIRECTORY", 0)
        directory_descriptor = os.open(path.parent, directory_flags)
        try:
            os.fsync(directory_descriptor)
        finally:
            os.close(directory_descriptor)
    finally:
        if temporary_descriptor >= 0:
            os.close(temporary_descriptor)
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass


def main() -> int:
    args = parse_args()
    try:
        (
            grid,
            start,
            goal,
            now_ns,
            goal_capture_failure,
            live_thresholds,
            live_freshness,
        ) = collect_ros_inputs(args)
        thresholds = live_thresholds or PlannerThresholds(
            min_traversability=args.min_traversability,
            max_slope=args.max_slope,
            mapper_max_slope=args.mapper_max_slope,
            max_roughness=args.max_roughness,
            max_step_height=args.max_step_height,
            start_snap_radius=args.start_snap_radius,
            snap_radius=args.snap_radius,
            min_observed_frames=args.min_observed_frames,
        )
        result = analyze_planner_inputs(
            grid,
            start,
            goal,
            thresholds,
            InputContract(
                map_frame=args.expected_map_frame,
                body_frame=args.expected_body_frame,
                max_map_age=(
                    live_freshness.max_map_age
                    if live_freshness is not None
                    else args.max_map_age
                ),
                max_odom_age=(
                    live_freshness.max_odom_age
                    if live_freshness is not None
                    else args.max_odom_age
                ),
                max_goal_age=args.max_goal_age,
                future_tolerance=(
                    live_freshness.future_tolerance
                    if live_freshness is not None
                    else args.future_tolerance
                ),
            ),
            now_ns=now_ns,
        )
        if live_freshness is not None:
            result["runtime_parameters"] = {
                "input_watchdog_rate": live_freshness.input_watchdog_rate,
            }
        if goal_capture_failure is not None:
            _mark_goal_capture_failure(result, goal_capture_failure)
    except (RuntimeError, ValueError) as error:
        print(f"inspect_planner_inputs: {error}", file=sys.stderr)
        return 1
    exit_code = _diagnosis_exit_code(result["diagnosis"])
    if args.json:
        print(json.dumps(result, indent=2, sort_keys=True))
    else:
        print_human(result)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
