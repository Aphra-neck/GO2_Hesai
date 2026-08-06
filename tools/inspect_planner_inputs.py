#!/usr/bin/env python3
"""Inspect planner map, odometry, and goal inputs without publishing commands."""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from collections import deque
from dataclasses import asdict, dataclass
from typing import Callable, Mapping, Sequence


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
    traversability: Sequence[float]


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
    max_step_height: float = 0.24
    snap_radius: float = 0.5


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
        ("traversability", grid.traversability),
    ):
        if len(values) != expected:
            raise ValueError(
                f"map {name} length {len(values)} does not match {expected} cells"
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
        if traversability == grid.unknown_value or slope == grid.unknown_value:
            continue
        known += 1
        if (
            math.isfinite(traversability)
            and math.isfinite(slope)
            and traversability >= thresholds.min_traversability
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
) -> dict[str, object]:
    grid_x, grid_y = _world_to_grid(grid, pose.x, pose.y)
    result: dict[str, object] = {
        "frame_id": pose.frame_id,
        "child_frame_id": pose.child_frame_id,
        "stamp_ns": pose.stamp_ns,
        "world_x": pose.x,
        "world_y": pose.y,
        "yaw_rad": pose.yaw,
        "grid_x": grid_x,
        "grid_y": grid_y,
        "inside_map": _inside(grid, grid_x, grid_y),
        "exact_cell_valid": False,
        "valid_cells_in_snap_square": 0,
        "snapped": False,
        "snapped_grid_x": None,
        "snapped_grid_y": None,
        "snapped_world_x": None,
        "snapped_world_y": None,
        "snap_grid_distance_m": None,
        "snap_world_to_center_distance_m": None,
        "snap_grid_distance_outside_nominal_radius": False,
    }
    if not result["inside_map"]:
        return result

    exact_valid = bool(mask[_address(grid, grid_x, grid_y)])
    result["exact_cell_valid"] = exact_valid
    radius = max(1, math.ceil(thresholds.snap_radius / grid.resolution))
    candidates: list[tuple[int, int, int]] = []
    for delta_y in range(-radius, radius + 1):
        for delta_x in range(-radius, radius + 1):
            x = grid_x + delta_x
            y = grid_y + delta_y
            if _inside(grid, x, y) and mask[_address(grid, x, y)]:
                candidates.append((delta_x * delta_x + delta_y * delta_y, x, y))
    result["valid_cells_in_snap_square"] = len(candidates)
    if not candidates:
        return result

    _, snapped_x, snapped_y = min(candidates, key=lambda candidate: candidate[0])
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
                grid_distance > thresholds.snap_radius + 1.0e-9
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
    _validate_pose("start", start)
    if goal is not None:
        _validate_pose("goal", goal)
    planner_mask, known_cells, valid_cells = _valid_mask(grid, thresholds)
    ground_mask, ground_cells = _continuous_ground_mask(grid, planner_mask)
    total_cells = grid.width * grid.height
    start_result = _snap_endpoint(grid, planner_mask, start, thresholds)
    goal_result = (
        _snap_endpoint(grid, planner_mask, goal, thresholds) if goal else None
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
            "valid_cells": valid_cells,
            "continuous_ground_cells": ground_cells,
            "known_percent": 100.0 * known_cells / total_cells,
            "valid_percent": 100.0 * valid_cells / total_cells,
            "continuous_ground_percent": 100.0 * ground_cells / total_cells,
        },
        "thresholds": asdict(thresholds),
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
        result["diagnosis"] = "start_has_no_valid_cell_in_snap_square"
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
        result["diagnosis"] = "goal_has_no_valid_cell_in_snap_square"
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


def _capture_runtime_messages(
    node: object,
    specs: Mapping[str, SubscriptionSpec],
    wait_for_goal: bool,
    input_timeout: float,
    goal_timeout: float,
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
    print(
        "Initial samples captured but not yet validated. "
        f"Set one RViz goal within {goal_timeout:.0f} seconds...",
        file=sys.stderr,
        flush=True,
    )
    goal = _capture_stage(node, (specs["goal"],), goal_timeout, wait_fn)
    if goal is None:
        raise RuntimeError("timed out waiting for a new goal message")

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
        traversability=getattr(message, "traversability"),
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
) -> tuple[GridSnapshot, Pose2D, Pose2D | None, int]:
    try:
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Odometry
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
        return grid, start, goal, now_ns
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _format_endpoint(name: str, endpoint: dict[str, object] | None) -> list[str]:
    if endpoint is None:
        return [f"{name}: not captured"]
    return [
        f"{name}: frame={endpoint['frame_id']} child={endpoint['child_frame_id'] or '-'} "
        f"position=({endpoint['world_x']:.3f}, {endpoint['world_y']:.3f}) "
        f"grid=({endpoint['grid_x']}, {endpoint['grid_y']}) "
        f"inside={endpoint['inside_map']}",
        f"  exact_valid={endpoint['exact_cell_valid']} "
        f"valid_in_snap_square={endpoint['valid_cells_in_snap_square']} "
        f"snapped={endpoint['snapped']} "
        f"grid_distance_m={endpoint['snap_grid_distance_m']} "
        f"world_to_center_m={endpoint['snap_world_to_center_distance_m']}",
    ]


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
        f"cells: known={map_result['known_cells']} ({map_result['known_percent']:.3f}%) "
        f"planner_valid={map_result['valid_cells']} "
        f"({map_result['valid_percent']:.3f}%) ground="
        f"{map_result['continuous_ground_cells']} "
        f"({map_result['continuous_ground_percent']:.3f}%)"
    )
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
    parser.add_argument("--no-goal", action="store_true", help="inspect only map and start")
    parser.add_argument("--goal-x", type=float)
    parser.add_argument("--goal-y", type=float)
    parser.add_argument("--goal-yaw", type=float, default=0.0)
    parser.add_argument("--goal-frame", default="")
    parser.add_argument("--min-traversability", type=float, default=0.18)
    parser.add_argument("--max-slope", type=float, default=0.65)
    parser.add_argument("--max-step-height", type=float, default=0.24)
    parser.add_argument("--snap-radius", type=float, default=0.5)
    parser.add_argument("--expected-map-frame", default="world")
    parser.add_argument("--expected-body-frame", default="base_link")
    parser.add_argument("--max-map-age", type=float, default=1.0)
    parser.add_argument("--max-odom-age", type=float, default=0.5)
    parser.add_argument("--max-goal-age", type=float, default=2.0)
    parser.add_argument("--future-tolerance", type=float, default=0.2)
    parser.add_argument("--json", action="store_true", help="emit machine-readable JSON")
    args = parser.parse_args()
    if (args.goal_x is None) != (args.goal_y is None):
        parser.error("--goal-x and --goal-y must be provided together")
    if args.no_goal and args.goal_x is not None:
        parser.error("--no-goal cannot be combined with manual goal coordinates")
    for name in (
        "input_timeout",
        "goal_timeout",
        "max_slope",
        "max_step_height",
        "snap_radius",
        "max_map_age",
        "max_odom_age",
        "max_goal_age",
        "future_tolerance",
    ):
        if not math.isfinite(getattr(args, name)) or getattr(args, name) < 0.0:
            parser.error(f"--{name.replace('_', '-')} must be finite and non-negative")
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


def main() -> int:
    args = parse_args()
    try:
        grid, start, goal, now_ns = collect_ros_inputs(args)
        result = analyze_planner_inputs(
            grid,
            start,
            goal,
            PlannerThresholds(
                min_traversability=args.min_traversability,
                max_slope=args.max_slope,
                max_step_height=args.max_step_height,
                snap_radius=args.snap_radius,
            ),
            InputContract(
                map_frame=args.expected_map_frame,
                body_frame=args.expected_body_frame,
                max_map_age=args.max_map_age,
                max_odom_age=args.max_odom_age,
                max_goal_age=args.max_goal_age,
                future_tolerance=args.future_tolerance,
            ),
            now_ns=now_ns,
        )
    except (RuntimeError, ValueError) as error:
        print(f"inspect_planner_inputs: {error}", file=sys.stderr)
        return 1
    if args.json:
        print(json.dumps(result, indent=2, sort_keys=True))
    else:
        print_human(result)
    return _diagnosis_exit_code(result["diagnosis"])


if __name__ == "__main__":
    raise SystemExit(main())
