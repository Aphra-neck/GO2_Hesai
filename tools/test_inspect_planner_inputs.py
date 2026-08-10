#!/usr/bin/env python3

from __future__ import annotations

from contextlib import redirect_stderr, redirect_stdout
from io import StringIO
import json
import math
import os
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

from inspect_planner_inputs import (
    GridSnapshot,
    InputContract,
    PlannerFreshnessSettings,
    PlannerThresholds,
    Pose2D,
    SubscriptionSpec,
    VerifiedFlatStartConfig,
    _capture_goal_stage,
    _diagnosis_exit_code,
    _capture_runtime_messages,
    _capture_stage,
    _write_session_record_atomic,
    _grid_snapshot,
    _mark_goal_capture_failure,
    _read_live_planner_thresholds,
    _terrain_layer_stats,
    analyze_planner_inputs,
    print_human,
)


UNKNOWN = -1000.0
NOW_NS = 10_000_000_000


def make_grid(
    width: int = 10,
    height: int = 10,
    resolution: float = 1.0,
    stamp_ns: int = NOW_NS,
) -> GridSnapshot:
    cells = width * height
    return GridSnapshot(
        frame_id="world",
        stamp_ns=stamp_ns,
        resolution=resolution,
        width=width,
        height=height,
        origin_x=0.0,
        origin_y=0.0,
        unknown_value=UNKNOWN,
        elevation=[0.0] * cells,
        variance=[0.001] * cells,
        slope=[0.0] * cells,
        roughness=[0.0] * cells,
        traversability=[1.0] * cells,
        observation_count=[4] * cells,
        confidence=[1.0] * cells,
        age=[0.0] * cells,
    )


def make_planar_blind_ring() -> tuple[GridSnapshot, Pose2D, float]:
    grid = make_grid(width=31, height=31, resolution=0.2)
    cells = grid.width * grid.height
    elevation = [UNKNOWN] * cells
    slope = [UNKNOWN] * cells
    roughness = [UNKNOWN] * cells
    traversability = [UNKNOWN] * cells
    observation_count = [0] * cells
    start = pose(3.1, 3.1)
    gradient_x = 0.02
    gradient_y = -0.01
    plane_slope = math.atan(math.hypot(gradient_x, gradient_y))
    for y in range(grid.height):
        cell_y = grid.origin_y + (y + 0.5) * grid.resolution
        for x in range(grid.width):
            cell_x = grid.origin_x + (x + 0.5) * grid.resolution
            dx = cell_x - start.x
            dy = cell_y - start.y
            radius = math.hypot(dx, dy)
            if 1.0 <= radius <= 2.5:
                index = y * grid.width + x
                elevation[index] = 0.4 + gradient_x * dx + gradient_y * dy
                slope[index] = plane_slope
                roughness[index] = 0.01
                traversability[index] = 0.8
                observation_count[index] = 4
    return (
        replace(
            grid,
            elevation=elevation,
            slope=slope,
            roughness=roughness,
            traversability=traversability,
            observation_count=observation_count,
        ),
        start,
        plane_slope,
    )


def pose(
    x: float,
    y: float,
    frame: str = "world",
    stamp_ns: int = NOW_NS,
    child_frame: str = "base_link",
) -> Pose2D:
    return Pose2D(
        x=x,
        y=y,
        yaw=0.0,
        frame_id=frame,
        stamp_ns=stamp_ns,
        child_frame_id=child_frame,
    )


def stamped_message(stamp_ns: int) -> SimpleNamespace:
    seconds, nanoseconds = divmod(stamp_ns, 1_000_000_000)
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=seconds, nanosec=nanoseconds)
        )
    )


class FakeSubscription:
    def __init__(self, topic: str, callback: object, qos: object) -> None:
        self.topic = topic
        self.callback = callback
        self.qos = qos


class FakeNode:
    def __init__(
        self,
        fail_topic: str | None = None,
        publisher_count: int = 1,
    ) -> None:
        self.fail_topic = fail_topic
        self.publisher_count = publisher_count
        self.clock_ns = NOW_NS
        self.active: list[FakeSubscription] = []
        self.events: list[tuple[str, str, object]] = []

    def create_subscription(
        self,
        message_type: object,
        topic: str,
        callback: object,
        qos: object,
    ) -> FakeSubscription:
        del message_type
        if topic == self.fail_topic:
            raise RuntimeError("subscription creation failed")
        subscription = FakeSubscription(topic, callback, qos)
        self.active.append(subscription)
        self.events.append(("create", topic, qos))
        return subscription

    def destroy_subscription(self, subscription: FakeSubscription) -> None:
        self.active.remove(subscription)
        self.events.append(("destroy", subscription.topic, subscription.qos))

    def count_publishers(self, topic: str) -> int:
        del topic
        return self.publisher_count

    def get_clock(self) -> SimpleNamespace:
        return SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=self.clock_ns)
        )

    def publish(self, topic: str, message: object) -> None:
        for subscription in list(self.active):
            if subscription.topic == topic:
                subscription.callback(message)


class FakeParameterType:
    PARAMETER_BOOL = 1
    PARAMETER_INTEGER = 2
    PARAMETER_DOUBLE = 3


class FakeGetParameters:
    class Request:
        def __init__(self) -> None:
            self.names: list[str] = []


class FakeParameterValue:
    def __init__(
        self,
        parameter_type: int,
        *,
        bool_value: bool = False,
        integer_value: int = 0,
        double_value: float = 0.0,
    ) -> None:
        self.type = parameter_type
        self.bool_value = bool_value
        self.integer_value = integer_value
        self.double_value = double_value


class FakeParameterResponse:
    def __init__(self, values: list[FakeParameterValue]) -> None:
        self.values = values


class FakeParameterFuture:
    def __init__(
        self,
        *,
        response: FakeParameterResponse | None = None,
        error: Exception | None = None,
        done: bool = True,
    ) -> None:
        self.response = response
        self.error = error
        self.completed = done
        self.cancelled = False

    def done(self) -> bool:
        return self.completed

    def exception(self) -> Exception | None:
        return self.error

    def result(self) -> FakeParameterResponse | None:
        return self.response

    def cancel(self) -> None:
        self.cancelled = True


class FakeParameterClient:
    def __init__(
        self,
        futures: list[FakeParameterFuture],
        availability: list[bool] | None = None,
    ) -> None:
        self.futures = list(futures)
        self.availability = list(availability or [])
        self.requests: list[list[str]] = []
        self.discovery_timeouts: list[float] = []

    def wait_for_service(self, timeout_sec: float) -> bool:
        self.discovery_timeouts.append(timeout_sec)
        return self.availability.pop(0) if self.availability else True

    def call_async(self, request: FakeGetParameters.Request) -> FakeParameterFuture:
        self.requests.append(list(request.names))
        return self.futures.pop(0)


class FakeParameterNode:
    def __init__(self, clients: dict[str, FakeParameterClient]) -> None:
        self.clients = clients
        self.created_services: list[str] = []
        self.destroyed_clients: list[FakeParameterClient] = []

    def create_client(
        self, service_type: object, service_name: str
    ) -> FakeParameterClient:
        self.asserted_service_type = service_type
        self.created_services.append(service_name)
        return self.clients[service_name]

    def destroy_client(self, client: FakeParameterClient) -> None:
        self.destroyed_clients.append(client)


def parameter_response(
    min_observed_frames: int | None = None,
    *values: float,
) -> FakeParameterResponse:
    encoded: list[FakeParameterValue] = []
    if min_observed_frames is not None:
        encoded.append(
            FakeParameterValue(
                FakeParameterType.PARAMETER_INTEGER,
                integer_value=min_observed_frames,
            )
        )
    encoded.extend(
        FakeParameterValue(
            FakeParameterType.PARAMETER_DOUBLE,
            double_value=value,
        )
        for value in values
    )
    return FakeParameterResponse(encoded)


def full_planner_parameter_response() -> FakeParameterResponse:
    values = [
        FakeParameterValue(FakeParameterType.PARAMETER_DOUBLE, double_value=value)
        for value in (0.21, 0.61, 0.19, 0.55, 0.45, 0.8, 0.4, 0.15, 12.0)
    ]
    values.extend(
        [
            FakeParameterValue(
                FakeParameterType.PARAMETER_BOOL, bool_value=True
            ),
            FakeParameterValue(FakeParameterType.PARAMETER_DOUBLE, double_value=1.0),
            FakeParameterValue(FakeParameterType.PARAMETER_DOUBLE, double_value=2.5),
            FakeParameterValue(FakeParameterType.PARAMETER_DOUBLE, double_value=1.35),
            FakeParameterValue(FakeParameterType.PARAMETER_INTEGER, integer_value=8),
            FakeParameterValue(FakeParameterType.PARAMETER_INTEGER, integer_value=7),
            FakeParameterValue(FakeParameterType.PARAMETER_INTEGER, integer_value=3),
            FakeParameterValue(FakeParameterType.PARAMETER_INTEGER, integer_value=32),
            FakeParameterValue(FakeParameterType.PARAMETER_INTEGER, integer_value=4),
            FakeParameterValue(FakeParameterType.PARAMETER_DOUBLE, double_value=0.15),
            FakeParameterValue(FakeParameterType.PARAMETER_DOUBLE, double_value=0.04),
            FakeParameterValue(FakeParameterType.PARAMETER_DOUBLE, double_value=0.10),
            FakeParameterValue(FakeParameterType.PARAMETER_DOUBLE, double_value=0.18),
            FakeParameterValue(FakeParameterType.PARAMETER_DOUBLE, double_value=0.20),
            FakeParameterValue(FakeParameterType.PARAMETER_DOUBLE, double_value=0.20),
        ]
    )
    values.extend(
        FakeParameterValue(
            FakeParameterType.PARAMETER_DOUBLE, double_value=value
        )
        for value in (2.0, 30.0)
    )
    return FakeParameterResponse(values)


class LiveParameterServiceTests(unittest.TestCase):
    def test_transient_response_failure_is_retried_with_batched_requests(
        self,
    ) -> None:
        mapper_client = FakeParameterClient(
            [
                FakeParameterFuture(error=RuntimeError("transient DDS failure")),
                FakeParameterFuture(
                    response=parameter_response(6, 0.58, 0.07)
                ),
            ]
        )
        planner_client = FakeParameterClient(
            [
                FakeParameterFuture(
                    response=full_planner_parameter_response()
                )
            ]
        )
        node = FakeParameterNode(
            {
                "/terrain_mapper/get_parameters": mapper_client,
                "/body_lattice_planner/get_parameters": planner_client,
            }
        )
        response_timeouts: list[float] = []

        def spin_until_complete(
            unused_node: object,
            unused_future: object,
            timeout_sec: float,
        ) -> None:
            del unused_node, unused_future
            response_timeouts.append(timeout_sec)

        thresholds, freshness, verified_flat_start = _read_live_planner_thresholds(
            node,
            FakeGetParameters,
            FakeParameterType,
            spin_until_complete,
            service_timeout=0.25,
            response_timeout=0.5,
            max_attempts=3,
        )

        self.assertEqual(
            thresholds,
            PlannerThresholds(
                min_traversability=0.21,
                max_slope=0.61,
                mapper_max_slope=0.58,
                max_roughness=0.07,
                max_step_height=0.19,
                start_snap_radius=0.55,
                snap_radius=0.45,
                min_observed_frames=6,
            ),
        )
        self.assertEqual(
            freshness,
            PlannerFreshnessSettings(
                max_map_age=0.8,
                max_odom_age=0.4,
                future_tolerance=0.15,
                input_watchdog_rate=12.0,
                max_goal_age=2.0,
                goal_retention_timeout=30.0,
            ),
        )
        self.assertEqual(
            verified_flat_start,
            VerifiedFlatStartConfig(enabled=True),
        )
        self.assertEqual(
            mapper_client.requests,
            [
                ["min_observed_frames", "max_slope", "max_roughness"],
                ["min_observed_frames", "max_slope", "max_roughness"],
            ],
        )
        self.assertEqual(
            planner_client.requests,
            [[
                "min_traversability",
                "max_slope",
                "max_step_height",
                "start_snap_radius",
                "snap_radius",
                "max_map_age",
                "max_odom_age",
                "timestamp_future_tolerance",
                "input_watchdog_rate",
                "verified_flat_start.enabled",
                "verified_flat_start.support_inner_radius",
                "verified_flat_start.support_outer_radius",
                "verified_flat_start.fill_radius",
                "verified_flat_start.sector_count",
                "verified_flat_start.min_supported_sectors",
                "verified_flat_start.min_cells_per_sector",
                "verified_flat_start.min_support_cells",
                "verified_flat_start.min_observation_count",
                "verified_flat_start.max_plane_slope",
                "verified_flat_start.max_plane_rmse",
                "verified_flat_start.max_plane_residual",
                "verified_flat_start.max_elevation_range",
                "verified_flat_start.inferred_traversability",
                "motion_step",
                "max_goal_age",
                "goal_retention_timeout",
            ]],
        )
        self.assertEqual(response_timeouts, [0.5, 0.5, 0.5])
        self.assertEqual(
            node.created_services,
            [
                "/terrain_mapper/get_parameters",
                "/body_lattice_planner/get_parameters",
            ],
        )

    def test_goal_freshness_parameters_are_requested_and_decoded(self) -> None:
        planner_client = FakeParameterClient(
            [
                FakeParameterFuture(
                    response=full_planner_parameter_response()
                )
            ]
        )
        node = FakeParameterNode(
            {
                "/terrain_mapper/get_parameters": FakeParameterClient(
                    [
                        FakeParameterFuture(
                            response=parameter_response(6, 0.58, 0.07)
                        )
                    ]
                ),
                "/body_lattice_planner/get_parameters": planner_client,
            }
        )

        _, freshness, _ = _read_live_planner_thresholds(
            node,
            FakeGetParameters,
            FakeParameterType,
            lambda unused_node, unused_future, timeout_sec: None,
            service_timeout=0.25,
            response_timeout=0.5,
            max_attempts=1,
        )

        self.assertEqual(freshness.max_goal_age, 2.0)
        self.assertEqual(freshness.goal_retention_timeout, 30.0)
        self.assertEqual(
            planner_client.requests[0][-2:],
            ["max_goal_age", "goal_retention_timeout"],
        )

    def test_permanently_unavailable_service_fails_closed_after_limit(
        self,
    ) -> None:
        mapper_client = FakeParameterClient(
            [],
            availability=[False, False, False],
        )
        planner_client = FakeParameterClient([])
        node = FakeParameterNode(
            {
                "/terrain_mapper/get_parameters": mapper_client,
                "/body_lattice_planner/get_parameters": planner_client,
            }
        )

        with self.assertRaisesRegex(
            RuntimeError,
            r"could not read \[min_observed_frames, max_slope, max_roughness\] "
            r"from /terrain_mapper/get_parameters after 3 attempts: "
            r"service discovery timed out after 0\.250 s",
        ):
            _read_live_planner_thresholds(
                node,
                FakeGetParameters,
                FakeParameterType,
                lambda unused_node, unused_future, timeout_sec: None,
                service_timeout=0.25,
                response_timeout=0.5,
                max_attempts=3,
            )

        self.assertEqual(mapper_client.discovery_timeouts, [0.25, 0.25, 0.25])
        self.assertEqual(mapper_client.requests, [])
        self.assertEqual(node.created_services, ["/terrain_mapper/get_parameters"])
        self.assertEqual(node.destroyed_clients, [mapper_client])

    def test_wrong_parameter_type_is_rejected_with_parameter_name(self) -> None:
        mapper_response = parameter_response(6, 0.58, 0.07)
        mapper_response.values[1] = FakeParameterValue(
            FakeParameterType.PARAMETER_INTEGER,
            integer_value=1,
        )
        node = FakeParameterNode(
            {
                "/terrain_mapper/get_parameters": FakeParameterClient(
                    [FakeParameterFuture(response=mapper_response)]
                ),
                "/body_lattice_planner/get_parameters": FakeParameterClient(
                    [
                        FakeParameterFuture(
                            response=full_planner_parameter_response()
                        )
                    ]
                ),
            }
        )

        with self.assertRaisesRegex(
            RuntimeError,
            r"/terrain_mapper/get_parameters parameter max_slope must be "
            r"a double; received ROS parameter type 2",
        ):
            _read_live_planner_thresholds(
                node,
                FakeGetParameters,
                FakeParameterType,
                lambda unused_node, unused_future, timeout_sec: None,
                service_timeout=0.25,
                response_timeout=0.5,
                max_attempts=3,
            )

    def test_nonfinite_parameter_value_is_rejected(self) -> None:
        node = FakeParameterNode(
            {
                "/terrain_mapper/get_parameters": FakeParameterClient(
                    [
                        FakeParameterFuture(
                            response=parameter_response(6, 0.58, float("nan"))
                        )
                    ]
                ),
                "/body_lattice_planner/get_parameters": FakeParameterClient(
                    [
                        FakeParameterFuture(
                            response=full_planner_parameter_response()
                        )
                    ]
                ),
            }
        )

        with self.assertRaisesRegex(
            RuntimeError,
            r"/terrain_mapper/get_parameters parameter max_roughness must be finite",
        ):
            _read_live_planner_thresholds(
                node,
                FakeGetParameters,
                FakeParameterType,
                lambda unused_node, unused_future, timeout_sec: None,
                service_timeout=0.25,
                response_timeout=0.5,
                max_attempts=3,
            )


def make_specs() -> dict[str, SubscriptionSpec]:
    return {
        "latched_map": SubscriptionSpec(
            "map", object, "/terrain_map", "latched"
        ),
        "live_map": SubscriptionSpec("map", object, "/terrain_map", "live"),
        "odom": SubscriptionSpec("odom", object, "/lio/body_odom", "sensor"),
        "goal": SubscriptionSpec("goal", object, "/goal_pose", "goal"),
    }


class PlannerInputInspectionTests(unittest.TestCase):
    @unittest.skipUnless(os.name == "posix", "session append uses POSIX locking")
    def test_session_record_is_appended_as_durable_jsonl(self) -> None:
        result = analyze_planner_inputs(
            make_grid(),
            pose(1.5, 1.5),
            None,
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "planner_input_inspections.jsonl"
            _write_session_record_atomic(
                path,
                Path(directory),
                0,
                inspection=result,
                recorded_at="2026-08-06T00:00:00+00:00",
            )
            _write_session_record_atomic(
                path,
                Path(directory),
                2,
                inspection=result,
                recorded_at="2026-08-06T00:00:01+00:00",
            )
            records = [json.loads(line) for line in path.read_text().splitlines()]
        self.assertEqual(len(records), 2)
        self.assertEqual(records[0]["schema_version"], 1)
        self.assertEqual(records[0]["status"], "ok")
        self.assertEqual(
            records[0]["inspection"]["diagnosis"],
            "start_ready_waiting_for_goal",
        )
        self.assertEqual(records[1]["status"], "diagnostic_failure")
        self.assertEqual(
            records[1]["recorded_at"],
            "2026-08-06T00:00:01+00:00",
        )

    @unittest.skipUnless(os.name == "posix", "session append uses POSIX files")
    def test_session_record_rejects_unsafe_existing_payloads(self) -> None:
        result = analyze_planner_inputs(
            make_grid(),
            pose(1.5, 1.5),
            None,
        )
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            path = root / "planner_input_inspections.jsonl"
            unsafe = b'{"schema_version":1}\n\0'
            path.write_bytes(unsafe)
            with self.assertRaisesRegex(ValueError, "NUL"):
                _write_session_record_atomic(
                    path,
                    root,
                    0,
                    inspection=result,
                )
            self.assertEqual(path.read_bytes(), unsafe)

            target = root / "other.jsonl"
            target.write_text("", encoding="utf-8")
            path.unlink()
            path.symlink_to(target)
            with self.assertRaises(OSError):
                _write_session_record_atomic(
                    path,
                    root,
                    0,
                    inspection=result,
                )
            self.assertEqual(target.read_text(encoding="utf-8"), "")

    @unittest.skipUnless(os.name == "posix", "session append uses POSIX files")
    def test_session_record_rejects_nonfinite_json(self) -> None:
        result = analyze_planner_inputs(
            make_grid(),
            pose(1.5, 1.5),
            None,
        )
        result["thresholds"]["max_slope"] = float("nan")
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            path = root / "planner_input_inspections.jsonl"
            with self.assertRaisesRegex(ValueError, "JSON"):
                _write_session_record_atomic(
                    path,
                    root,
                    0,
                    inspection=result,
                )
            self.assertFalse(path.exists())

    def test_observation_buckets_are_exhaustive(self) -> None:
        grid = replace(
            make_grid(width=5, height=1),
            observation_count=[0, 1, 3, 4, 8],
        )
        stats = _terrain_layer_stats(
            grid,
            PlannerThresholds(min_observed_frames=4),
        )
        self.assertEqual(stats["cell_count"], 5)
        self.assertEqual(stats["observation_zero"], 1)
        self.assertEqual(stats["observation_below_min"], 2)
        self.assertEqual(stats["observation_ready"], 2)

    def test_slope_quantiles_use_known_finite_values_and_linear_interpolation(self) -> None:
        grid = replace(
            make_grid(width=7, height=1),
            slope=[0.0, 0.1, 0.2, 0.3, 0.4, UNKNOWN, float("nan")],
        )

        stats = _terrain_layer_stats(grid, PlannerThresholds())

        self.assertEqual(
            {
                key: stats[key]
                for key in (
                    "slope_min_rad",
                    "slope_p50_rad",
                    "slope_p90_rad",
                    "slope_p95_rad",
                    "slope_max_rad",
                )
            },
            {
                "slope_min_rad": 0.0,
                "slope_p50_rad": 0.2,
                "slope_p90_rad": 0.36,
                "slope_p95_rad": 0.38,
                "slope_max_rad": 0.4,
            },
        )

    def test_slope_quantiles_are_null_when_no_slope_is_known(self) -> None:
        grid = replace(
            make_grid(width=2, height=1),
            slope=[UNKNOWN, float("inf")],
        )

        stats = _terrain_layer_stats(grid, PlannerThresholds())

        self.assertEqual(
            [
                stats[key]
                for key in (
                    "slope_min_rad",
                    "slope_p50_rad",
                    "slope_p90_rad",
                    "slope_p95_rad",
                    "slope_max_rad",
                )
            ],
            [None, None, None, None, None],
        )

    def test_known_elevation_is_distinct_from_known_features(self) -> None:
        grid = replace(
            make_grid(width=1, height=1),
            observation_count=[0],
            slope=[UNKNOWN],
            traversability=[UNKNOWN],
        )
        stats = _terrain_layer_stats(grid, PlannerThresholds())
        self.assertEqual(stats["elevation_known"], 1)
        self.assertEqual(stats["features_known"], 0)
        self.assertEqual(stats["observation_zero"], 1)

    def test_threshold_rejection_counts_are_independent(self) -> None:
        grid = replace(
            make_grid(width=5, height=1),
            slope=[0.66, 0.0, 0.0, 0.0, 0.0],
            roughness=[0.0, 0.081, 0.0, 0.0, 0.0],
            traversability=[1.0, 1.0, 0.10, 0.18, 1.0],
        )
        stats = _terrain_layer_stats(grid, PlannerThresholds())
        self.assertEqual(stats["slope_over_limit"], 1)
        self.assertEqual(stats["roughness_over_limit"], 1)
        self.assertEqual(stats["traversability_below_min_nonzero"], 1)
        self.assertEqual(stats["traversability_at_or_above_min"], 4)
        self.assertEqual(stats["planner_valid"], 3)

    def test_hard_rejection_candidate_uses_strict_feature_limits(self) -> None:
        grid = replace(
            make_grid(width=3, height=1),
            slope=[0.10, 0.65, 0.10],
            roughness=[0.01, 0.01, 0.08],
            traversability=[0.0, 0.0, 0.0],
        )
        stats = _terrain_layer_stats(grid, PlannerThresholds())
        self.assertEqual(stats["traversability_zero"], 3)
        self.assertEqual(
            stats[
                "zero_traversability_with_slope_and_roughness_below_limits"
            ],
            1,
        )

    def test_hard_rejection_uses_mapper_not_planner_slope_limit(self) -> None:
        grid = replace(
            make_grid(width=1, height=1),
            slope=[0.60],
            roughness=[0.01],
            traversability=[0.0],
        )
        stats = _terrain_layer_stats(
            grid,
            PlannerThresholds(max_slope=0.70, mapper_max_slope=0.50),
        )
        self.assertEqual(stats["slope_over_limit"], 0)
        self.assertEqual(stats["slope_at_or_above_mapper_limit"], 1)
        self.assertEqual(
            stats[
                "zero_traversability_with_slope_and_roughness_below_limits"
            ],
            0,
        )

    def test_snap_square_layer_stats_are_local_and_map_clipped(self) -> None:
        observation_count = [4] * 25
        observation_count[0] = 0
        observation_count[24] = 1
        grid = replace(
            make_grid(width=5, height=5, resolution=0.1),
            observation_count=observation_count,
        )
        result = analyze_planner_inputs(
            grid,
            pose(0.05, 0.05),
            pose(0.45, 0.45),
            PlannerThresholds(snap_radius=0.1),
        )
        global_stats = result["map"]["terrain_layers"]
        start_stats = result["start"]["snap_square_terrain_layers"]
        goal_stats = result["goal"]["snap_square_terrain_layers"]
        start_radius_stats = result["start"]["snap_radius_terrain_layers"]
        goal_radius_stats = result["goal"]["snap_radius_terrain_layers"]
        self.assertEqual(global_stats["observation_zero"], 1)
        self.assertEqual(global_stats["observation_below_min"], 1)
        self.assertEqual(start_stats["cell_count"], 4)
        self.assertEqual(start_stats["observation_zero"], 1)
        self.assertEqual(start_stats["observation_below_min"], 0)
        self.assertEqual(goal_stats["cell_count"], 4)
        self.assertEqual(goal_stats["observation_zero"], 0)
        self.assertEqual(goal_stats["observation_below_min"], 1)
        self.assertEqual(start_radius_stats["cell_count"], 3)
        self.assertEqual(goal_radius_stats["cell_count"], 3)

    def test_radial_coverage_reports_distance_rings_and_sectors(self) -> None:
        grid = make_grid(width=9, height=9, resolution=1.0)
        cells = grid.width * grid.height
        elevation = [UNKNOWN] * cells
        slope = [UNKNOWN] * cells
        roughness = [UNKNOWN] * cells
        traversability = [UNKNOWN] * cells
        observations = [0] * cells

        east_observed = 4 * grid.width + 5
        east_elevation = 4 * grid.width + 6
        north_planner_valid = 7 * grid.width + 4
        observations[east_observed] = 1
        observations[east_elevation] = 4
        elevation[east_elevation] = 0.0
        observations[north_planner_valid] = 4
        elevation[north_planner_valid] = 0.0
        slope[north_planner_valid] = 0.0
        roughness[north_planner_valid] = 0.0
        traversability[north_planner_valid] = 1.0
        grid = replace(
            grid,
            elevation=elevation,
            slope=slope,
            roughness=roughness,
            traversability=traversability,
            observation_count=observations,
        )

        result = analyze_planner_inputs(
            grid,
            pose(4.5, 4.5),
            None,
            PlannerThresholds(start_snap_radius=0.5),
        )
        coverage = result["start"]["radial_coverage"]
        nearest = coverage["nearest_distance_m"]
        self.assertEqual(nearest["observed"], 1.0)
        self.assertEqual(nearest["observation_ready"], 2.0)
        self.assertEqual(nearest["elevation_known"], 2.0)
        self.assertEqual(nearest["features_known"], 3.0)
        self.assertEqual(nearest["planner_valid"], 3.0)

        self.assertEqual(coverage["rings"][1]["observed"], 1)
        self.assertEqual(coverage["rings"][2]["elevation_known"], 2)
        self.assertEqual(coverage["rings"][2]["planner_valid"], 1)
        self.assertEqual(coverage["sectors"][0]["observed"], 2)
        self.assertEqual(coverage["sectors"][2]["planner_valid"], 1)
        self.assertEqual(
            coverage["sectors"][2]["nearest_distance_m"]["planner_valid"],
            3.0,
        )
        self.assertEqual(
            sum(ring["cell_count"] for ring in coverage["rings"]), 29
        )
        self.assertEqual(
            sum(sector["cell_count"] for sector in coverage["sectors"]),
            29,
        )
        for key in (
            "observed",
            "observation_ready",
            "elevation_known",
            "features_known",
            "planner_valid",
        ):
            self.assertEqual(
                sum(ring[key] for ring in coverage["rings"]),
                sum(sector[key] for sector in coverage["sectors"]),
            )

        rotated_result = analyze_planner_inputs(
            grid,
            replace(pose(4.5, 4.5), yaw=math.pi / 2.0),
            None,
            PlannerThresholds(start_snap_radius=0.5),
        )
        rotated_coverage = rotated_result["start"]["radial_coverage"]
        self.assertEqual(rotated_coverage["sectors"][0]["planner_valid"], 1)

    def test_grid_snapshot_captures_diagnostic_layers(self) -> None:
        message = SimpleNamespace(
            header=SimpleNamespace(
                frame_id="world",
                stamp=SimpleNamespace(sec=2, nanosec=3),
            ),
            resolution=0.1,
            width=1,
            height=1,
            origin_x=-1.0,
            origin_y=-2.0,
            unknown_value=UNKNOWN,
            elevation=[0.2],
            variance=[0.003],
            slope=[0.1],
            roughness=[0.02],
            traversability=[0.7],
            observation_count=[6],
            confidence=[0.8],
            age=[0.4],
        )
        snapshot = _grid_snapshot(message)
        self.assertEqual(snapshot.variance, [0.003])
        self.assertEqual(snapshot.roughness, [0.02])
        self.assertEqual(snapshot.observation_count, [6])
        self.assertEqual(snapshot.confidence, [0.8])
        self.assertEqual(snapshot.age, [0.4])

    def test_exit_codes_distinguish_ready_from_rejected_diagnoses(self) -> None:
        self.assertEqual(
            _diagnosis_exit_code(
                "same_continuous_ground_component_not_planner_approval"
            ),
            0,
        )
        self.assertEqual(_diagnosis_exit_code("start_ready_waiting_for_goal"), 0)
        self.assertEqual(
            _diagnosis_exit_code(
                "start_ready_with_verified_flat_start_waiting_for_goal"
            ),
            0,
        )
        self.assertEqual(_diagnosis_exit_code("goal_stale"), 2)
        self.assertEqual(_diagnosis_exit_code("goal_wait_timeout"), 2)
        self.assertEqual(
            _diagnosis_exit_code("goal_publisher_discovery_timeout"), 2
        )

    def test_goal_capture_failure_preserves_start_map_diagnosis(self) -> None:
        result: dict[str, object] = {
            "diagnosis": "start_has_no_valid_cell_in_snap_square"
        }

        _mark_goal_capture_failure(result, "goal_wait_timeout")

        self.assertEqual(result["diagnosis"], "goal_wait_timeout")
        self.assertEqual(
            result["start_map_diagnosis"],
            "start_has_no_valid_cell_in_snap_square",
        )
        self.assertEqual(result["goal_capture_failure"], "goal_wait_timeout")

    def test_flat_map_reports_continuous_ground_only(self) -> None:
        result = analyze_planner_inputs(
            make_grid(), pose(1.5, 1.5), pose(6.5, 1.5)
        )
        self.assertEqual(
            result["diagnosis"],
            "same_continuous_ground_component_not_planner_approval",
        )
        self.assertEqual(result["start_component_cells"], 100)
        self.assertTrue(result["goal_in_start_component"])

    def test_start_without_valid_snap_cell_is_explicit(self) -> None:
        grid = make_grid(width=5, height=5, resolution=0.1)
        grid = replace(
            grid,
            slope=[UNKNOWN] * 25,
            traversability=[UNKNOWN] * 25,
        )
        result = analyze_planner_inputs(grid, pose(0.25, 0.25), None)
        self.assertEqual(
            result["diagnosis"], "start_has_no_valid_cell_in_snap_radius"
        )
        self.assertEqual(result["start"]["valid_cells_in_snap_square"], 0)
        self.assertEqual(result["start"]["valid_cells_in_snap_radius"], 0)

    def test_verified_flat_start_disabled_preserves_existing_diagnosis(self) -> None:
        grid = make_grid(width=5, height=5, resolution=0.1)
        grid = replace(
            grid,
            slope=[UNKNOWN] * 25,
            traversability=[UNKNOWN] * 25,
        )

        result = analyze_planner_inputs(
            grid,
            pose(0.25, 0.25),
            None,
            verified_flat_start=VerifiedFlatStartConfig(enabled=False),
        )

        self.assertEqual(
            result["diagnosis"], "start_has_no_valid_cell_in_snap_radius"
        )
        self.assertEqual(result["verified_flat_start"]["status"], "disabled")

    def test_verified_flat_start_is_not_evaluated_after_ordinary_snap(self) -> None:
        result = analyze_planner_inputs(
            make_grid(),
            pose(1.5, 1.5),
            None,
            verified_flat_start=VerifiedFlatStartConfig(enabled=True),
        )

        self.assertEqual(result["diagnosis"], "start_ready_waiting_for_goal")
        self.assertEqual(result["verified_flat_start"]["status"], "not_needed")
        self.assertIsNone(result["verified_flat_start"]["fit"])

    def test_verified_flat_start_applies_planar_multi_sector_overlay(self) -> None:
        grid, start, expected_slope = make_planar_blind_ring()
        center = 15 * grid.width + 15
        self.assertEqual(grid.elevation[center], UNKNOWN)

        result = analyze_planner_inputs(
            grid,
            start,
            None,
            verified_flat_start=VerifiedFlatStartConfig(enabled=True),
        )

        assessment = result["verified_flat_start"]
        self.assertEqual(
            result["diagnosis"],
            "start_ready_with_verified_flat_start_waiting_for_goal",
        )
        self.assertEqual(assessment["status"], "applied")
        self.assertGreaterEqual(assessment["support_cells"], 32)
        self.assertEqual(assessment["support_sectors"], 8)
        self.assertGreater(assessment["filled_cells"], 0)
        self.assertEqual(
            assessment["planner_valid_filled_cells"],
            assessment["filled_cells"],
        )
        self.assertAlmostEqual(
            assessment["fit"]["slope_rad"], expected_slope, places=6
        )
        self.assertAlmostEqual(assessment["fit"]["rmse_m"], 0.0, places=9)
        self.assertTrue(result["start"]["snapped"])
        self.assertTrue(result["start"]["inferred"])
        self.assertTrue(assessment["exact_start_inferred"])
        self.assertEqual(result["map"]["valid_cells"], assessment["support_cells"])
        self.assertEqual(grid.elevation[center], UNKNOWN)

    def test_verified_flat_start_fit_is_reported_in_human_output(self) -> None:
        grid, start, _ = make_planar_blind_ring()
        result = analyze_planner_inputs(
            grid,
            start,
            None,
            verified_flat_start=VerifiedFlatStartConfig(enabled=True),
        )
        output = StringIO()

        with redirect_stdout(output):
            print_human(result)

        text = output.getvalue()
        self.assertIn("verified_flat_start: status=applied enabled=True", text)
        self.assertIn("connected_support=", text)
        self.assertIn("support=1.000-2.500 m fill=1.350 m", text)
        self.assertIn("motion_step=0.200 m", text)
        self.assertIn("plane_slope=", text)
        self.assertIn("max_residual=", text)

    def test_verified_flat_start_reports_fit_before_elevation_range_rejection(
        self,
    ) -> None:
        grid, start, _ = make_planar_blind_ring()
        elevation = list(grid.elevation)
        for y in range(grid.height):
            for x in range(grid.width):
                index = y * grid.width + x
                if elevation[index] == UNKNOWN:
                    continue
                cell_x = grid.origin_x + (x + 0.5) * grid.resolution
                elevation[index] += 0.06 * (cell_x - start.x)
        grid = replace(grid, elevation=elevation)

        result = analyze_planner_inputs(
            grid,
            start,
            None,
            verified_flat_start=VerifiedFlatStartConfig(enabled=True),
        )

        assessment = result["verified_flat_start"]
        support_elevation = assessment["support_elevation"]
        fit = assessment["fit"]
        self.assertEqual(
            result["diagnosis"], "start_has_no_valid_cell_in_snap_radius"
        )
        self.assertEqual(assessment["status"], "support_not_flat")
        self.assertEqual(
            assessment["rejection_reason"],
            "elevation_range_exceeds_limit",
        )
        self.assertGreater(
            support_elevation["range_m"],
            assessment["config"]["max_elevation_range"],
        )
        self.assertAlmostEqual(
            fit["slope_rad"], math.atan(math.hypot(0.08, -0.01)), places=6
        )
        self.assertAlmostEqual(fit["rmse_m"], 0.0, places=9)
        self.assertAlmostEqual(fit["residual_range_m"], 0.0, places=9)
        self.assertEqual(len(assessment["support_sector_elevation"]), 8)
        self.assertEqual(
            assessment["support_sector_convention"]["zero_direction"],
            "map_positive_x",
        )
        self.assertEqual(
            assessment["support_sector_convention"]["rotation"],
            "counterclockwise",
        )
        self.assertIsNotNone(assessment["support_extrema"]["minimum"])
        self.assertIsNotNone(assessment["support_extrema"]["maximum"])
        self.assertIsNotNone(
            assessment["support_extrema"]["maximum_absolute_residual"]
        )
        self.assertAlmostEqual(
            assessment["support_extrema"]["minimum"]["elevation_m"],
            support_elevation["min_m"],
        )
        self.assertAlmostEqual(
            assessment["support_extrema"]["maximum"]["elevation_m"],
            support_elevation["max_m"],
        )
        self.assertEqual(assessment["filled_cells"], 0)

    def test_verified_flat_start_rejects_insufficient_support_sectors(self) -> None:
        grid, start, _ = make_planar_blind_ring()
        elevation = list(grid.elevation)
        slope = list(grid.slope)
        roughness = list(grid.roughness)
        traversability = list(grid.traversability)
        observations = list(grid.observation_count)
        for y in range(grid.height):
            cell_y = grid.origin_y + (y + 0.5) * grid.resolution
            for x in range(grid.width):
                cell_x = grid.origin_x + (x + 0.5) * grid.resolution
                angle = math.atan2(cell_y - start.y, cell_x - start.x)
                sector = int((angle % (2.0 * math.pi)) / (math.pi / 4.0))
                if sector < 5:
                    continue
                index = y * grid.width + x
                elevation[index] = UNKNOWN
                slope[index] = UNKNOWN
                roughness[index] = UNKNOWN
                traversability[index] = UNKNOWN
                observations[index] = 0
        grid = replace(
            grid,
            elevation=elevation,
            slope=slope,
            roughness=roughness,
            traversability=traversability,
            observation_count=observations,
        )

        result = analyze_planner_inputs(
            grid,
            start,
            None,
            verified_flat_start=VerifiedFlatStartConfig(enabled=True),
        )

        assessment = result["verified_flat_start"]
        self.assertEqual(
            result["diagnosis"], "start_has_no_valid_cell_in_snap_radius"
        )
        self.assertGreaterEqual(assessment["support_cells"], 32)
        self.assertEqual(assessment["support_sectors"], 5)
        self.assertEqual(assessment["status"], "insufficient_sectors")
        self.assertEqual(
            assessment["rejection_reason"], "min_supported_sectors"
        )
        self.assertIsNone(assessment["fit"])

    def test_verified_flat_start_rejects_large_plane_residual(self) -> None:
        grid, start, _ = make_planar_blind_ring()
        elevation = list(grid.elevation)
        outlier = 23 * grid.width + 7
        self.assertEqual(grid.observation_count[outlier], 4)
        elevation[outlier] += 0.11
        grid = replace(
            grid,
            elevation=elevation,
        )

        result = analyze_planner_inputs(
            grid,
            start,
            None,
            verified_flat_start=VerifiedFlatStartConfig(enabled=True),
        )

        assessment = result["verified_flat_start"]
        self.assertEqual(
            result["diagnosis"], "start_has_no_valid_cell_in_snap_radius"
        )
        self.assertEqual(assessment["status"], "support_not_flat")
        self.assertEqual(
            assessment["rejection_reason"], "plane_residual_exceeds_limit"
        )
        self.assertGreater(
            assessment["fit"]["max_residual_m"],
            assessment["config"]["max_plane_residual"],
        )
        residual_point = assessment["support_extrema"][
            "maximum_absolute_residual"
        ]
        self.assertEqual(residual_point["grid_x"], 7)
        self.assertEqual(residual_point["grid_y"], 23)
        self.assertAlmostEqual(
            residual_point["absolute_residual_m"],
            assessment["fit"]["max_residual_m"],
        )
        self.assertEqual(residual_point["variance_m2"], 0.001)
        self.assertEqual(residual_point["confidence"], 1.0)
        self.assertEqual(residual_point["age_sec"], 0.0)
        self.assertEqual(assessment["filled_cells"], 0)

    def test_verified_flat_start_never_accepts_an_inferred_goal(self) -> None:
        grid, start, _ = make_planar_blind_ring()

        result = analyze_planner_inputs(
            grid,
            start,
            pose(start.x + 0.4, start.y),
            verified_flat_start=VerifiedFlatStartConfig(enabled=True),
        )

        self.assertEqual(result["verified_flat_start"]["status"], "applied")
        self.assertEqual(
            result["diagnosis"], "goal_has_no_valid_cell_in_snap_radius"
        )

    def test_verified_flat_start_requires_completely_unknown_exact_start(self) -> None:
        grid, start, _ = make_planar_blind_ring()
        center = 15 * grid.width + 15
        slope = list(grid.slope)
        slope[center] = 0.0
        grid = replace(grid, slope=slope)

        result = analyze_planner_inputs(
            grid,
            start,
            None,
            verified_flat_start=VerifiedFlatStartConfig(enabled=True),
        )

        assessment = result["verified_flat_start"]
        self.assertEqual(
            result["diagnosis"], "start_has_no_valid_cell_in_snap_radius"
        )
        self.assertEqual(assessment["status"], "no_inferred_start_cell")
        self.assertEqual(
            assessment["rejection_reason"], "exact_start_cell_not_inferred"
        )

    def test_verified_flat_start_rejects_unreachable_support_annulus(self) -> None:
        grid, start, _ = make_planar_blind_ring()

        result = analyze_planner_inputs(
            grid,
            start,
            None,
            verified_flat_start=VerifiedFlatStartConfig(
                enabled=True,
                support_inner_radius=1.6,
                fill_radius=1.35,
                motion_step=0.2,
            ),
        )

        assessment = result["verified_flat_start"]
        self.assertEqual(
            result["diagnosis"], "start_has_no_valid_cell_in_snap_radius"
        )
        self.assertEqual(assessment["status"], "invalid_configuration")
        self.assertEqual(
            assessment["rejection_reason"],
            "fill_radius_plus_motion_step_below_support_inner_radius",
        )
        self.assertEqual(assessment["config"]["motion_step"], 0.2)

    def test_verified_flat_start_rejects_disconnected_observed_support(self) -> None:
        grid, start, _ = make_planar_blind_ring()
        elevation = list(grid.elevation)
        slope = list(grid.slope)
        roughness = list(grid.roughness)
        traversability = list(grid.traversability)
        observations = list(grid.observation_count)
        for y in range(grid.height):
            cell_y = grid.origin_y + (y + 0.5) * grid.resolution
            for x in range(grid.width):
                cell_x = grid.origin_x + (x + 0.5) * grid.resolution
                radius = math.hypot(cell_x - start.x, cell_y - start.y)
                if radius >= 1.8:
                    continue
                index = y * grid.width + x
                elevation[index] = UNKNOWN
                slope[index] = UNKNOWN
                roughness[index] = UNKNOWN
                traversability[index] = UNKNOWN
                observations[index] = 0
        grid = replace(
            grid,
            elevation=elevation,
            slope=slope,
            roughness=roughness,
            traversability=traversability,
            observation_count=observations,
        )

        result = analyze_planner_inputs(
            grid,
            start,
            None,
            verified_flat_start=VerifiedFlatStartConfig(enabled=True),
        )

        assessment = result["verified_flat_start"]
        self.assertEqual(
            result["diagnosis"], "start_has_no_valid_cell_in_snap_radius"
        )
        self.assertGreaterEqual(
            assessment["support_cells"],
            assessment["config"]["min_support_cells"],
        )
        self.assertGreaterEqual(
            assessment["support_sectors"],
            assessment["config"]["min_supported_sectors"],
        )
        self.assertIsNotNone(assessment["fit"])
        self.assertEqual(assessment["connected_support_cells"], 0)
        self.assertEqual(assessment["status"], "no_observed_connection")
        self.assertEqual(
            assessment["rejection_reason"],
            "inferred_component_does_not_reach_observed_support",
        )

    def test_goal_outside_map_is_explicit(self) -> None:
        result = analyze_planner_inputs(
            make_grid(), pose(1.5, 1.5), pose(12.0, 1.5)
        )
        self.assertEqual(result["diagnosis"], "goal_outside_map")
        self.assertIsNone(result["goal"]["radial_coverage"])

    def test_radial_coverage_stays_bounded_at_dense_resolution(self) -> None:
        grid = make_grid(width=121, height=121, resolution=0.05)
        result = analyze_planner_inputs(grid, pose(3.025, 3.025), None)
        coverage = result["start"]["radial_coverage"]
        self.assertEqual(len(coverage["rings"]), 60)
        self.assertEqual(len(coverage["sectors"]), 8)
        encoded = json.dumps(
            result,
            allow_nan=False,
            separators=(",", ":"),
        ).encode("utf-8")
        self.assertLess(len(encoded), 100_000)

    def test_radial_coverage_caps_rings_for_tiny_resolution(self) -> None:
        grid = make_grid(width=5, height=5, resolution=0.0001)
        result = analyze_planner_inputs(grid, pose(0.00025, 0.00025), None)
        coverage = result["start"]["radial_coverage"]
        self.assertEqual(len(coverage["rings"]), 64)
        self.assertAlmostEqual(coverage["ring_width_m"], 3.0 / 64.0)

        encoded = json.dumps(
            result,
            allow_nan=False,
            separators=(",", ":"),
        ).encode("utf-8")
        self.assertLess(len(encoded), 100_000)

    def test_disconnected_valid_regions_are_detected(self) -> None:
        grid = make_grid(width=8, height=3)
        traversability = [UNKNOWN] * 24
        slope = [UNKNOWN] * 24
        for x in (0, 1, 6, 7):
            traversability[8 + x] = 1.0
            slope[8 + x] = 0.0
        grid = replace(grid, slope=slope, traversability=traversability)
        result = analyze_planner_inputs(
            grid,
            pose(0.5, 1.5),
            pose(7.5, 1.5),
            PlannerThresholds(snap_radius=0.5),
        )
        self.assertEqual(
            result["diagnosis"],
            "start_and_goal_continuous_ground_disconnected",
        )
        self.assertEqual(result["start_component_cells"], 2)
        self.assertFalse(result["goal_in_start_component"])

    def test_height_cliff_disconnects_continuous_ground(self) -> None:
        grid = replace(make_grid(width=2, height=1), elevation=[0.0, 1.0])
        result = analyze_planner_inputs(
            grid,
            pose(0.5, 0.5),
            pose(1.5, 0.5),
            PlannerThresholds(max_step_height=0.24),
        )
        self.assertEqual(
            result["diagnosis"],
            "start_and_goal_continuous_ground_disconnected",
        )
        self.assertEqual(result["start_component_cells"], 1)

    def test_diagonal_corner_cut_is_not_continuous_ground(self) -> None:
        grid = make_grid(width=2, height=2)
        grid = replace(
            grid,
            slope=[0.0, UNKNOWN, UNKNOWN, 0.0],
            traversability=[1.0, UNKNOWN, UNKNOWN, 1.0],
        )
        result = analyze_planner_inputs(
            grid,
            pose(0.5, 0.5),
            pose(1.5, 1.5),
        )
        self.assertEqual(
            result["diagnosis"],
            "start_and_goal_continuous_ground_disconnected",
        )

    def test_diagonal_endpoints_do_not_cross_high_orthogonal_cells(self) -> None:
        grid = replace(
            make_grid(width=2, height=2),
            elevation=[0.0, 1.0, 1.0, 0.0],
        )
        result = analyze_planner_inputs(
            grid,
            pose(0.5, 0.5),
            pose(1.5, 1.5),
            PlannerThresholds(max_step_height=0.24),
        )
        self.assertEqual(
            result["diagnosis"],
            "start_and_goal_continuous_ground_disconnected",
        )

    def test_unknown_endpoint_elevation_is_not_planner_valid(self) -> None:
        grid = replace(make_grid(width=1, height=1), elevation=[UNKNOWN])
        result = analyze_planner_inputs(grid, pose(0.5, 0.5), None)
        self.assertFalse(result["start"]["exact_cell_valid"])
        self.assertEqual(result["map"]["valid_cells"], 0)
        self.assertEqual(result["map"]["continuous_ground_cells"], 0)
        self.assertEqual(
            result["diagnosis"],
            "start_has_no_valid_cell_in_snap_radius",
        )

    def test_snap_rejects_candidate_outside_euclidean_radius(self) -> None:
        grid = make_grid(width=5, height=5)
        traversability = [UNKNOWN] * 25
        slope = [UNKNOWN] * 25
        traversability[18] = 1.0
        slope[18] = 0.0
        grid = replace(grid, slope=slope, traversability=traversability)
        result = analyze_planner_inputs(
            grid,
            pose(2.5, 2.5),
            None,
            PlannerThresholds(snap_radius=1.0),
        )
        self.assertFalse(result["start"]["snapped"])
        self.assertEqual(result["start"]["valid_cells_in_snap_square"], 1)
        self.assertEqual(result["start"]["valid_cells_in_snap_radius"], 0)
        self.assertEqual(
            result["diagnosis"],
            "start_has_no_valid_cell_in_snap_radius",
        )

    def test_snap_accepts_candidate_on_euclidean_radius_boundary(self) -> None:
        grid = make_grid(width=8, height=8, resolution=0.1)
        traversability = [UNKNOWN] * 64
        slope = [UNKNOWN] * 64
        traversability[6 * 8 + 1] = 1.0
        slope[6 * 8 + 1] = 0.0
        grid = replace(grid, slope=slope, traversability=traversability)
        result = analyze_planner_inputs(
            grid,
            pose(0.15, 0.15),
            None,
            PlannerThresholds(snap_radius=0.5),
        )
        self.assertTrue(result["start"]["snapped"])
        self.assertEqual(result["start"]["valid_cells_in_snap_radius"], 1)
        self.assertAlmostEqual(result["start"]["snap_grid_distance_m"], 0.5)
        self.assertFalse(
            result["start"]["snap_grid_distance_outside_nominal_radius"]
        )
        self.assertEqual(result["diagnosis"], "start_ready_waiting_for_goal")

    def test_snap_distance_is_continuous_across_source_cell_boundary(self) -> None:
        grid = make_grid(width=8, height=8, resolution=0.2)
        traversability = [UNKNOWN] * 64
        slope = [UNKNOWN] * 64
        traversability[4 * 8 + 1] = 1.0
        slope[4 * 8 + 1] = 0.0
        grid = replace(grid, slope=slope, traversability=traversability)
        thresholds = PlannerThresholds(
            start_snap_radius=0.55,
            snap_radius=0.5,
        )

        below_boundary = analyze_planner_inputs(
            grid, pose(0.3, 0.399), None, thresholds
        )
        above_boundary = analyze_planner_inputs(
            grid, pose(0.3, 0.401), None, thresholds
        )

        self.assertTrue(below_boundary["start"]["snapped"])
        self.assertTrue(above_boundary["start"]["snapped"])
        self.assertEqual(
            below_boundary["start"]["snapped_grid_y"],
            above_boundary["start"]["snapped_grid_y"],
        )
        self.assertLess(
            below_boundary["start"]["snap_world_to_center_distance_m"],
            thresholds.start_snap_radius,
        )

    def test_goal_keeps_its_narrower_snap_radius(self) -> None:
        grid = make_grid(width=8, height=8, resolution=0.2)
        traversability = [UNKNOWN] * 64
        slope = [UNKNOWN] * 64
        traversability[4 * 8 + 1] = 1.0
        slope[4 * 8 + 1] = 0.0
        grid = replace(grid, slope=slope, traversability=traversability)
        thresholds = PlannerThresholds(
            start_snap_radius=0.55,
            snap_radius=0.5,
        )

        result = analyze_planner_inputs(
            grid,
            pose(0.3, 0.9),
            pose(0.3, 0.399),
            thresholds,
        )

        self.assertTrue(result["start"]["snapped"])
        self.assertFalse(result["goal"]["snapped"])
        self.assertEqual(
            result["diagnosis"], "goal_has_no_valid_cell_in_snap_radius"
        )

    def test_legacy_snap_radius_also_controls_start(self) -> None:
        grid = make_grid(width=8, height=8, resolution=0.1)
        traversability = [UNKNOWN] * 64
        slope = [UNKNOWN] * 64
        traversability[3 * 8 + 1] = 1.0
        slope[3 * 8 + 1] = 0.0
        grid = replace(grid, slope=slope, traversability=traversability)

        # Preserve the original positional field order when the new field is omitted.
        thresholds = PlannerThresholds(0.18, 0.65, 0.65, 0.08, 0.24, 0.1, 4)
        result = analyze_planner_inputs(
            grid,
            pose(0.15, 0.251),
            None,
            thresholds,
        )

        self.assertIsNone(thresholds.start_snap_radius)
        self.assertTrue(result["start"]["snapped"])
        self.assertEqual(result["thresholds"]["start_snap_radius"], 0.1)
        self.assertEqual(
            result["diagnosis"], "start_ready_waiting_for_goal"
        )

    def test_exact_valid_cell_does_not_require_snapping(self) -> None:
        result = analyze_planner_inputs(
            make_grid(width=2, height=1),
            pose(0.99, 0.5),
            None,
            PlannerThresholds(start_snap_radius=0.0, snap_radius=0.0),
        )

        self.assertTrue(result["start"]["exact_cell_valid"])
        self.assertTrue(result["start"]["snapped"])
        self.assertEqual(result["start"]["snapped_grid_x"], 0)
        self.assertEqual(result["start"]["valid_cells_in_snap_radius"], 1)
        self.assertAlmostEqual(
            result["start"]["snap_world_to_center_distance_m"], 0.49
        )
        self.assertEqual(result["diagnosis"], "start_ready_waiting_for_goal")

    def test_parent_frame_mismatch_is_rejected_before_topology_claim(self) -> None:
        result = analyze_planner_inputs(
            make_grid(), pose(1.5, 1.5, frame="odom"), None
        )
        self.assertEqual(result["diagnosis"], "start_frame_mismatch")

    def test_raw_imu_child_frame_is_rejected(self) -> None:
        result = analyze_planner_inputs(
            make_grid(), pose(1.5, 1.5, child_frame="imu"), None
        )
        self.assertEqual(result["diagnosis"], "start_child_frame_mismatch")

    def test_map_frame_contract_is_enforced(self) -> None:
        grid = replace(make_grid(), frame_id="map")
        result = analyze_planner_inputs(
            grid,
            pose(1.5, 1.5, frame="map"),
            None,
        )
        self.assertEqual(result["diagnosis"], "map_frame_mismatch")

    def test_stale_map_is_rejected(self) -> None:
        grid = replace(make_grid(), stamp_ns=NOW_NS - 1_100_000_000)
        result = analyze_planner_inputs(
            grid,
            pose(1.5, 1.5),
            None,
            now_ns=NOW_NS,
        )
        self.assertEqual(result["diagnosis"], "map_stale")

    def test_stale_odometry_is_rejected(self) -> None:
        result = analyze_planner_inputs(
            make_grid(),
            pose(1.5, 1.5, stamp_ns=NOW_NS - 600_000_000),
            None,
            now_ns=NOW_NS,
        )
        self.assertEqual(result["diagnosis"], "odom_stale")

    def test_stale_goal_is_rejected(self) -> None:
        result = analyze_planner_inputs(
            make_grid(),
            pose(1.5, 1.5),
            pose(2.5, 1.5, stamp_ns=NOW_NS - 2_100_000_000),
            now_ns=NOW_NS,
        )
        self.assertEqual(result["diagnosis"], "goal_stale")

    def test_goal_fresh_at_receipt_is_not_stale_after_capture_delay(self) -> None:
        goal_stamp_ns = NOW_NS - 3_000_000_000
        goal_received_ns = goal_stamp_ns + 100_000_000

        result = analyze_planner_inputs(
            make_grid(),
            pose(1.5, 1.5),
            pose(2.5, 1.5, stamp_ns=goal_stamp_ns),
            now_ns=NOW_NS,
            goal_received_ns=goal_received_ns,
            goal_received_monotonic_ns=1_000_000_000,
            inspection_monotonic_ns=3_900_000_000,
        )

        self.assertEqual(
            result["diagnosis"],
            "same_continuous_ground_component_not_planner_approval",
        )
        freshness = result["freshness"]
        self.assertAlmostEqual(freshness["goal_age_sec"], 3.0)
        self.assertAlmostEqual(
            freshness["goal_age_at_inspection_sec"], 3.0
        )
        self.assertAlmostEqual(
            freshness["goal_header_age_at_receipt_sec"], 0.1
        )
        self.assertAlmostEqual(freshness["goal_retention_age_sec"], 2.9)
        self.assertEqual(freshness["goal_retention_clock"], "steady_monotonic")
        self.assertEqual(
            freshness["goal_receipt_scope"],
            "inspector_subscription_callback",
        )

    def test_goal_retention_expiry_is_distinct_from_header_freshness(self) -> None:
        goal_stamp_ns = NOW_NS - 3_000_000_000
        goal_received_ns = goal_stamp_ns + 100_000_000

        result = analyze_planner_inputs(
            make_grid(),
            pose(1.5, 1.5),
            pose(2.5, 1.5, stamp_ns=goal_stamp_ns),
            contract=InputContract(goal_retention_timeout=2.0),
            now_ns=NOW_NS,
            goal_received_ns=goal_received_ns,
            goal_received_monotonic_ns=1_000_000_000,
            inspection_monotonic_ns=3_900_000_000,
        )

        self.assertEqual(result["diagnosis"], "goal_retention_expired")
        freshness = result["freshness"]
        self.assertAlmostEqual(
            freshness["goal_header_age_at_receipt_sec"], 0.1
        )
        self.assertAlmostEqual(freshness["goal_retention_age_sec"], 2.9)

    def test_goal_retention_expiry_precedes_stale_map_diagnosis(self) -> None:
        result = analyze_planner_inputs(
            replace(make_grid(), stamp_ns=NOW_NS - 2_000_000_000),
            pose(1.5, 1.5),
            pose(2.5, 1.5),
            contract=InputContract(goal_retention_timeout=2.0),
            now_ns=NOW_NS,
            goal_received_ns=NOW_NS,
            goal_received_monotonic_ns=1_000_000_000,
            inspection_monotonic_ns=3_100_000_000,
        )

        self.assertEqual(result["diagnosis"], "goal_retention_expired")

    def test_invalid_goal_retention_contract_is_rejected(self) -> None:
        with self.assertRaisesRegex(
            ValueError, "goal_retention_timeout must be finite"
        ):
            analyze_planner_inputs(
                make_grid(),
                pose(1.5, 1.5),
                None,
                contract=InputContract(goal_retention_timeout=0.0),
            )

    def test_future_stamp_is_rejected(self) -> None:
        grid = replace(make_grid(), stamp_ns=NOW_NS + 300_000_000)
        result = analyze_planner_inputs(
            grid,
            pose(1.5, 1.5),
            None,
            now_ns=NOW_NS,
        )
        self.assertEqual(result["diagnosis"], "map_stamp_from_future")

    def test_custom_contract_thresholds_are_used(self) -> None:
        contract = InputContract(max_map_age=0.1)
        grid = replace(make_grid(), stamp_ns=NOW_NS - 200_000_000)
        result = analyze_planner_inputs(
            grid,
            pose(1.5, 1.5),
            None,
            contract=contract,
            now_ns=NOW_NS,
        )
        self.assertEqual(result["diagnosis"], "map_stale")

    def test_malformed_map_is_rejected(self) -> None:
        grid = replace(make_grid(), slope=[0.0])
        with self.assertRaisesRegex(ValueError, "slope length"):
            analyze_planner_inputs(grid, pose(1.5, 1.5), None)

    def test_malformed_diagnostic_layers_are_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "variance length"):
            analyze_planner_inputs(
                replace(make_grid(), variance=[0.0]),
                pose(1.5, 1.5),
                None,
            )
        with self.assertRaisesRegex(ValueError, "roughness length"):
            analyze_planner_inputs(
                replace(make_grid(), roughness=[0.0]),
                pose(1.5, 1.5),
                None,
            )
        with self.assertRaisesRegex(ValueError, "confidence length"):
            analyze_planner_inputs(
                replace(make_grid(), confidence=[1.0]),
                pose(1.5, 1.5),
                None,
            )
        with self.assertRaisesRegex(ValueError, "age length"):
            analyze_planner_inputs(
                replace(make_grid(), age=[0.0]),
                pose(1.5, 1.5),
                None,
            )
        with self.assertRaisesRegex(ValueError, "observation_count length"):
            analyze_planner_inputs(
                replace(make_grid(), observation_count=[4]),
                pose(1.5, 1.5),
                None,
            )

    def test_nonfinite_goal_position_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "goal x must be finite"):
            analyze_planner_inputs(
                make_grid(),
                pose(1.5, 1.5),
                pose(float("inf"), 1.5),
            )


class SubscriptionLifecycleTests(unittest.TestCase):
    def test_goal_callback_records_receipt_time(self) -> None:
        node = FakeNode()
        goal = stamped_message(1_200_000_000)
        receipt_ns = 1_250_000_000

        def wait_fn(
            fake_node: object,
            predicate: object,
            timeout: float,
        ) -> bool:
            del fake_node, timeout
            node.clock_ns = receipt_ns
            node.publish("/goal_pose", goal)
            node.clock_ns = receipt_ns + 500_000_000
            return predicate()

        with redirect_stderr(StringIO()):
            captured, failure = _capture_goal_stage(
                node,
                make_specs()["goal"],
                publisher_timeout=1.0,
                goal_timeout=1.0,
                wait_fn=wait_fn,
                monotonic_now_ns=lambda: 9_000_000_000,
            )

        self.assertIsNone(failure)
        self.assertIsNotNone(captured)
        self.assertIs(captured["goal"], goal)
        self.assertEqual(captured["goal_received_ns"], receipt_ns)
        self.assertEqual(
            captured["goal_received_monotonic_ns"], 9_000_000_000
        )
        self.assertEqual(node.active, [])

    def test_goal_wait_starts_after_publisher_discovery(self) -> None:
        class DelayedPublisherNode(FakeNode):
            def __init__(self) -> None:
                super().__init__(publisher_count=0)
                self.publisher_queries = 0

            def count_publishers(self, topic: str) -> int:
                del topic
                self.publisher_queries += 1
                return 0 if self.publisher_queries == 1 else 1

        node = DelayedPublisherNode()
        initial_map = stamped_message(1_000_000_000)
        initial_odom = stamped_message(1_100_000_000)
        goal = stamped_message(1_200_000_000)
        fresh_map = stamped_message(1_500_000_000)
        fresh_odom = stamped_message(1_600_000_000)
        stage_payloads = [
            {
                "/terrain_map": [initial_map],
                "/lio/body_odom": [initial_odom],
            },
            {},
            {"/goal_pose": [goal]},
            {
                "/terrain_map": [fresh_map],
                "/lio/body_odom": [fresh_odom],
            },
        ]
        stderr = StringIO()
        wait_calls = 0

        def wait_fn(
            fake_node: object,
            predicate: object,
            timeout: float,
        ) -> bool:
            nonlocal wait_calls
            del fake_node, timeout
            wait_calls += 1
            if wait_calls == 2:
                self.assertNotIn("Goal listener ready", stderr.getvalue())
            if wait_calls == 3:
                self.assertIn("Goal listener ready", stderr.getvalue())
            payloads = stage_payloads.pop(0)
            for topic, messages in payloads.items():
                for message in messages:
                    node.publish(topic, message)
            return predicate()

        with redirect_stderr(stderr):
            captured = _capture_runtime_messages(
                node,
                make_specs(),
                wait_for_goal=True,
                input_timeout=1.0,
                goal_timeout=1.0,
                goal_discovery_timeout=1.0,
                wait_fn=wait_fn,
            )

        self.assertIs(captured["goal"], goal)
        self.assertIs(captured["map"], fresh_map)
        self.assertIs(captured["odom"], fresh_odom)
        self.assertGreaterEqual(node.publisher_queries, 3)
        self.assertEqual(stage_payloads, [])
        self.assertEqual(node.active, [])

    def test_goal_wait_has_only_goal_subscription_and_recaptures_inputs(self) -> None:
        node = FakeNode()
        initial_map = stamped_message(1_000_000_000)
        initial_odom = stamped_message(1_100_000_000)
        goal = stamped_message(1_200_000_000)
        fresh_map = stamped_message(1_500_000_000)
        fresh_odom = stamped_message(1_600_000_000)
        stage_payloads = [
            {
                "/terrain_map": [initial_map],
                "/lio/body_odom": [initial_odom],
            },
            {"/goal_pose": [goal]},
            {
                "/terrain_map": [initial_map, fresh_map],
                "/lio/body_odom": [initial_odom, fresh_odom],
            },
        ]
        active_topics: list[set[str]] = []
        stderr = StringIO()

        def wait_fn(
            fake_node: object,
            predicate: object,
            timeout: float,
        ) -> bool:
            del timeout
            self.assertIs(fake_node, node)
            topics = {subscription.topic for subscription in node.active}
            active_topics.append(topics)
            if topics == {"/goal_pose"}:
                self.assertIn("Goal listener ready", stderr.getvalue())
            payloads = stage_payloads.pop(0)
            for topic, messages in payloads.items():
                for message in messages:
                    node.publish(topic, message)
            return predicate()

        with redirect_stderr(stderr):
            captured = _capture_runtime_messages(
                node,
                make_specs(),
                wait_for_goal=True,
                input_timeout=1.0,
                goal_timeout=1.0,
                wait_fn=wait_fn,
            )

        self.assertIs(captured["map"], fresh_map)
        self.assertIs(captured["odom"], fresh_odom)
        self.assertIs(captured["goal"], goal)
        self.assertEqual(
            active_topics,
            [
                {"/terrain_map", "/lio/body_odom"},
                {"/goal_pose"},
                {"/terrain_map", "/lio/body_odom"},
            ],
        )
        self.assertEqual(node.active, [])
        self.assertEqual(
            node.events,
            [
                ("create", "/terrain_map", "latched"),
                ("create", "/lio/body_odom", "sensor"),
                ("destroy", "/terrain_map", "latched"),
                ("destroy", "/lio/body_odom", "sensor"),
                ("create", "/goal_pose", "goal"),
                ("destroy", "/goal_pose", "goal"),
                ("create", "/terrain_map", "live"),
                ("create", "/lio/body_odom", "sensor"),
                ("destroy", "/terrain_map", "live"),
                ("destroy", "/lio/body_odom", "sensor"),
            ],
        )

    def test_no_goal_capture_uses_one_live_input_stage(self) -> None:
        node = FakeNode()
        live_map = stamped_message(2_000_000_000)
        live_odom = stamped_message(2_100_000_000)
        active_topics: list[set[str]] = []

        def wait_fn(
            fake_node: object,
            predicate: object,
            timeout: float,
        ) -> bool:
            del fake_node, timeout
            active_topics.append({subscription.topic for subscription in node.active})
            node.publish("/terrain_map", live_map)
            node.publish("/lio/body_odom", live_odom)
            return predicate()

        captured = _capture_runtime_messages(
            node,
            make_specs(),
            wait_for_goal=False,
            input_timeout=1.0,
            goal_timeout=1.0,
            wait_fn=wait_fn,
        )

        self.assertIs(captured["map"], live_map)
        self.assertIs(captured["odom"], live_odom)
        self.assertEqual(active_topics, [{"/terrain_map", "/lio/body_odom"}])
        self.assertEqual(node.active, [])
        self.assertIn(("create", "/terrain_map", "live"), node.events)

    def test_goal_timeout_can_recapture_fresh_start_inputs(self) -> None:
        node = FakeNode()
        initial_map = stamped_message(1_000_000_000)
        initial_odom = stamped_message(1_100_000_000)
        fresh_map = stamped_message(1_500_000_000)
        fresh_odom = stamped_message(1_600_000_000)
        stage_payloads = [
            {
                "/terrain_map": [initial_map],
                "/lio/body_odom": [initial_odom],
            },
            {},
            {
                "/terrain_map": [initial_map, fresh_map],
                "/lio/body_odom": [initial_odom, fresh_odom],
            },
        ]
        active_topics: list[set[str]] = []

        def wait_fn(
            fake_node: object,
            predicate: object,
            timeout: float,
        ) -> bool:
            del timeout
            self.assertIs(fake_node, node)
            active_topics.append({subscription.topic for subscription in node.active})
            payloads = stage_payloads.pop(0)
            for topic, messages in payloads.items():
                for message in messages:
                    node.publish(topic, message)
            return predicate()

        captured = _capture_runtime_messages(
            node,
            make_specs(),
            wait_for_goal=True,
            input_timeout=1.0,
            goal_timeout=1.0,
            record_start_on_goal_timeout=True,
            wait_fn=wait_fn,
        )

        self.assertIs(captured["map"], fresh_map)
        self.assertIs(captured["odom"], fresh_odom)
        self.assertNotIn("goal", captured)
        self.assertEqual(captured["goal_capture_failure"], "goal_wait_timeout")
        self.assertEqual(
            active_topics,
            [
                {"/terrain_map", "/lio/body_odom"},
                {"/goal_pose"},
                {"/terrain_map", "/lio/body_odom"},
            ],
        )
        self.assertEqual(stage_payloads, [])
        self.assertEqual(node.active, [])

    def test_goal_publisher_discovery_timeout_recaptures_start_inputs(self) -> None:
        node = FakeNode(publisher_count=0)
        initial_map = stamped_message(1_000_000_000)
        initial_odom = stamped_message(1_100_000_000)
        fresh_map = stamped_message(1_500_000_000)
        fresh_odom = stamped_message(1_600_000_000)
        stage_payloads = [
            {
                "/terrain_map": [initial_map],
                "/lio/body_odom": [initial_odom],
            },
            {},
            {
                "/terrain_map": [fresh_map],
                "/lio/body_odom": [fresh_odom],
            },
        ]

        def wait_fn(
            fake_node: object,
            predicate: object,
            timeout: float,
        ) -> bool:
            del fake_node, timeout
            payloads = stage_payloads.pop(0)
            for topic, messages in payloads.items():
                for message in messages:
                    node.publish(topic, message)
            return predicate()

        captured = _capture_runtime_messages(
            node,
            make_specs(),
            wait_for_goal=True,
            input_timeout=1.0,
            goal_timeout=1.0,
            goal_discovery_timeout=1.0,
            record_start_on_goal_timeout=True,
            wait_fn=wait_fn,
        )

        self.assertIs(captured["map"], fresh_map)
        self.assertIs(captured["odom"], fresh_odom)
        self.assertEqual(
            captured["goal_capture_failure"],
            "goal_publisher_discovery_timeout",
        )
        self.assertEqual(stage_payloads, [])
        self.assertEqual(node.active, [])

    def test_timeout_destroys_every_subscription(self) -> None:
        node = FakeNode()
        captured = _capture_stage(
            node,
            (make_specs()["live_map"], make_specs()["odom"]),
            0.0,
            wait_fn=lambda unused_node, predicate, timeout: False,
        )
        self.assertIsNone(captured)
        self.assertEqual(node.active, [])

    def test_partial_creation_failure_destroys_first_subscription(self) -> None:
        node = FakeNode(fail_topic="/lio/body_odom")
        with self.assertRaisesRegex(RuntimeError, "creation failed"):
            _capture_stage(
                node,
                (make_specs()["live_map"], make_specs()["odom"]),
                1.0,
                wait_fn=lambda unused_node, predicate, timeout: False,
            )
        self.assertEqual(node.active, [])
        self.assertEqual(
            node.events,
            [
                ("create", "/terrain_map", "live"),
                ("destroy", "/terrain_map", "live"),
            ],
        )

    def test_wait_exception_destroys_every_subscription(self) -> None:
        node = FakeNode()

        def wait_fn(
            unused_node: object,
            predicate: object,
            timeout: float,
        ) -> bool:
            del unused_node, predicate, timeout
            raise KeyboardInterrupt

        with self.assertRaises(KeyboardInterrupt):
            _capture_stage(
                node,
                (make_specs()["live_map"], make_specs()["odom"]),
                1.0,
                wait_fn=wait_fn,
            )
        self.assertEqual(node.active, [])


if __name__ == "__main__":
    unittest.main()
