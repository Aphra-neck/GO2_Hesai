#!/usr/bin/env python3

from __future__ import annotations

from contextlib import redirect_stderr
from io import StringIO
import json
import os
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

from inspect_planner_inputs import (
    GridSnapshot,
    InputContract,
    PlannerThresholds,
    Pose2D,
    SubscriptionSpec,
    _diagnosis_exit_code,
    _capture_runtime_messages,
    _capture_stage,
    _write_session_record_atomic,
    _grid_snapshot,
    _mark_goal_capture_failure,
    _terrain_layer_stats,
    analyze_planner_inputs,
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
        slope=[0.0] * cells,
        roughness=[0.0] * cells,
        traversability=[1.0] * cells,
        observation_count=[4] * cells,
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

    def publish(self, topic: str, message: object) -> None:
        for subscription in list(self.active):
            if subscription.topic == topic:
                subscription.callback(message)


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
        self.assertEqual(global_stats["observation_zero"], 1)
        self.assertEqual(global_stats["observation_below_min"], 1)
        self.assertEqual(start_stats["cell_count"], 4)
        self.assertEqual(start_stats["observation_zero"], 1)
        self.assertEqual(start_stats["observation_below_min"], 0)
        self.assertEqual(goal_stats["cell_count"], 4)
        self.assertEqual(goal_stats["observation_zero"], 0)
        self.assertEqual(goal_stats["observation_below_min"], 1)

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
            slope=[0.1],
            roughness=[0.02],
            traversability=[0.7],
            observation_count=[6],
        )
        snapshot = _grid_snapshot(message)
        self.assertEqual(snapshot.roughness, [0.02])
        self.assertEqual(snapshot.observation_count, [6])

    def test_exit_codes_distinguish_ready_from_rejected_diagnoses(self) -> None:
        self.assertEqual(
            _diagnosis_exit_code(
                "same_continuous_ground_component_not_planner_approval"
            ),
            0,
        )
        self.assertEqual(_diagnosis_exit_code("start_ready_waiting_for_goal"), 0)
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
            result["diagnosis"], "start_has_no_valid_cell_in_snap_square"
        )
        self.assertEqual(result["start"]["valid_cells_in_snap_square"], 0)

    def test_goal_outside_map_is_explicit(self) -> None:
        result = analyze_planner_inputs(
            make_grid(), pose(1.5, 1.5), pose(12.0, 1.5)
        )
        self.assertEqual(result["diagnosis"], "goal_outside_map")

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

    def test_unknown_endpoint_elevation_is_not_called_ground(self) -> None:
        grid = replace(make_grid(width=1, height=1), elevation=[UNKNOWN])
        result = analyze_planner_inputs(grid, pose(0.5, 0.5), None)
        self.assertTrue(result["start"]["exact_cell_valid"])
        self.assertEqual(result["map"]["valid_cells"], 1)
        self.assertEqual(result["map"]["continuous_ground_cells"], 0)
        self.assertEqual(
            result["diagnosis"],
            "start_elevation_invalid_for_ground_topology",
        )

    def test_square_snap_can_exceed_nominal_euclidean_radius(self) -> None:
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
        self.assertAlmostEqual(
            result["start"]["snap_grid_distance_m"], 2.0**0.5
        )
        self.assertAlmostEqual(
            result["start"]["snap_world_to_center_distance_m"], 2.0**0.5
        )
        self.assertTrue(
            result["start"]["snap_grid_distance_outside_nominal_radius"]
        )

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
        with self.assertRaisesRegex(ValueError, "roughness length"):
            analyze_planner_inputs(
                replace(make_grid(), roughness=[0.0]),
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
