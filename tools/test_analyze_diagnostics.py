#!/usr/bin/env python3

from __future__ import annotations

import copy
import csv
import json
import math
import os
import tempfile
import unittest
from pathlib import Path

from analyze_diagnostics import (
    generate_report,
    read_jsonl,
    summarize_planner_inspections,
    summarize_planner_series,
    summarize_topic_timing,
)
from inspect_planner_inputs import (
    GridSnapshot,
    Pose2D,
    _write_session_record_atomic,
    analyze_planner_inputs,
)


def layer_stats(seed: int, planner_valid: int = 30) -> dict[str, int]:
    return {
        "cell_count": 100 + seed,
        "observation_zero": 10,
        "observation_below_min": 20 + seed,
        "observation_ready": 70,
        "elevation_known": 60,
        "elevation_unknown_or_nonfinite": 40 + seed,
        "elevation_known_below_min_observations": 2,
        "slope_known": 50,
        "slope_unknown_or_nonfinite": 50 + seed,
        "roughness_known": 60,
        "roughness_unknown_or_nonfinite": 40 + seed,
        "traversability_known": 50,
        "traversability_unknown_or_nonfinite": 50 + seed,
        "features_known": 50,
        "slope_over_limit": 5,
        "slope_at_or_above_mapper_limit": 4,
        "roughness_over_limit": 6,
        "traversability_zero": 7,
        "traversability_below_min_nonzero": 8,
        "traversability_at_or_above_min": 35,
        "zero_traversability_with_slope_and_roughness_below_limits": 3,
        "planner_valid": planner_valid,
    }


def add_slope_quantiles(
    stats: dict[str, object],
    *,
    minimum: float,
) -> None:
    stats.update(
        {
            "slope_min_rad": minimum,
            "slope_p50_rad": minimum + 0.1,
            "slope_p90_rad": minimum + 0.2,
            "slope_p95_rad": minimum + 0.3,
            "slope_max_rad": minimum + 0.4,
        }
    )


def series_record(
    index: int,
    *,
    recorded_second: int,
    planner_valid: int,
    slope_over: int,
    slope_minimum: float,
    start_component_cells: int,
    goal_connected: bool | None,
) -> dict[str, object]:
    record = copy.deepcopy(inspection_record())
    record["recorded_at"] = (
        f"2026-08-06T00:00:{recorded_second:02d}+00:00"
    )
    inspection = record["inspection"]
    assert isinstance(inspection, dict)
    inspection["series"] = {
        "id": "slope-run-a",
        "index": index,
        "count": 3,
        "interval_sec": 2.0,
    }
    inspection["start_component_cells"] = start_component_cells
    inspection["start_component_percent_of_ground"] = (
        100.0 * start_component_cells / planner_valid
    )
    inspection["goal_in_start_component"] = goal_connected

    map_data = inspection["map"]
    map_layers = map_data["terrain_layers"]
    map_layers["planner_valid"] = planner_valid
    map_layers["slope_over_limit"] = slope_over
    map_data["valid_cells"] = planner_valid
    map_data["continuous_ground_cells"] = planner_valid
    map_data["valid_percent"] = float(planner_valid)
    map_data["continuous_ground_percent"] = float(planner_valid)
    add_slope_quantiles(map_layers, minimum=slope_minimum)
    for endpoint_data in (inspection["start"], inspection["goal"]):
        for key in (
            "snap_square_terrain_layers",
            "snap_radius_terrain_layers",
        ):
            endpoint_layers = endpoint_data[key]
            add_slope_quantiles(
                endpoint_layers,
                minimum=slope_minimum + 0.01,
            )
    return record


def endpoint(
    scope: str,
    seed: int,
    *,
    valid_cells: int,
    inside: bool = True,
) -> dict[str, object]:
    snapped = valid_cells > 0
    return {
        "frame_id": "world",
        "child_frame_id": "base_link" if scope == "start" else "",
        "stamp_ns": 1_000_000_000 + seed,
        "world_x": 0.1 + seed,
        "world_y": 0.2 + seed,
        "yaw_rad": 0.0,
        "snap_radius_m": 0.55 if scope == "start" else 0.5,
        "grid_x": 4 + seed,
        "grid_y": 5 + seed,
        "inside_map": inside,
        "exact_cell_valid": False,
        "valid_cells_in_snap_square": valid_cells if inside else 0,
        "valid_cells_in_snap_radius": valid_cells if inside else 0,
        "snapped": snapped if inside else False,
        "snapped_grid_x": 4 + seed if snapped and inside else None,
        "snapped_grid_y": 5 + seed if snapped and inside else None,
        "snapped_world_x": 0.15 + seed if snapped and inside else None,
        "snapped_world_y": 0.25 + seed if snapped and inside else None,
        "snap_grid_distance_m": 0.05 if snapped and inside else None,
        "snap_world_to_center_distance_m": 0.07 if snapped and inside else None,
        "snap_grid_distance_outside_nominal_radius": False,
        "snap_square_terrain_layers": (
            layer_stats(seed, valid_cells) if inside else None
        ),
        "snap_radius_terrain_layers": (
            layer_stats(seed, valid_cells) if inside else None
        ),
    }


def inspection_record() -> dict[str, object]:
    return {
        "schema_version": 1,
        "recorded_at": "2026-08-06T00:00:00+00:00",
        "exit_code": 2,
        "status": "diagnostic_failure",
        "inspection": {
            "diagnosis": "start_has_no_valid_cell_in_snap_square",
            "map": {
                "frame_id": "world",
                "stamp_ns": 1_000_000_100,
                "width": 10,
                "height": 10,
                "resolution": 0.05,
                "origin_x": -0.25,
                "origin_y": -0.25,
                "total_cells": 100,
                "known_cells": 50,
                "planner_gate_known_cells": 50,
                "valid_cells": 30,
                "continuous_ground_cells": 25,
                "known_percent": 50.0,
                "valid_percent": 30.0,
                "continuous_ground_percent": 25.0,
                "terrain_layers": layer_stats(0),
            },
            "thresholds": {
                "min_observed_frames": 4,
                "min_traversability": 0.18,
                "max_slope": 0.65,
                "mapper_max_slope": 0.65,
                "max_roughness": 0.08,
                "max_step_height": 0.24,
                "start_snap_radius": 0.55,
                "snap_radius": 0.5,
            },
            "contract": {
                "map_frame": "world",
                "body_frame": "base_link",
                "max_map_age": 1.0,
                "max_odom_age": 0.5,
                "max_goal_age": 2.0,
                "future_tolerance": 0.2,
            },
            "freshness": {
                "evaluated": True,
                "map_age_sec": 0.2,
                "odom_age_sec": 0.1,
                "goal_age_sec": 0.0,
                "map_odom_stamp_delta_sec": 0.05,
            },
            "start": endpoint("start", 1, valid_cells=0),
            "goal": endpoint("goal", 2, valid_cells=2),
            "start_component_cells": 0,
            "start_component_percent_of_ground": 0.0,
            "goal_in_start_component": None,
            "limitations": "read-only topology diagnostic",
        },
    }


def capture_error_record() -> dict[str, object]:
    return {
        "schema_version": 1,
        "recorded_at": "2026-08-06T00:00:01+00:00",
        "exit_code": 1,
        "status": "capture_error",
        "error": {"type": "timeout", "message": "no terrain map"},
    }


def no_goal_record() -> dict[str, object]:
    record = copy.deepcopy(inspection_record())
    record["exit_code"] = 0
    record["status"] = "ok"
    inspection = record["inspection"]
    assert isinstance(inspection, dict)
    inspection["diagnosis"] = "start_ready_waiting_for_goal"
    inspection["start"] = endpoint("start", 1, valid_cells=3)
    inspection["goal"] = None
    inspection["freshness"]["goal_age_sec"] = None
    inspection["start_component_cells"] = 20
    inspection["start_component_percent_of_ground"] = 80.0
    return record


class PlannerInspectionAnalysisTests(unittest.TestCase):
    @unittest.skipUnless(os.name == "posix", "atomic writer uses POSIX files")
    def test_real_inspector_record_is_accepted_end_to_end(self) -> None:
        grid = GridSnapshot(
            frame_id="world",
            stamp_ns=2_000_000_000,
            resolution=1.0,
            width=3,
            height=3,
            origin_x=0.0,
            origin_y=0.0,
            unknown_value=-999.0,
            elevation=[0.0] * 9,
            slope=[0.0] * 9,
            roughness=[0.0] * 9,
            traversability=[1.0] * 9,
            observation_count=[4] * 9,
        )
        start = Pose2D(
            x=1.5,
            y=1.5,
            yaw=0.0,
            frame_id="world",
            stamp_ns=2_000_000_000,
            child_frame_id="base_link",
        )
        inspection = analyze_planner_inputs(
            grid,
            start,
            None,
            now_ns=2_000_000_000,
        )

        with tempfile.TemporaryDirectory() as directory:
            session = Path(directory) / "sessions" / "real-inspector"
            output = session / "analysis"
            session.mkdir(parents=True)
            _write_session_record_atomic(
                session / "planner_input_inspections.jsonl",
                Path(directory),
                0,
                inspection=inspection,
                recorded_at="2026-08-06T00:00:00+00:00",
            )
            (session / "ros_dds_environment.txt").write_text(
                "GO2_LIO_DENSE_OUTPUT=false\n",
                encoding="utf-8",
            )
            products = generate_report(session, output)
            report = products[0].read_text(encoding="utf-8")
            with products[5].open(encoding="utf-8", newline="") as stream:
                rows = list(csv.DictReader(stream))

        self.assertEqual([row["scope"] for row in rows], ["map", "start"])
        self.assertIn("Accepted records: 1", report)
        self.assertIn("start_ready_waiting_for_goal", report)
        self.assertIn("| LIO dense cloud output | false |", report)

    def test_completed_inspection_expands_to_map_start_and_goal_rows(self) -> None:
        summary = summarize_planner_inspections([inspection_record()])
        self.assertEqual(summary["schema_invalid"], 0)
        self.assertEqual(summary["capture_errors"], 0)
        self.assertEqual(len(summary["rows"]), 3)
        self.assertEqual(
            [row["scope"] for row in summary["rows"]],
            ["map", "start", "goal"],
        )
        self.assertEqual(summary["rows"][1]["observation_below_min"], 21)
        self.assertEqual(summary["rows"][1]["hard_reject_candidate"], 3)
        self.assertEqual(summary["rows"][0]["max_step_height"], 0.24)
        self.assertEqual(summary["rows"][0]["start_snap_radius"], 0.55)
        self.assertEqual(summary["rows"][0]["snap_radius"], 0.5)
        self.assertEqual(summary["rows"][1]["endpoint_snap_radius"], 0.55)
        self.assertEqual(summary["rows"][0]["mapper_max_slope"], 0.65)
        self.assertEqual(
            summary["rows"][0]["slope_at_or_above_mapper_limit"], 4
        )
        self.assertEqual(summary["rows"][0]["map_odom_stamp_delta_sec"], 0.05)
        self.assertEqual(summary["rows"][0]["map_planner_gate_known_cells"], 50)
        self.assertEqual(summary["rows"][0]["map_continuous_ground_cells"], 25)
        self.assertEqual(summary["rows"][0]["slope_p95_rad"], "")
        self.assertEqual(
            summary["diagnoses"]["start_has_no_valid_cell_in_snap_square"],
            1,
        )

    def test_radius_fields_allow_square_only_candidate_without_snap(self) -> None:
        record = inspection_record()
        inspection = record["inspection"]
        start = inspection["start"]
        start["valid_cells_in_snap_square"] = 1
        start["snap_square_terrain_layers"] = layer_stats(1, 1)
        inspection["diagnosis"] = "start_has_no_valid_cell_in_snap_radius"

        summary = summarize_planner_inspections([record])

        self.assertEqual(summary["schema_invalid"], 0)
        self.assertEqual(len(summary["completed_records"]), 1)
        self.assertEqual(summary["rows"][1]["valid_cells_in_snap_square"], 1)
        self.assertEqual(summary["rows"][1]["valid_cells_in_snap_radius"], 0)

    def test_previous_schema_without_endpoint_radii_remains_readable(self) -> None:
        record = inspection_record()
        inspection = record["inspection"]
        inspection["thresholds"].pop("start_snap_radius")
        for endpoint_data in (inspection["start"], inspection["goal"]):
            endpoint_data.pop("snap_radius_m")

        summary = summarize_planner_inspections([record])

        self.assertEqual(summary["schema_invalid"], 0)
        self.assertEqual(len(summary["completed_records"]), 1)

    def test_legacy_endpoints_without_radius_fields_remain_readable(self) -> None:
        record = inspection_record()
        inspection = record["inspection"]
        inspection["thresholds"].pop("start_snap_radius")
        for endpoint_data in (inspection["start"], inspection["goal"]):
            endpoint_data.pop("snap_radius_m")
            endpoint_data.pop("valid_cells_in_snap_radius")
            endpoint_data.pop("snap_radius_terrain_layers")

        summary = summarize_planner_inspections([record])

        self.assertEqual(summary["schema_invalid"], 0)
        self.assertEqual(len(summary["completed_records"]), 1)

    def test_world_distance_snap_may_cross_integer_grid_radius(self) -> None:
        record = inspection_record()
        goal = record["inspection"]["goal"]
        goal["snap_grid_distance_m"] = 0.6
        goal["snap_world_to_center_distance_m"] = 0.49
        goal["snap_grid_distance_outside_nominal_radius"] = True

        summary = summarize_planner_inspections([record])

        self.assertEqual(summary["schema_invalid"], 0)
        self.assertEqual(len(summary["completed_records"]), 1)

    def test_grid_radius_flag_must_match_snapped_distance(self) -> None:
        record = inspection_record()
        goal = record["inspection"]["goal"]
        goal["snap_grid_distance_m"] = 0.6

        summary = summarize_planner_inspections([record])

        self.assertEqual(summary["schema_invalid"], 1)
        self.assertEqual(len(summary["completed_records"]), 0)

    def test_unsnapped_endpoint_cannot_set_grid_radius_flag(self) -> None:
        record = inspection_record()
        start = record["inspection"]["start"]
        start["snap_grid_distance_outside_nominal_radius"] = True

        summary = summarize_planner_inspections([record])

        self.assertEqual(summary["schema_invalid"], 1)
        self.assertEqual(len(summary["completed_records"]), 0)

    def test_snap_outside_endpoint_world_radius_is_rejected(self) -> None:
        record = inspection_record()
        goal = record["inspection"]["goal"]
        goal["snap_world_to_center_distance_m"] = 0.51

        summary = summarize_planner_inspections([record])

        self.assertEqual(summary["schema_invalid"], 1)
        self.assertEqual(len(summary["completed_records"]), 0)

    def test_non_numeric_endpoint_snap_radius_is_rejected(self) -> None:
        record = inspection_record()
        record["inspection"]["goal"]["snap_radius_m"] = "0.55"

        summary = summarize_planner_inspections([record])

        self.assertEqual(summary["schema_invalid"], 1)
        self.assertEqual(len(summary["completed_records"]), 0)

    def test_non_numeric_endpoint_world_distance_is_rejected(self) -> None:
        record = inspection_record()
        record["inspection"]["goal"][
            "snap_world_to_center_distance_m"
        ] = "0.1"

        summary = summarize_planner_inspections([record])

        self.assertEqual(summary["schema_invalid"], 1)
        self.assertEqual(len(summary["completed_records"]), 0)

    def test_endpoint_radius_must_match_its_threshold(self) -> None:
        record = inspection_record()
        record["inspection"]["goal"]["snap_radius_m"] = 0.55

        summary = summarize_planner_inspections([record])

        self.assertEqual(summary["schema_invalid"], 1)
        self.assertEqual(len(summary["completed_records"]), 0)

    def test_exact_valid_cell_may_exceed_snap_radius(self) -> None:
        record = inspection_record()
        goal = record["inspection"]["goal"]
        goal["exact_cell_valid"] = True
        goal["snap_grid_distance_m"] = 0.0
        goal["snap_world_to_center_distance_m"] = 0.51

        summary = summarize_planner_inspections([record])

        self.assertEqual(summary["schema_invalid"], 0)
        self.assertEqual(len(summary["completed_records"]), 1)

    def test_optional_slope_quantiles_and_series_metadata_are_accepted(self) -> None:
        record = series_record(
            1,
            recorded_second=0,
            planner_valid=20,
            slope_over=4,
            slope_minimum=0.1,
            start_component_cells=10,
            goal_connected=True,
        )

        summary = summarize_planner_inspections([record])

        self.assertEqual(summary["schema_invalid"], 0)
        self.assertEqual(summary["rows"][0]["series_id"], "slope-run-a")
        self.assertEqual(summary["rows"][0]["series_index"], 1)
        self.assertAlmostEqual(summary["rows"][0]["slope_p90_rad"], 0.3)

    def test_partial_or_nonmonotonic_slope_quantiles_are_rejected(self) -> None:
        partial = copy.deepcopy(inspection_record())
        partial_layers = partial["inspection"]["map"]["terrain_layers"]
        partial_layers["slope_min_rad"] = 0.1

        nonmonotonic = series_record(
            1,
            recorded_second=0,
            planner_valid=20,
            slope_over=4,
            slope_minimum=0.1,
            start_component_cells=10,
            goal_connected=True,
        )
        nonmonotonic_layers = nonmonotonic["inspection"]["map"][
            "terrain_layers"
        ]
        nonmonotonic_layers["slope_p95_rad"] = 0.05

        summary = summarize_planner_inspections([partial, nonmonotonic])

        self.assertEqual(summary["schema_invalid"], 2)
        self.assertEqual(summary["rows"], [])

    def test_series_summary_aggregates_each_scope_without_bridging_null_flips(
        self,
    ) -> None:
        records = [
            series_record(
                1,
                recorded_second=0,
                planner_valid=10,
                slope_over=3,
                slope_minimum=0.1,
                start_component_cells=5,
                goal_connected=True,
            ),
            series_record(
                2,
                recorded_second=2,
                planner_valid=20,
                slope_over=6,
                slope_minimum=0.2,
                start_component_cells=15,
                goal_connected=False,
            ),
            series_record(
                3,
                recorded_second=4,
                planner_valid=15,
                slope_over=9,
                slope_minimum=0.3,
                start_component_cells=10,
                goal_connected=None,
            ),
        ]
        inspection_summary = summarize_planner_inspections(records)

        rows = summarize_planner_series(inspection_summary["rows"])

        self.assertEqual([row["scope"] for row in rows], ["map", "start", "goal"])
        map_row = rows[0]
        self.assertEqual(map_row["captured"], 3)
        self.assertEqual(map_row["expected"], 3)
        self.assertEqual(map_row["first_recorded_at"], "2026-08-06T00:00:00+00:00")
        self.assertEqual(map_row["last_recorded_at"], "2026-08-06T00:00:04+00:00")
        self.assertEqual(map_row["duration_sec"], 4.0)
        self.assertEqual(map_row["planner_valid_min"], 10)
        self.assertEqual(map_row["planner_valid_max"], 20)
        self.assertEqual(map_row["planner_valid_mean"], 15.0)
        self.assertEqual(map_row["slope_over_limit_mean"], 6.0)
        self.assertAlmostEqual(map_row["slope_p90_rad_mean"], 0.4)
        self.assertEqual(map_row["start_component_cells_min"], 5)
        self.assertEqual(map_row["start_component_cells_max"], 15)
        self.assertEqual(map_row["start_component_cells_mean"], 10.0)
        self.assertEqual(map_row["goal_connectivity_true"], 1)
        self.assertEqual(map_row["goal_connectivity_false"], 1)
        self.assertEqual(map_row["goal_connectivity_unknown"], 1)
        self.assertEqual(map_row["goal_connectivity_bool_flips"], 1)

    def test_no_goal_inspection_expands_to_map_and_start_rows(self) -> None:
        summary = summarize_planner_inspections([no_goal_record()])
        self.assertEqual(summary["schema_invalid"], 0)
        self.assertEqual(
            [row["scope"] for row in summary["rows"]],
            ["map", "start"],
        )

    def test_goal_timeout_retains_start_map_rows_and_diagnosis(self) -> None:
        record = no_goal_record()
        record["exit_code"] = 2
        record["status"] = "diagnostic_failure"
        inspection = record["inspection"]
        assert isinstance(inspection, dict)
        inspection["diagnosis"] = "goal_wait_timeout"
        inspection["start_map_diagnosis"] = "start_ready_waiting_for_goal"
        inspection["goal_capture_failure"] = "goal_wait_timeout"

        summary = summarize_planner_inspections([record])

        self.assertEqual(summary["schema_invalid"], 0)
        self.assertEqual(
            [row["scope"] for row in summary["rows"]],
            ["map", "start"],
        )
        self.assertEqual(
            summary["rows"][0]["start_map_diagnosis"],
            "start_ready_waiting_for_goal",
        )
        self.assertEqual(summary["diagnoses"]["goal_wait_timeout"], 1)

    def test_outside_endpoint_accepts_explicit_null_layer_stats(self) -> None:
        record = copy.deepcopy(inspection_record())
        inspection = record["inspection"]
        assert isinstance(inspection, dict)
        inspection["diagnosis"] = "start_outside_map"
        inspection["start"] = endpoint(
            "start", 1, valid_cells=0, inside=False
        )
        summary = summarize_planner_inspections([record])
        self.assertEqual(summary["schema_invalid"], 0)
        self.assertEqual(summary["rows"][1]["cell_count"], "")

    def test_frame_mismatch_records_preserve_empty_frame_evidence(self) -> None:
        records: list[dict[str, object]] = []

        empty_map = copy.deepcopy(inspection_record())
        empty_map["inspection"]["diagnosis"] = "map_frame_mismatch"
        empty_map["inspection"]["map"]["frame_id"] = ""
        records.append(empty_map)

        empty_start = copy.deepcopy(inspection_record())
        empty_start["inspection"]["diagnosis"] = "start_frame_mismatch"
        empty_start["inspection"]["start"]["frame_id"] = ""
        records.append(empty_start)

        empty_goal = copy.deepcopy(inspection_record())
        empty_goal["inspection"]["diagnosis"] = "goal_frame_mismatch"
        empty_goal["inspection"]["goal"]["frame_id"] = ""
        records.append(empty_goal)

        summary = summarize_planner_inspections(records)
        self.assertEqual(summary["schema_invalid"], 0)
        self.assertEqual(len(summary["completed_records"]), 3)
        self.assertEqual(summary["diagnoses"]["map_frame_mismatch"], 1)
        self.assertEqual(summary["diagnoses"]["start_frame_mismatch"], 1)
        self.assertEqual(summary["diagnoses"]["goal_frame_mismatch"], 1)

    def test_rejects_invalid_wrapper_contracts(self) -> None:
        invalid_records: list[dict[str, object]] = []

        boolean_version = copy.deepcopy(inspection_record())
        boolean_version["schema_version"] = True
        invalid_records.append(boolean_version)

        boolean_exit = capture_error_record()
        boolean_exit["exit_code"] = True
        invalid_records.append(boolean_exit)

        local_timestamp = copy.deepcopy(inspection_record())
        local_timestamp["recorded_at"] = "2026-08-06T08:00:00+08:00"
        invalid_records.append(local_timestamp)

        missing_timestamp = copy.deepcopy(inspection_record())
        del missing_timestamp["recorded_at"]
        invalid_records.append(missing_timestamp)

        both_payloads = capture_error_record()
        both_payloads["inspection"] = inspection_record()["inspection"]
        invalid_records.append(both_payloads)

        completed_with_error = copy.deepcopy(inspection_record())
        completed_with_error["error"] = {"type": "extra", "message": "extra"}
        invalid_records.append(completed_with_error)

        wrong_status = copy.deepcopy(inspection_record())
        wrong_status["status"] = "ok"
        wrong_status["exit_code"] = 0
        invalid_records.append(wrong_status)

        summary = summarize_planner_inspections(invalid_records)
        self.assertEqual(summary["schema_invalid"], len(invalid_records))
        self.assertEqual(summary["accepted_records"], [])
        self.assertEqual(summary["rows"], [])

    def test_rejects_missing_nonfinite_and_inconsistent_nested_fields(self) -> None:
        invalid_records: list[dict[str, object]] = []

        missing_layer = copy.deepcopy(inspection_record())
        del missing_layer["inspection"]["map"]["terrain_layers"]["observation_ready"]
        invalid_records.append(missing_layer)

        nonfinite_layer = copy.deepcopy(inspection_record())
        nonfinite_layer["inspection"]["map"]["terrain_layers"]["slope_known"] = math.nan
        invalid_records.append(nonfinite_layer)

        inconsistent_layer = copy.deepcopy(inspection_record())
        inconsistent_layer["inspection"]["start"]["snap_square_terrain_layers"][
            "observation_ready"
        ] += 1
        invalid_records.append(inconsistent_layer)

        missing_threshold = copy.deepcopy(inspection_record())
        del missing_threshold["inspection"]["thresholds"]["max_step_height"]
        invalid_records.append(missing_threshold)

        missing_mapper_threshold = copy.deepcopy(inspection_record())
        del missing_mapper_threshold["inspection"]["thresholds"][
            "mapper_max_slope"
        ]
        invalid_records.append(missing_mapper_threshold)

        missing_mapper_layer = copy.deepcopy(inspection_record())
        del missing_mapper_layer["inspection"]["map"]["terrain_layers"][
            "slope_at_or_above_mapper_limit"
        ]
        invalid_records.append(missing_mapper_layer)

        missing_optional_key = copy.deepcopy(inspection_record())
        del missing_optional_key["inspection"]["goal"]
        invalid_records.append(missing_optional_key)

        invalid_topology = copy.deepcopy(inspection_record())
        invalid_topology["inspection"]["goal_in_start_component"] = []
        invalid_records.append(invalid_topology)

        summary = summarize_planner_inspections(invalid_records)
        self.assertEqual(summary["schema_invalid"], len(invalid_records))
        self.assertEqual(summary["rows"], [])

    def test_jsonl_reader_rejects_nonstandard_constants_and_blank_records(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "planner.jsonl"
            path.write_text(
                '{"value":NaN}\n{"value":Infinity}\n{"value":-Infinity}\n\n'
                '{"valid":true}\n',
                encoding="utf-8",
            )
            records, invalid = read_jsonl(path)
        self.assertEqual(records, [{"valid": True}])
        self.assertEqual(invalid, 4)

    def test_latest_capture_error_is_reported_after_completed_inspection(self) -> None:
        summary = summarize_planner_inspections(
            [inspection_record(), capture_error_record()]
        )
        self.assertEqual(summary["latest_attempt"]["status"], "capture_error")
        self.assertEqual(summary["latest"]["status"], "diagnostic_failure")

        with tempfile.TemporaryDirectory() as directory:
            session = Path(directory) / "sessions" / "test-session"
            output = session / "analysis"
            session.mkdir(parents=True)
            (session / "metadata.json").write_text(
                json.dumps({"session_id": "test-session"}),
                encoding="utf-8",
            )
            (session / "planner_input_inspections.jsonl").write_text(
                json.dumps(inspection_record())
                + "\n"
                + json.dumps(capture_error_record())
                + "\n",
                encoding="utf-8",
            )
            products = generate_report(session, output)
            report = products[0].read_text(encoding="utf-8")

        self.assertIn("Latest attempt:", report)
        self.assertIn("status=`capture_error`", report)
        self.assertIn("timeout: no terrain map", report)
        self.assertIn("Latest completed inspection:", report)

    def test_report_contains_planner_markdown_and_csv(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            session = Path(directory) / "sessions" / "test-session"
            output = session / "analysis"
            session.mkdir(parents=True)
            (session / "metadata.json").write_text(
                json.dumps({"session_id": "test-session"}),
                encoding="utf-8",
            )
            (session / "planner_input_inspections.jsonl").write_text(
                json.dumps(inspection_record()) + "\n{malformed\n",
                encoding="utf-8",
            )
            products = generate_report(session, output)
            report = products[0].read_text(encoding="utf-8")
            with products[5].open(encoding="utf-8", newline="") as stream:
                rows = list(csv.DictReader(stream))

        self.assertEqual(len(products), 7)
        self.assertEqual(len(rows), 3)
        self.assertEqual(rows[0]["max_step_height"], "0.24")
        self.assertEqual(rows[0]["start_snap_radius"], "0.55")
        self.assertEqual(rows[0]["snap_radius"], "0.5")
        self.assertEqual(rows[0]["mapper_max_slope"], "0.65")
        self.assertEqual(rows[0]["slope_at_or_above_mapper_limit"], "4")
        self.assertEqual(rows[0]["map_continuous_ground_cells"], "25")
        self.assertEqual(rows[0]["start_component_cells"], "0")
        self.assertIn("## Planner Input Inspections", report)
        self.assertIn("malformed JSONL: 1", report)
        self.assertIn("T accepted", report)
        self.assertIn("max_step_height=0.24", report)
        self.assertIn("mapper_max_slope=0.65", report)
        self.assertIn("Slope >= mapper", report)
        self.assertIn("start_has_no_valid_cell_in_snap_square", report)

    def test_report_writes_planner_series_summary_and_overview(self) -> None:
        records = [
            series_record(
                index,
                recorded_second=2 * (index - 1),
                planner_valid=5 * index,
                slope_over=2 * index,
                slope_minimum=0.1 * index,
                start_component_cells=index,
                goal_connected=connected,
            )
            for index, connected in ((1, True), (2, False), (3, None))
        ]
        with tempfile.TemporaryDirectory() as directory:
            session = Path(directory) / "sessions" / "series-session"
            output = session / "analysis"
            session.mkdir(parents=True)
            (session / "metadata.json").write_text(
                json.dumps({"session_id": "series-session"}),
                encoding="utf-8",
            )
            (session / "planner_input_inspections.jsonl").write_text(
                "\n".join(json.dumps(record) for record in records) + "\n",
                encoding="utf-8",
            )

            products = generate_report(session, output)
            report = products[0].read_text(encoding="utf-8")
            with products[6].open(encoding="utf-8", newline="") as stream:
                rows = list(csv.DictReader(stream))

        self.assertEqual(products[6].name, "planner_input_series_summary.csv")
        self.assertEqual(len(rows), 3)
        self.assertEqual(rows[0]["series_id"], "slope-run-a")
        self.assertEqual(rows[0]["captured"], "3")
        self.assertEqual(rows[0]["goal_connectivity_bool_flips"], "1")
        self.assertIn("### Planner Input Series", report)
        self.assertIn("slope-run-a", report)

    def test_missing_planner_log_remains_backward_compatible(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            session = Path(directory) / "sessions" / "old-session"
            output = session / "analysis"
            session.mkdir(parents=True)
            (session / "metadata.json").write_text(
                json.dumps({"session_id": "old-session"}),
                encoding="utf-8",
            )
            products = generate_report(session, output)
            report = products[0].read_text(encoding="utf-8")
            with products[5].open(encoding="utf-8", newline="") as stream:
                rows = list(csv.DictReader(stream))

        self.assertEqual(rows, [])
        self.assertIn("Accepted records: 0", report)
        self.assertIn("No completed planner input inspection was captured.", report)


class ResourceDiagnosticsAnalysisTests(unittest.TestCase):
    def test_topic_timing_preserves_future_header_age(self) -> None:
        summaries = summarize_topic_timing(
            [
                {
                    "topic": "/lio/odom",
                    "status": "ok",
                    "window_duration_sec": "5",
                    "message_count": "5",
                    "latest_header_age_ms": "-25.5",
                },
                {
                    "topic": "/lio/odom",
                    "status": "ok",
                    "window_duration_sec": "5",
                    "message_count": "5",
                    "latest_header_age_ms": "40.0",
                },
            ]
        )

        self.assertEqual(summaries[0]["min_latest_header_age_ms"], "-25.500000")
        self.assertEqual(summaries[0]["max_latest_header_age_ms"], "40.000000")

    def test_report_summarizes_new_runtime_diagnostics(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            session = Path(directory) / "sessions" / "resource-session"
            output = session / "analysis"
            session.mkdir(parents=True)
            (session / "metadata.json").write_text(
                json.dumps({"session_id": "resource-session"}),
                encoding="utf-8",
            )
            (session / "process_health.csv").write_text(
                "timestamp,component,running,pids,states,cpu_percent_interval,"
                "rss_kib,threads,metrics_status\n"
                "2026-08-07T00:00:00Z,super_lio,1,101,101:S,,1000,4,baseline\n"
                "2026-08-07T00:00:05Z,super_lio,1,101,101:S,25.5,1200,5,ok\n"
                "2026-08-07T00:00:10Z,super_lio,1,101,101:R,50.5,1400,6,partial\n",
                encoding="utf-8",
            )
            (session / "topic_timing.csv").write_text(
                "window_end,window_duration_sec,topic,status,message_count,"
                "first_receive_ns,last_receive_ns,first_header_ns,last_header_ns,"
                "latest_header_age_ms,header_span_ms,first_local_sequence,"
                "last_local_sequence,unique_header_count,duplicate_header_count,"
                "nonmonotonic_header_count,invalid_header_count,min_header_gap_ms,"
                "max_header_gap_ms,max_receive_gap_ms\n"
                "2026-08-07T00:00:05Z,5,/lio/odom,ok,6,1,6,1,6,12,5000,1,6,"
                "5,1,1,0,90,110,150\n"
                "2026-08-07T00:00:10Z,5,/lio/odom,ok,11,7,17,7,17,18,5000,1,11,"
                "11,0,0,1,80,120,220\n"
                "2026-08-07T00:00:10Z,5,/lio/body_odom,no_data,0,,,,,,,,,0,0,0,0,,,\n",
                encoding="utf-8",
            )
            (session / "system_health.csv").write_text(
                "timestamp,monotonic_ns,load1,load5,load15,mem_available_kib,"
                "swap_free_kib,thermal_max_millic,thermal_zone,status\n"
                "2026-08-07T00:00:00Z,1,1.0,0.8,0.5,4000000,2000000,55000,cpu,ok\n"
                "2026-08-07T00:00:05Z,2,2.5,1.2,0.7,3500000,1500000,62000,gpu,partial\n",
                encoding="utf-8",
            )
            (session / "network_health.csv").write_text(
                "timestamp,monotonic_ns,interface,operstate,carrier,rx_bytes,"
                "rx_packets,rx_errors,rx_dropped,tx_bytes,tx_packets,tx_errors,"
                "tx_dropped,status\n"
                "2026-08-07T00:00:00Z,1,enP8p1s0,up,1,1000,10,1,2,2000,20,0,1,ok\n"
                "2026-08-07T00:00:05Z,2,enP8p1s0,up,1,1600,16,3,5,2900,29,1,3,ok\n",
                encoding="utf-8",
            )
            (session / "hesai_summary.csv").write_text(
                "timestamp,status,file_size_bytes,mtime_epoch,last_frame,frame_delta,"
                "last_points,last_packets,last_start_time,last_end_time,"
                "tail_warning_lines,tail_error_lines\n"
                "2026-08-07T00:00:00Z,ok,10000,1,100,,64000,500,10.0,10.1,1,0\n"
                "2026-08-07T00:00:05Z,ok,12000,2,110,10,64000,500,10.5,10.6,2,1\n",
                encoding="utf-8",
            )

            products = generate_report(session, output)
            report = products[0].read_text(encoding="utf-8")
            with (output / "process_health_summary.csv").open(
                encoding="utf-8", newline=""
            ) as stream:
                process_rows = list(csv.DictReader(stream))
            with (output / "topic_timing_summary.csv").open(
                encoding="utf-8", newline=""
            ) as stream:
                timing_rows = list(csv.DictReader(stream))
            with (output / "system_health_summary.csv").open(
                encoding="utf-8", newline=""
            ) as stream:
                system_rows = list(csv.DictReader(stream))
            with (output / "network_health_summary.csv").open(
                encoding="utf-8", newline=""
            ) as stream:
                network_rows = list(csv.DictReader(stream))
            with (output / "hesai_driver_summary.csv").open(
                encoding="utf-8", newline=""
            ) as stream:
                hesai_rows = list(csv.DictReader(stream))

        self.assertEqual(len(products), 7)
        self.assertEqual(process_rows[0]["cpu_samples"], "2")
        self.assertEqual(process_rows[0]["mean_cpu_percent"], "38.000000")
        self.assertEqual(process_rows[0]["max_rss_kib"], "1400.000000")
        self.assertEqual(process_rows[0]["max_threads"], "6")
        self.assertEqual(process_rows[0]["last_states"], "101:R")
        self.assertEqual(timing_rows[0]["topic"], "/lio/odom")
        self.assertEqual(timing_rows[0]["total_messages"], "17")
        self.assertEqual(timing_rows[0]["duplicate_headers"], "1")
        self.assertEqual(timing_rows[0]["nonmonotonic_headers"], "1")
        self.assertEqual(timing_rows[0]["invalid_headers"], "1")
        self.assertEqual(timing_rows[0]["max_receive_gap_ms"], "220.000000")
        self.assertEqual(system_rows[0]["max_load1"], "2.500000")
        self.assertEqual(system_rows[0]["min_mem_available_kib"], "3500000.000000")
        self.assertEqual(system_rows[0]["max_thermal_millic"], "62000.000000")
        self.assertEqual(network_rows[0]["rx_bytes_delta"], "600")
        self.assertEqual(network_rows[0]["rx_errors_delta"], "2")
        self.assertEqual(network_rows[0]["tx_dropped_delta"], "2")
        self.assertEqual(hesai_rows[0]["frame_advance_total"], "10")
        self.assertEqual(hesai_rows[0]["last_frame"], "110")
        self.assertEqual(hesai_rows[0]["max_tail_error_lines"], "1")
        self.assertIn("## Topic Timing", report)
        self.assertIn("## System Health", report)
        self.assertIn("## Network Health", report)
        self.assertIn("## Hesai Driver Summary", report)
        self.assertIn("Mean CPU %", report)

    def test_legacy_process_rows_and_missing_new_files_remain_supported(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            session = Path(directory) / "sessions" / "legacy-session"
            output = session / "analysis"
            session.mkdir(parents=True)
            (session / "metadata.json").write_text(
                json.dumps({"session_id": "legacy-session"}),
                encoding="utf-8",
            )
            (session / "process_health.csv").write_text(
                "timestamp,component,running,pids\n"
                "2026-08-06T00:00:00Z,super_lio,1,42\n",
                encoding="utf-8",
            )

            products = generate_report(session, output)
            report = products[0].read_text(encoding="utf-8")
            with products[2].open(encoding="utf-8", newline="") as stream:
                process_rows = list(csv.DictReader(stream))
            with (output / "topic_timing_summary.csv").open(
                encoding="utf-8", newline=""
            ) as stream:
                timing_rows = list(csv.DictReader(stream))
            with (output / "network_health_summary.csv").open(
                encoding="utf-8", newline=""
            ) as stream:
                network_rows = list(csv.DictReader(stream))

        self.assertEqual(len(products), 7)
        self.assertEqual(process_rows[0]["last_state"], "running")
        self.assertEqual(process_rows[0]["last_pids"], "42")
        self.assertEqual(process_rows[0]["cpu_samples"], "0")
        self.assertEqual(process_rows[0]["mean_cpu_percent"], "")
        self.assertEqual(process_rows[0]["max_rss_kib"], "")
        self.assertEqual(timing_rows, [])
        self.assertEqual(network_rows, [])
        self.assertIn("## Topic Timing", report)
        self.assertIn("No topic timing windows were captured.", report)
        self.assertIn("No system health samples were captured.", report)
        self.assertIn("No network health samples were captured.", report)
        self.assertIn("No Hesai driver samples were captured.", report)


if __name__ == "__main__":
    unittest.main()
