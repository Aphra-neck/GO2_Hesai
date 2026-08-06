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
        "grid_x": 4 + seed,
        "grid_y": 5 + seed,
        "inside_map": inside,
        "exact_cell_valid": False,
        "valid_cells_in_snap_square": valid_cells if inside else 0,
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
            products = generate_report(session, output)
            report = products[0].read_text(encoding="utf-8")
            with products[5].open(encoding="utf-8", newline="") as stream:
                rows = list(csv.DictReader(stream))

        self.assertEqual([row["scope"] for row in rows], ["map", "start"])
        self.assertIn("Accepted records: 1", report)
        self.assertIn("start_ready_waiting_for_goal", report)

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
        self.assertEqual(summary["rows"][0]["snap_radius"], 0.5)
        self.assertEqual(summary["rows"][0]["mapper_max_slope"], 0.65)
        self.assertEqual(
            summary["rows"][0]["slope_at_or_above_mapper_limit"], 4
        )
        self.assertEqual(summary["rows"][0]["map_odom_stamp_delta_sec"], 0.05)
        self.assertEqual(summary["rows"][0]["map_planner_gate_known_cells"], 50)
        self.assertEqual(summary["rows"][0]["map_continuous_ground_cells"], 25)
        self.assertEqual(
            summary["diagnoses"]["start_has_no_valid_cell_in_snap_square"],
            1,
        )

    def test_no_goal_inspection_expands_to_map_and_start_rows(self) -> None:
        summary = summarize_planner_inspections([no_goal_record()])
        self.assertEqual(summary["schema_invalid"], 0)
        self.assertEqual(
            [row["scope"] for row in summary["rows"]],
            ["map", "start"],
        )

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

        self.assertEqual(len(products), 6)
        self.assertEqual(len(rows), 3)
        self.assertEqual(rows[0]["max_step_height"], "0.24")
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


if __name__ == "__main__":
    unittest.main()
