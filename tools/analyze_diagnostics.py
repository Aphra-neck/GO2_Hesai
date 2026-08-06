#!/usr/bin/env python3
"""Build compact Markdown and CSV summaries for one go2-log session."""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
import math
import re
import statistics
from collections import Counter, defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable


EXPECTED_TOPICS = (
    "/imu/data",
    "/lio/odom",
    "/lio/body_odom",
    "/body_path",
    "/terrain_costmap",
)
PARAMETER_SCALAR = re.compile(r"^\s*([A-Za-z0-9_.-]+):\s*([^#]+?)\s*(?:#.*)?$")
YAW_AUDIT_TOLERANCE_RAD = 0.15
RFC3339_UTC = re.compile(
    r"^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}(?:\.\d+)?(?:Z|\+00:00)$"
)
PLANNER_SUCCESS_DIAGNOSES = {
    "start_ready_waiting_for_goal",
    "same_continuous_ground_component_not_planner_approval",
}
PLANNER_LAYER_SOURCE_FIELDS = (
    "cell_count",
    "observation_zero",
    "observation_below_min",
    "observation_ready",
    "elevation_known",
    "elevation_unknown_or_nonfinite",
    "elevation_known_below_min_observations",
    "slope_known",
    "slope_unknown_or_nonfinite",
    "roughness_known",
    "roughness_unknown_or_nonfinite",
    "traversability_known",
    "traversability_unknown_or_nonfinite",
    "features_known",
    "slope_over_limit",
    "slope_at_or_above_mapper_limit",
    "roughness_over_limit",
    "traversability_zero",
    "traversability_below_min_nonzero",
    "traversability_at_or_above_min",
    "zero_traversability_with_slope_and_roughness_below_limits",
    "planner_valid",
)
PLANNER_LAYER_FIELDS = (
    "cell_count",
    "observation_zero",
    "observation_below_min",
    "observation_ready",
    "elevation_known",
    "elevation_unknown_or_nonfinite",
    "elevation_known_below_min_observations",
    "slope_known",
    "slope_unknown_or_nonfinite",
    "roughness_known",
    "roughness_unknown_or_nonfinite",
    "traversability_known",
    "traversability_unknown_or_nonfinite",
    "features_known",
    "slope_over_limit",
    "slope_at_or_above_mapper_limit",
    "roughness_over_limit",
    "traversability_zero",
    "traversability_below_min_nonzero",
    "traversability_at_or_above_min",
    "hard_reject_candidate",
    "planner_valid",
)
PLANNER_CSV_FIELDS = (
    "recorded_at",
    "status",
    "exit_code",
    "diagnosis",
    "scope",
    "map_frame",
    "map_width",
    "map_height",
    "resolution",
    "map_stamp_ns",
    "map_total_cells",
    "map_planner_gate_known_cells",
    "map_valid_cells",
    "map_continuous_ground_cells",
    "map_known_percent",
    "map_valid_percent",
    "map_continuous_ground_percent",
    "min_observed_frames",
    "min_traversability",
    "max_slope",
    "mapper_max_slope",
    "max_roughness",
    "max_step_height",
    "snap_radius",
    "map_age_sec",
    "odom_age_sec",
    "goal_age_sec",
    "map_odom_stamp_delta_sec",
    "start_component_cells",
    "start_component_percent_of_ground",
    "goal_in_start_component",
    *PLANNER_LAYER_FIELDS,
    "world_x",
    "world_y",
    "grid_x",
    "grid_y",
    "inside_map",
    "exact_cell_valid",
    "valid_cells_in_snap_square",
    "snapped",
    "snap_grid_distance_m",
    "snap_world_to_center_distance_m",
    "start_map_diagnosis",
)


@dataclass(frozen=True)
class CapturedSetting:
    value: float | None
    source: str

    @property
    def display_value(self) -> str:
        return "not captured" if self.value is None else f"{self.value:.6g}"


@dataclass(frozen=True)
class OdometrySample:
    stamp_ns: int
    yaw: float
    quaternion: tuple[float, float, float, float]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path, help="downloaded sessions/<session-id> directory")
    parser.add_argument(
        "--output-dir",
        type=Path,
        help="output directory (default: <session>/analysis)",
    )
    return parser.parse_args()


def read_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}
    return value if isinstance(value, dict) else {}


def _reject_json_constant(value: str) -> None:
    raise ValueError(f"invalid JSON numeric constant: {value}")


def read_jsonl(path: Path) -> tuple[list[dict[str, Any]], int]:
    records: list[dict[str, Any]] = []
    invalid = 0
    try:
        lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    except OSError:
        return records, invalid
    for line in lines:
        if not line.strip():
            invalid += 1
            continue
        try:
            value = json.loads(line, parse_constant=_reject_json_constant)
        except (json.JSONDecodeError, ValueError):
            invalid += 1
            continue
        if isinstance(value, dict):
            records.append(value)
        else:
            invalid += 1
    return records, invalid


def _is_strict_int(value: object, *, minimum: int = 0) -> bool:
    return type(value) is int and value >= minimum


def _is_finite_number(
    value: object,
    *,
    minimum: float | None = None,
    maximum: float | None = None,
) -> bool:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return False
    number = float(value)
    if not math.isfinite(number):
        return False
    if minimum is not None and number < minimum:
        return False
    if maximum is not None and number > maximum:
        return False
    return True


def _is_optional_finite_number(value: object) -> bool:
    return value is None or _is_finite_number(value)


def _is_rfc3339_utc(value: object) -> bool:
    if not isinstance(value, str) or RFC3339_UTC.fullmatch(value) is None:
        return False
    normalized = value[:-1] + "+00:00" if value.endswith("Z") else value
    try:
        parsed = dt.datetime.fromisoformat(normalized)
    except ValueError:
        return False
    return parsed.utcoffset() == dt.timedelta(0)


def _valid_planner_layers(value: object) -> bool:
    if not isinstance(value, dict):
        return False
    if any(
        not _is_strict_int(value.get(field))
        for field in PLANNER_LAYER_SOURCE_FIELDS
    ):
        return False

    cell_count = value["cell_count"]
    if (
        value["observation_zero"]
        + value["observation_below_min"]
        + value["observation_ready"]
        != cell_count
    ):
        return False
    for known, unknown in (
        ("elevation_known", "elevation_unknown_or_nonfinite"),
        ("slope_known", "slope_unknown_or_nonfinite"),
        ("roughness_known", "roughness_unknown_or_nonfinite"),
        ("traversability_known", "traversability_unknown_or_nonfinite"),
    ):
        if value[known] + value[unknown] != cell_count:
            return False
    if value["elevation_known_below_min_observations"] > value["elevation_known"]:
        return False
    if value["elevation_known_below_min_observations"] > (
        value["observation_zero"] + value["observation_below_min"]
    ):
        return False
    if value["features_known"] > min(
        value["slope_known"],
        value["roughness_known"],
        value["traversability_known"],
    ):
        return False
    if value["slope_over_limit"] > value["slope_known"]:
        return False
    if value["slope_at_or_above_mapper_limit"] > value["slope_known"]:
        return False
    if value["roughness_over_limit"] > value["roughness_known"]:
        return False
    if any(
        value[field] > value["traversability_known"]
        for field in (
            "traversability_zero",
            "traversability_below_min_nonzero",
            "traversability_at_or_above_min",
        )
    ):
        return False
    if value[
        "zero_traversability_with_slope_and_roughness_below_limits"
    ] > min(value["traversability_zero"], value["features_known"]):
        return False
    return value["planner_valid"] <= min(
        value["traversability_at_or_above_min"],
        value["slope_known"],
    )


def _valid_planner_endpoint(value: object) -> bool:
    if not isinstance(value, dict):
        return False
    required = {
        "frame_id",
        "child_frame_id",
        "stamp_ns",
        "world_x",
        "world_y",
        "yaw_rad",
        "grid_x",
        "grid_y",
        "inside_map",
        "exact_cell_valid",
        "valid_cells_in_snap_square",
        "snapped",
        "snapped_grid_x",
        "snapped_grid_y",
        "snapped_world_x",
        "snapped_world_y",
        "snap_grid_distance_m",
        "snap_world_to_center_distance_m",
        "snap_grid_distance_outside_nominal_radius",
        "snap_square_terrain_layers",
    }
    if (
        not required.issubset(value)
        or not isinstance(value.get("frame_id"), str)
        or not isinstance(value.get("child_frame_id"), str)
        or not _is_strict_int(value.get("stamp_ns"))
        or not all(
            _is_finite_number(value.get(field))
            for field in ("world_x", "world_y", "yaw_rad")
        )
        or not all(
            _is_strict_int(value.get(field), minimum=-2**63)
            for field in ("grid_x", "grid_y")
        )
        or type(value.get("inside_map")) is not bool
        or type(value.get("exact_cell_valid")) is not bool
        or not _is_strict_int(value.get("valid_cells_in_snap_square"))
        or type(value.get("snapped")) is not bool
        or type(value.get("snap_grid_distance_outside_nominal_radius")) is not bool
    ):
        return False

    inside = value["inside_map"]
    layers = value.get("snap_square_terrain_layers")
    if inside:
        if not _valid_planner_layers(layers):
            return False
        if layers["cell_count"] < 1:
            return False
        if layers["planner_valid"] != value["valid_cells_in_snap_square"]:
            return False
    elif layers is not None:
        return False

    snapped = value["snapped"]
    snapped_fields = (
        "snapped_grid_x",
        "snapped_grid_y",
        "snapped_world_x",
        "snapped_world_y",
        "snap_grid_distance_m",
        "snap_world_to_center_distance_m",
    )
    if snapped:
        if value["valid_cells_in_snap_square"] < 1:
            return False
        if not all(
            _is_strict_int(value.get(field), minimum=-2**63)
            for field in ("snapped_grid_x", "snapped_grid_y")
        ):
            return False
        if not all(
            _is_finite_number(
                value.get(field),
                minimum=0.0 if field.endswith("distance_m") else None,
            )
            for field in snapped_fields[2:]
        ):
            return False
    elif any(value.get(field) is not None for field in snapped_fields):
        return False
    if value["exact_cell_valid"] and not snapped:
        return False
    if not inside and (
        value["exact_cell_valid"]
        or value["valid_cells_in_snap_square"] != 0
        or snapped
    ):
        return False
    return (value["valid_cells_in_snap_square"] > 0) == snapped


def _valid_planner_inspection(value: object) -> bool:
    if not isinstance(value, dict):
        return False
    required = {
        "map",
        "thresholds",
        "contract",
        "freshness",
        "start",
        "goal",
        "start_component_cells",
        "start_component_percent_of_ground",
        "goal_in_start_component",
        "diagnosis",
        "limitations",
    }
    if not required.issubset(value):
        return False
    diagnosis = value.get("diagnosis")
    map_data = value.get("map")
    thresholds = value.get("thresholds")
    contract = value.get("contract")
    freshness = value.get("freshness")
    start = value.get("start")
    goal = value.get("goal")
    if (
        not isinstance(diagnosis, str)
        or not diagnosis
        or not isinstance(map_data, dict)
        or not isinstance(thresholds, dict)
        or not isinstance(contract, dict)
        or not isinstance(freshness, dict)
        or not _valid_planner_endpoint(start)
        or (goal is not None and not _valid_planner_endpoint(goal))
    ):
        return False

    width = map_data.get("width")
    height = map_data.get("height")
    total_cells = map_data.get("total_cells")
    map_required = {
        "frame_id",
        "stamp_ns",
        "resolution",
        "width",
        "height",
        "origin_x",
        "origin_y",
        "total_cells",
        "known_cells",
        "planner_gate_known_cells",
        "valid_cells",
        "continuous_ground_cells",
        "known_percent",
        "valid_percent",
        "continuous_ground_percent",
        "terrain_layers",
    }
    if (
        not map_required.issubset(map_data)
        or not isinstance(map_data.get("frame_id"), str)
        or not _is_strict_int(map_data.get("stamp_ns"))
        or not _is_finite_number(map_data.get("resolution"), minimum=0.0)
        or float(map_data["resolution"]) == 0.0
        or not _is_strict_int(width, minimum=1)
        or not _is_strict_int(height, minimum=1)
        or not _is_finite_number(map_data.get("origin_x"))
        or not _is_finite_number(map_data.get("origin_y"))
        or not _is_strict_int(total_cells, minimum=1)
        or total_cells != width * height
        or not _valid_planner_layers(map_data.get("terrain_layers"))
        or map_data["terrain_layers"]["cell_count"] != total_cells
    ):
        return False

    count_fields = (
        "known_cells",
        "planner_gate_known_cells",
        "valid_cells",
        "continuous_ground_cells",
    )
    if any(not _is_strict_int(map_data.get(field)) for field in count_fields):
        return False
    if (
        map_data["known_cells"] != map_data["planner_gate_known_cells"]
        or map_data["valid_cells"] != map_data["terrain_layers"]["planner_valid"]
        or map_data["known_cells"] > total_cells
        or map_data["valid_cells"] > map_data["known_cells"]
        or map_data["continuous_ground_cells"] > map_data["valid_cells"]
    ):
        return False
    for field in (
        "known_percent",
        "valid_percent",
        "continuous_ground_percent",
    ):
        if not _is_finite_number(map_data.get(field), minimum=0.0, maximum=100.0):
            return False
    for field, count_field in (
        ("known_percent", "known_cells"),
        ("valid_percent", "valid_cells"),
        ("continuous_ground_percent", "continuous_ground_cells"),
    ):
        if not math.isclose(
            float(map_data[field]),
            100.0 * map_data[count_field] / total_cells,
            rel_tol=1.0e-9,
            abs_tol=1.0e-9,
        ):
            return False

    thresholds_required = {
        "min_observed_frames",
        "min_traversability",
        "max_slope",
        "mapper_max_slope",
        "max_roughness",
        "max_step_height",
        "snap_radius",
    }
    contract_required = {
        "map_frame",
        "body_frame",
        "max_map_age",
        "max_odom_age",
        "max_goal_age",
        "future_tolerance",
    }
    freshness_required = {
        "evaluated",
        "map_age_sec",
        "odom_age_sec",
        "goal_age_sec",
        "map_odom_stamp_delta_sec",
    }
    if (
        not thresholds_required.issubset(thresholds)
        or not contract_required.issubset(contract)
        or not freshness_required.issubset(freshness)
        or not _is_strict_int(thresholds.get("min_observed_frames"), minimum=1)
        or any(
            not _is_finite_number(thresholds.get(field), minimum=0.0)
            for field in (
                "min_traversability",
                "max_slope",
                "mapper_max_slope",
                "max_roughness",
                "max_step_height",
                "snap_radius",
            )
        )
        or not isinstance(contract.get("map_frame"), str)
        or not contract["map_frame"]
        or not isinstance(contract.get("body_frame"), str)
        or not contract["body_frame"]
        or any(
            not _is_finite_number(contract.get(field), minimum=0.0)
            for field in (
                "max_map_age",
                "max_odom_age",
                "max_goal_age",
                "future_tolerance",
            )
        )
        or type(freshness.get("evaluated")) is not bool
        or any(
            not _is_optional_finite_number(freshness.get(field))
            for field in (
                "map_age_sec",
                "odom_age_sec",
                "goal_age_sec",
                "map_odom_stamp_delta_sec",
            )
        )
        or not _is_strict_int(value.get("start_component_cells"))
        or not _is_finite_number(
            value.get("start_component_percent_of_ground"),
            minimum=0.0,
            maximum=100.0,
        )
        or (
            value.get("goal_in_start_component") is not None
            and type(value.get("goal_in_start_component")) is not bool
        )
        or not isinstance(value.get("limitations"), str)
        or not value["limitations"]
    ):
        return False
    if value["start_component_cells"] > map_data["continuous_ground_cells"]:
        return False
    expected_component_percent = (
        100.0
        * value["start_component_cells"]
        / map_data["continuous_ground_cells"]
        if map_data["continuous_ground_cells"]
        else 0.0
    )
    if not math.isclose(
        float(value["start_component_percent_of_ground"]),
        expected_component_percent,
        rel_tol=1.0e-9,
        abs_tol=1.0e-9,
    ):
        return False
    return True


def _planner_layer_row(
    record: dict[str, Any],
    inspection: dict[str, Any],
    scope: str,
    layers: dict[str, Any],
    endpoint: dict[str, Any] | None,
) -> dict[str, object]:
    row: dict[str, object] = {field: "" for field in PLANNER_CSV_FIELDS}
    map_data = inspection.get("map", {})
    thresholds = inspection.get("thresholds", {})
    freshness = inspection.get("freshness", {})
    if not isinstance(map_data, dict):
        map_data = {}
    if not isinstance(thresholds, dict):
        thresholds = {}
    if not isinstance(freshness, dict):
        freshness = {}
    row.update(
        {
            "recorded_at": record.get("recorded_at", ""),
            "status": record.get("status", ""),
            "exit_code": record.get("exit_code", ""),
            "diagnosis": inspection.get("diagnosis", ""),
            "start_map_diagnosis": inspection.get("start_map_diagnosis", ""),
            "scope": scope,
            "map_frame": map_data.get("frame_id", ""),
            "map_width": map_data.get("width", ""),
            "map_height": map_data.get("height", ""),
            "resolution": map_data.get("resolution", ""),
            "map_stamp_ns": map_data.get("stamp_ns", ""),
            "map_total_cells": map_data.get("total_cells", ""),
            "map_planner_gate_known_cells": map_data.get(
                "planner_gate_known_cells", ""
            ),
            "map_valid_cells": map_data.get("valid_cells", ""),
            "map_continuous_ground_cells": map_data.get(
                "continuous_ground_cells", ""
            ),
            "map_known_percent": map_data.get("known_percent", ""),
            "map_valid_percent": map_data.get("valid_percent", ""),
            "map_continuous_ground_percent": map_data.get(
                "continuous_ground_percent", ""
            ),
            "min_observed_frames": thresholds.get(
                "min_observed_frames", ""
            ),
            "min_traversability": thresholds.get("min_traversability", ""),
            "max_slope": thresholds.get("max_slope", ""),
            "mapper_max_slope": thresholds.get("mapper_max_slope", ""),
            "max_roughness": thresholds.get("max_roughness", ""),
            "max_step_height": thresholds.get("max_step_height", ""),
            "snap_radius": thresholds.get("snap_radius", ""),
            "map_age_sec": freshness.get("map_age_sec", ""),
            "odom_age_sec": freshness.get("odom_age_sec", ""),
            "goal_age_sec": freshness.get("goal_age_sec", ""),
            "map_odom_stamp_delta_sec": freshness.get(
                "map_odom_stamp_delta_sec", ""
            ),
            "start_component_cells": inspection.get(
                "start_component_cells", ""
            ),
            "start_component_percent_of_ground": inspection.get(
                "start_component_percent_of_ground", ""
            ),
            "goal_in_start_component": inspection.get(
                "goal_in_start_component", ""
            ),
        }
    )
    for field in PLANNER_LAYER_FIELDS:
        source_field = (
            "zero_traversability_with_slope_and_roughness_below_limits"
            if field == "hard_reject_candidate"
            else field
        )
        row[field] = layers.get(source_field, "")
    if endpoint is not None:
        for field in (
            "world_x",
            "world_y",
            "grid_x",
            "grid_y",
            "inside_map",
            "exact_cell_valid",
            "valid_cells_in_snap_square",
            "snapped",
            "snap_grid_distance_m",
            "snap_world_to_center_distance_m",
        ):
            row[field] = endpoint.get(field, "")
    return row


def summarize_planner_inspections(
    records: list[dict[str, Any]],
) -> dict[str, Any]:
    accepted_records: list[dict[str, Any]] = []
    completed: list[dict[str, Any]] = []
    rows: list[dict[str, object]] = []
    latest_rows: list[dict[str, object]] = []
    schema_invalid = 0
    capture_errors = 0
    diagnoses: Counter[str] = Counter()

    for record in records:
        status = record.get("status")
        exit_code = record.get("exit_code")
        if (
            type(record.get("schema_version")) is not int
            or record["schema_version"] != 1
            or not _is_rfc3339_utc(record.get("recorded_at"))
            or type(exit_code) is not int
            or status not in {
                "ok",
                "diagnostic_failure",
                "capture_error",
            }
        ):
            schema_invalid += 1
            continue
        if status == "capture_error":
            error = record.get("error")
            if (
                exit_code != 1
                or "inspection" in record
                or not isinstance(error, dict)
                or not isinstance(error.get("type"), str)
                or not error["type"]
                or not isinstance(error.get("message"), str)
                or not error["message"]
            ):
                schema_invalid += 1
                continue
            accepted_records.append(record)
            capture_errors += 1
            continue

        expected_exit = 0 if status == "ok" else 2
        inspection = record.get("inspection")
        if (
            exit_code != expected_exit
            or "error" in record
            or not _valid_planner_inspection(inspection)
        ):
            schema_invalid += 1
            continue
        assert isinstance(inspection, dict)
        diagnosis = inspection.get("diagnosis")
        if (status == "ok") != (diagnosis in PLANNER_SUCCESS_DIAGNOSES):
            schema_invalid += 1
            continue
        map_data = inspection["map"]
        start = inspection["start"]
        goal = inspection.get("goal")
        assert isinstance(diagnosis, str)
        assert isinstance(map_data, dict)
        assert isinstance(start, dict)
        start_layers = start.get("snap_square_terrain_layers")

        accepted_records.append(record)
        completed.append(record)
        diagnoses[diagnosis] += 1
        record_rows = [
            _planner_layer_row(
                record,
                inspection,
                "map",
                map_data["terrain_layers"],
                None,
            )
        ]
        record_rows.append(
            _planner_layer_row(
                record,
                inspection,
                "start",
                start_layers if isinstance(start_layers, dict) else {},
                start,
            )
        )
        if goal is not None:
            record_rows.append(
                _planner_layer_row(
                    record,
                    inspection,
                    "goal",
                    goal.get("snap_square_terrain_layers")
                    if isinstance(goal.get("snap_square_terrain_layers"), dict)
                    else {},
                    goal,
                )
            )
        rows.extend(record_rows)
        latest_rows = record_rows

    latest = completed[-1] if completed else None
    latest_attempt = accepted_records[-1] if accepted_records else None
    return {
        "accepted_records": accepted_records,
        "completed_records": completed,
        "rows": rows,
        "latest": latest,
        "latest_attempt": latest_attempt,
        "latest_rows": latest_rows,
        "schema_invalid": schema_invalid,
        "capture_errors": capture_errors,
        "diagnoses": diagnoses,
    }


def read_csv(path: Path) -> list[dict[str, str]]:
    try:
        with path.open("r", encoding="utf-8", errors="replace", newline="") as stream:
            return list(csv.DictReader(stream))
    except OSError:
        return []


def as_float(value: str | None) -> float | None:
    try:
        parsed = float(value) if value not in (None, "") else None
    except ValueError:
        return None
    return parsed if parsed is not None and math.isfinite(parsed) else None


def read_environment(path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    try:
        lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    except OSError:
        return values
    for line in lines:
        key, separator, value = line.partition("=")
        if separator and key:
            values[key.strip()] = value.strip()
    return values


def read_parameter_scalars(path: Path) -> dict[str, str]:
    """Read numeric scalar candidates from a ROS 2 parameter dump.

    ROS parameter dumps use nested YAML, but the settings needed here are
    unique scalar keys. Keeping this reader scalar-only avoids making local
    diagnostics analysis depend on PyYAML while rejecting lists and mappings.
    """

    values: dict[str, str] = {}
    try:
        lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    except OSError:
        return values
    for line in lines:
        match = PARAMETER_SCALAR.match(line)
        if not match:
            continue
        key, raw_value = match.groups()
        value = raw_value.strip().strip("'\"")
        if as_float(value) is not None:
            values[key] = value
    return values


def find_parameter_setting(
    session: Path,
    key: str,
    filename_hints: tuple[str, ...],
    *,
    positive: bool | None,
) -> CapturedSetting:
    for path in sorted(session.glob("parameters_*.yaml")):
        lower_name = path.name.lower()
        if not any(hint in lower_name for hint in filename_hints):
            continue
        value = as_float(read_parameter_scalars(path).get(key))
        if value is None:
            continue
        if positive is True and value <= 0.0:
            continue
        if positive is False and value < 0.0:
            continue
        return CapturedSetting(value, f"{path.name}:{key}")
    return CapturedSetting(None, "not captured; comparison skipped")


def load_runtime_settings(session: Path) -> dict[str, CapturedSetting]:
    environment = read_environment(session / "ros_dds_environment.txt")
    imu_environment_rate = as_float(environment.get("GO2_IMU_RATE"))
    if imu_environment_rate is not None and imu_environment_rate > 0.0:
        imu_rate = CapturedSetting(
            imu_environment_rate,
            "ros_dds_environment.txt:GO2_IMU_RATE",
        )
    else:
        imu_rate = find_parameter_setting(
            session,
            "publish_rate",
            ("go2_imu_bridge",),
            positive=True,
        )

    body_environment_offset = as_float(environment.get("GO2_BODY_YAW_OFFSET_RAD"))
    if body_environment_offset is not None:
        body_yaw_offset = CapturedSetting(
            body_environment_offset,
            "ros_dds_environment.txt:GO2_BODY_YAW_OFFSET_RAD",
        )
    else:
        body_yaw_offset = find_parameter_setting(
            session,
            "yaw_offset",
            ("body_odom_adapter",),
            positive=None,
        )

    sdk2_hints = ("go2_sdk2_bridge", "sdk2_bridge")
    return {
        "imu_target_hz": imu_rate,
        "body_yaw_offset": body_yaw_offset,
        "max_vx": find_parameter_setting(
            session, "max_vx", sdk2_hints, positive=False
        ),
        "max_vy": find_parameter_setting(
            session, "max_vy", sdk2_hints, positive=False
        ),
        "max_yaw_rate": find_parameter_setting(
            session, "max_yaw_rate", sdk2_hints, positive=False
        ),
    }


def markdown_cell(value: object) -> str:
    return str(value).replace("|", "\\|").replace("\n", " ")


def format_rate(value: float | None) -> str:
    return "-" if value is None else f"{value:.3f}"


def write_csv_atomic(path: Path, fieldnames: Iterable[str], rows: Iterable[dict[str, object]]) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(fieldnames))
        writer.writeheader()
        writer.writerows(rows)
    temporary.replace(path)


def write_text_atomic(path: Path, content: str) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(content, encoding="utf-8")
    temporary.replace(path)


def summarize_rates(rows: list[dict[str, str]]) -> list[dict[str, object]]:
    grouped: dict[str, list[dict[str, str]]] = defaultdict(list)
    for row in rows:
        topic = row.get("topic", "")
        if topic:
            grouped[topic].append(row)

    topics = list(EXPECTED_TOPICS)
    topics.extend(sorted(set(grouped).difference(topics)))
    summaries: list[dict[str, object]] = []
    for topic in topics:
        topic_rows = grouped.get(topic, [])
        rates = [rate for row in topic_rows if (rate := as_float(row.get("average_hz"))) is not None]
        latest = rates[-1] if rates else None
        summaries.append(
            {
                "topic": topic,
                "samples": len(topic_rows),
                "ok_samples": len(rates),
                "no_data_samples": sum(row.get("status") != "ok" for row in topic_rows),
                "mean_hz": f"{statistics.fmean(rates):.6f}" if rates else "",
                "min_hz": f"{min(rates):.6f}" if rates else "",
                "max_hz": f"{max(rates):.6f}" if rates else "",
                "latest_hz": f"{latest:.6f}" if latest is not None else "",
            }
        )
    return summaries


def summarize_processes(rows: list[dict[str, str]]) -> list[dict[str, object]]:
    grouped: dict[str, list[dict[str, str]]] = defaultdict(list)
    for row in rows:
        component = row.get("component", "")
        if component:
            grouped[component].append(row)

    summaries: list[dict[str, object]] = []
    for component in sorted(grouped):
        component_rows = grouped[component]
        running = sum(row.get("running") == "1" for row in component_rows)
        samples = len(component_rows)
        summaries.append(
            {
                "component": component,
                "samples": samples,
                "running_samples": running,
                "running_percent": f"{(100.0 * running / samples):.2f}" if samples else "",
                "last_state": "running" if component_rows[-1].get("running") == "1" else "stopped",
                "last_pids": component_rows[-1].get("pids", ""),
            }
        )
    return summaries


def summarize_sdk2_commands(rows: list[dict[str, str]]) -> dict[str, object]:
    valid: list[tuple[float, float, float]] = []
    for row in rows:
        vx = as_float(row.get("vx"))
        vy = as_float(row.get("vy"))
        yaw_rate = as_float(row.get("yaw_rate"))
        if vx is not None and vy is not None and yaw_rate is not None:
            valid.append((vx, vy, yaw_rate))

    latest = valid[-1] if valid else (None, None, None)
    return {
        "samples": len(rows),
        "ok_samples": len(valid),
        "no_data_samples": len(rows) - len(valid),
        "max_abs_vx": f"{max((abs(item[0]) for item in valid), default=0.0):.6f}" if valid else "",
        "max_abs_vy": f"{max((abs(item[1]) for item in valid), default=0.0):.6f}" if valid else "",
        "max_abs_yaw_rate": (
            f"{max((abs(item[2]) for item in valid), default=0.0):.6f}" if valid else ""
        ),
        "latest_vx": f"{latest[0]:.6f}" if latest[0] is not None else "",
        "latest_vy": f"{latest[1]:.6f}" if latest[1] is not None else "",
        "latest_yaw_rate": f"{latest[2]:.6f}" if latest[2] is not None else "",
    }


def normalize_angle(angle: float) -> float:
    return math.remainder(angle, 2.0 * math.pi)


def summarize_odometry(
    rows: list[dict[str, str]],
    topic: str,
    parent_frame: str,
    child_frames: set[str],
) -> tuple[dict[str, object], list[OdometrySample]]:
    valid: list[OdometrySample] = []
    no_data = 0
    invalid = 0
    frame_mismatch = 0
    for row in rows:
        status = row.get("status", "")
        if status != "ok":
            if status in {"no_data", "ros_unavailable"}:
                no_data += 1
            else:
                invalid += 1
            continue

        if (
            row.get("topic") != topic
            or row.get("frame_id") != parent_frame
            or row.get("child_frame_id", "") not in child_frames
        ):
            frame_mismatch += 1
            invalid += 1
            continue

        try:
            stamp_ns = int(row.get("ros_stamp_ns", ""))
        except ValueError:
            stamp_ns = 0
        position = [as_float(row.get(axis)) for axis in ("x", "y", "z")]
        quaternion = [as_float(row.get(axis)) for axis in ("qx", "qy", "qz", "qw")]
        captured_yaw = as_float(row.get("yaw"))
        if stamp_ns <= 0 or any(value is None for value in position + quaternion):
            invalid += 1
            continue

        qx, qy, qz, qw = (float(value) for value in quaternion)
        norm_squared = qx * qx + qy * qy + qz * qz + qw * qw
        if abs(norm_squared - 1.0) > 1.0e-3 or norm_squared <= 1.0e-12:
            invalid += 1
            continue
        norm = math.sqrt(norm_squared)
        qx, qy, qz, qw = (value / norm for value in (qx, qy, qz, qw))
        computed_yaw = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz),
        )
        if captured_yaw is None or abs(normalize_angle(captured_yaw - computed_yaw)) > 1.0e-6:
            invalid += 1
            continue
        valid.append(
            OdometrySample(
                stamp_ns=stamp_ns,
                yaw=computed_yaw,
                quaternion=(qx, qy, qz, qw),
            )
        )

    return (
        {
            "topic": topic,
            "samples": len(rows),
            "ok_samples": len(valid),
            "no_data_samples": no_data,
            "invalid_samples": invalid,
            "frame_mismatch_samples": frame_mismatch,
            "latest_yaw_deg": f"{math.degrees(valid[-1].yaw):.6f}" if valid else "",
        },
        valid,
    )


def summarize_yaw_correction(
    raw_samples: list[OdometrySample],
    body_samples: list[OdometrySample],
    configured_offset: CapturedSetting,
) -> dict[str, object]:
    raw_by_stamp = {sample.stamp_ns: sample for sample in raw_samples}
    offsets: list[float] = []
    relative_quaternions: list[tuple[float, float, float, float]] = []
    for body in sorted(body_samples, key=lambda sample: sample.stamp_ns):
        raw = raw_by_stamp.get(body.stamp_ns)
        if raw is None:
            continue
        rx, ry, rz, rw = raw.quaternion
        bx, by, bz, bw = body.quaternion
        relative = (
            rw * bx - rx * bw - ry * bz + rz * by,
            rw * by + rx * bz - ry * bw - rz * bx,
            rw * bz - rx * by + ry * bx - rz * bw,
            rw * bw + rx * bx + ry * by + rz * bz,
        )
        relative_norm = math.sqrt(sum(value * value for value in relative))
        relative = tuple(value / relative_norm for value in relative)
        qx, qy, qz, qw = relative
        offsets.append(
            math.atan2(
                2.0 * (qw * qz + qx * qy),
                1.0 - 2.0 * (qy * qy + qz * qz),
            )
        )
        relative_quaternions.append(relative)

    observed = None
    if offsets:
        observed = math.atan2(
            statistics.fmean(math.sin(value) for value in offsets),
            statistics.fmean(math.cos(value) for value in offsets),
        )
    errors: list[float] = []
    if configured_offset.value is not None:
        expected = (
            0.0,
            0.0,
            math.sin(0.5 * configured_offset.value),
            math.cos(0.5 * configured_offset.value),
        )
        for relative in relative_quaternions:
            dot = abs(sum(left * right for left, right in zip(relative, expected)))
            errors.append(2.0 * math.acos(max(-1.0, min(1.0, dot))))
    if not offsets:
        status = "no_pairs"
    elif configured_offset.value is None:
        status = "offset_not_captured"
    elif max(errors, default=0.0) > YAW_AUDIT_TOLERANCE_RAD:
        status = "mismatch"
    else:
        status = "ok"
    return {
        "matched_pairs": len(offsets),
        "configured_offset_rad": (
            f"{configured_offset.value:.9f}" if configured_offset.value is not None else ""
        ),
        "configured_offset_source": configured_offset.source,
        "observed_mean_offset_rad": f"{observed:.9f}" if observed is not None else "",
        "max_abs_error_rad": f"{max(errors):.9f}" if errors else "",
        "tolerance_rad": f"{YAW_AUDIT_TOLERANCE_RAD:.9f}",
        "status": status,
    }


def odometry_observations(
    raw_summary: dict[str, object],
    body_summary: dict[str, object],
    correction_summary: dict[str, object],
) -> list[str]:
    observations: list[str] = []
    for label, summary in (("raw", raw_summary), ("corrected body", body_summary)):
        if int(summary["ok_samples"]) == 0:
            observations.append(f"No valid {label} odometry pose sample was captured.")
        if int(summary["no_data_samples"]) > 0:
            observations.append(
                f"{label.capitalize()} odometry had {summary['no_data_samples']} no-data samples."
            )
        if int(summary["invalid_samples"]) > 0:
            observations.append(
                f"{label.capitalize()} odometry had {summary['invalid_samples']} invalid samples."
            )
        if int(summary["frame_mismatch_samples"]) > 0:
            observations.append(
                f"{label.capitalize()} odometry had {summary['frame_mismatch_samples']} frame mismatches."
            )

    status = correction_summary["status"]
    if status == "no_pairs":
        observations.append("Raw and corrected odometry had no timestamp-aligned yaw samples.")
    elif status == "offset_not_captured":
        observations.append("Body yaw offset was not captured; yaw-correction comparison was skipped.")
    elif status == "mismatch":
        observations.append(
            "Observed body yaw correction exceeded the configured tolerance: "
            f"mean={correction_summary['observed_mean_offset_rad']} rad, "
            f"configured={correction_summary['configured_offset_rad']} rad, "
            f"max error={correction_summary['max_abs_error_rad']} rad."
        )
    return observations


def count_rosout_levels(path: Path) -> Counter[str]:
    counts: Counter[str] = Counter()
    try:
        content = path.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return counts
    for block in content.split("---"):
        level = ""
        for line in block.splitlines():
            if line.startswith("level:"):
                level = line.partition(":")[2].strip()
                break
        counts[{"30": "warning", "40": "error", "50": "fatal"}.get(level, "unknown")] += 1
    counts.pop("unknown", None)
    return counts


def rate_value(summary: dict[str, object], key: str) -> float | None:
    return as_float(str(summary.get(key, "")))


def build_observations(
    rate_summaries: list[dict[str, object]],
    events: list[dict[str, Any]],
    invalid_events: int,
    rosout_counts: Counter[str],
    session: Path,
    imu_target: CapturedSetting,
) -> list[str]:
    observations: list[str] = []
    rates_by_topic = {str(row["topic"]): row for row in rate_summaries}
    for topic in EXPECTED_TOPICS:
        summary = rates_by_topic.get(topic, {})
        if int(summary.get("ok_samples", 0)) == 0:
            observations.append(f"No valid rate sample was received for `{topic}`.")

    imu_mean = rate_value(rates_by_topic.get("/imu/data", {}), "mean_hz")
    if imu_mean is not None and imu_target.value is None:
        observations.append(
            "IMU target rate was not captured; configured-rate comparison was skipped."
        )
    elif imu_mean is not None and imu_target.value is not None and imu_mean < imu_target.value:
        observations.append(
            f"Mean IMU delivery rate was {imu_mean:.1f} Hz, below the captured "
            f"{imu_target.value:.1f} Hz target ({imu_target.source})."
        )

    warning_or_error_events = sum(
        str(event.get("level", "")).lower() in {"warning", "error", "fatal"} for event in events
    )
    if warning_or_error_events:
        observations.append(f"Structured events contain {warning_or_error_events} warning/error lifecycle entries.")
    if sum(rosout_counts.values()):
        observations.append(
            "ROS logs contain "
            f"{rosout_counts['warning']} warnings, {rosout_counts['error']} errors, "
            f"and {rosout_counts['fatal']} fatal messages."
        )
    if invalid_events:
        observations.append(f"Ignored {invalid_events} malformed JSONL event records.")
    if (session / "capacity_reached.txt").exists():
        observations.append("Collection stopped at the protective session-size threshold.")
    if not (session / "ended_at.txt").exists():
        observations.append("The session has no clean collector end marker.")
    return observations


def planner_inspection_observations(
    summary: dict[str, Any],
    malformed_records: int,
) -> list[str]:
    observations: list[str] = []
    if malformed_records:
        observations.append(
            "Planner input log contains "
            f"{malformed_records} malformed JSONL record(s)."
        )
    if summary["schema_invalid"]:
        observations.append(
            "Planner input log contains "
            f"{summary['schema_invalid']} schema-invalid record(s)."
        )
    if summary["capture_errors"]:
        observations.append(
            f"Planner input inspection had {summary['capture_errors']} capture error(s)."
        )
    latest_attempt = summary["latest_attempt"]
    if latest_attempt is not None and latest_attempt["status"] == "capture_error":
        error = latest_attempt["error"]
        observations.append(
            "Latest planner input inspection attempt failed at "
            f"{markdown_cell(latest_attempt['recorded_at'])}: "
            f"{markdown_cell(error['type'])}: "
            f"{planner_error_message(error['message'])}."
        )
    latest = summary["latest"]
    if latest is None:
        observations.append("No completed planner input inspection was captured.")
        return observations
    inspection = latest["inspection"]
    diagnosis = inspection["diagnosis"]
    if diagnosis in {
        "goal_wait_timeout",
        "goal_publisher_discovery_timeout",
    }:
        start_map_diagnosis = inspection.get("start_map_diagnosis", "unknown")
        observations.append(
            f"Planner goal capture failed with `{diagnosis}`; fresh map/start "
            "evidence was retained "
            f"with start-map diagnosis `{start_map_diagnosis}`."
        )
    if diagnosis not in {
        "start_ready_waiting_for_goal",
        "same_continuous_ground_component_not_planner_approval",
    }:
        observations.append(
            f"Latest planner input inspection diagnosis: `{diagnosis}`."
        )
    return observations


def planner_markdown_value(value: object) -> str:
    if value is None or value == "":
        return "-"
    return markdown_cell(value)


def planner_error_message(value: object, limit: int = 500) -> str:
    message = markdown_cell(value)
    return message if len(message) <= limit else message[:limit] + "..."


def generate_report(
    session: Path,
    output_dir: Path,
) -> tuple[Path, Path, Path, Path, Path, Path]:
    metadata = read_json(session / "metadata.json")
    runtime_settings = load_runtime_settings(session)
    rate_rows = read_csv(session / "topic_rates.csv")
    process_rows = read_csv(session / "process_health.csv")
    sdk2_rows = read_csv(session / "sdk2_commands.csv")
    raw_odometry_rows = read_csv(session / "odom_position.csv")
    body_odometry_rows = read_csv(session / "body_odom_pose.csv")
    events, invalid_events = read_jsonl(session / "events.jsonl")
    planner_records, malformed_planner_records = read_jsonl(
        session / "planner_input_inspections.jsonl"
    )
    planner_summary = summarize_planner_inspections(planner_records)
    rate_summaries = summarize_rates(rate_rows)
    process_summaries = summarize_processes(process_rows)
    sdk2_summary = summarize_sdk2_commands(sdk2_rows)
    raw_odometry_summary, raw_odometry_samples = summarize_odometry(
        raw_odometry_rows,
        "/lio/odom",
        "world",
        {"", "imu"},
    )
    body_odometry_summary, body_odometry_samples = summarize_odometry(
        body_odometry_rows,
        "/lio/body_odom",
        "world",
        {"base_link"},
    )
    yaw_correction_summary = summarize_yaw_correction(
        raw_odometry_samples,
        body_odometry_samples,
        runtime_settings["body_yaw_offset"],
    )
    rosout_counts = count_rosout_levels(session / "rosout_warn_error.txt")

    output_dir.mkdir(parents=True, exist_ok=True)
    rate_csv = output_dir / "topic_rates_summary.csv"
    process_csv = output_dir / "process_health_summary.csv"
    sdk2_csv = output_dir / "sdk2_command_summary.csv"
    odometry_csv = output_dir / "body_odometry_audit.csv"
    planner_csv = output_dir / "planner_input_summary.csv"
    report_path = output_dir / "report.md"

    write_csv_atomic(
        rate_csv,
        ("topic", "samples", "ok_samples", "no_data_samples", "mean_hz", "min_hz", "max_hz", "latest_hz"),
        rate_summaries,
    )
    write_csv_atomic(
        process_csv,
        ("component", "samples", "running_samples", "running_percent", "last_state", "last_pids"),
        process_summaries,
    )
    sdk2_output = {
        **sdk2_summary,
        "configured_max_vx": runtime_settings["max_vx"].display_value,
        "configured_max_vy": runtime_settings["max_vy"].display_value,
        "configured_max_yaw_rate": runtime_settings["max_yaw_rate"].display_value,
        "max_vx_source": runtime_settings["max_vx"].source,
        "max_vy_source": runtime_settings["max_vy"].source,
        "max_yaw_rate_source": runtime_settings["max_yaw_rate"].source,
    }
    write_csv_atomic(
        sdk2_csv,
        (
            "samples",
            "ok_samples",
            "no_data_samples",
            "max_abs_vx",
            "max_abs_vy",
            "max_abs_yaw_rate",
            "latest_vx",
            "latest_vy",
            "latest_yaw_rate",
            "configured_max_vx",
            "configured_max_vy",
            "configured_max_yaw_rate",
            "max_vx_source",
            "max_vy_source",
            "max_yaw_rate_source",
        ),
        [sdk2_output],
    )
    odometry_output = {
        "raw_samples": raw_odometry_summary["samples"],
        "raw_ok_samples": raw_odometry_summary["ok_samples"],
        "raw_no_data_samples": raw_odometry_summary["no_data_samples"],
        "raw_invalid_samples": raw_odometry_summary["invalid_samples"],
        "raw_frame_mismatch_samples": raw_odometry_summary["frame_mismatch_samples"],
        "body_samples": body_odometry_summary["samples"],
        "body_ok_samples": body_odometry_summary["ok_samples"],
        "body_no_data_samples": body_odometry_summary["no_data_samples"],
        "body_invalid_samples": body_odometry_summary["invalid_samples"],
        "body_frame_mismatch_samples": body_odometry_summary["frame_mismatch_samples"],
        **yaw_correction_summary,
    }
    write_csv_atomic(
        odometry_csv,
        (
            "raw_samples",
            "raw_ok_samples",
            "raw_no_data_samples",
            "raw_invalid_samples",
            "raw_frame_mismatch_samples",
            "body_samples",
            "body_ok_samples",
            "body_no_data_samples",
            "body_invalid_samples",
            "body_frame_mismatch_samples",
            "matched_pairs",
            "configured_offset_rad",
            "configured_offset_source",
            "observed_mean_offset_rad",
            "max_abs_error_rad",
            "tolerance_rad",
            "status",
        ),
        [odometry_output],
    )
    write_csv_atomic(
        planner_csv,
        PLANNER_CSV_FIELDS,
        planner_summary["rows"],
    )

    event_levels = Counter(str(event.get("level", "unknown")) for event in events)
    event_categories = Counter(str(event.get("category", "unknown")) for event in events)
    observations = build_observations(
        rate_summaries,
        events,
        invalid_events,
        rosout_counts,
        session,
        runtime_settings["imu_target_hz"],
    )
    for key, setting_key, label in (
        ("max_abs_vx", "max_vx", "vx"),
        ("max_abs_vy", "max_vy", "vy"),
        ("max_abs_yaw_rate", "max_yaw_rate", "yaw rate"),
    ):
        value = as_float(str(sdk2_summary.get(key, "")))
        setting = runtime_settings[setting_key]
        if value is not None and setting.value is None:
            observations.append(
                f"SDK2 {label} limit was not captured; limit comparison was skipped."
            )
        elif value is not None and setting.value is not None and value > setting.value + 1e-6:
            observations.append(
                f"Observed SDK2 {label} {value:.3f}, above the captured limit "
                f"{setting.value:.3f} ({setting.source})."
            )
    observations.extend(
        odometry_observations(
            raw_odometry_summary,
            body_odometry_summary,
            yaw_correction_summary,
        )
    )
    observations.extend(
        planner_inspection_observations(
            planner_summary,
            malformed_planner_records,
        )
    )
    if not observations:
        observations.append(
            "No automatic warning condition was detected; inspect the raw logs for behavior-specific issues."
        )

    lines = [
        f"# Diagnostic Report: {markdown_cell(metadata.get('session_id', session.name))}",
        "",
        f"Generated: {dt.datetime.now(dt.timezone.utc).isoformat()}",
        "",
        "## Session",
        "",
        "| Field | Value |",
        "| --- | --- |",
    ]
    for key in ("started_at", "hostname", "machine", "platform", "source_repository", "size_limit_bytes"):
        lines.append(f"| {key} | {markdown_cell(metadata.get(key, '-'))} |")

    lines.extend(
        [
            "",
            "## Captured Runtime Configuration",
            "",
            "| Setting | Value | Source / fallback |",
            "| --- | ---: | --- |",
        ]
    )
    for label, key in (
        ("IMU target rate (Hz)", "imu_target_hz"),
        ("Body yaw offset (rad)", "body_yaw_offset"),
        ("SDK2 max vx", "max_vx"),
        ("SDK2 max vy", "max_vy"),
        ("SDK2 max yaw rate", "max_yaw_rate"),
    ):
        setting = runtime_settings[key]
        lines.append(
            f"| {label} | {setting.display_value} | {markdown_cell(setting.source)} |"
        )

    lines.extend(
        [
            "",
            "## Topic Rates",
            "",
            "| Topic | Samples | Valid | No data | Mean Hz | Min Hz | Max Hz | Latest Hz |",
            "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
        ]
    )
    for summary in rate_summaries:
        lines.append(
            "| {topic} | {samples} | {ok_samples} | {no_data_samples} | {mean} | {minimum} | {maximum} | {latest} |".format(
                topic=markdown_cell(summary["topic"]),
                samples=summary["samples"],
                ok_samples=summary["ok_samples"],
                no_data_samples=summary["no_data_samples"],
                mean=format_rate(rate_value(summary, "mean_hz")),
                minimum=format_rate(rate_value(summary, "min_hz")),
                maximum=format_rate(rate_value(summary, "max_hz")),
                latest=format_rate(rate_value(summary, "latest_hz")),
            )
        )

    lines.extend(
        [
            "",
            "## Planner Input Inspections",
            "",
            "Accepted records: "
            f"{len(planner_summary['accepted_records'])}; completed inspections: "
            f"{len(planner_summary['completed_records'])}; capture errors: "
            f"{planner_summary['capture_errors']}; malformed JSONL: "
            f"{malformed_planner_records}; schema-invalid: "
            f"{planner_summary['schema_invalid']}.",
            "",
            "| Diagnosis | Count |",
            "| --- | ---: |",
        ]
    )
    if planner_summary["diagnoses"]:
        for diagnosis, count in sorted(planner_summary["diagnoses"].items()):
            lines.append(f"| {markdown_cell(diagnosis)} | {count} |")
    else:
        lines.append("| - | 0 |")

    latest_attempt = planner_summary["latest_attempt"]
    if latest_attempt is not None:
        if latest_attempt["status"] == "capture_error":
            error = latest_attempt["error"]
            lines.extend(
                [
                    "",
                    "Latest attempt: "
                    f"{markdown_cell(latest_attempt['recorded_at'])}; "
                    "status=`capture_error`; "
                    f"error={markdown_cell(error['type'])}: "
                    f"{planner_error_message(error['message'])}.",
                ]
            )
        else:
            lines.extend(
                [
                    "",
                    "Latest attempt: "
                    f"{markdown_cell(latest_attempt['recorded_at'])}; "
                    f"status=`{latest_attempt['status']}`; "
                    "diagnosis=`"
                    f"{markdown_cell(latest_attempt['inspection']['diagnosis'])}`.",
                ]
            )

    latest_planner = planner_summary["latest"]
    if latest_planner is not None:
        latest_inspection = latest_planner["inspection"]
        latest_map = latest_inspection["map"]
        lines.extend(
            [
                "",
                "Latest completed inspection: "
                f"{markdown_cell(latest_planner.get('recorded_at', '-'))}; "
                f"diagnosis=`{markdown_cell(latest_inspection['diagnosis'])}`; "
                f"map={markdown_cell(latest_map.get('frame_id', '-'))} "
                f"{latest_map.get('width', '-')}x{latest_map.get('height', '-')} "
                f"at {latest_map.get('resolution', '-')} m.",
                "Map gates: "
                f"planner_gate_known={latest_map['planner_gate_known_cells']} "
                f"({latest_map['known_percent']:.3f}%); "
                f"planner_valid={latest_map['valid_cells']} "
                f"({latest_map['valid_percent']:.3f}%); "
                f"continuous_ground={latest_map['continuous_ground_cells']} "
                f"({latest_map['continuous_ground_percent']:.3f}%).",
                "Thresholds: "
                f"min_observed_frames={latest_inspection['thresholds']['min_observed_frames']}; "
                f"min_traversability={latest_inspection['thresholds']['min_traversability']}; "
                f"max_slope={latest_inspection['thresholds']['max_slope']}; "
                f"mapper_max_slope={latest_inspection['thresholds']['mapper_max_slope']}; "
                f"max_roughness={latest_inspection['thresholds']['max_roughness']}; "
                f"max_step_height={latest_inspection['thresholds']['max_step_height']}; "
                f"snap_radius={latest_inspection['thresholds']['snap_radius']}.",
                "Timing/topology: "
                "map_minus_odom="
                f"{planner_markdown_value(latest_inspection['freshness']['map_odom_stamp_delta_sec'])} s; "
                f"start_component={latest_inspection['start_component_cells']} cells "
                f"({latest_inspection['start_component_percent_of_ground']:.3f}% of ground); "
                "goal_in_start_component="
                f"{planner_markdown_value(latest_inspection['goal_in_start_component'])}.",
                "",
                "| Scope | Cells | Obs 0 | Obs below min | Obs ready | Elevation known | Features known | Slope over planner | Slope >= mapper | Roughness over | T=0 | T low | T accepted | Hard reject candidate | Planner valid |",
                "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
            ]
        )
        for row in planner_summary["latest_rows"]:
            lines.append(
                f"| {row['scope']} | {planner_markdown_value(row['cell_count'])} | "
                f"{planner_markdown_value(row['observation_zero'])} | "
                f"{planner_markdown_value(row['observation_below_min'])} | "
                f"{planner_markdown_value(row['observation_ready'])} | "
                f"{planner_markdown_value(row['elevation_known'])} | "
                f"{planner_markdown_value(row['features_known'])} | "
                f"{planner_markdown_value(row['slope_over_limit'])} | "
                f"{planner_markdown_value(row['slope_at_or_above_mapper_limit'])} | "
                f"{planner_markdown_value(row['roughness_over_limit'])} | "
                f"{planner_markdown_value(row['traversability_zero'])} | "
                f"{planner_markdown_value(row['traversability_below_min_nonzero'])} | "
                f"{planner_markdown_value(row['traversability_at_or_above_min'])} | "
                f"{planner_markdown_value(row['hard_reject_candidate'])} | "
                f"{planner_markdown_value(row['planner_valid'])} |"
            )
        lines.extend(
            [
                "",
                "Layer counts overlap. Hole-filled cells can have known elevation "
                "below the observation threshold.",
                "",
                "| Endpoint | Inside | Exact valid | Valid in snap square | Snapped | Grid | World position |",
                "| --- | --- | --- | ---: | --- | --- | --- |",
            ]
        )
        endpoint_rows = [
            row
            for row in planner_summary["latest_rows"]
            if row["scope"] in {"start", "goal"}
        ]
        for row in endpoint_rows:
            lines.append(
                f"| {row['scope']} | {row['inside_map']} | "
                f"{row['exact_cell_valid']} | "
                f"{planner_markdown_value(row['valid_cells_in_snap_square'])} | "
                f"{row['snapped']} | ({row['grid_x']}, {row['grid_y']}) | "
                f"({row['world_x']}, {row['world_y']}) |"
            )
    else:
        lines.extend(["", "No completed planner input inspection was captured."])

    lines.extend(
        [
            "",
            "## Body Odometry Audit",
            "",
            "| Topic | Samples | Valid | No data | Invalid | Frame mismatch | Latest yaw (deg) |",
            "| --- | ---: | ---: | ---: | ---: | ---: | ---: |",
        ]
    )
    for summary in (raw_odometry_summary, body_odometry_summary):
        lines.append(
            f"| {summary['topic']} | {summary['samples']} | {summary['ok_samples']} | "
            f"{summary['no_data_samples']} | {summary['invalid_samples']} | "
            f"{summary['frame_mismatch_samples']} | {summary['latest_yaw_deg'] or '-'} |"
        )
    lines.extend(
        [
            "",
            "| Matched pairs | Configured offset (rad) | Observed mean (rad) | Max error (rad) | Tolerance (rad) | Status |",
            "| ---: | ---: | ---: | ---: | ---: | --- |",
            "| {matched_pairs} | {configured} | {observed} | {error} | {tolerance} | {status} |".format(
                matched_pairs=yaw_correction_summary["matched_pairs"],
                configured=yaw_correction_summary["configured_offset_rad"] or "-",
                observed=yaw_correction_summary["observed_mean_offset_rad"] or "-",
                error=yaw_correction_summary["max_abs_error_rad"] or "-",
                tolerance=yaw_correction_summary["tolerance_rad"],
                status=yaw_correction_summary["status"],
            ),
            "",
            "## SDK2 Commands",
            "",
            "| Samples | Valid | No data | Max abs(vx) | Max abs(vy) | Max abs(yaw rate) | Latest vx | Latest vy | Latest yaw rate |",
            "| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
            "| {samples} | {ok_samples} | {no_data_samples} | {max_vx} | {max_vy} | {max_yaw} | {latest_vx} | {latest_vy} | {latest_yaw} |".format(
                samples=sdk2_summary["samples"],
                ok_samples=sdk2_summary["ok_samples"],
                no_data_samples=sdk2_summary["no_data_samples"],
                max_vx=format_rate(as_float(str(sdk2_summary["max_abs_vx"]))),
                max_vy=format_rate(as_float(str(sdk2_summary["max_abs_vy"]))),
                max_yaw=format_rate(as_float(str(sdk2_summary["max_abs_yaw_rate"]))),
                latest_vx=format_rate(as_float(str(sdk2_summary["latest_vx"]))),
                latest_vy=format_rate(as_float(str(sdk2_summary["latest_vy"]))),
                latest_yaw=format_rate(as_float(str(sdk2_summary["latest_yaw_rate"]))),
            ),
            "",
            "## Process Health",
            "",
            "| Component | Samples | Running | Running % | Last state | Last PIDs |",
            "| --- | ---: | ---: | ---: | --- | --- |",
        ]
    )
    if process_summaries:
        for summary in process_summaries:
            lines.append(
                f"| {markdown_cell(summary['component'])} | {summary['samples']} | "
                f"{summary['running_samples']} | {summary['running_percent']} | "
                f"{summary['last_state']} | {markdown_cell(summary['last_pids']) or '-'} |"
            )
    else:
        lines.append("| - | 0 | 0 | - | - | - |")

    lines.extend(
        [
            "",
            "## Events",
            "",
            f"Valid JSONL records: {len(events)}; malformed records: {invalid_events}.",
            "",
            "| Group | Count |",
            "| --- | ---: |",
        ]
    )
    for key, count in sorted(event_levels.items()):
        lines.append(f"| level: {markdown_cell(key)} | {count} |")
    for key, count in sorted(event_categories.items()):
        lines.append(f"| category: {markdown_cell(key)} | {count} |")
    if not events:
        lines.append("| events | 0 |")

    lines.extend(
        [
            "",
            "## ROS Warnings And Errors",
            "",
            "| Severity | Count |",
            "| --- | ---: |",
            f"| Warning | {rosout_counts['warning']} |",
            f"| Error | {rosout_counts['error']} |",
            f"| Fatal | {rosout_counts['fatal']} |",
            "",
            "## Observations",
            "",
        ]
    )
    lines.extend(f"- {item}" for item in observations)
    lines.extend(
        [
            "",
            "## Raw Inputs",
            "",
            "The analyzer does not modify raw files. Review `events.jsonl`, `topic_rates.csv`, "
            "`odom_position.csv`, `body_odom_pose.csv`, `sdk2_commands.csv`, `process_health.csv`, "
            "`planner_input_inspections.jsonl`, `rosout_warn_error.txt`, parameter dumps, "
            "and the Git/network/environment snapshots alongside this report.",
            "",
        ]
    )
    write_text_atomic(report_path, "\n".join(lines))
    return report_path, rate_csv, process_csv, sdk2_csv, odometry_csv, planner_csv


def main() -> int:
    args = parse_args()
    session = args.session.expanduser().resolve()
    if not session.is_dir():
        raise SystemExit(f"session directory does not exist: {session}")
    if not (session / "metadata.json").is_file():
        raise SystemExit(f"not a go2-log session (metadata.json is missing): {session}")
    output_dir = (args.output_dir or (session / "analysis")).expanduser().resolve()
    report, rate_csv, process_csv, sdk2_csv, odometry_csv, planner_csv = (
        generate_report(session, output_dir)
    )
    print(f"Report: {report}")
    print(f"Topic summary: {rate_csv}")
    print(f"Process summary: {process_csv}")
    print(f"SDK2 summary: {sdk2_csv}")
    print(f"Body odometry audit: {odometry_csv}")
    print(f"Planner input summary: {planner_csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
