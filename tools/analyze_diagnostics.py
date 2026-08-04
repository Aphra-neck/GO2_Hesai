#!/usr/bin/env python3
"""Build compact Markdown and CSV summaries for one go2-log session."""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
import re
import statistics
from collections import Counter, defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable


EXPECTED_TOPICS = ("/imu/data", "/lio/odom", "/body_path", "/terrain_costmap")
PARAMETER_SCALAR = re.compile(r"^\s*([A-Za-z0-9_.-]+):\s*([^#]+?)\s*(?:#.*)?$")


@dataclass(frozen=True)
class CapturedSetting:
    value: float | None
    source: str

    @property
    def display_value(self) -> str:
        return "not captured" if self.value is None else f"{self.value:.6g}"


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


def read_jsonl(path: Path) -> tuple[list[dict[str, Any]], int]:
    records: list[dict[str, Any]] = []
    invalid = 0
    try:
        lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    except OSError:
        return records, invalid
    for line in lines:
        if not line.strip():
            continue
        try:
            value = json.loads(line)
        except json.JSONDecodeError:
            invalid += 1
            continue
        if isinstance(value, dict):
            records.append(value)
        else:
            invalid += 1
    return records, invalid


def read_csv(path: Path) -> list[dict[str, str]]:
    try:
        with path.open("r", encoding="utf-8", errors="replace", newline="") as stream:
            return list(csv.DictReader(stream))
    except OSError:
        return []


def as_float(value: str | None) -> float | None:
    try:
        return float(value) if value not in (None, "") else None
    except ValueError:
        return None


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
    positive: bool,
) -> CapturedSetting:
    for path in sorted(session.glob("parameters_*.yaml")):
        lower_name = path.name.lower()
        if not any(hint in lower_name for hint in filename_hints):
            continue
        value = as_float(read_parameter_scalars(path).get(key))
        if value is None or (positive and value <= 0.0) or (not positive and value < 0.0):
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

    sdk2_hints = ("go2_sdk2_bridge", "sdk2_bridge")
    return {
        "imu_target_hz": imu_rate,
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
    if not observations:
        observations.append("No automatic warning condition was detected; inspect the raw logs for behavior-specific issues.")
    return observations


def generate_report(session: Path, output_dir: Path) -> tuple[Path, Path, Path, Path]:
    metadata = read_json(session / "metadata.json")
    runtime_settings = load_runtime_settings(session)
    rate_rows = read_csv(session / "topic_rates.csv")
    process_rows = read_csv(session / "process_health.csv")
    sdk2_rows = read_csv(session / "sdk2_commands.csv")
    events, invalid_events = read_jsonl(session / "events.jsonl")
    rate_summaries = summarize_rates(rate_rows)
    process_summaries = summarize_processes(process_rows)
    sdk2_summary = summarize_sdk2_commands(sdk2_rows)
    rosout_counts = count_rosout_levels(session / "rosout_warn_error.txt")

    output_dir.mkdir(parents=True, exist_ok=True)
    rate_csv = output_dir / "topic_rates_summary.csv"
    process_csv = output_dir / "process_health_summary.csv"
    sdk2_csv = output_dir / "sdk2_command_summary.csv"
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
            "`odom_position.csv`, `sdk2_commands.csv`, `process_health.csv`, "
            "`rosout_warn_error.txt`, parameter dumps, "
            "and the Git/network/environment snapshots alongside this report.",
            "",
        ]
    )
    write_text_atomic(report_path, "\n".join(lines))
    return report_path, rate_csv, process_csv, sdk2_csv


def main() -> int:
    args = parse_args()
    session = args.session.expanduser().resolve()
    if not session.is_dir():
        raise SystemExit(f"session directory does not exist: {session}")
    if not (session / "metadata.json").is_file():
        raise SystemExit(f"not a go2-log session (metadata.json is missing): {session}")
    output_dir = (args.output_dir or (session / "analysis")).expanduser().resolve()
    report, rate_csv, process_csv, sdk2_csv = generate_report(session, output_dir)
    print(f"Report: {report}")
    print(f"Topic summary: {rate_csv}")
    print(f"Process summary: {process_csv}")
    print(f"SDK2 summary: {sdk2_csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
