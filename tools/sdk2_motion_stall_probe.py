#!/usr/bin/env python3
"""Synchronize read-only ROS and Unitree feedback for motion-stall diagnosis."""

from __future__ import annotations

import argparse
import bisect
from dataclasses import asdict, dataclass
import json
import math
import os
from pathlib import Path
import secrets
import signal
import stat
import statistics
import subprocess
import sys
import threading
import time
from typing import Any, Iterable, Sequence


NS_PER_SECOND = 1_000_000_000
MAX_CAPTURE_EVENTS = 120_000
MAX_CAPTURE_BYTES = 24 * 1024 * 1024
MAX_CAPTURE_METADATA_BYTES = 4 * 1024 * 1024
MAX_CAPTURE_DIAGNOSTIC_BYTES = 1 * 1024 * 1024


@dataclass(frozen=True)
class AnalysisConfig:
    command_gap_sec: float = 0.25
    odom_gap_sec: float = 0.50
    sport_gap_sec: float = 0.25
    remote_gap_sec: float = 0.25
    command_linear_min: float = 0.03
    command_yaw_min: float = 0.08
    sport_linear_min: float = 0.04
    sport_yaw_min: float = 0.05
    odom_linear_min: float = 0.04
    odom_yaw_min: float = 0.05
    response_max_age_sec: float = 0.55
    stall_min_sec: float = 0.75
    remote_axis_min: float = 0.08
    wake_window_sec: float = 2.0
    wake_sustain_sec: float = 0.75
    yaw_evidence_min_rad: float = 0.05
    yaw_opposite_fraction: float = 0.70


@dataclass(frozen=True)
class SessionWatch:
    active_session_file: Path
    expected_session: Path
    collector_pid: int
    proc_root: Path = Path("/proc")


def _read_bounded_regular_file(
    path: Path, max_bytes: int
) -> tuple[bytes | None, str | None]:
    try:
        path_status = os.lstat(path)
    except FileNotFoundError:
        return None, "is missing"
    except OSError:
        return None, "could not be inspected"
    if stat.S_ISLNK(path_status.st_mode):
        return None, "became a symbolic link"
    if not stat.S_ISREG(path_status.st_mode):
        return None, "is not a regular file"

    flags = os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0)
    try:
        file_fd = os.open(path, flags)
    except OSError:
        return None, "could not be opened"
    try:
        opened_status = os.fstat(file_fd)
        if not stat.S_ISREG(opened_status.st_mode):
            return None, "is not a regular file"
        if (
            path_status.st_dev,
            path_status.st_ino,
        ) != (
            opened_status.st_dev,
            opened_status.st_ino,
        ):
            return None, "changed while being read"
        content = os.read(file_fd, max_bytes + 1)
    except OSError:
        return None, "could not be read"
    finally:
        os.close(file_fd)
    if len(content) > max_bytes:
        return None, f"exceeds {max_bytes} bytes"
    return content, None


def session_invalidation_reason(watch: SessionWatch) -> str | None:
    try:
        session_status = os.lstat(watch.expected_session)
    except FileNotFoundError:
        return "active diagnostic session directory is missing"
    except OSError:
        return "active diagnostic session directory cannot be inspected"
    if stat.S_ISLNK(session_status.st_mode):
        return "active diagnostic session directory became a symbolic link"
    if not stat.S_ISDIR(session_status.st_mode):
        return "active diagnostic session path is not a directory"

    ended_marker = watch.expected_session / "ended_at.txt"
    uploaded_marker = watch.expected_session / ".uploaded"
    if ended_marker.exists() or ended_marker.is_symlink():
        return "active diagnostic session ended"
    if uploaded_marker.exists() or uploaded_marker.is_symlink():
        return "active diagnostic session was uploaded"

    active_content, active_error = _read_bounded_regular_file(
        watch.active_session_file, 4096
    )
    if active_error is not None:
        return f"active diagnostic session marker {active_error}"
    assert active_content is not None
    try:
        active_lines = active_content.decode("utf-8").splitlines()
    except UnicodeDecodeError:
        return "active diagnostic session marker is not UTF-8"
    if len(active_lines) != 1 or not active_lines[0]:
        return "active diagnostic session marker is invalid"
    try:
        active_session = Path(active_lines[0]).resolve(strict=True)
        expected_session = watch.expected_session.resolve(strict=True)
    except OSError:
        return "active diagnostic session changed"
    if active_session != expected_session:
        return "active diagnostic session changed"

    collector_file = watch.expected_session / "collector.pid"
    collector_content, collector_error = _read_bounded_regular_file(
        collector_file, 64
    )
    if collector_error is not None:
        return f"active diagnostic collector PID file {collector_error}"
    assert collector_content is not None
    try:
        collector_pid_text = collector_content.decode("ascii").strip()
        collector_pid = int(collector_pid_text)
    except (UnicodeDecodeError, ValueError):
        return "active diagnostic collector PID file is invalid"
    if collector_pid != watch.collector_pid or collector_pid <= 0:
        return "active diagnostic collector changed"
    try:
        os.kill(collector_pid, 0)
    except PermissionError:
        pass
    except (OSError, ValueError):
        return "active diagnostic collector stopped"

    command_file = watch.proc_root / str(collector_pid) / "cmdline"
    command_content, command_error = _read_bounded_regular_file(
        command_file, 1024 * 1024
    )
    if command_error is not None:
        return "active diagnostic collector stopped"
    assert command_content is not None
    arguments = [argument for argument in command_content.split(b"\0") if argument]
    expected_bytes = os.fsencode(str(expected_session))
    if (
        not any(b"go2-log" in argument for argument in arguments)
        or b"_collect" not in arguments
        or not any(expected_bytes in argument for argument in arguments)
    ):
        return "active diagnostic collector identity changed"
    return None


def _number(event: dict[str, Any], key: str) -> float | None:
    value = event.get(key)
    if isinstance(value, bool):
        return None
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _finite_or_none(value: Any) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _mono_ns(event: dict[str, Any]) -> int | None:
    value = event.get("mono_ns")
    if isinstance(value, bool):
        return None
    try:
        result = int(value)
    except (TypeError, ValueError):
        return None
    return result if result >= 0 else None


def _normalize_angle(angle: float) -> float:
    return math.remainder(angle, 2.0 * math.pi)


def _percentile(values: Sequence[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = int(math.ceil(fraction * len(ordered))) - 1
    return ordered[max(0, min(index, len(ordered) - 1))]


def _source_events(
    events: Iterable[dict[str, Any]], source: str
) -> list[dict[str, Any]]:
    selected = [
        event
        for event in events
        if event.get("source") == source and _mono_ns(event) is not None
    ]
    selected.sort(key=lambda event: int(event["mono_ns"]))
    return selected


def stream_summary(
    events: Sequence[dict[str, Any]], gap_limit_sec: float
) -> dict[str, Any]:
    times = [int(event["mono_ns"]) for event in events]
    gaps_ms = [
        (current - previous) / 1_000_000.0
        for previous, current in zip(times, times[1:])
        if current >= previous
    ]
    if not times:
        status = "no_data"
        rate_hz = None
    elif len(times) == 1:
        status = "single_sample"
        rate_hz = None
    else:
        span_sec = (times[-1] - times[0]) / NS_PER_SECOND
        rate_hz = (len(times) - 1) / span_sec if span_sec > 0.0 else None
        max_gap_sec = max(gaps_ms) / 1000.0 if gaps_ms else math.inf
        status = "gapped" if max_gap_sec > gap_limit_sec else "ok"
    return {
        "status": status,
        "count": len(times),
        "rate_hz": rate_hz,
        "gap_limit_ms": gap_limit_sec * 1000.0,
        "gap_median_ms": statistics.median(gaps_ms) if gaps_ms else None,
        "gap_p95_ms": _percentile(gaps_ms, 0.95),
        "gap_max_ms": max(gaps_ms) if gaps_ms else None,
    }


def derive_odom_motion(
    odometry: Sequence[dict[str, Any]], max_pair_gap_sec: float
) -> list[dict[str, Any]]:
    derived: list[dict[str, Any]] = []
    for previous, current in zip(odometry, odometry[1:]):
        previous_ns = int(previous["mono_ns"])
        current_ns = int(current["mono_ns"])
        dt = (current_ns - previous_ns) / NS_PER_SECOND
        if not 0.0 < dt <= max_pair_gap_sec:
            continue
        values = [
            _number(previous, "x"),
            _number(previous, "y"),
            _number(previous, "yaw"),
            _number(current, "x"),
            _number(current, "y"),
            _number(current, "yaw"),
        ]
        if any(value is None for value in values):
            continue
        previous_x, previous_y, previous_yaw, current_x, current_y, current_yaw = (
            float(value) for value in values
        )
        dx = current_x - previous_x
        dy = current_y - previous_y
        cosine = math.cos(previous_yaw)
        sine = math.sin(previous_yaw)
        body_vx = (cosine * dx + sine * dy) / dt
        body_vy = (-sine * dx + cosine * dy) / dt
        delta_yaw = _normalize_angle(current_yaw - previous_yaw)
        derived.append(
            {
                "mono_ns": current_ns,
                "dt": dt,
                "body_vx": body_vx,
                "body_vy": body_vy,
                "linear_speed": math.hypot(dx, dy) / dt,
                "yaw_rate": delta_yaw / dt,
                "delta_yaw": delta_yaw,
            }
        )
    return derived


def _latest_at_or_before(
    events: Sequence[dict[str, Any]],
    times: Sequence[int],
    timestamp_ns: int,
    max_age_sec: float,
) -> dict[str, Any] | None:
    index = bisect.bisect_right(times, timestamp_ns) - 1
    if index < 0:
        return None
    age_sec = (timestamp_ns - times[index]) / NS_PER_SECOND
    return events[index] if age_sec <= max_age_sec else None


def _motion_response(evidence: Sequence[bool | None]) -> bool | None:
    if any(value is True for value in evidence):
        return True
    if evidence and all(value is False for value in evidence):
        return False
    return None


def command_evaluations(
    commands: Sequence[dict[str, Any]],
    sport: Sequence[dict[str, Any]],
    odom_motion: Sequence[dict[str, Any]],
    config: AnalysisConfig,
) -> list[dict[str, Any]]:
    sport_times = [int(event["mono_ns"]) for event in sport]
    odom_times = [int(event["mono_ns"]) for event in odom_motion]
    evaluations: list[dict[str, Any]] = []
    for command in commands:
        timestamp_ns = int(command["mono_ns"])
        vx = _number(command, "vx")
        vy = _number(command, "vy")
        yaw_rate = _number(command, "yaw_rate")
        if vx is None or vy is None or yaw_rate is None:
            continue
        linear_commanded = math.hypot(vx, vy) >= config.command_linear_min
        yaw_commanded = abs(yaw_rate) >= config.command_yaw_min
        if not linear_commanded and not yaw_commanded:
            evaluations.append(
                {
                    "mono_ns": timestamp_ns,
                    "active": False,
                    "moving": None,
                    "sport_moving": None,
                    "odom_moving": None,
                }
            )
            continue

        sport_sample = _latest_at_or_before(
            sport, sport_times, timestamp_ns, config.response_max_age_sec
        )
        odom_sample = _latest_at_or_before(
            odom_motion, odom_times, timestamp_ns, config.response_max_age_sec
        )
        sport_evidence: list[bool | None] = []
        if sport_sample is not None:
            sport_vx = _number(sport_sample, "vx")
            sport_vy = _number(sport_sample, "vy")
            sport_yaw = _number(sport_sample, "yaw_rate")
            if linear_commanded:
                sport_evidence.append(
                    None
                    if sport_vx is None or sport_vy is None
                    else math.hypot(sport_vx, sport_vy)
                    >= config.sport_linear_min
                )
            if yaw_commanded:
                sport_evidence.append(
                    None
                    if sport_yaw is None
                    else abs(sport_yaw) >= config.sport_yaw_min
                )
        sport_moving = _motion_response(sport_evidence)

        odom_evidence: list[bool | None] = []
        if odom_sample is not None:
            odom_linear = _number(odom_sample, "linear_speed")
            odom_yaw = _number(odom_sample, "yaw_rate")
            if linear_commanded:
                odom_evidence.append(
                    None
                    if odom_linear is None
                    else odom_linear >= config.odom_linear_min
                )
            if yaw_commanded:
                odom_evidence.append(
                    None
                    if odom_yaw is None
                    else abs(odom_yaw) >= config.odom_yaw_min
                )
        odom_moving = _motion_response(odom_evidence)
        combined_moving = _motion_response(
            [sport_moving, odom_moving]
        )
        evaluations.append(
            {
                "mono_ns": timestamp_ns,
                "active": True,
                "moving": combined_moving,
                "sport_moving": sport_moving,
                "odom_moving": odom_moving,
                "sport_response_mono_ns": (
                    int(sport_sample["mono_ns"])
                    if sport_sample is not None
                    else None
                ),
                "odom_response_mono_ns": (
                    int(odom_sample["mono_ns"])
                    if odom_sample is not None
                    else None
                ),
                "vx": vx,
                "vy": vy,
                "yaw_rate": yaw_rate,
            }
        )
    return evaluations


def find_stall_episodes(
    evaluations: Sequence[dict[str, Any]],
    config: AnalysisConfig,
    response_field: str = "moving",
) -> list[dict[str, Any]]:
    episodes: list[dict[str, Any]] = []
    start_ns: int | None = None
    end_ns: int | None = None
    samples = 0

    def finish() -> None:
        nonlocal start_ns, end_ns, samples
        if start_ns is not None and end_ns is not None:
            duration = (end_ns - start_ns) / NS_PER_SECOND
            if duration >= config.stall_min_sec:
                episodes.append(
                    {
                        "start_mono_ns": start_ns,
                        "end_mono_ns": end_ns,
                        "duration_sec": duration,
                        "command_samples": samples,
                    }
                )
        start_ns = None
        end_ns = None
        samples = 0

    previous_ns: int | None = None
    for evaluation in evaluations:
        timestamp_ns = int(evaluation["mono_ns"])
        continuous = (
            previous_ns is None
            or (timestamp_ns - previous_ns) / NS_PER_SECOND <= config.command_gap_sec
        )
        stalled = bool(evaluation.get("active")) and (
            evaluation.get(response_field) is False
        )
        if stalled and continuous:
            if start_ns is None:
                start_ns = timestamp_ns
            end_ns = timestamp_ns
            samples += 1
        elif stalled:
            finish()
            start_ns = timestamp_ns
            end_ns = timestamp_ns
            samples = 1
        else:
            finish()
        previous_ns = timestamp_ns
    finish()
    return episodes


def _has_active_command_window(
    evaluations: Sequence[dict[str, Any]],
    config: AnalysisConfig,
    response_field: str | None = None,
) -> bool:
    start_ns: int | None = None
    previous_ns: int | None = None
    for evaluation in evaluations:
        timestamp_ns = int(evaluation["mono_ns"])
        continuous = (
            previous_ns is not None
            and (timestamp_ns - previous_ns) / NS_PER_SECOND
            <= config.command_gap_sec
        )
        observable = (
            response_field is None or evaluation.get(response_field) is not None
        )
        if evaluation.get("active") and observable:
            if start_ns is None or not continuous:
                start_ns = timestamp_ns
            if (timestamp_ns - start_ns) / NS_PER_SECOND >= config.stall_min_sec:
                return True
        else:
            start_ns = None
        previous_ns = timestamp_ns
    return False


def _active_commands_cover_interval(
    evaluations: Sequence[dict[str, Any]],
    start_ns: int,
    end_ns: int,
    config: AnalysisConfig,
) -> bool:
    if not evaluations:
        return False
    timestamps = [int(item["mono_ns"]) for item in evaluations]
    first_index = bisect.bisect_right(timestamps, start_ns) - 1
    last_index = bisect.bisect_left(timestamps, end_ns)
    if first_index < 0 or last_index >= len(evaluations):
        return False
    selected = evaluations[first_index : last_index + 1]
    maximum_gap_ns = int(config.command_gap_sec * NS_PER_SECOND)
    if start_ns - int(selected[0]["mono_ns"]) > maximum_gap_ns:
        return False
    if int(selected[-1]["mono_ns"]) - end_ns > maximum_gap_ns:
        return False
    if any(not item.get("active") for item in selected):
        return False
    return all(
        int(current["mono_ns"]) - int(previous["mono_ns"]) <= maximum_gap_ns
        for previous, current in zip(selected, selected[1:])
    )


def _feedback_covers_active_commands(
    evaluations: Sequence[dict[str, Any]],
    feedback: Sequence[dict[str, Any]],
    response_field: str,
    max_age_sec: float,
) -> bool:
    active = [item for item in evaluations if item.get("active")]
    if not active:
        return True
    if not feedback:
        return False
    first_command_ns = int(active[0]["mono_ns"])
    last_command_ns = int(active[-1]["mono_ns"])
    first_feedback_ns = int(feedback[0]["mono_ns"])
    last_feedback_ns = int(feedback[-1]["mono_ns"])
    maximum_age_ns = int(max_age_sec * NS_PER_SECOND)
    if first_feedback_ns - first_command_ns > maximum_age_ns:
        return False
    if last_command_ns - last_feedback_ns > maximum_age_ns:
        return False
    return all(
        item.get(response_field) is not None
        for item in active
        if first_feedback_ns <= int(item["mono_ns"]) <= last_feedback_ns
    )


def _stream_covers_active_commands(
    events: Sequence[dict[str, Any]],
    evaluations: Sequence[dict[str, Any]],
    max_edge_gap_sec: float,
) -> bool:
    active = [item for item in evaluations if item.get("active")]
    if not active:
        return True
    if not events:
        return False
    maximum_gap_ns = int(max_edge_gap_sec * NS_PER_SECOND)
    first_active_ns = int(active[0]["mono_ns"])
    last_active_ns = int(active[-1]["mono_ns"])
    first_event_ns = int(events[0]["mono_ns"])
    last_event_ns = int(events[-1]["mono_ns"])
    return (
        first_event_ns - first_active_ns <= maximum_gap_ns
        and last_active_ns - last_event_ns <= maximum_gap_ns
    )


def remote_nudges(
    remote: Sequence[dict[str, Any]], config: AnalysisConfig
) -> list[dict[str, Any]]:
    nudges: list[dict[str, Any]] = []
    current: dict[str, Any] | None = None
    previous_ns: int | None = None
    for event in remote:
        timestamp_ns = int(event["mono_ns"])
        axes = {
            axis: _number(event, axis) or 0.0 for axis in ("lx", "ly", "rx", "ry")
        }
        dominant_axis, dominant_value = max(
            axes.items(), key=lambda item: abs(item[1])
        )
        active = abs(dominant_value) >= config.remote_axis_min
        stream_continuous = (
            previous_ns is None
            or (timestamp_ns - previous_ns) / NS_PER_SECOND <= config.remote_gap_sec
        )
        if active:
            if current is None or not stream_continuous:
                if current is not None:
                    nudges.append(current)
                current = {
                    "start_mono_ns": timestamp_ns,
                    "end_mono_ns": timestamp_ns,
                    "peak_axis": dominant_axis,
                    "peak_value": dominant_value,
                }
            else:
                current["end_mono_ns"] = timestamp_ns
                if abs(dominant_value) > abs(float(current["peak_value"])):
                    current["peak_axis"] = dominant_axis
                    current["peak_value"] = dominant_value
        elif current is not None:
            nudges.append(current)
            current = None
        previous_ns = timestamp_ns
    if current is not None:
        nudges.append(current)
    for nudge in nudges:
        nudge["duration_sec"] = (
            int(nudge["end_mono_ns"]) - int(nudge["start_mono_ns"])
        ) / NS_PER_SECOND
        nudge["forward_axis"] = nudge["peak_axis"] == "ly"
    return nudges


def remote_wakes(
    nudges: Sequence[dict[str, Any]],
    evaluations: Sequence[dict[str, Any]],
    config: AnalysisConfig,
) -> list[dict[str, Any]]:
    def sustained_response(
        samples: Sequence[dict[str, Any]], field: str
    ) -> dict[str, Any] | None:
        start_ns: int | None = None
        previous_ns: int | None = None
        for sample in samples:
            timestamp_ns = int(sample["mono_ns"])
            continuous = (
                previous_ns is None
                or (timestamp_ns - previous_ns) / NS_PER_SECOND
                <= config.command_gap_sec
            )
            responding = bool(sample.get("active")) and bool(sample.get(field))
            if not responding or not continuous:
                start_ns = timestamp_ns if responding else None
            elif start_ns is None:
                start_ns = timestamp_ns
            if (
                start_ns is not None
                and (timestamp_ns - start_ns) / NS_PER_SECOND
                >= config.wake_sustain_sec
            ):
                return {
                    "response_mono_ns": start_ns,
                    "confirmed_mono_ns": timestamp_ns,
                    "sustain_sec": (timestamp_ns - start_ns) / NS_PER_SECOND,
                }
            previous_ns = timestamp_ns
        return None

    wakes: list[dict[str, Any]] = []
    for index, nudge in enumerate(nudges):
        onset_ns = int(nudge["start_mono_ns"])
        centered_ns = int(nudge["end_mono_ns"])
        pre_start_ns = onset_ns - int(config.stall_min_sec * NS_PER_SECOND)
        post_end_ns = centered_ns + int(config.wake_window_sec * NS_PER_SECOND)
        if index + 1 < len(nudges):
            post_end_ns = min(
                post_end_ns, int(nudges[index + 1]["start_mono_ns"])
            )
        if not _active_commands_cover_interval(
            evaluations, pre_start_ns, onset_ns, config
        ):
            continue
        if not _active_commands_cover_interval(
            evaluations, onset_ns, centered_ns, config
        ):
            continue
        pre = [
            item
            for item in evaluations
            if pre_start_ns <= int(item["mono_ns"]) <= onset_ns
            and item.get("active")
        ]
        post = [
            item
            for item in evaluations
            if centered_ns < int(item["mono_ns"]) <= post_end_ns
            and item.get("active")
        ]
        if len(pre) < 2:
            continue
        pre_span = (int(pre[-1]["mono_ns"]) - int(pre[0]["mono_ns"])) / NS_PER_SECOND
        channel_results: dict[str, dict[str, Any]] = {}
        for channel, field in (
            ("sport", "sport_moving"),
            ("odom", "odom_moving"),
        ):
            known_pre = [
                item for item in pre if item.get(field) is not None
            ]
            if len(known_pre) >= 2:
                known_pre_span = (
                    int(known_pre[-1]["mono_ns"])
                    - int(known_pre[0]["mono_ns"])
                ) / NS_PER_SECOND
            else:
                known_pre_span = 0.0
            if known_pre_span >= config.stall_min_sec * 0.80:
                stationary_fraction = sum(
                    item.get(field) is False for item in known_pre
                ) / len(known_pre)
                resumed = sustained_response(post, field)
                if (
                    stationary_fraction >= 0.80
                    and resumed is not None
                    and _active_commands_cover_interval(
                        evaluations,
                        onset_ns,
                        int(resumed["confirmed_mono_ns"]),
                        config,
                    )
                ):
                    channel_results[channel] = {
                        "pre_stationary_span_sec": known_pre_span,
                        "pre_stationary_fraction": stationary_fraction,
                        **resumed,
                        "response_latency_sec": (
                            int(resumed["response_mono_ns"]) - centered_ns
                        ) / NS_PER_SECOND,
                    }
        if channel_results:
            response_ns = min(
                int(result["response_mono_ns"])
                for result in channel_results.values()
            )
            wakes.append(
                {
                    "nudge_start_mono_ns": onset_ns,
                    "nudge_end_mono_ns": centered_ns,
                    "nudge_peak_axis": nudge["peak_axis"],
                    "nudge_peak_value": nudge["peak_value"],
                    "pre_stall_span_sec": pre_span,
                    "response_mono_ns": response_ns,
                    "response_latency_sec": (
                        response_ns - centered_ns
                    ) / NS_PER_SECOND,
                    "sport_response": "sport" in channel_results,
                    "odom_response": "odom" in channel_results,
                    "channels": channel_results,
                }
            )
    return wakes


def _yaw_evidence_status(
    aligned: float,
    opposite: float,
    minimum: float,
    opposite_fraction: float,
) -> dict[str, Any]:
    total = aligned + opposite
    fraction = opposite / total if total > 0.0 else None
    if total < minimum:
        status = "insufficient_response"
    elif fraction is not None and fraction >= opposite_fraction:
        status = "mismatch"
    else:
        status = "match_or_mixed"
    return {
        "status": status,
        "aligned_evidence": aligned,
        "opposite_evidence": opposite,
        "opposite_fraction": fraction,
    }


def yaw_sign_analysis(
    commands: Sequence[dict[str, Any]],
    sport: Sequence[dict[str, Any]],
    odom_motion: Sequence[dict[str, Any]],
    config: AnalysisConfig,
) -> dict[str, Any]:
    command_times = [int(event["mono_ns"]) for event in commands]
    odom_aligned = 0.0
    odom_opposite = 0.0
    for sample in odom_motion:
        timestamp_ns = int(sample["mono_ns"])
        command = _latest_at_or_before(
            commands, command_times, timestamp_ns, config.command_gap_sec
        )
        if command is None:
            continue
        command_yaw = _number(command, "yaw_rate")
        delta_yaw = _number(sample, "delta_yaw")
        if (
            command_yaw is None
            or delta_yaw is None
            or abs(command_yaw) < config.command_yaw_min
        ):
            continue
        if abs(delta_yaw) < 1e-4:
            continue
        if math.copysign(1.0, command_yaw) == math.copysign(1.0, delta_yaw):
            odom_aligned += abs(delta_yaw)
        else:
            odom_opposite += abs(delta_yaw)

    sport_aligned = 0.0
    sport_opposite = 0.0
    previous_ns: int | None = None
    for sample in sport:
        timestamp_ns = int(sample["mono_ns"])
        dt = (
            (timestamp_ns - previous_ns) / NS_PER_SECOND
            if previous_ns is not None
            else 0.0
        )
        previous_ns = timestamp_ns
        command = _latest_at_or_before(
            commands, command_times, timestamp_ns, config.command_gap_sec
        )
        sport_yaw = _number(sample, "yaw_rate")
        if command is None or sport_yaw is None:
            continue
        command_yaw = _number(command, "yaw_rate")
        if (
            command_yaw is None
            or abs(command_yaw) < config.command_yaw_min
            or abs(sport_yaw) < config.sport_yaw_min
        ):
            continue
        if not 0.0 < dt <= config.sport_gap_sec:
            continue
        evidence = abs(sport_yaw) * dt
        if math.copysign(1.0, command_yaw) == math.copysign(1.0, sport_yaw):
            sport_aligned += evidence
        else:
            sport_opposite += evidence

    odom_result = _yaw_evidence_status(
        odom_aligned,
        odom_opposite,
        config.yaw_evidence_min_rad,
        config.yaw_opposite_fraction,
    )
    sport_result = _yaw_evidence_status(
        sport_aligned,
        sport_opposite,
        config.yaw_evidence_min_rad,
        config.yaw_opposite_fraction,
    )
    statuses = {odom_result["status"], sport_result["status"]}
    if "mismatch" in statuses and "match_or_mixed" in statuses:
        overall = "odom_sport_disagree"
    elif "mismatch" in statuses:
        overall = "mismatch"
    elif "match_or_mixed" in statuses:
        overall = "match_or_mixed"
    else:
        overall = "insufficient_response"
    return {"status": overall, "odom": odom_result, "sport": sport_result}


def _ordered_nonnegative_integers(
    events: Sequence[dict[str, Any]], key: str
) -> list[int]:
    result: list[int] = []
    seen: set[int] = set()
    for event in events:
        value = event.get(key)
        if isinstance(value, bool):
            continue
        try:
            candidate = int(value)
        except (TypeError, ValueError):
            continue
        if candidate < 0 or candidate in seen:
            continue
        seen.add(candidate)
        result.append(candidate)
    return result


def planning_summary(events: Sequence[dict[str, Any]]) -> dict[str, Any]:
    goals = _source_events(events, "goal")
    paths = _source_events(events, "path")
    pose_counts = _ordered_nonnegative_integers(paths, "pose_count")
    path_sequences = _ordered_nonnegative_integers(paths, "sequence")
    if paths:
        status = "path_observed"
    elif goals:
        status = "goal_without_path"
    else:
        status = "not_observed"
    return {
        "status": status,
        "goal_count": len(goals),
        "path_count": len(paths),
        "goal_generations_ns": _ordered_nonnegative_integers(
            goals, "goal_generation_ns"
        ),
        "path_goal_generations_ns": _ordered_nonnegative_integers(
            paths, "goal_generation_ns"
        ),
        "path_sequences": path_sequences,
        "path_pose_count_min": min(pose_counts) if pose_counts else None,
        "path_pose_count_max": max(pose_counts) if pose_counts else None,
    }


def analyze_events(
    events: Iterable[dict[str, Any]], config: AnalysisConfig | None = None
) -> dict[str, Any]:
    active_config = config or AnalysisConfig()
    event_list = [event for event in events if _mono_ns(event) is not None]
    event_list.sort(key=lambda event: int(event["mono_ns"]))
    commands = _source_events(event_list, "command")
    odometry = _source_events(event_list, "odom")
    sport = _source_events(event_list, "sport")
    remote = _source_events(event_list, "remote")
    odom_motion = derive_odom_motion(
        odometry, max_pair_gap_sec=active_config.odom_gap_sec
    )
    evaluations = command_evaluations(
        commands, sport, odom_motion, active_config
    )
    combined_stalls = find_stall_episodes(evaluations, active_config)
    sport_stalls = find_stall_episodes(
        evaluations, active_config, "sport_moving"
    )
    odom_stalls = find_stall_episodes(
        evaluations, active_config, "odom_moving"
    )
    nudges = remote_nudges(remote, active_config)
    streams = {
        "command": stream_summary(commands, active_config.command_gap_sec),
        "odom": stream_summary(odometry, active_config.odom_gap_sec),
        "sport": stream_summary(sport, active_config.sport_gap_sec),
        "remote": stream_summary(remote, active_config.remote_gap_sec),
    }
    active_evaluations = [item for item in evaluations if item.get("active")]
    active_commands = len(active_evaluations)
    active_window_complete = _has_active_command_window(evaluations, active_config)
    remote_coverage_complete = _stream_covers_active_commands(
        remote, evaluations, active_config.remote_gap_sec
    )
    if streams["remote"]["status"] == "ok" and not remote_coverage_complete:
        streams["remote"]["status"] = "partial"
    odom_active_window_complete = _has_active_command_window(
        evaluations, active_config, "odom_moving"
    )
    sport_active_window_complete = _has_active_command_window(
        evaluations, active_config, "sport_moving"
    )
    odom_feedback_healthy = (
        streams["odom"]["status"] == "ok"
        and (not active_window_complete or odom_active_window_complete)
        and _feedback_covers_active_commands(
            evaluations,
            odom_motion,
            "odom_moving",
            active_config.response_max_age_sec,
        )
    )
    sport_feedback_healthy = (
        streams["sport"]["status"] == "ok"
        and (not active_window_complete or sport_active_window_complete)
        and _feedback_covers_active_commands(
            evaluations,
            sport,
            "sport_moving",
            active_config.response_max_age_sec,
        )
    )
    required_complete = (
        all(
            streams[name]["status"] == "ok"
            for name in ("command", "odom", "sport", "remote")
        )
        and odom_feedback_healthy
        and sport_feedback_healthy
    )
    wakes = (
        remote_wakes(nudges, evaluations, active_config)
        if required_complete
        else []
    )
    if active_commands == 0 or not active_window_complete:
        stall_status = "insufficient_active_command"
    elif (
        streams["command"]["status"] != "ok"
        or not odom_feedback_healthy
    ):
        stall_status = "insufficient_feedback"
    elif odom_stalls:
        stall_status = "observed"
    else:
        stall_status = "not_observed"
    if streams["remote"]["status"] != "ok":
        wake_status = "insufficient_feedback"
    elif not nudges:
        wake_status = "no_remote_nudge"
    elif active_commands == 0:
        wake_status = "insufficient_active_command"
    elif not required_complete:
        wake_status = "insufficient_feedback"
    elif wakes:
        wake_status = "observed"
    elif any(
        any(
            any(
                int(nudge["start_mono_ns"])
                - int(active_config.stall_min_sec * NS_PER_SECOND)
                <= int(item["mono_ns"])
                <= int(nudge["start_mono_ns"])
                and item.get("active")
                and item.get(field) is False
                for item in evaluations
            )
            and any(
                int(nudge["start_mono_ns"])
                <= int(item["mono_ns"])
                <= int(nudge["end_mono_ns"])
                and item.get("active")
                and item.get(field) is True
                for item in evaluations
            )
            for field in ("sport_moving", "odom_moving")
        )
        and _active_commands_cover_interval(
            evaluations,
            int(nudge["end_mono_ns"]),
            int(nudge["end_mono_ns"])
            + int(active_config.wake_window_sec * NS_PER_SECOND),
            active_config,
        )
        and not any(
            item.get("active")
            and (
                (
                    item.get("sport_moving") is True
                    and item.get("sport_response_mono_ns") is not None
                    and int(nudge["end_mono_ns"])
                    < int(item["sport_response_mono_ns"])
                    <= int(nudge["end_mono_ns"])
                    + int(active_config.wake_window_sec * NS_PER_SECOND)
                )
                or (
                    item.get("odom_moving") is True
                    and item.get("odom_response_mono_ns") is not None
                    and int(nudge["end_mono_ns"])
                    < int(item["odom_response_mono_ns"])
                    <= int(nudge["end_mono_ns"])
                    + int(active_config.wake_window_sec * NS_PER_SECOND)
                )
            )
            for item in evaluations
        )
        for nudge in nudges
    ):
        wake_status = "remote_only_motion"
    else:
        wake_status = "not_observed"
    return {
        "schema": 1,
        "required_streams_complete": required_complete,
        "config": asdict(active_config),
        "streams": streams,
        "active_command_samples": active_commands,
        "stall": {
            "status": stall_status,
            "episodes": odom_stalls,
            "odom_episodes": odom_stalls,
            "sport_episodes": sport_stalls,
            "combined_episodes": combined_stalls,
        },
        "remote": {
            "nudge_count": len(nudges),
            "nudges": nudges,
            "wake_status": wake_status,
            "wakes": wakes,
        },
        "yaw_sign": yaw_sign_analysis(
            commands, sport, odom_motion, active_config
        ),
        # Goal/path evidence is intentionally not part of required stream
        # completeness: neither exists until the operator publishes a goal.
        "planning": planning_summary(event_list),
    }


def _header_stamp_ns(message: Any) -> int | None:
    try:
        seconds = int(message.header.stamp.sec)
        nanoseconds = int(message.header.stamp.nanosec)
    except (AttributeError, TypeError, ValueError):
        return None
    if seconds < 0 or not 0 <= nanoseconds < NS_PER_SECOND:
        return None
    return seconds * NS_PER_SECOND + nanoseconds


def _quaternion_yaw(orientation: Any) -> float | None:
    try:
        x = float(orientation.x)
        y = float(orientation.y)
        z = float(orientation.z)
        w = float(orientation.w)
    except (AttributeError, TypeError, ValueError):
        return None
    if not all(math.isfinite(value) for value in (x, y, z, w)):
        return None
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-9:
        return None
    x /= norm
    y /= norm
    z /= norm
    w /= norm
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def _stamped_pose_geometry(message: Any) -> dict[str, Any]:
    position = message.pose.position
    orientation = message.pose.orientation
    return {
        "header_ns": _header_stamp_ns(message),
        "frame_id": str(message.header.frame_id),
        "x": _finite_or_none(position.x),
        "y": _finite_or_none(position.y),
        "z": _finite_or_none(position.z),
        "qx": _finite_or_none(orientation.x),
        "qy": _finite_or_none(orientation.y),
        "qz": _finite_or_none(orientation.z),
        "qw": _finite_or_none(orientation.w),
        "yaw": _quaternion_yaw(orientation),
    }


def _path_goal_generation_ns(poses: Sequence[Any]) -> int | None:
    if not poses:
        return None
    generation = _header_stamp_ns(poses[0])
    if generation is None or generation <= 0:
        return None
    for pose in poses[1:]:
        if _header_stamp_ns(pose) != generation:
            return None
    return generation


def _goal_event(message: Any, mono_ns: int) -> dict[str, Any]:
    geometry = _stamped_pose_geometry(message)
    header_ns = geometry["header_ns"]
    return {
        "schema": 1,
        "source": "goal",
        "topic": "/goal_pose",
        "mono_ns": mono_ns,
        "goal_generation_ns": (
            header_ns if isinstance(header_ns, int) and header_ns > 0 else None
        ),
        **geometry,
    }


def _path_event(message: Any, mono_ns: int, sequence: int) -> dict[str, Any]:
    poses = list(message.poses)
    return {
        "schema": 1,
        "source": "path",
        "topic": "/body_path",
        "mono_ns": mono_ns,
        "header_ns": _header_stamp_ns(message),
        "frame_id": str(message.header.frame_id),
        "sequence": sequence,
        "goal_generation_ns": _path_goal_generation_ns(poses),
        "pose_count": len(poses),
        "poses": [
            {"index": index, **_stamped_pose_geometry(pose)}
            for index, pose in enumerate(poses)
        ],
    }


class EventStore:
    def __init__(
        self,
        max_events: int = MAX_CAPTURE_EVENTS,
        max_bytes: int = MAX_CAPTURE_BYTES,
    ) -> None:
        self._events: list[dict[str, Any]] = []
        self._lock = threading.Lock()
        self._max_events = max_events
        self._max_bytes = max_bytes
        self._encoded_bytes = 0
        self._limit_reason: str | None = None

    def append(self, event: dict[str, Any]) -> None:
        if _mono_ns(event) is None:
            return
        encoded_bytes = len(
            json.dumps(event, sort_keys=True, separators=(",", ":")).encode("utf-8")
        ) + 1
        with self._lock:
            if self._limit_reason is not None:
                return
            if len(self._events) >= self._max_events:
                self._limit_reason = f"event limit reached ({self._max_events})"
                return
            if self._encoded_bytes + encoded_bytes > self._max_bytes:
                self._limit_reason = f"event byte limit reached ({self._max_bytes})"
                return
            self._events.append(event)
            self._encoded_bytes += encoded_bytes

    def snapshot(self) -> list[dict[str, Any]]:
        with self._lock:
            return list(self._events)

    def clear(self) -> None:
        with self._lock:
            self._events.clear()
            self._encoded_bytes = 0
            self._limit_reason = None

    def has_source(self, source: str) -> bool:
        with self._lock:
            return any(event.get("source") == source for event in self._events)

    def latest_source(self, source: str) -> dict[str, Any] | None:
        with self._lock:
            for event in reversed(self._events):
                if event.get("source") == source:
                    return dict(event)
        return None

    def limit_reason(self) -> str | None:
        with self._lock:
            return self._limit_reason


class BoundedDiagnosticStore:
    def __init__(self, max_bytes: int = MAX_CAPTURE_DIAGNOSTIC_BYTES) -> None:
        self._max_bytes = max_bytes
        self._limit_reason = f"reader diagnostic byte limit reached ({max_bytes})"
        self._marker = f"[truncated: {self._limit_reason}]"
        self._marker_bytes = len(self._marker.encode("utf-8")) + 1
        if max_bytes < self._marker_bytes:
            raise ValueError("diagnostic byte limit cannot hold its truncation marker")
        self._lines: list[str] = []
        self._encoded_bytes = 0
        self._truncated = False
        self._lock = threading.Lock()

    def append(self, line: str) -> None:
        encoded_bytes = len(line.encode("utf-8")) + 1
        with self._lock:
            if self._truncated:
                return
            payload_limit = self._max_bytes - self._marker_bytes
            if self._encoded_bytes + encoded_bytes > payload_limit:
                self._truncated = True
                return
            self._lines.append(line)
            self._encoded_bytes += encoded_bytes

    def snapshot(self) -> list[str]:
        with self._lock:
            lines = list(self._lines)
            if self._truncated:
                lines.append(self._marker)
            return lines

    def limit_reason(self) -> str | None:
        with self._lock:
            return self._limit_reason if self._truncated else None


def _reader_output_thread(
    stream: Any,
    store: EventStore,
    errors: BoundedDiagnosticStore,
) -> None:
    for line in stream:
        line = line.strip()
        if not line:
            continue
        try:
            event = json.loads(line)
        except json.JSONDecodeError:
            errors.append(f"invalid reader JSON: {line[:200]}")
            continue
        if not isinstance(event, dict):
            errors.append("reader emitted a non-object JSON value")
            continue
        store.append(event)


def _reader_error_thread(stream: Any, errors: BoundedDiagnosticStore) -> None:
    for line in stream:
        line = line.strip()
        if line:
            errors.append(line)


def _remote_discovery_problem(store: EventStore) -> str | None:
    if store.has_source("remote"):
        return None
    lowstate = store.latest_source("lowstate")
    if lowstate is None:
        return "missing rt/lowstate"

    def nonnegative_integer(name: str) -> int | None:
        value = lowstate.get(name)
        if isinstance(value, bool) or not isinstance(value, int) or value < 0:
            return None
        return value

    details = []
    lowstate_count = nonnegative_integer("lowstate_frames")
    invalid_count = nonnegative_integer("invalid_remote_frames")
    packet_head = nonnegative_integer("packet_head")
    nonzero_bytes = nonnegative_integer("nonzero_bytes")
    if lowstate_count is not None:
        details.append(f"observed_lowstate_frames>={lowstate_count}")
    if invalid_count is not None:
        details.append(f"invalid_remote_frames>={invalid_count}")
    if packet_head is not None:
        details.append(f"latest_packet_head=0x{packet_head:04x}")
    if nonzero_bytes is not None:
        details.append(f"latest_nonzero_bytes={nonzero_bytes}")
    details.append("expected_packet_head=0x5551")
    return (
        "rt/lowstate present but wireless_remote packets are invalid ("
        + ", ".join(details)
        + ")"
    )


def capture_live(
    reader: Path,
    network_interface: str,
    duration_sec: float,
    discovery_timeout_sec: float,
    session_watch: SessionWatch | None = None,
) -> tuple[
    list[dict[str, Any]],
    list[str],
    int | None,
    str | None,
    str | None,
]:
    try:
        import rclpy
        from geometry_msgs.msg import PoseStamped, TwistStamped
        from nav_msgs.msg import Odometry, Path as PathMessage
        from rclpy.node import Node
        from rclpy.qos import (
            DurabilityPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )
    except ImportError as error:
        raise RuntimeError("ROS 2 Humble Python message packages are required") from error

    if not reader.is_file() or not os.access(reader, os.X_OK):
        raise RuntimeError(f"SDK2 state reader is not executable: {reader}")
    store = EventStore()
    errors = BoundedDiagnosticStore()
    child = subprocess.Popen(
        [str(reader), network_interface],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        bufsize=1,
    )
    assert child.stdout is not None
    assert child.stderr is not None
    output_thread = threading.Thread(
        target=_reader_output_thread,
        args=(child.stdout, store, errors),
        daemon=True,
    )
    error_thread = threading.Thread(
        target=_reader_error_thread,
        args=(child.stderr, errors),
        daemon=True,
    )
    child_status: int | None = None
    node: Any | None = None
    output_thread_started = False
    error_thread_started = False
    cleanup_done = False

    def cleanup() -> None:
        nonlocal child_status, cleanup_done
        if cleanup_done:
            return
        cleanup_done = True
        if node is not None:
            try:
                node.destroy_node()
            except Exception as error:  # pragma: no cover - defensive cleanup
                errors.append(f"ROS node cleanup failed: {error}")
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as error:  # pragma: no cover - defensive cleanup
            errors.append(f"ROS shutdown failed: {error}")
        try:
            if child.poll() is None:
                child.send_signal(signal.SIGINT)
                try:
                    child_status = child.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    child.kill()
                    child_status = child.wait(timeout=3.0)
            else:
                child_status = child.returncode
        except Exception as error:  # pragma: no cover - defensive cleanup
            errors.append(f"SDK2 reader cleanup failed: {error}")
            try:
                child.kill()
                child_status = child.wait(timeout=3.0)
            except Exception as kill_error:
                errors.append(f"SDK2 reader kill failed: {kill_error}")
        if output_thread_started:
            output_thread.join(timeout=1.0)
        if error_thread_started:
            error_thread.join(timeout=1.0)
        child.stdout.close()
        child.stderr.close()

    try:
        output_thread.start()
        output_thread_started = True
        error_thread.start()
        error_thread_started = True

        rclpy.init(args=[])
        node = Node(f"sdk2_motion_stall_probe_{os.getpid()}")
        command_qos = QoSProfile(
            depth=50,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        odom_qos = QoSProfile(
            depth=50,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        goal_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        path_qos = QoSProfile(
            depth=50,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        path_sequence = 0

        def command_callback(message: Any) -> None:
            store.append(
                {
                    "schema": 1,
                    "source": "command",
                    "topic": "/sdk2_command",
                    "mono_ns": time.monotonic_ns(),
                    "header_ns": _header_stamp_ns(message),
                    "vx": _finite_or_none(message.twist.linear.x),
                    "vy": _finite_or_none(message.twist.linear.y),
                    "yaw_rate": _finite_or_none(message.twist.angular.z),
                }
            )

        def odom_callback(message: Any) -> None:
            position = message.pose.pose.position
            store.append(
                {
                    "schema": 1,
                    "source": "odom",
                    "topic": "/lio/body_odom",
                    "mono_ns": time.monotonic_ns(),
                    "header_ns": _header_stamp_ns(message),
                    "frame_id": str(message.header.frame_id),
                    "child_frame_id": str(message.child_frame_id),
                    "x": _finite_or_none(position.x),
                    "y": _finite_or_none(position.y),
                    "z": _finite_or_none(position.z),
                    "yaw": _quaternion_yaw(message.pose.pose.orientation),
                }
            )

        def goal_callback(message: Any) -> None:
            store.append(_goal_event(message, time.monotonic_ns()))

        def path_callback(message: Any) -> None:
            nonlocal path_sequence
            path_sequence += 1
            store.append(_path_event(message, time.monotonic_ns(), path_sequence))

        subscriptions = [
            node.create_subscription(
                TwistStamped, "/sdk2_command", command_callback, command_qos
            ),
            node.create_subscription(
                Odometry, "/lio/body_odom", odom_callback, odom_qos
            ),
            node.create_subscription(
                PoseStamped, "/goal_pose", goal_callback, goal_qos
            ),
            node.create_subscription(
                PathMessage, "/body_path", path_callback, path_qos
            ),
        ]
        del subscriptions
    except BaseException:
        cleanup()
        raise

    capture_start_ns: int | None = None
    capture_end_ns: int | None = None
    capture_invalidation_reason: str | None = None

    def check_session() -> bool:
        nonlocal capture_invalidation_reason
        if session_watch is None or capture_invalidation_reason is not None:
            return capture_invalidation_reason is not None
        capture_invalidation_reason = session_invalidation_reason(session_watch)
        return capture_invalidation_reason is not None

    try:
        discovery_deadline = time.monotonic() + discovery_timeout_sec
        discovered = False
        while time.monotonic() < discovery_deadline:
            if check_session():
                break
            rclpy.spin_once(node, timeout_sec=0.05)
            if check_session():
                break
            if child.poll() is not None:
                raise RuntimeError(
                    f"SDK2 state reader exited during discovery with {child.returncode}"
                )
            if (
                node.count_publishers("/sdk2_command") > 0
                and node.count_publishers("/lio/body_odom") > 0
                and store.has_source("sport")
                and store.has_source("remote")
            ):
                discovered = True
                break
        if capture_invalidation_reason is None and not discovered:
            problems = []
            if node.count_publishers("/sdk2_command") == 0:
                problems.append("missing /sdk2_command publisher")
            if node.count_publishers("/lio/body_odom") == 0:
                problems.append("missing /lio/body_odom publisher")
            if not store.has_source("sport"):
                problems.append("missing rt/sportmodestate")
            remote_problem = _remote_discovery_problem(store)
            if remote_problem is not None:
                problems.append(remote_problem)
            raise RuntimeError("discovery timeout; " + "; ".join(problems))

        if capture_invalidation_reason is None:
            store.clear()
            path_sequence = 0
            capture_start_ns = time.monotonic_ns()
            print("READY: publish one fresh goal now. Keep the native remote centered.")
            print("Do not command the native remote while the SDK2 bridge is armed.")
            print("The probe is read-only and does not arm or control the robot.")
            deadline_ns = capture_start_ns + int(duration_sec * NS_PER_SECOND)
            while time.monotonic_ns() < deadline_ns:
                if check_session():
                    break
                remaining = max(
                    0.0, (deadline_ns - time.monotonic_ns()) / NS_PER_SECOND
                )
                rclpy.spin_once(node, timeout_sec=min(0.05, remaining))
                if check_session():
                    break
                if child.poll() is not None:
                    raise RuntimeError(
                        "SDK2 state reader exited during capture with "
                        f"{child.returncode}"
                    )
                if store.limit_reason() is not None:
                    break
            if capture_invalidation_reason is None:
                check_session()
            capture_end_ns = time.monotonic_ns()
    finally:
        cleanup()

    error_lines = errors.snapshot()
    events = store.snapshot()
    if capture_start_ns is not None and capture_end_ns is not None:
        events = [
            event
            for event in events
            if capture_start_ns <= int(event["mono_ns"]) <= capture_end_ns
        ]
    limit_reasons = [
        reason
        for reason in (store.limit_reason(), errors.limit_reason())
        if reason is not None
    ]
    capture_limit_reason = "; ".join(limit_reasons) if limit_reasons else None
    return (
        events,
        error_lines,
        child_status,
        capture_limit_reason,
        capture_invalidation_reason,
    )


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    events: list[dict[str, Any]] = []
    with path.open(encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, start=1):
            line = line.strip()
            if not line:
                continue
            try:
                event = json.loads(line)
            except json.JSONDecodeError as error:
                raise RuntimeError(
                    f"invalid JSON at {path}:{line_number}: {error}"
                ) from error
            if not isinstance(event, dict):
                raise RuntimeError(f"non-object JSON at {path}:{line_number}")
            events.append(event)
    return events


def write_capture(
    output_dir: Path,
    events: Iterable[dict[str, Any]],
    report: dict[str, Any],
    errors: Sequence[str],
    directory_fd: int | None = None,
) -> None:
    close_directory = False
    if directory_fd is None:
        output_dir.mkdir(parents=False, exist_ok=False, mode=0o700)
        directory_fd = _open_capture_directory(output_dir)
        close_directory = True
    event_list = [event for event in events if _mono_ns(event) is not None]
    event_list.sort(key=lambda event: int(event["mono_ns"]))
    events_text = "".join(
        json.dumps(event, sort_keys=True, separators=(",", ":")) + "\n"
        for event in event_list
    )
    report_text = json.dumps(report, indent=2, sort_keys=True) + "\n"
    errors_text = "\n".join(errors) + ("\n" if errors else "")
    if len(events_text.encode("utf-8")) > MAX_CAPTURE_BYTES:
        raise RuntimeError("serialized events exceed the capture byte limit")
    if len(report_text.encode("utf-8")) + len(errors_text.encode("utf-8")) > (
        MAX_CAPTURE_METADATA_BYTES
    ):
        raise RuntimeError("capture report exceeds the metadata byte limit")
    try:
        _write_capture_file(directory_fd, "events.jsonl", events_text)
        _write_capture_file(directory_fd, "report.json", report_text)
        _write_capture_file(directory_fd, "reader_stderr.txt", errors_text)
    finally:
        if close_directory:
            os.close(directory_fd)


def _open_capture_directory(output_dir: Path) -> int:
    if not output_dir.is_absolute():
        raise RuntimeError("live capture directory must be absolute")
    directory_flags = os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW
    parent_fd = os.open(output_dir.parent, directory_flags)
    try:
        return os.open(output_dir.name, directory_flags, dir_fd=parent_fd)
    finally:
        os.close(parent_fd)


def _write_capture_file(directory_fd: int, name: str, content: str) -> None:
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW
    temporary_name = f".{name}.tmp-{os.getpid()}-{secrets.token_hex(8)}"
    file_fd = os.open(temporary_name, flags, 0o600, dir_fd=directory_fd)
    published = False
    try:
        with os.fdopen(file_fd, "w", encoding="utf-8", newline="\n") as stream:
            stream.write(content)
            stream.flush()
            os.fsync(stream.fileno())
        os.link(
            temporary_name,
            name,
            src_dir_fd=directory_fd,
            dst_dir_fd=directory_fd,
            follow_symlinks=False,
        )
        published = True
    finally:
        try:
            os.unlink(temporary_name, dir_fd=directory_fd)
        except FileNotFoundError:
            pass
        if published:
            os.fsync(directory_fd)


def print_report(report: dict[str, Any], output_dir: Path | None) -> None:
    print("===== SDK2 MOTION STALL RESULT =====")
    print(
        "required_streams="
        + ("complete" if report["required_streams_complete"] else "incomplete")
    )
    for name in ("command", "odom", "sport", "remote"):
        stream = report["streams"][name]
        rate = stream["rate_hz"]
        max_gap = stream["gap_max_ms"]
        print(
            f"{name}: status={stream['status']} count={stream['count']} "
            f"rate_hz={rate if rate is not None else 'NA'} "
            f"max_gap_ms={max_gap if max_gap is not None else 'NA'}"
        )
    print(
        f"stall={report['stall']['status']} "
        f"episodes={len(report['stall']['episodes'])}"
    )
    print(
        f"remote_nudges={report['remote']['nudge_count']} "
        f"remote_wake={report['remote']['wake_status']}"
    )
    print(f"yaw_sign={report['yaw_sign']['status']}")
    planning = report["planning"]
    print(
        f"planning={planning['status']} goals={planning['goal_count']} "
        f"paths={planning['path_count']}"
    )
    if report.get("capture_invalidated") is not None:
        print(f"capture_invalidated={report['capture_invalidated']}")
    if output_dir is not None:
        print(f"capture_dir={output_dir}")


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Read-only synchronized Go2 motion-stall diagnostic",
        allow_abbrev=False,
    )
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--reader", type=Path, help="compiled SDK2 state reader")
    mode.add_argument("--replay", type=Path, help="analyze an existing events JSONL")
    parser.add_argument("--interface", default="enP8p1s0")
    parser.add_argument("--duration", type=float, default=45.0)
    parser.add_argument("--discovery-timeout", type=float, default=10.0)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--active-session-file", type=Path)
    parser.add_argument("--expected-session", type=Path)
    parser.add_argument("--collector-pid", type=int)
    args = parser.parse_args(argv)
    if args.reader is not None and args.output_dir is None:
        parser.error("--output-dir is required for a live capture")
    session_arguments = (
        args.active_session_file,
        args.expected_session,
        args.collector_pid,
    )
    if args.reader is not None and any(value is None for value in session_arguments):
        parser.error(
            "--active-session-file, --expected-session, and --collector-pid "
            "are required for a live capture"
        )
    if args.replay is not None and any(value is not None for value in session_arguments):
        parser.error("diagnostic session watch arguments are only valid in live mode")
    if args.reader is not None:
        assert args.active_session_file is not None
        assert args.expected_session is not None
        assert args.collector_pid is not None
        if not args.active_session_file.is_absolute():
            parser.error("--active-session-file must be absolute")
        if not args.expected_session.is_absolute():
            parser.error("--expected-session must be absolute")
        if args.collector_pid <= 0:
            parser.error("--collector-pid must be positive")
    if not math.isfinite(args.duration) or not 5.0 <= args.duration <= 120.0:
        parser.error("--duration must be finite and in [5, 120] seconds")
    if (
        not math.isfinite(args.discovery_timeout)
        or not 1.0 <= args.discovery_timeout <= 30.0
    ):
        parser.error("--discovery-timeout must be finite and in [1, 30] seconds")
    if not args.interface or any(character.isspace() for character in args.interface):
        parser.error("--interface must be one non-empty interface name")
    return args


def main(argv: Sequence[str] | None = None) -> int:
    capture_directory_fd: int | None = None
    try:
        args = parse_args(argv)
        errors: list[str] = []
        child_status: int | None = None
        capture_limit_reason: str | None = None
        capture_invalidation_reason: str | None = None
        if args.replay is not None:
            events = read_jsonl(args.replay)
            output_dir = args.output_dir
        else:
            if args.output_dir is None:
                raise RuntimeError("live capture output directory was not configured")
            if (
                args.active_session_file is None
                or args.expected_session is None
                or args.collector_pid is None
            ):
                raise RuntimeError("live capture session watch was not configured")
            capture_directory_fd = _open_capture_directory(args.output_dir)
            session_watch = SessionWatch(
                active_session_file=args.active_session_file,
                expected_session=args.expected_session,
                collector_pid=args.collector_pid,
            )
            (
                events,
                errors,
                child_status,
                capture_limit_reason,
                capture_invalidation_reason,
            ) = capture_live(
                args.reader.resolve(),
                args.interface,
                args.duration,
                args.discovery_timeout,
                session_watch,
            )
            output_dir = args.output_dir
        report = analyze_events(events)
        report["reader_exit"] = child_status
        report["capture_invalidated"] = capture_invalidation_reason
        if args.replay is None:
            report["capture_limit"] = capture_limit_reason
            if capture_limit_reason is not None:
                errors.append(capture_limit_reason)
                report["required_streams_complete"] = False
                report["stall"]["status"] = "insufficient_capture"
                report["remote"]["wake_status"] = "insufficient_feedback"
                report["yaw_sign"]["status"] = "insufficient_capture"
            if capture_invalidation_reason is not None:
                errors.append(
                    f"capture_invalidated: {capture_invalidation_reason}"
                )
                report["required_streams_complete"] = False
                report["stall"]["status"] = "insufficient_capture"
                report["remote"]["wake_status"] = "insufficient_feedback"
                report["yaw_sign"]["status"] = "insufficient_capture"
        if output_dir is not None:
            write_capture(
                output_dir,
                events,
                report,
                errors,
                directory_fd=capture_directory_fd,
            )
        print_report(report, output_dir)
        return 0 if report["required_streams_complete"] else 3
    except (OSError, RuntimeError, ValueError) as error:
        print(f"probe_error={error}", file=sys.stderr)
        return 1
    finally:
        if capture_directory_fd is not None:
            os.close(capture_directory_fd)


if __name__ == "__main__":
    raise SystemExit(main())
