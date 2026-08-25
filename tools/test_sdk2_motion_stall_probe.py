#!/usr/bin/env python3

from __future__ import annotations

import ast
import io
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import types
import unittest
from unittest import mock

from sdk2_motion_stall_probe import (
    AnalysisConfig,
    BoundedDiagnosticStore,
    EventStore,
    MAX_CAPTURE_BYTES,
    SessionWatch,
    _remote_discovery_problem,
    analyze_events,
    capture_live,
    main,
    parse_args,
    session_invalidation_reason,
    write_capture,
)


TOOLS_ROOT = Path(__file__).resolve().parent
READER_SOURCE = TOOLS_ROOT / "sdk2_motion_stall_probe" / "state_reader.cpp"
PROBE_SOURCE = TOOLS_ROOT / "sdk2_motion_stall_probe.py"
RUNNER_SOURCE = TOOLS_ROOT / "run_sdk2_motion_stall_probe.sh"
NS = 1_000_000_000


def event(source: str, seconds: float, **values: object) -> dict[str, object]:
    return {
        "schema": 1,
        "source": source,
        "mono_ns": int(seconds * NS),
        **values,
    }


def regular_times(duration: float, rate_hz: float) -> list[float]:
    count = int(duration * rate_hz) + 1
    return [index / rate_hz for index in range(count)]


def complete_remote_stream(
    duration: float,
    nudge_start: float | None = None,
    nudge_end: float | None = None,
) -> list[dict[str, object]]:
    result = []
    for timestamp in regular_times(duration, 50.0):
        nudged = (
            nudge_start is not None
            and nudge_end is not None
            and nudge_start <= timestamp <= nudge_end
        )
        result.append(
            event(
                "remote",
                timestamp,
                lx=0.0,
                ly=0.22 if nudged else 0.0,
                rx=0.0,
                ry=0.0,
                buttons=0,
            )
        )
    return result


class StallClassificationTests(unittest.TestCase):
    def test_detects_stall_then_remote_nudge_wake(self) -> None:
        duration = 4.0
        events: list[dict[str, object]] = []
        for timestamp in regular_times(duration, 20.0):
            events.append(
                event("command", timestamp, vx=0.30, vy=0.0, yaw_rate=0.0)
            )
        for timestamp in regular_times(duration, 100.0):
            events.append(
                event(
                    "sport",
                    timestamp,
                    vx=0.20 if timestamp >= 2.12 else 0.0,
                    vy=0.0,
                    yaw_rate=0.0,
                )
            )
        x = 0.0
        for timestamp in regular_times(duration, 10.0):
            if timestamp >= 2.2:
                x += 0.02
            events.append(event("odom", timestamp, x=x, y=0.0, yaw=0.0))
        events.extend(complete_remote_stream(duration, 2.0, 2.10))

        report = analyze_events(events)

        self.assertTrue(report["required_streams_complete"])
        self.assertEqual(report["stall"]["status"], "observed")
        self.assertGreaterEqual(len(report["stall"]["episodes"]), 1)
        self.assertEqual(report["remote"]["nudge_count"], 1)
        self.assertEqual(report["remote"]["wake_status"], "observed")
        wake = report["remote"]["wakes"][0]
        self.assertEqual(wake["nudge_peak_axis"], "ly")
        self.assertLess(wake["response_latency_sec"], 1.0)
        self.assertTrue(wake["sport_response"] or wake["odom_response"])

    def test_motion_only_while_remote_is_deflected_is_not_a_wake(self) -> None:
        duration = 5.0
        events: list[dict[str, object]] = []
        for timestamp in regular_times(duration, 20.0):
            events.append(
                event("command", timestamp, vx=0.30, vy=0.0, yaw_rate=0.0)
            )
        for timestamp in regular_times(duration, 100.0):
            moving = 2.0 <= timestamp <= 2.10
            events.append(
                event(
                    "sport",
                    timestamp,
                    vx=0.20 if moving else 0.0,
                    vy=0.0,
                    yaw_rate=0.0,
                )
            )
        x = 0.0
        for timestamp in regular_times(duration, 10.0):
            if 2.0 <= timestamp <= 2.10:
                x += 0.02
            events.append(event("odom", timestamp, x=x, y=0.0, yaw=0.0))
        events.extend(complete_remote_stream(duration, 2.0, 2.10))

        report = analyze_events(events)

        self.assertEqual(report["remote"]["wake_status"], "remote_only_motion")
        self.assertFalse(report["remote"]["wakes"])

    def test_motion_continuing_after_remote_centers_is_not_remote_only(self) -> None:
        duration = 4.0
        events: list[dict[str, object]] = []
        for timestamp in regular_times(duration, 20.0):
            events.append(
                event("command", timestamp, vx=0.30, vy=0.0, yaw_rate=0.0)
            )
        for timestamp in regular_times(duration, 100.0):
            moving = 2.0 <= timestamp <= 2.60
            events.append(
                event(
                    "sport",
                    timestamp,
                    vx=0.20 if moving else 0.0,
                    vy=0.0,
                    yaw_rate=0.0,
                )
            )
        x = 0.0
        for timestamp in regular_times(duration, 10.0):
            if 2.0 <= timestamp <= 2.60:
                x += 0.02
            events.append(event("odom", timestamp, x=x, y=0.0, yaw=0.0))
        events.extend(complete_remote_stream(duration, 2.0, 2.10))

        report = analyze_events(events)

        self.assertEqual(report["remote"]["wake_status"], "not_observed")
        self.assertFalse(report["remote"]["wakes"])

    def test_remote_only_requires_both_feedback_channels_to_stop(self) -> None:
        duration = 4.0
        events: list[dict[str, object]] = []
        for timestamp in regular_times(duration, 20.0):
            events.append(
                event("command", timestamp, vx=0.30, vy=0.0, yaw_rate=0.0)
            )
        for timestamp in regular_times(duration, 100.0):
            moving = 2.0 <= timestamp <= 2.10
            events.append(
                event(
                    "sport",
                    timestamp,
                    vx=0.20 if moving else 0.0,
                    vy=0.0,
                    yaw_rate=0.0,
                )
            )
        x = 0.0
        for timestamp in regular_times(duration, 10.0):
            if 2.0 <= timestamp <= 2.60:
                x += 0.02
            events.append(event("odom", timestamp, x=x, y=0.0, yaw=0.0))
        events.extend(complete_remote_stream(duration, 2.0, 2.10))

        report = analyze_events(events)

        self.assertEqual(report["remote"]["wake_status"], "not_observed")
        self.assertFalse(report["remote"]["wakes"])

    def test_remote_only_requires_active_commands_after_centering(self) -> None:
        duration = 5.0
        events: list[dict[str, object]] = []
        for timestamp in regular_times(duration, 20.0):
            command_active = not 2.15 <= timestamp <= 2.80
            events.append(
                event(
                    "command",
                    timestamp,
                    vx=0.30 if command_active else 0.0,
                    vy=0.0,
                    yaw_rate=0.0,
                )
            )
        for timestamp in regular_times(duration, 100.0):
            moving = 2.0 <= timestamp <= 2.60
            events.append(
                event(
                    "sport",
                    timestamp,
                    vx=0.20 if moving else 0.0,
                    vy=0.0,
                    yaw_rate=0.0,
                )
            )
        x = 0.0
        for timestamp in regular_times(duration, 10.0):
            if 2.0 <= timestamp <= 2.60:
                x += 0.02
            events.append(event("odom", timestamp, x=x, y=0.0, yaw=0.0))
        events.extend(complete_remote_stream(duration, 2.0, 2.10))

        report = analyze_events(events)

        self.assertTrue(report["required_streams_complete"])
        self.assertEqual(report["remote"]["wake_status"], "not_observed")
        self.assertFalse(report["remote"]["wakes"])

    def test_feedback_gap_recovery_after_nudge_is_not_a_remote_wake(self) -> None:
        duration = 4.0
        events: list[dict[str, object]] = []
        for timestamp in regular_times(duration, 20.0):
            events.append(
                event("command", timestamp, vx=0.30, vy=0.0, yaw_rate=0.0)
            )
        for timestamp in regular_times(duration, 100.0):
            if 1.0 < timestamp < 2.2:
                continue
            events.append(
                event(
                    "sport",
                    timestamp,
                    vx=0.20 if timestamp >= 2.2 else 0.0,
                    vy=0.0,
                    yaw_rate=0.0,
                )
            )
        x = 0.0
        for timestamp in regular_times(duration, 10.0):
            if 1.0 < timestamp < 2.2:
                continue
            if timestamp >= 2.2:
                x += 0.02
            events.append(event("odom", timestamp, x=x, y=0.0, yaw=0.0))
        events.extend(complete_remote_stream(duration, 2.0, 2.10))

        report = analyze_events(events)

        self.assertFalse(report["required_streams_complete"])
        self.assertEqual(report["stall"]["status"], "insufficient_feedback")
        self.assertEqual(report["remote"]["wake_status"], "insufficient_feedback")
        self.assertFalse(report["remote"]["wakes"])

    def test_unknown_feedback_warmup_does_not_count_as_pre_nudge_stall(self) -> None:
        duration = 3.0
        events: list[dict[str, object]] = []
        for timestamp in regular_times(duration, 20.0):
            events.append(
                event("command", timestamp, vx=0.30, vy=0.0, yaw_rate=0.0)
            )
        for timestamp in regular_times(duration - 0.5, 100.0):
            sample_time = timestamp + 0.5
            events.append(
                event(
                    "sport",
                    sample_time,
                    vx=0.20 if sample_time >= 0.9 else 0.0,
                    vy=0.0,
                    yaw_rate=0.0,
                )
            )
        x = 0.0
        for timestamp in regular_times(duration - 0.4, 10.0):
            sample_time = timestamp + 0.4
            if sample_time >= 0.9:
                x += 0.02
            events.append(event("odom", sample_time, x=x, y=0.0, yaw=0.0))
        events.extend(complete_remote_stream(duration, 0.7, 0.8))

        report = analyze_events(events)

        self.assertTrue(report["required_streams_complete"])
        self.assertEqual(report["remote"]["wake_status"], "not_observed")
        self.assertFalse(report["remote"]["wakes"])

    def test_detects_odom_yaw_sign_opposite_to_command_and_sport(self) -> None:
        duration = 3.0
        events: list[dict[str, object]] = []
        for timestamp in regular_times(duration, 20.0):
            events.append(
                event("command", timestamp, vx=0.0, vy=0.0, yaw_rate=-0.8)
            )
        for timestamp in regular_times(duration, 100.0):
            events.append(
                event("sport", timestamp, vx=0.0, vy=0.0, yaw_rate=-0.4)
            )
        for timestamp in regular_times(duration, 10.0):
            events.append(
                event("odom", timestamp, x=0.0, y=0.0, yaw=0.4 * timestamp)
            )
        events.extend(complete_remote_stream(duration))

        report = analyze_events(events)

        self.assertEqual(report["yaw_sign"]["status"], "odom_sport_disagree")
        self.assertEqual(report["yaw_sign"]["odom"]["status"], "mismatch")
        self.assertEqual(
            report["yaw_sign"]["sport"]["status"], "match_or_mixed"
        )
        self.assertGreater(
            report["yaw_sign"]["odom"]["opposite_fraction"], 0.95
        )

    def test_odom_stall_is_visible_when_sport_already_reports_velocity(self) -> None:
        duration = 4.0
        events: list[dict[str, object]] = []
        for timestamp in regular_times(duration, 20.0):
            events.append(
                event("command", timestamp, vx=0.25, vy=0.0, yaw_rate=0.0)
            )
        for timestamp in regular_times(duration, 100.0):
            events.append(
                event("sport", timestamp, vx=0.20, vy=0.0, yaw_rate=0.0)
            )
        x = 0.0
        for timestamp in regular_times(duration, 10.0):
            if timestamp >= 2.2:
                x += 0.02
            events.append(event("odom", timestamp, x=x, y=0.0, yaw=0.0))
        events.extend(complete_remote_stream(duration, 2.0, 2.10))

        report = analyze_events(events)

        self.assertEqual(report["stall"]["status"], "observed")
        self.assertTrue(report["stall"]["odom_episodes"])
        self.assertFalse(report["stall"]["sport_episodes"])
        self.assertFalse(report["stall"]["combined_episodes"])
        self.assertEqual(report["remote"]["wake_status"], "observed")
        wake = report["remote"]["wakes"][0]
        self.assertTrue(wake["odom_response"])
        self.assertFalse(wake["sport_response"])

    def test_classifies_independent_command_odom_and_sport_gaps(self) -> None:
        config = AnalysisConfig()
        events = [
            event("command", 0.0, vx=0.2, vy=0.0, yaw_rate=0.0),
            event("command", 0.05, vx=0.2, vy=0.0, yaw_rate=0.0),
            event("command", 0.80, vx=0.2, vy=0.0, yaw_rate=0.0),
            event("odom", 0.0, x=0.0, y=0.0, yaw=0.0),
            event("odom", 0.1, x=0.01, y=0.0, yaw=0.0),
            event("odom", 0.75, x=0.02, y=0.0, yaw=0.0),
            event("sport", 0.0, vx=0.1, vy=0.0, yaw_rate=0.0),
            event("sport", 0.01, vx=0.1, vy=0.0, yaw_rate=0.0),
            event("sport", 0.40, vx=0.1, vy=0.0, yaw_rate=0.0),
            event("remote", 0.0, lx=0.0, ly=0.0, rx=0.0, ry=0.0),
            event("remote", 0.02, lx=0.0, ly=0.0, rx=0.0, ry=0.0),
        ]

        report = analyze_events(events, config)

        self.assertEqual(report["streams"]["command"]["status"], "gapped")
        self.assertEqual(report["streams"]["odom"]["status"], "gapped")
        self.assertEqual(report["streams"]["sport"]["status"], "gapped")
        self.assertEqual(report["streams"]["remote"]["status"], "partial")

    def test_gapped_feedback_is_not_classified_as_a_physical_stall(self) -> None:
        duration = 2.0
        events: list[dict[str, object]] = []
        for timestamp in regular_times(duration, 20.0):
            events.append(
                event("command", timestamp, vx=0.30, vy=0.0, yaw_rate=0.0)
            )
        for timestamp in (0.0, duration):
            events.append(event("odom", timestamp, x=0.0, y=0.0, yaw=0.0))
            events.append(
                event("sport", timestamp, vx=0.0, vy=0.0, yaw_rate=0.0)
            )
            events.append(
                event(
                    "remote",
                    timestamp,
                    lx=0.0,
                    ly=0.0,
                    rx=0.0,
                    ry=0.0,
                )
            )

        report = analyze_events(events)

        self.assertFalse(report["required_streams_complete"])
        self.assertEqual(report["stall"]["status"], "insufficient_feedback")
        self.assertFalse(report["stall"]["odom_episodes"])

    def test_no_active_command_is_not_reported_as_a_motion_stall(self) -> None:
        duration = 1.0
        events: list[dict[str, object]] = []
        for timestamp in regular_times(duration, 20.0):
            events.append(
                event("command", timestamp, vx=0.0, vy=0.0, yaw_rate=0.0)
            )
        for timestamp in regular_times(duration, 10.0):
            events.append(event("odom", timestamp, x=0.0, y=0.0, yaw=0.0))
        for timestamp in regular_times(duration, 100.0):
            events.append(
                event("sport", timestamp, vx=0.0, vy=0.0, yaw_rate=0.0)
            )
        events.extend(complete_remote_stream(duration))

        report = analyze_events(events)

        self.assertEqual(
            report["stall"]["status"], "insufficient_active_command"
        )
        self.assertEqual(report["remote"]["wake_status"], "no_remote_nudge")

    def test_short_active_command_window_is_insufficient_to_exclude_a_stall(self) -> None:
        duration = 2.0
        events: list[dict[str, object]] = []
        for timestamp in regular_times(duration, 20.0):
            events.append(
                event(
                    "command",
                    timestamp,
                    vx=0.30 if 1.0 <= timestamp <= 1.20 else 0.0,
                    vy=0.0,
                    yaw_rate=0.0,
                )
            )
        for timestamp in regular_times(duration, 100.0):
            events.append(
                event("sport", timestamp, vx=0.0, vy=0.0, yaw_rate=0.0)
            )
        for timestamp in regular_times(duration, 10.0):
            events.append(event("odom", timestamp, x=0.0, y=0.0, yaw=0.0))
        events.extend(complete_remote_stream(duration))

        report = analyze_events(events)

        self.assertTrue(report["required_streams_complete"])
        self.assertEqual(
            report["stall"]["status"], "insufficient_active_command"
        )

    def test_short_odom_evidence_window_is_insufficient_to_exclude_a_stall(
        self,
    ) -> None:
        events: list[dict[str, object]] = []
        for timestamp in regular_times(0.8, 20.0):
            events.append(
                event("command", timestamp, vx=0.30, vy=0.0, yaw_rate=0.0)
            )
        for timestamp in regular_times(0.8, 100.0):
            events.append(
                event("sport", timestamp, vx=0.20, vy=0.0, yaw_rate=0.0)
            )
        for index, timestamp in enumerate((0.45, 0.55, 0.65, 0.75, 0.85)):
            events.append(
                event("odom", timestamp, x=0.01 * index, y=0.0, yaw=0.0)
            )
        events.extend(complete_remote_stream(0.8))

        report = analyze_events(events)

        self.assertFalse(report["required_streams_complete"])
        self.assertEqual(report["stall"]["status"], "insufficient_feedback")

    def test_remote_wake_requires_nonzero_command_through_the_nudge(self) -> None:
        duration = 4.0
        events: list[dict[str, object]] = []
        for timestamp in regular_times(duration, 20.0):
            command_active = not 2.0 <= timestamp <= 2.10
            events.append(
                event(
                    "command",
                    timestamp,
                    vx=0.30 if command_active else 0.0,
                    vy=0.0,
                    yaw_rate=0.0,
                )
            )
        for timestamp in regular_times(duration, 100.0):
            events.append(
                event(
                    "sport",
                    timestamp,
                    vx=0.20 if timestamp >= 2.2 else 0.0,
                    vy=0.0,
                    yaw_rate=0.0,
                )
            )
        x = 0.0
        for timestamp in regular_times(duration, 10.0):
            if timestamp >= 2.2:
                x += 0.02
            events.append(event("odom", timestamp, x=x, y=0.0, yaw=0.0))
        events.extend(complete_remote_stream(duration, 2.0, 2.10))

        report = analyze_events(events)

        self.assertTrue(report["required_streams_complete"])
        self.assertEqual(report["remote"]["wake_status"], "not_observed")
        self.assertFalse(report["remote"]["wakes"])

    def test_remote_wake_requires_nonzero_command_after_centering(self) -> None:
        duration = 5.0
        events: list[dict[str, object]] = []
        for timestamp in regular_times(duration, 20.0):
            command_active = not 2.15 <= timestamp <= 2.50
            events.append(
                event(
                    "command",
                    timestamp,
                    vx=0.30 if command_active else 0.0,
                    vy=0.0,
                    yaw_rate=0.0,
                )
            )
        for timestamp in regular_times(duration, 100.0):
            events.append(
                event(
                    "sport",
                    timestamp,
                    vx=0.20 if timestamp >= 2.60 else 0.0,
                    vy=0.0,
                    yaw_rate=0.0,
                )
            )
        x = 0.0
        for timestamp in regular_times(duration, 10.0):
            if timestamp >= 2.60:
                x += 0.02
            events.append(event("odom", timestamp, x=x, y=0.0, yaw=0.0))
        events.extend(complete_remote_stream(duration, 2.0, 2.10))

        report = analyze_events(events)

        self.assertTrue(report["required_streams_complete"])
        self.assertEqual(report["remote"]["wake_status"], "not_observed")
        self.assertFalse(report["remote"]["wakes"])

    def test_remote_wake_requires_nonzero_command_before_the_nudge(self) -> None:
        duration = 5.0
        events: list[dict[str, object]] = []
        for timestamp in regular_times(duration, 20.0):
            command_active = not 1.60 <= timestamp <= 1.80
            events.append(
                event(
                    "command",
                    timestamp,
                    vx=0.30 if command_active else 0.0,
                    vy=0.0,
                    yaw_rate=0.0,
                )
            )
        for timestamp in regular_times(duration, 100.0):
            events.append(
                event(
                    "sport",
                    timestamp,
                    vx=0.20 if timestamp >= 2.20 else 0.0,
                    vy=0.0,
                    yaw_rate=0.0,
                )
            )
        x = 0.0
        for timestamp in regular_times(duration, 10.0):
            if timestamp >= 2.20:
                x += 0.02
            events.append(event("odom", timestamp, x=x, y=0.0, yaw=0.0))
        events.extend(complete_remote_stream(duration, 2.0, 2.10))

        report = analyze_events(events)

        self.assertTrue(report["required_streams_complete"])
        self.assertEqual(report["remote"]["wake_status"], "not_observed")
        self.assertFalse(report["remote"]["wakes"])

    def test_remote_stream_must_cover_the_active_command_interval(self) -> None:
        duration = 4.0
        events: list[dict[str, object]] = []
        for timestamp in regular_times(duration, 20.0):
            events.append(
                event("command", timestamp, vx=0.30, vy=0.0, yaw_rate=0.0)
            )
        for timestamp in regular_times(duration, 100.0):
            events.append(
                event("sport", timestamp, vx=0.20, vy=0.0, yaw_rate=0.0)
            )
        for timestamp in regular_times(duration, 10.0):
            events.append(
                event("odom", timestamp, x=0.10 * timestamp, y=0.0, yaw=0.0)
            )
        events.extend(complete_remote_stream(0.02))

        report = analyze_events(events)

        self.assertNotEqual(report["streams"]["remote"]["status"], "ok")
        self.assertFalse(report["required_streams_complete"])
        self.assertEqual(report["remote"]["wake_status"], "insufficient_feedback")

    def test_remote_only_motion_requires_motion_to_be_confined_to_nudge(self) -> None:
        duration = 4.0
        events: list[dict[str, object]] = []
        for timestamp in regular_times(duration, 20.0):
            events.append(
                event("command", timestamp, vx=0.30, vy=0.0, yaw_rate=0.0)
            )
        for timestamp in regular_times(duration, 100.0):
            events.append(
                event("sport", timestamp, vx=0.20, vy=0.0, yaw_rate=0.0)
            )
        for timestamp in regular_times(duration, 10.0):
            events.append(
                event("odom", timestamp, x=0.10 * timestamp, y=0.0, yaw=0.0)
            )
        events.extend(complete_remote_stream(duration, 2.0, 2.10))

        report = analyze_events(events)

        self.assertEqual(report["remote"]["wake_status"], "not_observed")


class ReadOnlySurfaceTests(unittest.TestCase):
    def test_session_watch_detects_an_ended_active_session(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            log_root = Path(directory)
            session = log_root / "sessions" / "session-real"
            session.mkdir(parents=True)
            active_file = log_root / "active_session"
            active_file.write_text(f"{session}\n", encoding="utf-8")
            (session / "ended_at.txt").write_text("ended\n", encoding="utf-8")
            watch = SessionWatch(active_file, session, os.getpid())

            reason = session_invalidation_reason(watch)

        self.assertEqual(reason, "active diagnostic session ended")

    def test_session_watch_accepts_the_pinned_collector_identity(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            log_root = Path(directory)
            session = log_root / "sessions" / "session-real"
            session.mkdir(parents=True)
            active_file = log_root / "active_session"
            active_file.write_text(f"{session}\n", encoding="utf-8")
            collector_pid = os.getpid()
            (session / "collector.pid").write_text(
                f"{collector_pid}\n", encoding="ascii"
            )
            proc_root = log_root / "proc"
            process_dir = proc_root / str(collector_pid)
            process_dir.mkdir(parents=True)
            (process_dir / "cmdline").write_bytes(
                b"bash\0/tools/go2-log\0_collect\0"
                + os.fsencode(str(session.resolve()))
                + b"\0"
            )
            watch = SessionWatch(
                active_file,
                session,
                collector_pid,
                proc_root=proc_root,
            )

            reason = session_invalidation_reason(watch)

        self.assertIsNone(reason)

    @unittest.skipUnless(
        hasattr(os, "O_DIRECTORY") and hasattr(os, "O_NOFOLLOW"),
        "directory-fd capture writes require Linux open flags",
    )
    def test_invalidated_capture_is_saved_and_returns_nonzero(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            output_dir = root / "capture"
            output_dir.mkdir(mode=0o700)
            active_file = root / "active_session"
            expected_session = root / "sessions" / "session-real"
            expected_session.mkdir(parents=True)
            active_file.write_text(f"{expected_session}\n", encoding="utf-8")
            invalidation = "active diagnostic collector stopped"
            with mock.patch(
                "sdk2_motion_stall_probe.capture_live",
                return_value=(
                    [event("reader", 0.0, event="started")],
                    [],
                    0,
                    None,
                    invalidation,
                ),
            ):
                status = main(
                    [
                        "--reader",
                        sys.executable,
                        "--output-dir",
                        str(output_dir),
                        "--active-session-file",
                        str(active_file),
                        "--expected-session",
                        str(expected_session),
                        "--collector-pid",
                        str(os.getpid()),
                        "--duration",
                        "5",
                    ]
                )

            report = json.loads(
                (output_dir / "report.json").read_text(encoding="utf-8")
            )
            events_text = (output_dir / "events.jsonl").read_text(encoding="utf-8")

        self.assertNotEqual(status, 0)
        self.assertEqual(report["capture_invalidated"], invalidation)
        self.assertFalse(report["required_streams_complete"])
        self.assertEqual(report["stall"]["status"], "insufficient_capture")
        self.assertIn('"source":"reader"', events_text)

    def test_capture_loop_stops_and_reaps_reader_when_session_is_invalidated(
        self,
    ) -> None:
        child = mock.Mock()
        mono_ns = time.monotonic_ns()
        child.stdout = io.StringIO(
            json.dumps({"schema": 1, "source": "sport", "mono_ns": mono_ns})
            + "\n"
            + json.dumps(
                {"schema": 1, "source": "remote", "mono_ns": mono_ns + 1}
            )
            + "\n"
        )
        child.stderr = io.StringIO()
        child.poll.return_value = None
        child.wait.return_value = 0
        child.returncode = 0

        class FakeNode:
            destroyed = False

            def __init__(self, _name: str) -> None:
                pass

            def create_subscription(self, *_args: object) -> object:
                return object()

            def count_publishers(self, _topic: str) -> int:
                return 1

            def destroy_node(self) -> None:
                self.destroyed = True

        rclpy = types.ModuleType("rclpy")
        rclpy.init = mock.Mock()
        rclpy.ok = mock.Mock(return_value=True)
        rclpy.shutdown = mock.Mock()
        rclpy.spin_once = mock.Mock(side_effect=lambda *_args, **_kwargs: time.sleep(0.01))
        rclpy_node = types.ModuleType("rclpy.node")
        rclpy_node.Node = FakeNode
        rclpy_qos = types.ModuleType("rclpy.qos")
        rclpy_qos.DurabilityPolicy = types.SimpleNamespace(VOLATILE=1)
        rclpy_qos.QoSProfile = lambda **_kwargs: object()
        rclpy_qos.ReliabilityPolicy = types.SimpleNamespace(
            RELIABLE=1, BEST_EFFORT=2
        )
        geometry_msgs = types.ModuleType("geometry_msgs")
        geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")
        geometry_msgs_msg.TwistStamped = object
        nav_msgs = types.ModuleType("nav_msgs")
        nav_msgs_msg = types.ModuleType("nav_msgs.msg")
        nav_msgs_msg.Odometry = object
        modules = {
            "rclpy": rclpy,
            "rclpy.node": rclpy_node,
            "rclpy.qos": rclpy_qos,
            "geometry_msgs": geometry_msgs,
            "geometry_msgs.msg": geometry_msgs_msg,
            "nav_msgs": nav_msgs,
            "nav_msgs.msg": nav_msgs_msg,
        }
        invalidation = "active diagnostic collector stopped"
        checks = iter((None, None, invalidation))

        with (
            mock.patch.dict(sys.modules, modules),
            mock.patch(
                "sdk2_motion_stall_probe.subprocess.Popen", return_value=child
            ),
            mock.patch(
                "sdk2_motion_stall_probe.session_invalidation_reason",
                side_effect=lambda _watch: next(checks, invalidation),
            ),
        ):
            result = capture_live(
                Path(sys.executable),
                "enP8p1s0",
                5.0,
                1.0,
                SessionWatch(Path("active"), Path("session"), os.getpid()),
            )

        self.assertEqual(result[4], invalidation)
        child.send_signal.assert_called_once_with(signal.SIGINT)
        child.wait.assert_called_once_with(timeout=3.0)
        self.assertTrue(child.stdout.closed)
        self.assertTrue(child.stderr.closed)

    @unittest.skipUnless(
        hasattr(os, "O_DIRECTORY") and hasattr(os, "O_NOFOLLOW"),
        "directory-fd capture writes require Linux open flags",
    )
    def test_diagnostic_overflow_preserves_bounded_main_evidence(self) -> None:
        diagnostics = BoundedDiagnosticStore(max_bytes=256)
        diagnostics.append("first reader warning")
        diagnostics.append("x" * 512)

        lines = diagnostics.snapshot()

        self.assertEqual(lines[0], "first reader warning")
        self.assertIn("truncated", lines[-1])
        self.assertEqual(
            diagnostics.limit_reason(),
            "reader diagnostic byte limit reached (256)",
        )
        self.assertLessEqual(
            len(("\n".join(lines) + "\n").encode("utf-8")), 256
        )
        with tempfile.TemporaryDirectory() as directory:
            output_dir = Path(directory) / "capture"
            write_capture(
                output_dir,
                [event("reader", 0.0, event="started")],
                {"schema": 1, "required_streams_complete": False},
                lines,
            )

            self.assertIn(
                '"source":"reader"',
                (output_dir / "events.jsonl").read_text(encoding="utf-8"),
            )
            self.assertIn(
                "truncated",
                (output_dir / "reader_stderr.txt").read_text(encoding="utf-8"),
            )

    @unittest.skipUnless(
        hasattr(os, "O_DIRECTORY") and hasattr(os, "O_NOFOLLOW"),
        "directory-fd capture writes require Linux open flags",
    )
    def test_capture_files_are_published_atomically_without_overwrite(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output_dir = Path(directory) / "capture"
            output_dir.mkdir(mode=0o700)
            directory_fd = os.open(
                output_dir,
                os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW,
            )
            try:
                write_capture(
                    output_dir,
                    [event("reader", 0.0, value="original")],
                    {"schema": 1, "required_streams_complete": False},
                    [],
                    directory_fd=directory_fd,
                )
                original = (output_dir / "events.jsonl").read_bytes()
                with self.assertRaises(FileExistsError):
                    write_capture(
                        output_dir,
                        [event("reader", 0.0, value="replacement")],
                        {"schema": 1, "required_streams_complete": True},
                        [],
                        directory_fd=directory_fd,
                    )
            finally:
                os.close(directory_fd)

            self.assertEqual((output_dir / "events.jsonl").read_bytes(), original)
            self.assertFalse(
                any(path.name.startswith(".") for path in output_dir.iterdir())
            )

    def test_live_event_store_stops_at_configured_bound(self) -> None:
        store = EventStore(max_events=2, max_bytes=4096)
        store.append(event("sport", 0.0, vx=0.0, vy=0.0, yaw_rate=0.0))
        store.append(event("sport", 0.01, vx=0.0, vy=0.0, yaw_rate=0.0))
        store.append(event("sport", 0.02, vx=0.0, vy=0.0, yaw_rate=0.0))

        self.assertEqual(len(store.snapshot()), 2)
        self.assertEqual(store.limit_reason(), "event limit reached (2)")

    def test_live_event_store_enforces_the_24_mib_byte_limit(self) -> None:
        store = EventStore()

        store.append(event("reader", 0.0, payload="x" * MAX_CAPTURE_BYTES))

        self.assertEqual(store.snapshot(), [])
        self.assertEqual(
            store.limit_reason(),
            f"event byte limit reached ({24 * 1024 * 1024})",
        )

    def test_live_event_store_counts_the_jsonl_newline(self) -> None:
        sample = event("sport", 0.0, vx=0.0, vy=0.0, yaw_rate=0.0)
        json_bytes = len(
            json.dumps(sample, sort_keys=True, separators=(",", ":")).encode(
                "utf-8"
            )
        )
        without_newline = EventStore(max_events=1, max_bytes=json_bytes)
        with_newline = EventStore(max_events=1, max_bytes=json_bytes + 1)

        without_newline.append(sample)
        with_newline.append(sample)

        self.assertEqual(without_newline.snapshot(), [])
        self.assertEqual(
            without_newline.limit_reason(),
            f"event byte limit reached ({json_bytes})",
        )
        self.assertEqual(with_newline.snapshot(), [sample])
        self.assertIsNone(with_newline.limit_reason())

    def test_remote_discovery_reports_missing_lowstate_separately(self) -> None:
        store = EventStore()

        self.assertEqual(_remote_discovery_problem(store), "missing rt/lowstate")

    def test_remote_discovery_reports_invalid_wireless_packet_details(self) -> None:
        store = EventStore()
        store.append(
            event(
                "lowstate",
                1.0,
                lowstate_frames=1450,
                invalid_remote_frames=1450,
                packet_head=0,
                nonzero_bytes=0,
            )
        )

        problem = _remote_discovery_problem(store)

        self.assertIsNotNone(problem)
        assert problem is not None
        self.assertIn("rt/lowstate present", problem)
        self.assertIn("observed_lowstate_frames>=1450", problem)
        self.assertIn("invalid_remote_frames>=1450", problem)
        self.assertIn("latest_packet_head=0x0000", problem)
        self.assertIn("latest_nonzero_bytes=0", problem)
        self.assertIn("expected_packet_head=0x5551", problem)

    def test_remote_discovery_accepts_a_valid_remote_source(self) -> None:
        store = EventStore()
        store.append(event("remote", 1.0))

        self.assertIsNone(_remote_discovery_problem(store))

    def test_cli_does_not_abbreviate_reserved_live_options(self) -> None:
        with self.assertRaises(SystemExit):
            parse_args(
                [
                    "--reader",
                    "reader",
                    "--output-dir",
                    "inside",
                    "--output-d",
                    "outside",
                ]
            )

    def test_reader_is_reaped_when_ros_initialization_fails(self) -> None:
        child = mock.Mock()
        child.stdout = io.StringIO()
        child.stderr = io.StringIO()
        child.poll.return_value = None
        child.wait.return_value = 0

        rclpy = types.ModuleType("rclpy")
        rclpy.init = mock.Mock(side_effect=RuntimeError("ROS init failed"))
        rclpy.ok = mock.Mock(return_value=False)
        rclpy.shutdown = mock.Mock()
        rclpy_node = types.ModuleType("rclpy.node")
        rclpy_node.Node = object
        rclpy_qos = types.ModuleType("rclpy.qos")
        rclpy_qos.DurabilityPolicy = object
        rclpy_qos.QoSProfile = object
        rclpy_qos.ReliabilityPolicy = object
        geometry_msgs = types.ModuleType("geometry_msgs")
        geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")
        geometry_msgs_msg.TwistStamped = object
        nav_msgs = types.ModuleType("nav_msgs")
        nav_msgs_msg = types.ModuleType("nav_msgs.msg")
        nav_msgs_msg.Odometry = object
        modules = {
            "rclpy": rclpy,
            "rclpy.node": rclpy_node,
            "rclpy.qos": rclpy_qos,
            "geometry_msgs": geometry_msgs,
            "geometry_msgs.msg": geometry_msgs_msg,
            "nav_msgs": nav_msgs,
            "nav_msgs.msg": nav_msgs_msg,
        }

        with (
            mock.patch.dict(sys.modules, modules),
            mock.patch(
                "sdk2_motion_stall_probe.subprocess.Popen", return_value=child
            ),
            self.assertRaisesRegex(RuntimeError, "ROS init failed"),
        ):
            capture_live(Path(sys.executable), "enP8p1s0", 5.0, 1.0)

        child.send_signal.assert_called_once_with(signal.SIGINT)
        child.wait.assert_called_once_with(timeout=3.0)
        self.assertTrue(child.stdout.closed)
        self.assertTrue(child.stderr.closed)

    def test_runtime_sources_have_only_subscription_surfaces(self) -> None:
        runtime_sources = (READER_SOURCE, PROBE_SOURCE, RUNNER_SOURCE)
        forbidden = (
            "SportClient",
            "MotionSwitcherClient",
            "RobotStateClient",
            "ChannelPublisher",
            "create_publisher",
            "create_client",
            "create_service",
            "Move(",
            "StopMove",
            "SwitchJoystick",
            "BalanceStand",
            "HandStand",
            "FreeJump",
            "StaticWalk",
            "TrotRun",
            "EconomicGait",
            "SelectMode",
            "ReleaseMode",
            "ServiceSwitch",
        )
        for path in runtime_sources:
            source = path.read_text(encoding="utf-8")
            with self.subTest(path=path.name):
                for token in forbidden:
                    self.assertNotIn(token, source)

        reader = READER_SOURCE.read_text(encoding="utf-8")
        self.assertEqual(reader.count("ChannelSubscriber<"), 2)
        self.assertIn('"rt/sportmodestate"', reader)
        self.assertIn('"rt/lowstate"', reader)
        self.assertIn("message.wireless_remote()", reader)
        self.assertIn("lowstate_frames.fetch_add", reader)
        self.assertIn("invalid_remote_frames.fetch_add", reader)
        self.assertIn('"{\\"schema\\":1,\\"source\\":\\"lowstate\\""', reader)
        self.assertIn(
            "(static_cast<std::uint16_t>(bytes[0]) << 8U) |\n"
            "    static_cast<std::uint16_t>(bytes[1])",
            reader,
        )
        gate_index = reader.index("if (packet_head != 0x5551U)")
        gate_return_index = reader.index("return;", gate_index)
        sequence_index = reader.index(
            "const auto sequence = remote_frames.fetch_add", gate_index
        )
        decode_index = reader.index("std::memcpy", gate_index)
        emit_index = reader.index('"{\\"schema\\":1,\\"source\\":\\"remote\\""')
        self.assertLess(gate_index, gate_return_index)
        self.assertLess(gate_return_index, sequence_index)
        self.assertLess(gate_return_index, decode_index)
        self.assertLess(gate_return_index, emit_index)
        self.assertIn("std::chrono::steady_clock", reader)
        self.assertNotIn("unitree/robot/go2/", reader)

        probe = PROBE_SOURCE.read_text(encoding="utf-8")
        self.assertIn("time.monotonic_ns()", probe)
        syntax = ast.parse(probe)
        create_calls = [
            node
            for node in ast.walk(syntax)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr.startswith("create_")
        ]
        self.assertEqual(
            [node.func.attr for node in create_calls],
            ["create_subscription", "create_subscription"],
        )

    def test_replay_cli_is_deterministic_and_requires_no_ros_import(self) -> None:
        events = []
        for timestamp in regular_times(1.0, 20.0):
            events.append(
                event("command", timestamp, vx=0.2, vy=0.0, yaw_rate=0.0)
            )
        for timestamp in regular_times(1.0, 10.0):
            events.append(event("odom", timestamp, x=timestamp * 0.1, y=0.0, yaw=0.0))
        for timestamp in regular_times(1.0, 100.0):
            events.append(
                event("sport", timestamp, vx=0.1, vy=0.0, yaw_rate=0.0)
            )
        events.extend(complete_remote_stream(1.0))

        with tempfile.TemporaryDirectory() as directory:
            fixture = Path(directory) / "events.jsonl"
            with fixture.open("w", encoding="utf-8") as stream:
                for item in reversed(events):
                    stream.write(json.dumps(item) + "\n")
            completed = subprocess.run(
                [sys.executable, str(PROBE_SOURCE), "--replay", str(fixture)],
                check=False,
                capture_output=True,
                text=True,
            )

        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertIn("required_streams=complete", completed.stdout)
        self.assertIn("stall=not_observed", completed.stdout)


if __name__ == "__main__":
    unittest.main()
