#!/usr/bin/env python3
"""Capture bounded timing summaries without retaining ROS message payloads."""

from __future__ import annotations

import argparse
import csv
import io
import math
import os
from pathlib import Path
import stat
import time
from typing import Any, Iterable


TOPICS = (
    "/lio/odom",
    "/lio/body_odom",
)
TIMING_FIELDS = (
    "window_end",
    "window_duration_sec",
    "topic",
    "status",
    "message_count",
    "first_receive_ns",
    "last_receive_ns",
    "first_header_ns",
    "last_header_ns",
    "latest_header_age_ms",
    "header_span_ms",
    "first_local_sequence",
    "last_local_sequence",
    "unique_header_count",
    "duplicate_header_count",
    "nonmonotonic_header_count",
    "invalid_header_count",
    "min_header_gap_ms",
    "max_header_gap_ms",
    "max_receive_gap_ms",
)
RATE_FIELDS = ("timestamp", "topic", "average_hz", "status")


def _stamp_ns(message: object) -> int | None:
    try:
        stamp = getattr(getattr(message, "header"), "stamp")
        seconds = int(stamp.sec)
        nanoseconds = int(stamp.nanosec)
    except (AttributeError, TypeError, ValueError):
        return None
    if seconds < 0 or not 0 <= nanoseconds < 1_000_000_000:
        return None
    value = seconds * 1_000_000_000 + nanoseconds
    return value if value > 0 else None


class TopicWindow:
    def __init__(self) -> None:
        self.message_count = 0
        self.first_receive_ns: int | None = None
        self.last_receive_ns: int | None = None
        self.first_receive_monotonic_ns: int | None = None
        self.last_receive_monotonic_ns: int | None = None
        self.max_receive_gap_ns: int | None = None
        self.first_header_ns: int | None = None
        self.last_header_ns: int | None = None
        self.unique_headers: set[int] = set()
        self.nonmonotonic_header_count = 0
        self.invalid_header_count = 0
        self.min_header_gap_ns: int | None = None
        self.max_header_gap_ns: int | None = None

    def observe(
        self,
        message: object,
        receive_ns: int,
        receive_monotonic_ns: int,
    ) -> None:
        self.message_count += 1
        if self.first_receive_ns is None:
            self.first_receive_ns = receive_ns
            self.first_receive_monotonic_ns = receive_monotonic_ns
        self.last_receive_ns = receive_ns
        if self.last_receive_monotonic_ns is not None:
            gap = receive_monotonic_ns - self.last_receive_monotonic_ns
            if gap >= 0 and (
                self.max_receive_gap_ns is None or gap > self.max_receive_gap_ns
            ):
                self.max_receive_gap_ns = gap
        self.last_receive_monotonic_ns = receive_monotonic_ns

        header_ns = _stamp_ns(message)
        if header_ns is None:
            self.invalid_header_count += 1
        else:
            if self.first_header_ns is None:
                self.first_header_ns = header_ns
            if self.last_header_ns is not None:
                gap = header_ns - self.last_header_ns
                if gap < 0:
                    self.nonmonotonic_header_count += 1
                elif gap > 0:
                    if self.min_header_gap_ns is None or gap < self.min_header_gap_ns:
                        self.min_header_gap_ns = gap
                    if self.max_header_gap_ns is None or gap > self.max_header_gap_ns:
                        self.max_header_gap_ns = gap
            self.last_header_ns = header_ns
            self.unique_headers.add(header_ns)

    def timing_row(
        self,
        topic: str,
        window_end: str,
        duration_sec: float,
        report_ns: int,
    ) -> dict[str, Any]:
        valid_header_count = self.message_count - self.invalid_header_count
        if self.message_count == 0:
            status = "no_data"
        elif valid_header_count == 0:
            status = "invalid_header"
        else:
            status = "ok"
        duplicate_count = max(0, valid_header_count - len(self.unique_headers))
        return {
            "window_end": window_end,
            "window_duration_sec": f"{duration_sec:.6f}",
            "topic": topic,
            "status": status,
            "message_count": self.message_count,
            "first_receive_ns": self.first_receive_ns or "",
            "last_receive_ns": self.last_receive_ns or "",
            "first_header_ns": self.first_header_ns or "",
            "last_header_ns": self.last_header_ns or "",
            "latest_header_age_ms": _milliseconds(
                report_ns - self.last_header_ns
                if self.last_header_ns is not None
                else None
            ),
            "header_span_ms": _milliseconds(
                self.last_header_ns - self.first_header_ns
                if self.last_header_ns is not None and self.first_header_ns is not None
                else None
            ),
            "first_local_sequence": 1 if self.message_count else "",
            "last_local_sequence": self.message_count or "",
            "unique_header_count": len(self.unique_headers),
            "duplicate_header_count": duplicate_count,
            "nonmonotonic_header_count": self.nonmonotonic_header_count,
            "invalid_header_count": self.invalid_header_count,
            "min_header_gap_ms": _milliseconds(self.min_header_gap_ns),
            "max_header_gap_ms": _milliseconds(self.max_header_gap_ns),
            "max_receive_gap_ms": _milliseconds(self.max_receive_gap_ns),
        }

    def rate_row(self, topic: str, timestamp: str) -> dict[str, Any]:
        elapsed_ns = None
        if (
            self.first_receive_monotonic_ns is not None
            and self.last_receive_monotonic_ns is not None
        ):
            elapsed_ns = (
                self.last_receive_monotonic_ns - self.first_receive_monotonic_ns
            )
        if self.message_count >= 2 and elapsed_ns is not None and elapsed_ns > 0:
            rate = (self.message_count - 1) * 1_000_000_000.0 / elapsed_ns
            return {
                "timestamp": timestamp,
                "topic": topic,
                "average_hz": f"{rate:.6f}",
                "status": "ok",
            }
        return {
            "timestamp": timestamp,
            "topic": topic,
            "average_hz": "",
            "status": "no_data",
        }


def _milliseconds(value_ns: int | None) -> str:
    return "" if value_ns is None else f"{value_ns / 1_000_000.0:.6f}"


def append_bounded_csv(
    path: Path,
    fieldnames: Iterable[str],
    rows: Iterable[dict[str, Any]],
    max_bytes: int,
) -> bool:
    output = io.StringIO(newline="")
    writer = csv.DictWriter(output, fieldnames=list(fieldnames), lineterminator="\n")
    writer.writerows(rows)
    payload = output.getvalue().encode("utf-8")
    flags = os.O_WRONLY | os.O_APPEND
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    descriptor = os.open(path, flags)
    try:
        details = os.fstat(descriptor)
        if not stat.S_ISREG(details.st_mode):
            raise OSError(f"not a regular file: {path}")
        if details.st_size + len(payload) > max_bytes:
            return False
        written = os.write(descriptor, payload)
        if written != len(payload):
            raise OSError(f"short write to {path}")
        os.fsync(descriptor)
        return True
    finally:
        os.close(descriptor)


def capture(duration_sec: float) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    try:
        import rclpy
        from nav_msgs.msg import Odometry
        from rclpy.node import Node
        from rclpy.qos import (
            DurabilityPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )
    except ImportError as error:
        raise RuntimeError("ROS 2 Python odometry messages are required") from error

    rclpy.init(args=None)
    node = None
    windows = {topic: TopicWindow() for topic in TOPICS}
    try:
        node = Node(f"go2_log_topic_timing_{os.getpid()}")
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        def callback(topic: str):
            def observe(message: object) -> None:
                windows[topic].observe(message, time.time_ns(), time.monotonic_ns())

            return observe

        subscriptions = []
        for topic in TOPICS:
            subscriptions.append(
                node.create_subscription(Odometry, topic, callback(topic), qos)
            )
        del subscriptions
        started_monotonic_ns = time.monotonic_ns()
        deadline_ns = started_monotonic_ns + int(duration_sec * 1_000_000_000)
        while time.monotonic_ns() < deadline_ns:
            remaining = max(0.0, (deadline_ns - time.monotonic_ns()) / 1e9)
            rclpy.spin_once(node, timeout_sec=min(0.1, remaining))
        ended_monotonic_ns = time.monotonic_ns()
        report_ns = time.time_ns()
        timestamp = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
        actual_duration = (ended_monotonic_ns - started_monotonic_ns) / 1e9
        timing_rows = [
            windows[topic].timing_row(
                topic, timestamp, actual_duration, report_ns
            )
            for topic in TOPICS
        ]
        rate_rows = [windows[topic].rate_row(topic, timestamp) for topic in TOPICS]
        return timing_rows, rate_rows
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timing-csv", required=True, type=Path)
    parser.add_argument("--rates-csv", required=True, type=Path)
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--max-bytes", type=int, default=4 * 1024 * 1024)
    args = parser.parse_args()
    if not math.isfinite(args.duration) or not 0.1 <= args.duration <= 10.0:
        parser.error("--duration must be finite and in [0.1, 10.0] seconds")
    if not 64 * 1024 <= args.max_bytes <= 8 * 1024 * 1024:
        parser.error("--max-bytes must be in [64 KiB, 8 MiB]")
    return args


def main() -> int:
    args = parse_args()
    timing_rows, rate_rows = capture(args.duration)
    timing_written = append_bounded_csv(
        args.timing_csv, TIMING_FIELDS, timing_rows, args.max_bytes
    )
    rates_written = append_bounded_csv(
        args.rates_csv, RATE_FIELDS, rate_rows, 8 * 1024 * 1024
    )
    if not timing_written:
        print(f"topic timing file reached its {args.max_bytes}-byte ceiling", file=os.sys.stderr)
    if not rates_written:
        print("topic rate file reached its 8 MiB ceiling", file=os.sys.stderr)
    return 0 if timing_written and rates_written else 3


if __name__ == "__main__":
    raise SystemExit(main())
