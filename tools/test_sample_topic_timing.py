#!/usr/bin/env python3

from __future__ import annotations

import csv
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

from sample_topic_timing import (
    RATE_FIELDS,
    TIMING_FIELDS,
    TopicWindow,
    _stamp_ns,
    append_bounded_csv,
)


def stamped_message(stamp_ns: int) -> SimpleNamespace:
    seconds, nanoseconds = divmod(stamp_ns, 1_000_000_000)
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=seconds, nanosec=nanoseconds)
        )
    )


def raw_stamp(seconds: object, nanoseconds: object) -> SimpleNamespace:
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=seconds, nanosec=nanoseconds)
        )
    )


class TopicWindowTests(unittest.TestCase):
    def test_duplicate_header_stamps_are_counted_separately_from_unique_stamps(
        self,
    ) -> None:
        window = TopicWindow()
        for sequence, stamp_ns in enumerate(
            (1_000_000_000, 1_000_000_000, 2_000_000_000),
            start=1,
        ):
            window.observe(
                stamped_message(stamp_ns),
                receive_ns=10_000_000_000 + sequence,
                receive_monotonic_ns=sequence * 100_000_000,
            )

        row = window.timing_row(
            "/lio/odom",
            "2026-08-07T00:00:00Z",
            duration_sec=0.2,
            report_ns=2_500_000_000,
        )

        self.assertEqual(row["message_count"], 3)
        self.assertEqual(row["unique_header_count"], 2)
        self.assertEqual(row["duplicate_header_count"], 1)
        self.assertEqual(row["nonmonotonic_header_count"], 0)
        self.assertEqual(row["min_header_gap_ms"], "1000.000000")
        self.assertEqual(row["max_header_gap_ms"], "1000.000000")

    def test_reverse_header_stamp_is_nonmonotonic_but_not_a_duplicate(self) -> None:
        window = TopicWindow()
        window.observe(stamped_message(2_000_000_000), 10, 100)
        window.observe(stamped_message(1_500_000_000), 20, 200)

        row = window.timing_row(
            "/lio/body_odom",
            "2026-08-07T00:00:00Z",
            duration_sec=0.1,
            report_ns=3_000_000_000,
        )

        self.assertEqual(row["unique_header_count"], 2)
        self.assertEqual(row["duplicate_header_count"], 0)
        self.assertEqual(row["nonmonotonic_header_count"], 1)
        self.assertEqual(row["min_header_gap_ms"], "")
        self.assertEqual(row["max_header_gap_ms"], "")

    def test_invalid_header_stamps_are_rejected_without_hiding_valid_data(
        self,
    ) -> None:
        messages = (
            SimpleNamespace(),
            raw_stamp(0, 0),
            raw_stamp(-1, 0),
            raw_stamp(1, 1_000_000_000),
            raw_stamp("bad", 0),
            stamped_message(4_000_000_000),
        )
        window = TopicWindow()
        for sequence, message in enumerate(messages, start=1):
            window.observe(message, sequence, sequence)

        row = window.timing_row(
            "/lio/odom",
            "2026-08-07T00:00:00Z",
            duration_sec=1.0,
            report_ns=4_250_000_000,
        )

        self.assertIsNone(_stamp_ns(raw_stamp(0, 0)))
        self.assertEqual(row["status"], "ok")
        self.assertEqual(row["message_count"], 6)
        self.assertEqual(row["invalid_header_count"], 5)
        self.assertEqual(row["unique_header_count"], 1)
        self.assertEqual(row["latest_header_age_ms"], "250.000000")

    def test_all_invalid_headers_report_invalid_header_status(self) -> None:
        window = TopicWindow()
        window.observe(SimpleNamespace(), 10, 10)
        window.observe(raw_stamp(0, 0), 20, 20)

        row = window.timing_row(
            "/lio/odom",
            "2026-08-07T00:00:00Z",
            duration_sec=1.0,
            report_ns=1_000_000_000,
        )

        self.assertEqual(row["status"], "invalid_header")
        self.assertEqual(row["invalid_header_count"], 2)
        self.assertEqual(row["latest_header_age_ms"], "")

    def test_receive_rate_header_timing_and_max_receive_gap_use_fixed_clocks(
        self,
    ) -> None:
        window = TopicWindow()
        observations = (
            (10_000_000_000, 50_000_000_000, 1_000_000_000),
            (10_100_000_000, 50_100_000_000, 1_200_000_000),
            (10_300_000_000, 50_300_000_000, 1_500_000_000),
            (10_600_000_000, 50_600_000_000, 2_000_000_000),
        )
        for header_ns, receive_ns, monotonic_ns in observations:
            window.observe(
                stamped_message(header_ns),
                receive_ns=receive_ns,
                receive_monotonic_ns=monotonic_ns,
            )

        timing = window.timing_row(
            "/lio/odom",
            "2026-08-07T00:00:00Z",
            duration_sec=1.0,
            report_ns=10_850_000_000,
        )
        rate = window.rate_row("/lio/odom", "2026-08-07T00:00:00Z")

        self.assertEqual(timing["first_receive_ns"], 50_000_000_000)
        self.assertEqual(timing["last_receive_ns"], 50_600_000_000)
        self.assertEqual(timing["header_span_ms"], "600.000000")
        self.assertEqual(timing["min_header_gap_ms"], "100.000000")
        self.assertEqual(timing["max_header_gap_ms"], "300.000000")
        self.assertEqual(timing["max_receive_gap_ms"], "500.000000")
        self.assertEqual(timing["latest_header_age_ms"], "250.000000")
        self.assertEqual(rate["status"], "ok")
        self.assertEqual(rate["average_hz"], "3.000000")

    def test_rate_requires_two_receives_with_positive_elapsed_time(self) -> None:
        empty = TopicWindow()
        one_message = TopicWindow()
        one_message.observe(stamped_message(1_000_000_000), 10, 100)
        zero_elapsed = TopicWindow()
        zero_elapsed.observe(stamped_message(1_000_000_000), 10, 100)
        zero_elapsed.observe(stamped_message(2_000_000_000), 20, 100)

        for window in (empty, one_message, zero_elapsed):
            with self.subTest(message_count=window.message_count):
                self.assertEqual(
                    window.rate_row("/lio/odom", "timestamp"),
                    {
                        "timestamp": "timestamp",
                        "topic": "/lio/odom",
                        "average_hz": "",
                        "status": "no_data",
                    },
                )


class BoundedCsvTests(unittest.TestCase):
    def test_csv_schema_field_order_matches_collector_headers(self) -> None:
        self.assertEqual(
            TIMING_FIELDS,
            (
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
            ),
        )
        self.assertEqual(
            RATE_FIELDS,
            ("timestamp", "topic", "average_hz", "status"),
        )

    def test_append_preserves_single_header_and_rejects_payload_over_ceiling(
        self,
    ) -> None:
        row = {
            "window_end": "2026-08-07T00:00:00Z",
            "window_duration_sec": "1.000000",
            "topic": "/lio/odom",
            "status": "ok",
            "message_count": 2,
            "first_receive_ns": 10,
            "last_receive_ns": 20,
            "first_header_ns": 1_000_000_000,
            "last_header_ns": 2_000_000_000,
            "latest_header_age_ms": "100.000000",
            "header_span_ms": "1000.000000",
            "first_local_sequence": 1,
            "last_local_sequence": 2,
            "unique_header_count": 2,
            "duplicate_header_count": 0,
            "nonmonotonic_header_count": 0,
            "invalid_header_count": 0,
            "min_header_gap_ms": "1000.000000",
            "max_header_gap_ms": "1000.000000",
            "max_receive_gap_ms": "10.000000",
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "topic_timing.csv"
            path.write_text(",".join(TIMING_FIELDS) + "\n", encoding="utf-8")
            header_size = path.stat().st_size

            self.assertTrue(
                append_bounded_csv(
                    path,
                    TIMING_FIELDS,
                    [row],
                    max_bytes=header_size + 4096,
                )
            )
            accepted = path.read_bytes()
            self.assertFalse(
                append_bounded_csv(
                    path,
                    TIMING_FIELDS,
                    [row],
                    max_bytes=len(accepted),
                )
            )
            self.assertEqual(path.read_bytes(), accepted)
            with path.open(encoding="utf-8", newline="") as stream:
                parsed = list(csv.DictReader(stream))

        self.assertEqual(parsed, [{key: str(value) for key, value in row.items()}])
        self.assertEqual(accepted.count(b"window_end,"), 1)


if __name__ == "__main__":
    unittest.main()
