#!/usr/bin/env python3

import hashlib
import json
import math
import os
from pathlib import Path
import socket
import struct
from typing import Any, Dict, Iterable, List, Tuple

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField


DEFAULT_TOPIC = "/flat_obstacle_filtered_map_3d"


def recorder_qos_profile() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=8,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def classify_source_stamp(stamp_ns: int, last_seen_stamp_ns: int) -> str:
    if stamp_ns <= 0:
        return "invalid"
    if stamp_ns == last_seen_stamp_ns:
        return "duplicate"
    if last_seen_stamp_ns >= 0 and stamp_ns < last_seen_stamp_ns:
        return "new_epoch"
    return "next"


def require_world_frame(frame_id: str) -> None:
    if frame_id != "world":
        raise ValueError(
            f"filtered obstacle map frame must be 'world', received {frame_id!r}"
        )


def validate_output_root(path: Path) -> Path:
    resolved = path.expanduser().resolve()
    if any(part.lower() == "g02_log" for part in resolved.parts):
        raise ValueError("filtered obstacle map output must not be inside G02_log")
    for candidate in (resolved, *resolved.parents):
        if (candidate / ".git").exists():
            raise ValueError(
                "filtered obstacle map output must be outside every Git workspace: "
                f"{resolved}"
            )
    return resolved


def extract_xyz(message: PointCloud2) -> List[Tuple[float, float, float]]:
    fields = {field.name: field for field in message.fields}
    required = ("x", "y", "z")
    if any(name not in fields for name in required):
        raise ValueError("PointCloud2 must contain x, y, and z fields")
    if any(
        fields[name].datatype != PointField.FLOAT32 or fields[name].count != 1
        for name in required
    ):
        raise ValueError("x, y, and z fields must be scalar float32 values")
    expected_size = message.row_step * message.height
    if message.point_step <= 0 or message.row_step < message.point_step * message.width:
        raise ValueError("PointCloud2 point_step or row_step is invalid")
    if len(message.data) < expected_size:
        raise ValueError("PointCloud2 data is shorter than its declared dimensions")

    byte_order = ">" if message.is_bigendian else "<"
    unpack_float = struct.Struct(byte_order + "f").unpack_from
    result: List[Tuple[float, float, float]] = []
    result_append = result.append
    for row in range(message.height):
        row_offset = row * message.row_step
        for column in range(message.width):
            point_offset = row_offset + column * message.point_step
            point = tuple(
                unpack_float(message.data, point_offset + fields[name].offset)[0]
                for name in required
            )
            if all(math.isfinite(value) for value in point):
                result_append(point)
    return result


def binary_pcd_bytes(points: Iterable[Tuple[float, float, float]]) -> bytes:
    finite_points = [
        point for point in points
        if len(point) == 3 and all(math.isfinite(value) for value in point)
    ]
    count = len(finite_points)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        f"WIDTH {count}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {count}\n"
        "DATA binary\n"
    ).encode("ascii")
    payload = bytearray(count * 12)
    pack_point = struct.Struct("<fff").pack_into
    for index, point in enumerate(finite_points):
        pack_point(payload, index * 12, *point)
    return header + payload


def write_binary_pcd(path: Path, points: Iterable[Tuple[float, float, float]]) -> int:
    content = binary_pcd_bytes(points)
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("wb") as stream:
        stream.write(content)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)
    return len(content)


def write_json_atomic(path: Path, payload: Dict[str, Any]) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8", newline="\n") as stream:
        json.dump(payload, stream, indent=2, sort_keys=True)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


def snapshot_navigation_config(
    source_path: str, session_directory: Path
) -> Dict[str, Any]:
    if not source_path or source_path == "unknown":
        return {"status": "unavailable", "source_path": source_path or "unknown"}
    source = Path(os.path.expanduser(source_path)).resolve(strict=True)
    content = source.read_bytes()
    destination = session_directory / "navigation_config.yaml"
    temporary = destination.with_suffix(destination.suffix + ".tmp")
    with temporary.open("wb") as stream:
        stream.write(content)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, destination)
    return {
        "status": "captured",
        "source_path": str(source),
        "file": destination.name,
        "sha256": hashlib.sha256(content).hexdigest(),
        "bytes": len(content),
    }


class FlatObstacleMapRecorder(Node):
    def __init__(self) -> None:
        super().__init__("flat_obstacle_map_recorder")
        topic = self.declare_parameter(
            "topic", DEFAULT_TOPIC
        ).get_parameter_value().string_value
        output_root = validate_output_root(Path(self.declare_parameter(
            "output_directory", "~/go2_map_exports"
        ).get_parameter_value().string_value))
        self.max_snapshots = self.declare_parameter(
            "max_snapshots", 120
        ).get_parameter_value().integer_value
        max_megabytes = self.declare_parameter(
            "max_total_megabytes", 100
        ).get_parameter_value().integer_value
        self.min_interval = self.declare_parameter(
            "min_interval", 0.0
        ).get_parameter_value().double_value
        source_git_sha = self.declare_parameter(
            "source_git_sha", "unknown"
        ).get_parameter_value().string_value
        navigation_config = self.declare_parameter(
            "navigation_config", "unknown"
        ).get_parameter_value().string_value
        planning_mode = self.declare_parameter(
            "planning_mode", "flat_obstacle"
        ).get_parameter_value().string_value
        flat_ground_confirmed = self.declare_parameter(
            "flat_ground_confirmed", False
        ).get_parameter_value().bool_value
        body_yaw_offset = self.declare_parameter(
            "body_yaw_offset", -1.5707963267948966
        ).get_parameter_value().double_value
        lidar_offset = [
            self.declare_parameter(
                "lidar_offset_x", 0.171
            ).get_parameter_value().double_value,
            self.declare_parameter(
                "lidar_offset_y", 0.0
            ).get_parameter_value().double_value,
            self.declare_parameter(
                "lidar_offset_z", 0.0908
            ).get_parameter_value().double_value,
        ]
        if not topic or self.max_snapshots < 1 or max_megabytes < 1:
            raise ValueError("recorder topic and limits must be valid")
        if not math.isfinite(self.min_interval) or self.min_interval < 0.0:
            raise ValueError("min_interval must be finite and non-negative")
        if planning_mode != "flat_obstacle" or not flat_ground_confirmed:
            raise ValueError(
                "filtered obstacle map capture requires flat_obstacle mode and "
                "explicit ground confirmation"
            )
        if not math.isfinite(body_yaw_offset) or not all(
            math.isfinite(value) for value in lidar_offset
        ):
            raise ValueError("capture pose and lidar offsets must be finite")
        self.max_total_bytes = max_megabytes * 1024 * 1024

        session_stamp = self.get_clock().now().to_msg()
        session_name = (
            f"{session_stamp.sec}_{session_stamp.nanosec:09d}-"
            f"{socket.gethostname()}-flat-obstacle-map"
        )
        self.session_directory = output_root / session_name
        self.session_directory.mkdir(parents=True, exist_ok=False)
        try:
            navigation_config_snapshot = snapshot_navigation_config(
                navigation_config, self.session_directory
            )
        except OSError as error:
            raise ValueError(
                f"cannot snapshot navigation configuration {navigation_config!r}: {error}"
            ) from error
        metadata = {
            "format": "pcd_binary_xyz_float32",
            "frame": "world",
            "topic": topic,
            "max_snapshots": self.max_snapshots,
            "max_total_megabytes": max_megabytes,
            "min_interval_seconds": self.min_interval,
            "navigation_config": navigation_config_snapshot,
            "launch_overrides": {
                "planning_mode": planning_mode,
                "flat_ground_confirmed": flat_ground_confirmed,
                "body_yaw_offset": body_yaw_offset,
                "lidar_offset_xyz": lidar_offset,
            },
            "qos": "reliable_volatile_keep_last_8",
            "source_git_sha": source_git_sha,
        }
        write_json_atomic(self.session_directory / "session.json", metadata)
        self.manifest = (self.session_directory / "manifest.jsonl").open(
            "a", encoding="utf-8", buffering=1, newline="\n"
        )
        self.saved_snapshots = 0
        self.total_bytes = 0
        self.last_seen_stamp_ns = -1
        self.last_saved_stamp_ns = -1
        self.source_epoch = 0
        self.status = "recording"
        self.stop_reason = ""
        self.exit_code = 0
        self.subscription = self.create_subscription(
            PointCloud2, topic, self._cloud_callback, recorder_qos_profile()
        )
        self.get_logger().info(
            f"Recording filtered 3D obstacle maps to {self.session_directory}"
        )

    def _stop(self, reason: str, failed: bool = False) -> None:
        if self.status != "recording":
            return
        if not failed and self.saved_snapshots == 0:
            failed = True
            reason = f"{reason} before any valid planning map was saved"
        self.status = "failed" if failed else "limit_reached"
        self.stop_reason = reason
        self.exit_code = 1 if failed else 0
        log = self.get_logger().error if failed else self.get_logger().info
        log(
            f"Filtered obstacle map recording {self.status}: {reason}; "
            f"snapshots={self.saved_snapshots}, bytes={self.total_bytes}, "
            f"directory={self.session_directory}"
        )
        if rclpy.ok():
            rclpy.shutdown()

    def _cloud_callback(self, message: PointCloud2) -> None:
        stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        stamp_class = classify_source_stamp(stamp_ns, self.last_seen_stamp_ns)
        if stamp_class in ("invalid", "duplicate"):
            return
        try:
            require_world_frame(message.header.frame_id)
        except ValueError as error:
            self._stop(str(error), failed=True)
            return
        if stamp_class == "new_epoch":
            self.source_epoch += 1
            self.last_saved_stamp_ns = -1
            self.get_logger().warning(
                "Planning map source clock moved backward; "
                f"recording source epoch {self.source_epoch}"
            )
        self.last_seen_stamp_ns = stamp_ns
        if self.last_saved_stamp_ns >= 0 and (
            stamp_ns - self.last_saved_stamp_ns
        ) * 1.0e-9 < self.min_interval:
            return
        try:
            points = extract_xyz(message)
            file_name = (
                f"epoch_{self.source_epoch:03d}_map_{message.header.stamp.sec}_"
                f"{message.header.stamp.nanosec:09d}.pcd"
            )
            estimated_bytes = 256 + len(points) * 12
            if self.total_bytes + estimated_bytes > self.max_total_bytes:
                self._stop("maximum byte limit reached")
                return
            file_path = self.session_directory / file_name
            written_bytes = write_binary_pcd(file_path, points)
            record = {
                "file": file_name,
                "frame_id": message.header.frame_id,
                "point_count": len(points),
                "source_epoch": self.source_epoch,
                "stamp_nanoseconds": stamp_ns,
                "written_bytes": written_bytes,
            }
            self.manifest.write(json.dumps(record, sort_keys=True) + "\n")
            self.manifest.flush()
            os.fsync(self.manifest.fileno())
            self.saved_snapshots += 1
            self.total_bytes += written_bytes
            self.last_saved_stamp_ns = stamp_ns
            if self.saved_snapshots == 1 or self.saved_snapshots % 10 == 0:
                self.get_logger().info(
                    f"Saved filtered 3D obstacle map {self.saved_snapshots}/"
                    f"{self.max_snapshots}: {file_name}, points={len(points)}"
                )
            if self.saved_snapshots >= self.max_snapshots:
                self._stop("maximum snapshot count reached")
        except (OSError, ValueError, struct.error) as error:
            self.get_logger().error(f"Filtered obstacle map capture failed: {error}")
            self._stop(str(error), failed=True)

    def close(self) -> None:
        if self.status == "recording":
            if self.saved_snapshots == 0:
                self.status = "failed"
                self.stop_reason = "operator or ROS shutdown before any valid map was saved"
                self.exit_code = 1
            else:
                self.status = "stopped"
                self.stop_reason = "operator or ROS shutdown"
        try:
            if not self.manifest.closed:
                self.manifest.flush()
                os.fsync(self.manifest.fileno())
                self.manifest.close()
            write_json_atomic(
                self.session_directory / "result.json",
                {
                    "status": self.status,
                    "reason": self.stop_reason,
                    "exit_code": self.exit_code,
                    "saved_snapshots": self.saved_snapshots,
                    "pcd_bytes": self.total_bytes,
                    "source_epoch_resets": self.source_epoch,
                },
            )
        except OSError as error:
            self.exit_code = 1
            self.status = "failed"
            self.get_logger().error(
                f"Failed to finalize filtered obstacle map session: {error}"
            )


def main(args=None) -> int:
    rclpy.init(args=args)
    node = FlatObstacleMapRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return node.exit_code


if __name__ == "__main__":
    raise SystemExit(main())
