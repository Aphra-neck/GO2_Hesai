import hashlib
import importlib.util
from pathlib import Path
import struct

from sensor_msgs.msg import PointCloud2, PointField


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "flat_obstacle_map_recorder.py"
)
SPEC = importlib.util.spec_from_file_location("flat_obstacle_map_recorder", SCRIPT_PATH)
RECORDER = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(RECORDER)


def make_cloud(points):
    message = PointCloud2()
    message.height = 1
    message.width = len(points)
    message.is_bigendian = False
    message.is_dense = True
    message.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    message.point_step = 16
    message.row_step = message.point_step * message.width
    payload = bytearray(message.row_step)
    for index, point in enumerate(points):
        struct.pack_into("<fff", payload, index * message.point_step, *point)
    message.data = bytes(payload)
    return message


def test_extract_xyz_respects_point_step_and_skips_nonfinite():
    message = make_cloud([(1.0, 2.0, 3.0), (float("nan"), 4.0, 5.0)])

    assert RECORDER.extract_xyz(message) == [(1.0, 2.0, 3.0)]


def test_recorder_uses_reliable_filtered_obstacle_map_topic():
    assert RECORDER.DEFAULT_TOPIC == "/flat_obstacle_filtered_map_3d"
    qos = RECORDER.recorder_qos_profile()
    assert qos.depth == 8
    assert qos.history == RECORDER.HistoryPolicy.KEEP_LAST
    assert qos.reliability == RECORDER.ReliabilityPolicy.RELIABLE
    assert qos.durability == RECORDER.DurabilityPolicy.VOLATILE


def test_write_binary_pcd_is_cloudcompare_compatible(tmp_path):
    path = tmp_path / "map.pcd"

    written = RECORDER.write_binary_pcd(path, [(1.0, 2.0, 3.0), (-1.0, 0.5, 0.25)])
    content = path.read_bytes()
    header, payload = content.split(b"DATA binary\n", 1)

    assert b"FIELDS x y z\n" in header
    assert b"WIDTH 2\n" in header
    assert b"POINTS 2\n" in header
    assert struct.unpack("<ffffff", payload) == (1.0, 2.0, 3.0, -1.0, 0.5, 0.25)
    assert written == len(content)


def test_write_binary_pcd_preserves_an_empty_filtered_obstacle_map(tmp_path):
    path = tmp_path / "empty.pcd"

    RECORDER.write_binary_pcd(path, [])
    content = path.read_bytes()

    assert b"WIDTH 0\n" in content
    assert b"POINTS 0\n" in content
    assert content.endswith(b"DATA binary\n")


def test_navigation_config_is_copied_and_hashed(tmp_path):
    source = tmp_path / "active.yaml"
    source.write_text("terrain_mapper:\n  ros__parameters: {}\n", encoding="utf-8")
    session = tmp_path / "session"
    session.mkdir()

    metadata = RECORDER.snapshot_navigation_config(str(source), session)

    content = source.read_bytes()
    assert (session / "navigation_config.yaml").read_bytes() == content
    assert metadata["status"] == "captured"
    assert metadata["sha256"] == hashlib.sha256(content).hexdigest()
    assert metadata["bytes"] == len(content)


def test_source_stamp_classification_keeps_clock_reset_as_a_new_epoch():
    assert RECORDER.classify_source_stamp(2_000_000_000, -1) == "next"
    assert RECORDER.classify_source_stamp(2_000_000_000, 2_000_000_000) == "duplicate"
    assert RECORDER.classify_source_stamp(1_000_000_000, 2_000_000_000) == "new_epoch"
    assert RECORDER.classify_source_stamp(0, 2_000_000_000) == "invalid"


def test_recorder_rejects_non_world_map_frame():
    RECORDER.require_world_frame("world")

    try:
        RECORDER.require_world_frame("base_link")
    except ValueError as error:
        assert "must be 'world'" in str(error)
    else:
        raise AssertionError("non-world planning map frame was accepted")


def test_capture_output_rejects_git_and_diagnostics_repositories(tmp_path):
    git_root = tmp_path / "mapping_repo"
    (git_root / ".git").mkdir(parents=True)

    try:
        RECORDER.validate_output_root(git_root / "captures")
    except ValueError as error:
        assert "Git workspace" in str(error)
    else:
        raise AssertionError("capture output inside a Git workspace was accepted")

    try:
        RECORDER.validate_output_root(tmp_path / "G02_log" / "maps")
    except ValueError as error:
        assert "G02_log" in str(error)
    else:
        raise AssertionError("capture output inside G02_log was accepted")

    assert RECORDER.validate_output_root(tmp_path / "exports") == (
        tmp_path / "exports"
    ).resolve()
