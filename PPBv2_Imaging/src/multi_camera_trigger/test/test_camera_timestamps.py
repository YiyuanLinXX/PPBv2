"""Tests for camera timestamp formatting helpers."""

from multi_camera_trigger.multi_camera_trigger_node import MultiCameraTriggerNode


def test_format_nanoseconds_preserves_nine_digits():
    assert MultiCameraTriggerNode._format_nanoseconds(1_234_000_005) == (
        '1.234000005'
    )


def test_format_nanoseconds_handles_large_camera_time():
    assert MultiCameraTriggerNode._format_nanoseconds(5_085_493_606_136) == (
        '5085.493606136'
    )


def test_sync_uses_exposure_time_not_absolute_camera_frame_id():
    node = MultiCameraTriggerNode.__new__(MultiCameraTriggerNode)
    node.cross_camera_sync_tolerance_ms = 20.0
    frames = [
        {'chunk_frame_id': 1457, 'exposure_ros_time_float': 100.000},
        {'chunk_frame_id': 375, 'exposure_ros_time_float': 100.001},
    ]

    assert node._frames_time_aligned(frames)


def test_sync_rejects_frames_from_different_trigger_events():
    node = MultiCameraTriggerNode.__new__(MultiCameraTriggerNode)
    node.cross_camera_sync_tolerance_ms = 20.0
    frames = [
        {'chunk_frame_id': 1457, 'exposure_ros_time_float': 100.000},
        {'chunk_frame_id': 374, 'exposure_ros_time_float': 99.500},
    ]

    assert not node._frames_time_aligned(frames)
