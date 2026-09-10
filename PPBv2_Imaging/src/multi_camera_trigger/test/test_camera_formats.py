"""Per-serial format selection and mixed raw/RGB acquisition regression tests."""

import csv
import io
from unittest.mock import Mock

from PIL import Image
import pytest

from multi_camera_trigger.image_io import (
    parse_camera_image_formats,
    resolve_camera_image_formats,
)
from multi_camera_trigger.multi_camera_trigger_node import MultiCameraTriggerNode


def test_formats_follow_serials_not_enumeration_order():
    overrides = parse_camera_image_formats('24071775=JPEG, 24071782=pgm')
    assert resolve_camera_image_formats(
        ['24071782', '24071783', '24071775'], 'png', overrides
    ) == {'24071782': 'pgm', '24071783': 'png', '24071775': 'jpg'}


def test_empty_overrides_preserve_default():
    assert resolve_camera_image_formats(
        ['24071775', '24071782'], 'pgm', parse_camera_image_formats(' ')
    ) == {'24071775': 'pgm', '24071782': 'pgm'}


@pytest.mark.parametrize('value', [
    '24071775=jpg,24071775=pgm', '24071775=tiff', '24071775',
    '=jpg', '24071775=', '24071775=jpg,', '../camera=jpg',
])
def test_bad_mapping_rejected(value):
    with pytest.raises(ValueError):
        parse_camera_image_formats(value)


def test_missing_configured_camera_rejected():
    with pytest.raises(ValueError, match='undetected.*24071784'):
        resolve_camera_image_formats(
            ['24071775'], 'jpg', {'24071784': 'pgm'}
        )


def test_duplicate_detected_camera_rejected():
    with pytest.raises(ValueError, match='duplicate detected'):
        resolve_camera_image_formats(['24071775', '24071775'], 'pgm', {})


def test_mixed_capture_writes_correct_pixels_and_csv(tmp_path):
    node = MultiCameraTriggerNode.__new__(MultiCameraTriggerNode)
    node.image_format = 'png'  # Deliberately differs from both per-camera formats.
    node.camera_timeout_ms = 1000
    node.jpeg_quality = 100
    node.jpeg_subsampling = 0
    node.png_compress_level = 3
    node.get_clock = Mock()
    node.get_clock.return_value.now.return_value.nanoseconds = 1_000_000_000
    node.get_logger = Mock()
    node.status_log_every_n_frames = 20
    node.image_processor = Mock()
    rgb = bytes([255, 0, 0] * 4)
    node.image_processor.Convert.return_value.GetData.return_value = rgb
    raw = bytes([0, 64, 128, 255])

    for serial, fmt in [('24071775', 'jpg'), ('24071782', 'pgm')]:
        camera_dir = tmp_path / serial
        camera_dir.mkdir()
        image = Mock()
        image.IsIncomplete.return_value = False
        image.GetData.return_value = raw
        image.GetWidth.return_value = 2
        image.GetHeight.return_value = 2
        image.GetChunkData.return_value.GetTimestamp.return_value = 1_000_000_000
        image.GetChunkData.return_value.GetFrameID.return_value = 42
        csv_file = io.StringIO()
        entry = {
            'serial': serial, 'image_format': fmt, 'cam': Mock(),
            'chunk_tick_ns': 1.0, 'timestamp_offset_ns': 0,
            'timestamp_uncertainty_ns': 1000,
            'dir': str(camera_dir), 'counter': 1,
            'csv_file': csv_file, 'csv_writer': csv.writer(csv_file),
        }
        entry['cam'].GetNextImage.return_value = image
        frame = node._acquire_frame(entry)
        assert frame['data'] == (raw if fmt == 'pgm' else rgb)
        image.Release.assert_called_once()
        node._save_frame(frame, None, None, True, 0.0)
        output = camera_dir / f'000001.{fmt}'
        with Image.open(output) as saved:
            if fmt == 'pgm':
                assert saved.mode == 'L'
                assert list(saved.getdata()) == list(raw)
            else:
                assert saved.mode == 'RGB'
                r, g, b = saved.getpixel((0, 0))
                assert r > 240 and g < 15 and b < 15
        row = next(csv.reader(io.StringIO(csv_file.getvalue())))
        assert row[0:2] == ['1', f'000001.{fmt}']
        assert row[-2:] == ['1', '0.000000']
    node.image_processor.Convert.assert_called_once()
