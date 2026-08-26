"""Unit tests for RGB/PGM image writing."""

from PIL import Image
import pytest

from multi_camera_trigger.image_io import (
    normalize_image_format,
    save_frame_atomic,
)


@pytest.mark.parametrize(
    'value, expected',
    [('PNG', 'png'), ('.jpg', 'jpg'), ('jpeg', 'jpg'), ('pgm', 'pgm')],
)
def test_normalize_image_format(value, expected):
    assert normalize_image_format(value) == expected


def test_normalize_image_format_rejects_unknown_value():
    with pytest.raises(ValueError, match='unsupported image format'):
        normalize_image_format('tiff')


@pytest.mark.parametrize('image_format', ['png', 'jpg'])
def test_save_rgb_image(image_format, tmp_path):
    output = tmp_path / f'frame.{image_format}'
    red_pixels = bytes([255, 0, 0] * 4)

    save_frame_atomic(
        str(output),
        red_pixels,
        width=2,
        height=2,
        image_format=image_format,
        jpeg_quality=100,
        jpeg_subsampling=2,
    )

    with Image.open(output) as image:
        assert image.mode == 'RGB'
        assert image.size == (2, 2)
        red, green, blue = image.getpixel((0, 0))
        assert red > 240
        assert green < 15
        assert blue < 15


def test_save_raw_pgm(tmp_path):
    output = tmp_path / 'frame.pgm'
    save_frame_atomic(
        str(output),
        bytes([0, 64, 128, 255]),
        width=2,
        height=2,
        image_format='pgm',
    )

    with Image.open(output) as image:
        assert image.mode == 'L'
        assert list(image.getdata()) == [0, 64, 128, 255]


def test_save_rejects_wrong_buffer_size(tmp_path):
    with pytest.raises(ValueError, match='expected'):
        save_frame_atomic(
            str(tmp_path / 'bad.png'),
            b'not-rgb',
            width=2,
            height=2,
            image_format='png',
        )
