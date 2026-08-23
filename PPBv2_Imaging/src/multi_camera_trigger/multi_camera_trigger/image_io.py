"""Image format validation and atomic frame writing."""

import os
import threading
from pathlib import Path

from PIL import Image


SUPPORTED_IMAGE_FORMATS = ('png', 'jpg', 'pgm')


def normalize_image_format(value: str) -> str:
    """Return the canonical image extension or raise a useful error."""
    normalized = value.strip().lower().lstrip('.')
    if normalized == 'jpeg':
        normalized = 'jpg'
    if normalized not in SUPPORTED_IMAGE_FORMATS:
        choices = ', '.join(SUPPORTED_IMAGE_FORMATS)
        raise ValueError(f'unsupported image format {value!r}; choose one of: {choices}')
    return normalized


def save_frame_atomic(
    output_path: str,
    data: bytes,
    width: int,
    height: int,
    image_format: str,
    jpeg_quality: int = 95,
    png_compress_level: int = 3,
) -> None:
    """Write one Bayer PGM or RGB PNG/JPEG frame using an atomic rename."""
    image_format = normalize_image_format(image_format)
    expected_size = width * height * (1 if image_format == 'pgm' else 3)
    if len(data) != expected_size:
        raise ValueError(
            f'frame has {len(data)} bytes; expected {expected_size} '
            f'for {width}x{height} {image_format}'
        )

    destination = Path(output_path)
    temporary = destination.with_name(
        f'.{destination.name}.{os.getpid()}.{threading.get_ident()}.tmp'
    )

    try:
        if image_format == 'pgm':
            with temporary.open('xb') as pgm_file:
                pgm_file.write(b'P5\n')
                pgm_file.write(f'{width} {height}\n255\n'.encode('ascii'))
                pgm_file.write(data)
        else:
            image = Image.frombytes('RGB', (width, height), data)
            if image_format == 'png':
                image.save(
                    temporary,
                    format='PNG',
                    compress_level=png_compress_level,
                )
            else:
                image.save(
                    temporary,
                    format='JPEG',
                    quality=jpeg_quality,
                    subsampling=0,
                )
        os.replace(temporary, destination)
    except Exception:
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass
        raise
