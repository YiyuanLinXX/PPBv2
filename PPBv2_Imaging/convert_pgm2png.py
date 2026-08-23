#!/usr/bin/env python3
"""Convert legacy BayerRG8 PGM frames to RGB PNG images."""

import argparse
import os
from concurrent.futures import ThreadPoolExecutor, as_completed

import cv2
import numpy as np
from PIL import Image
from tqdm import tqdm


DEFAULT_WORKERS = 4


def process_single_file(input_dir, output_dir, filename):
    """Convert one BayerRG8 PGM file and return its name and any error."""
    input_path = os.path.join(input_dir, filename)
    output_path = os.path.join(
        output_dir,
        os.path.splitext(filename)[0] + '.png',
    )

    try:
        with Image.open(input_path) as image:
            bayer = np.asarray(image)
        if bayer.ndim != 2:
            raise ValueError(f'expected one Bayer channel, got shape {bayer.shape}')

        # OpenCV writes BGR arrays. The explicit RGGB constant avoids the
        # confusing legacy COLOR_BAYER_RG2RGB alias while preserving behavior.
        bgr_image = cv2.cvtColor(bayer, cv2.COLOR_BayerRGGB2BGR)
        if not cv2.imwrite(output_path, bgr_image):
            raise OSError('cv2.imwrite failed')
        return filename, None
    except Exception as exc:
        return filename, str(exc)


def convert_pgm_to_png(input_dir, output_dir, workers=None):
    """Convert all visible PGM files in a directory and report failures."""
    if not os.path.isdir(input_dir):
        raise NotADirectoryError(f'input directory does not exist: {input_dir}')
    os.makedirs(output_dir, exist_ok=True)

    pgm_files = sorted(
        entry.name
        for entry in os.scandir(input_dir)
        if entry.is_file()
        and entry.name.lower().endswith('.pgm')
        and not entry.name.startswith('.')
    )
    worker_count = DEFAULT_WORKERS if workers is None else max(1, workers)
    failures = []

    print(f'Converting {len(pgm_files)} image(s) from {input_dir}')
    if worker_count == 1:
        results = (
            process_single_file(input_dir, output_dir, filename)
            for filename in tqdm(pgm_files, desc='Converting images')
        )
        for filename, error in results:
            if error:
                failures.append((filename, error))
    else:
        with ThreadPoolExecutor(max_workers=worker_count) as executor:
            futures = [
                executor.submit(
                    process_single_file,
                    input_dir,
                    output_dir,
                    filename,
                )
                for filename in pgm_files
            ]
            for future in tqdm(
                as_completed(futures),
                total=len(futures),
                desc=f'Converting images ({worker_count} threads)',
            ):
                filename, error = future.result()
                if error:
                    failures.append((filename, error))

    if failures:
        print('Failed files:')
        for filename, error in sorted(failures):
            print(f'  {filename}: {error}')
        raise RuntimeError(f'{len(failures)} image(s) failed to convert')

    print(f'Converted images were saved to {output_dir}')


def main():
    """Run the command-line converter."""
    parser = argparse.ArgumentParser(
        description='Convert BayerRG8 PGM images to RGB PNG files.',
    )
    parser.add_argument('input_dir', help='Directory containing PGM files')
    parser.add_argument('output_dir', help='Directory for converted PNG files')
    parser.add_argument(
        '--workers',
        type=int,
        default=DEFAULT_WORKERS,
        help=f'Worker threads (default: {DEFAULT_WORKERS}; use 1 for serial)',
    )
    args = parser.parse_args()
    convert_pgm_to_png(args.input_dir, args.output_dir, args.workers)


if __name__ == '__main__':
    main()
