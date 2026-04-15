import os
from concurrent.futures import ThreadPoolExecutor, as_completed
from PIL import Image
import numpy as np
import cv2
from tqdm import tqdm
import argparse

DEFAULT_WORKERS = 4

def process_single_file(input_dir, output_dir, filename):
    input_path = os.path.join(input_dir, filename)
    output_path = os.path.join(output_dir, os.path.splitext(filename)[0] + ".png")

    try:
        img_pil = Image.open(input_path)
        img_np = np.array(img_pil)

        if img_np is None:
            raise ValueError("Image loaded as None")

        rgb_image = cv2.cvtColor(img_np, cv2.COLOR_BAYER_RG2RGB)

        success = cv2.imwrite(output_path, rgb_image)
        if not success:
            raise IOError("cv2.imwrite failed")

        return filename, None
    except Exception as e:
        return filename, str(e)

def convert_pgm_to_png(input_dir, output_dir, workers=None):
    """
    Converts all .pgm files in the specified input directory to .png files,
    and saves them to the specified output directory.
    Skips problematic files and reports them at the end.
    """
    
    print(f"Converting images in folder: {input_dir}")
    
    # Ensure the output directory exists
    os.makedirs(output_dir, exist_ok=True)

    # List all .pgm files
    pgm_files = [
        entry.name for entry in os.scandir(input_dir)
        if entry.is_file() and entry.name.lower().endswith('.pgm') and not entry.name.startswith('.')
    ]

    failed_files = []

    if workers is None:
        workers = DEFAULT_WORKERS
    workers = max(1, workers)

    if workers == 1:
        for filename in tqdm(pgm_files, desc="Converting images"):
            failed_filename, error = process_single_file(input_dir, output_dir, filename)
            if error:
                failed_files.append(failed_filename)
                print(f"\n[WARNING] Failed to process: {failed_filename}")
                print(f"Reason: {error}")
    else:
        with ThreadPoolExecutor(max_workers=workers) as executor:
            futures = [
                executor.submit(process_single_file, input_dir, output_dir, filename)
                for filename in pgm_files
            ]

            for future in tqdm(as_completed(futures), total=len(futures), desc=f"Converting images ({workers} threads)"):
                failed_filename, error = future.result()
                if error:
                    failed_files.append(failed_filename)
                    print(f"\n[WARNING] Failed to process: {failed_filename}")
                    print(f"Reason: {error}")

    print(f"\nFinished converting images in folder: {input_dir}")
    print(f"Converted images are saved to {output_dir}")

    # Report failed files
    if failed_files:
        print("\n===== Failed Files =====")
        for f in failed_files:
            print(f)
        print(f"Total failed: {len(failed_files)}")
        print(f"\n===============================")
        print(f"\n")
        print(f"\n")
    else:
        print("\nAll files processed successfully!")
        print(f"\n===============================")
        print(f"\n")
        print(f"\n")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert PGM images to PNG format.")
    parser.add_argument("input_dir", help="Path to the input directory containing .pgm files.")
    parser.add_argument("output_dir", help="Path to the output directory to save .png files.")
    parser.add_argument(
        "--workers",
        type=int,
        default=None,
        help=f"Number of worker threads. Default: {DEFAULT_WORKERS}. Use 1 to disable parallelism.",
    )

    args = parser.parse_args()

    convert_pgm_to_png(args.input_dir, args.output_dir, workers=args.workers)
