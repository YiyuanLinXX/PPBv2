import os
from PIL import Image
import numpy as np
import cv2
from tqdm import tqdm
import argparse

def convert_pgm_to_png(input_dir, output_dir):
    """
    Converts all .pgm files in the specified input directory to .png files,
    and saves them to the specified output directory, in sorted order.
    """

    print(f"Converting images in folder: {input_dir}")
    
    # Ensure the output directory exists
    os.makedirs(output_dir, exist_ok=True)

    # List all .pgm files and sort them by filename
    pgm_files = sorted([f for f in os.listdir(input_dir) if f.lower().endswith('.pgm')])

    if not pgm_files:
        print("No .pgm files found in the input directory.")
        return

    for filename in tqdm(pgm_files, desc="Converting images"):
        input_path = os.path.join(input_dir, filename)
        output_path = os.path.join(output_dir, filename.replace('.pgm', '.png'))

        try:
            # Load the image using Pillow
            img_pil = Image.open(input_path)
            img_np = np.array(img_pil)

            # Check if image is valid
            if img_np is None or img_np.size == 0:
                print(f"Warning: Skipped invalid image {filename}")
                continue

            # Convert to RGB from Bayer RG8 (assuming BayerRG)
            rgb_image = cv2.cvtColor(img_np, cv2.COLOR_BAYER_RG2RGB)

            # Save as PNG
            cv2.imwrite(output_path, rgb_image)

        except Exception as e:
            print(f"Error processing {filename}: {e}")

    print(f"Finished converting {len(pgm_files)} image(s).")
    print(f"Converted images are saved to: {output_dir}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert PGM images to PNG format in sorted order.")
    parser.add_argument("input_dir", help="Path to the input directory containing .pgm files.")
    parser.add_argument("output_dir", help="Path to the output directory to save .png files.")
    args = parser.parse_args()

    convert_pgm_to_png(args.input_dir, args.output_dir)

