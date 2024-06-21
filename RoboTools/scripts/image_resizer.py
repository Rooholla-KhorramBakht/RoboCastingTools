import os
from PIL import Image
import argparse
from tqdm import tqdm

def resize_images(input_dir, output_dir, width, height):
    # Create the output directory if it doesn't exist
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # List all files in the input directory
    file_list = os.listdir(input_dir)

    for filename in tqdm(file_list):
        input_path = os.path.join(input_dir, filename)
        output_path = os.path.join(output_dir, filename)

        try:
            # Open the image
            img = Image.open(input_path)

            # Resize the image
            img = img.resize((width, height), Image.ANTIALIAS)

            # Save the resized image to the output directory
            img.save(output_path)
        except Exception as e:
            print(f"Error processing {filename}: {str(e)}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Resize images in a directory")
    parser.add_argument("input_dir", help="Input directory containing images")
    parser.add_argument("output_dir", help="Output directory for resized images")
    parser.add_argument("width", type=int, help="Width for resized images")
    parser.add_argument("height", type=int, help="Height for resized images")

    args = parser.parse_args()

    resize_images(args.input_dir, args.output_dir, args.width, args.height)

