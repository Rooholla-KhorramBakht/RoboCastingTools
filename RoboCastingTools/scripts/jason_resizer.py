import os
import json
from tqdm import tqdm
import argparse

def resize_json_files(input_dir, output_dir, new_width, new_height):
    # Create the output directory if it doesn't exist
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # List all JSON files in the input directory
    json_files = [f for f in os.listdir(input_dir) if f.endswith('.json')]

    for filename in tqdm(json_files):
        json_path = os.path.join(input_dir, filename)

        # Open and parse the JSON file
        with open(json_path, 'r') as file:
            data = json.load(file)

        # Update the points in each shape
        for shape in data['shapes']:
            for point in shape['points']:
                point[0] *= new_width / data['imageWidth']
                point[1] *= new_height / data['imageHeight']
        # Update imageWidth and imageHeight
        data['imageWidth'] = new_width
        data['imageHeight'] = new_height
        # Save the modified JSON to the output directory
        output_path = os.path.join(output_dir, filename)
        with open(output_path, 'w') as file:
            json.dump(data, file, indent=4)

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Resize Jason instance segmentation labels in a directory")
    parser.add_argument("input_dir", help="Input directory containing the Jason files")
    parser.add_argument("output_dir", help="Output directory where the resized jason files should be saved")
    parser.add_argument("width", type=int, help="Width for resized Jason files")
    parser.add_argument("height", type=int, help="Height for resized Jason files")

    args = parser.parse_args()
    resize_json_files(args.input_dir, args.output_dir, args.width, args.height)
