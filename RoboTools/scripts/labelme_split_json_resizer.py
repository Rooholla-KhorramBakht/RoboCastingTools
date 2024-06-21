import os
import json
from tqdm import tqdm
import argparse
import cv2
'''
This script was originally written to change information in split (dev.json, train.json, and test.json) config files of the armbench dataset
after resizing the labels and images to a desired target_h and target_w values.
'''
def modify_json_file(img_dir, json_file, target_h=480, target_w=640):
    
    # Open and parse the JSON file
    with open(json_file, 'r') as file:
        data = json.load(file)

    # Update the data for each sample
    img_id_to_img_size = {}
    print('Changing the image info ...')
    for i in tqdm(range(len(data['images']))):
        # Extract the data that are to be modified
        image = data['images'][i]
        # Get the name and the path of the image file
        img_name = image['file_name']
        img_path = os.path.join(img_dir, img_name)
        # Get the shape of the original image
        h, w, _ = cv2.imread(img_path).shape
        img_id_to_img_size[image['id']]=(h,w)
        # Modify the image shape
        data['images'][i]['width']=target_w
        data['images'][i]['height']=target_h
    
    print('Resizing the annotations ...')
    for i in tqdm(range(len(data['annotations']))):
        annotation = data['annotations'][i]
        segmentations = annotation['segmentation']
        bbox = annotation['bbox']
        h,w = img_id_to_img_size[annotation['image_id']]
        # Modify the segment values. Even entries are x and odd entries are y
        for segmentation in segmentations:
            seg_x = [s*target_w//w for s in segmentation[0::2]]
            seg_y = [s*target_h//h for s in segmentation[1::2]]
            segmentation[0::2]=seg_x.copy()
            segmentation[1::2]=seg_y.copy()
        # Modify the bounding box values
        bbox[0::2] = [s*target_w//w for s in bbox[0::2]]
        bbox[1::2] = [s*target_h//h for s in bbox[1::2]]
        # Store the modified values
        data['annotations'][i]['segmentation'] = segmentations.copy()
        data['annotations'][i]['bbox'] = bbox.copy()
    json_file_name = json_file.split('/')[-1].split('.')[0]
    with open(f'{json_file_name}_resized.json', 'w') as file:
        json.dump(data, file, indent=4)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Resize JSON files corresponding to the train-test-dev split configuration.")
    parser.add_argument("img_dir", help="Directory containing the image files")
    parser.add_argument("json_file", help="Path to the JSON file")
    parser.add_argument("target_width", type=int, help="Target image height")
    parser.add_argument("target_height", type=int, help="Target image width")
    args = parser.parse_args()
    modify_json_file(args.img_dir, args.json_file, args.target_height, args.target_width)