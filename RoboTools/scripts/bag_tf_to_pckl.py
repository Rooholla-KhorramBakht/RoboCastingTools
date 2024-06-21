#!/usr/bin/env python3
import pickle
import argparse
from RoboTools.rosbag_utils import extract_tf_data

def save_to_pickle(data, file):
    with open(file, 'wb') as f:
        pickle.dump(data, f)

def main():
    parser = argparse.ArgumentParser(description="Extract TF data from ROS bag file and save it to a pickle file.")
    parser.add_argument('bagfile', type=str, help="The path to the bag file.")
    parser.add_argument('picklefile', type=str, help="The path to the output pickle file.")
    args = parser.parse_args()
    
    tf_data = extract_tf_data(args.bagfile)
    save_to_pickle(tf_data, args.picklefile)
    print(f"TF data extracted and saved to {args.picklefile}.")
    return


if __name__ == "__main__":
    main()