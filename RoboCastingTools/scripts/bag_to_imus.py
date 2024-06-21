#!/usr/bin/env python3
"""Extract images from a rosbag.
"""

import argparse
from RoboCastingTools.rosbag_utils import *


def main():
    """Extract a folder of images from a rosbag."""
    parser = argparse.ArgumentParser(description="Extract images from a ROS bag.")
    parser.add_argument("bag_file", help="Input ROS bag.")
    parser.add_argument("output_dir", help="Output directory.")
    parser.add_argument("image_topic", help="Image topic.")

    args = parser.parse_args()
    bag2Images(args.bag_file, args.output_dir, args.image_topic)

    return


if __name__ == "__main__":
    main()
