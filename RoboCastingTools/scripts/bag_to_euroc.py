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

    parser.add_argument("--imu_topic", help="IMU topic.", default=None)
    parser.add_argument("--vicon_topic", help="Vicon topic.", default=None)
    parser.add_argument("--img0_topic", help="Optional image topic for img0.", default=None)
    parser.add_argument("--img1_topic", help="Optional image topic for img1.", default=None)
    parser.add_argument("--depth_topic", help="Optional depth image topic.", default=None)
    parser.add_argument("--jointstate_topic", help="Optional jointstates topic.", default=None)
    parser.add_argument("--feet_contact_topic", help="Optional foot contact topic.", default=None)

    args = parser.parse_args()

    if os.path.exists(os.path.join(args.output_dir)):
        shutil.rmtree(args.output_dir)
        os.mkdir(os.path.join(args.output_dir))
        if args.imu_topic is not None:
            os.mkdir(os.path.join(args.output_dir, 'imu0'))
        if args.img0_topic is not None:
            os.mkdir(os.path.join(args.output_dir, 'imu0'))
        if args.img1_topic is not None:
            os.mkdir(os.path.join(args.output_dir, 'imu1'))
        if args.vicon_topic is not None:
            os.mkdir(os.path.join(args.output_dir, 'vicon0'))
        if args.depth_topic is not None:
            os.mkdir(os.path.join(args.output_dir, 'depth0'))
        if args.jointstate_topic is not None:
            os.mkdir(os.path.join(args.output_dir, 'joints0'))
        if args.feet_contact_topic is not None:
            os.mkdir(os.path.join(args.output_dir, 'feet_contact0'))
        
    else:
        if args.imu_topic is not None:
            os.mkdir(os.path.join(args.output_dir, 'imu0'))
        if args.img0_topic is not None:
            os.mkdir(os.path.join(args.output_dir, 'imu0'))
        if args.img1_topic is not None:
            os.mkdir(os.path.join(args.output_dir, 'imu1'))
        if args.vicon_topic is not None:
            os.mkdir(os.path.join(args.output_dir, 'vicon0'))
        if args.depth_topic is not None:
            os.mkdir(os.path.join(args.output_dir, 'depth0'))
        if args.jointstate_topic is not None:
            os.mkdir(os.path.join(args.output_dir, 'joints0'))
        if args.feet_contact_topic is not None:
            os.mkdir(os.path.join(args.output_dir, 'feet_contact0'))
    
    if args.img0_topic is not None:
        print('Extracting the img0 data')
        bag2Images(args.bag_file, os.path.join(args.output_dir, 'cam0'), args.img0_topic)
    if args.img1_topic is not None:
        print('Extracting the img1 data')
        bag2Images(args.bag_file, os.path.join(args.output_dir, 'cam1'), args.img1_topic)
    if args.imu_topic is not None:
        print('Extracting the IMU data')
        bag2IMUs(args.bag_file, os.path.join(args.output_dir, 'imu0'), args.imu_topic)
    if args.vicon_topic is not None:
        print('Extracting the Vicon data')
        bag2Poses(args.bag_file, os.path.join(args.output_dir, 'vicon0'), args.vicon_topic)
    return

if __name__ == "__main__":
    main()
