import rosbag
try:
    from unitree_legged_msgs.msg import HighState
except:
    print('The unitree message definitions are not installed. Compile and source the unitree')
    print('unitree_ros_to_real ROS node at https://github.com/unitreerobotics/unitree_ros_to_real')
    exit(-1)

import pandas as pd
from tqdm import tqdm
import numpy as np
import os


def UnitreeBag2CSV(bag_file_path, output_dir, topic_name):
    """
    UnitreeBag2CSV extracts the important sensory data of the unitree quadruped robot
    as published in a ROS bagfile and stores them as a set of csv files.

    :param bag_file_path: path to the rosbag file
    :param output_dir:    the output directory where the extracted images and stamps should be stored
    :param highstate_topic:   the name of the robot state topic published in the rosbag file.
    :returns: None
    """

    imu_data = []  # IMU samples
    kinematic_data = []  # Joint Kinematic Samples
    foot_force_data = []  # Foot force estimates from the online controller
    robot_pos_data = []  # Robot estimated position

    bag = rosbag.Bag(bag_file_path, "r")
    for topic, msg, t in tqdm(bag.read_messages(topics=[topic_name])):
        # Store IMU data
        imu_entry = np.hstack([t.to_nsec(),
                               msg.imu.quaternion,
                               msg.imu.gyroscope,
                               msg.imu.accelerometer,
                               msg.imu.rpy,
                               msg.imu.temperature]).squeeze()
        imu_data.append(imu_entry)
        # Store Kinematic data
        kin_entry = [(m.q, m.dq, m.ddq, m.tauEst, m.temperature)
                     for m in msg.motorState]
        kin_entry = np.hstack([t.to_nsec()]+kin_entry)[0:12*5+1].squeeze()
        kinematic_data.append(kin_entry)
        foot_force_data.append(msg.footForce)
        # Store the robot position
        robot_position = np.hstack([msg.position, msg.velocity, msg.yawSpeed])
        robot_pos_data.append(robot_position)

    bag.close()
    # Store the kinematic dataset
    kinematic_columns = np.array([[f'q_{i}', f'dq_{i}', f'ddq_{i}', f'tauEst_{i}', f'temperature_{i}']
                                  for i in range(1, 13)]).reshape(-1).tolist()
    kinematic_columns = ['stamp_ns'] + kinematic_columns
    kinematic_data = np.vstack(kinematic_data)
    kin_df = pd.DataFrame(data=kinematic_data.squeeze(),
                          columns=kinematic_columns)

    foot_force_data = np.array(foot_force_data)
    foot_force_columns = [f'f{i}' for i in range(1, 5)]
    foot_force_df = pd.DataFrame(data=foot_force_data.squeeze(),
                                 columns=foot_force_columns)
    foot_force_df.to_csv(os.path.join(
        output_dir, "foot_force.csv"), index=False)
    kin_df.to_csv(os.path.join(output_dir, "kinematics.csv"), index=False)
    # Store the IMU dataset
    imu_columns = ['stamp_ns', 'q1', 'q2', 'q3', 'q4',
                   'wx', 'wy', 'wz', 'ax', 'ay', 'az',
                   'roll', 'pitch', 'yaw', 'temperature']
    imu_data = np.vstack(imu_data)
    imu_df = pd.DataFrame(data=imu_data.squeeze(), columns=imu_columns)
    imu_df.to_csv(os.path.join(output_dir, "imu.csv"), index=False)
    # Store robot postion dataset
    robot_pos_data = np.array(robot_pos_data)
    robot_pos_df = pd.DataFrame(data=robot_pos_data,
                                columns=['x', 'y', 'yaw', 'vx', 'vy', 'omega_z', 'yaw_spedd'])
    robot_pos_df.to_csv(os.path.join(
        output_dir, "position_velocity.csv"), index=False)
    return kin_df, imu_df, foot_force_df, robot_pos_df, msg
