import os
import argparse
import cv2
import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from PIL import Image
import pickle
import pandas as pd
import numpy as np
from tqdm import tqdm

import rosbag
from tf2_msgs.msg import TFMessage
from collections import defaultdict
from scipy.spatial.transform import Rotation
import shutil

def extract_tf_data(bagfile):
    """
    Extracts transformation (TF) data from a given ROS bag file.

    Parameters:
    bagfile (str): The path to the bag file from which to extract TF data.

    Returns:
    dict: A dictionary where the keys are the TF's frame IDs (child frame ID) and the values are lists of tuples.
    Each tuple represents the time stamp of the frame, pose and the corresponding parent ID of the tf message.

    Example:
    {'tag_id1': [(timestamp2, (x, y, z, rotation), parent_id), ...],
    'tag_id2': [(timestamp2, (x, y, z, rotation), parent_id), ...],
    ...}
    """
    bag = rosbag.Bag(bagfile)
    tf_data = defaultdict(list)
    for topic, msg, t in bag.read_messages(topics=["/tf", "/tf_static"]):
        for transform in msg.transforms:
            # You can also use child_frame_id depending on your needs
            tag_id = transform.child_frame_id
            parent_id = transform.header.frame_id
            pose = transform.transform
            r = np.array([pose.translation.x, pose.translation.y, pose.translation.z])
            q = [pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w]
            R = Rotation.from_quat(q).as_matrix()
            T = np.vstack([np.hstack([R, r.reshape(3, 1)]), np.array([0, 0, 0, 1])])
            timestamp = t.to_sec()  # Convert to seconds
            tf_data[tag_id].append((timestamp, parent_id, T))

    bag.close()
    return tf_data


def bag2Images(bag_file_path, output_dir, image_topic):
    """
    bag2Images extracts the RGB images published in a ROS bagfile and
    stores them alongside the corresponding timestamps into a csv file.

    :param bag_file_path: path to the rosbag file
    :param output_dir:    the output directory where the extracted images and stamps should be stored
    :param image_topic:   the name of the image topic published in the rosbag file.
    :returns: None
    """
    if os.path.exists(os.path.join(output_dir)):
        shutil.rmtree(os.path.join(output_dir))
        os.mkdir(os.path.join(output_dir))
        os.mkdir(os.path.join(output_dir, 'data'))
    else:
        os.mkdir(os.path.join(output_dir))
        os.mkdir(os.path.join(output_dir, 'data'))

    data = []  # Store the image timestamps
    bag = rosbag.Bag(bag_file_path, "r")
    bridge = CvBridge()
    for topic, msg, t in tqdm(bag.read_messages(topics=[image_topic])):
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        stamp = msg.header.stamp.to_nsec()
        img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        cv2.imwrite(os.path.join(output_dir,'data', f"{stamp}.png"), img)
        cv2.imshow("preview", img)
        cv2.waitKey(1)
        data.append([stamp, f"{stamp}.png"])

    bag.close()
    df = pd.DataFrame(data=data, columns=['#timestamp [ns]', 'filename'])
    df.to_csv(os.path.join(output_dir,"data.csv"), index=False)
    cv2.destroyAllWindows()

def bag2DepthImages(bag_file_path, output_dir, depth_image_topic):
    """
    bag2DepthImages extracts the depth images published in a ROS bagfile and
    stores them as 16-bit PNG files alongside the corresponding timestamps into a csv file.

    :param bag_file_path: path to the rosbag file
    :param output_dir:    the output directory where the extracted images and stamps should be stored
    :param depth_image_topic:   the name of the depth image topic published in the rosbag file.
    :returns: None
    """
    if os.path.exists(os.path.join(output_dir)):
        shutil.rmtree(os.path.join(output_dir))
    os.mkdir(output_dir)
    os.mkdir(os.path.join(output_dir, 'data'))

    data = []  # Store the image timestamps
    bag = rosbag.Bag(bag_file_path, "r")
    bridge = CvBridge()
    for topic, msg, t in tqdm(bag.read_messages(topics=[depth_image_topic])):
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        stamp = msg.header.stamp.to_nsec()
        cv2.imwrite(os.path.join(output_dir, 'data', f"{stamp}.png"), cv_img, [cv2.IMWRITE_PNG_COMPRESSION, 0])
        data.append([stamp, f"{stamp}.png"])

    bag.close()
    df = pd.DataFrame(data=data, columns=['#timestamp [ns]', 'filename'])
    df.to_csv(os.path.join(output_dir, "data.csv"), index=False)


def bag2IMUs(bag_file_path, output_dir, imu_topic):
    """
    bag2Imus extracts the IMU data published in a ROS bagfile and returns a pandas dataframe.

    :param bag_file_path: path to the rosbag file
    :param output_dir:    the output directory where the video and stamps should be stored
    :param imu_topic:   the name of the IMU topic published in the rosbag file
    :returns: None
    """

    bag = rosbag.Bag(bag_file_path, "r")
    column_names = ["#timestamp [ns]", 
                    "w_RS_S_x [rad s^-1]", 
                    "w_RS_S_y [rad s^-1]",
                    "w_RS_S_z [rad s^-1]",
                    "a_RS_S_x [m s^-2]",
                    "a_RS_S_y [m s^-2]",
                    "a_RS_S_z [m s^-2]"]
    
    accels = []
    gyros = []
    stamps = []
    for topic, msg, t in tqdm(bag.read_messages(topics=[imu_topic])):
        stamp = msg.header.stamp.to_nsec()
        ax = msg.linear_acceleration.x 
        ay = msg.linear_acceleration.y 
        az = msg.linear_acceleration.z 
        wx = msg.angular_velocity.x 
        wy = msg.angular_velocity.y 
        wz = msg.angular_velocity.z 
        accel = [ax, ay, az]
        gyro = [wx, wy, wz]
        accels.append(accel)
        gyros.append(gyro)
        stamps.append(stamp)
    assert len(stamps) > 0, 'There are no IMU topics with the given name in the selected bag file or the number of published topics is zero.'
    accels = np.array(accels)
    gyros = np.array(gyros)
    stamps = np.array(stamps).reshape(-1,1)
    data = np.hstack([stamps, gyros, accels])
    df = pd.DataFrame(data=data, columns= column_names)
    df.to_csv(os.path.join(output_dir, 'data.csv'), index = False)
    return df

def bag2Poses(bag_file_path, output_dir, pose_topic):
    """
    bag2Poses extracts the stamp poses published in a ROS bagfile and returns a pandas dataframe.

    :param bag_file_path: path to the rosbag file
    :param output_dir:    the output directory where the video and stamps should be stored
    :param imu_topic:   the name of the IMU topic published in the rosbag file
    :returns: None
    """

    bag = rosbag.Bag(bag_file_path, "r")
    column_names = ["#timestamp [ns]", 
                    "p_RS_R_x [m]", 
                    "p_RS_R_y [m]",
                    "p_RS_R_z [m]",
                    "q_RS_w []",
                    "q_RS_x []",
                    "q_RS_y []",
                    "q_RS_z []",
                    ]
    ts = []
    qs = []
    stamps = []
    for topic, msg, t in tqdm(bag.read_messages(topics=[pose_topic])):
        stamp = msg.header.stamp.to_nsec()
        px = msg.pose.position.x 
        py = msg.pose.position.y 
        pz = msg.pose.position.z 
        qx = msg.pose.orientation.x 
        qy = msg.pose.orientation.y 
        qz = msg.pose.orientation.z 
        qw = msg.pose.orientation.w 
        t = [px, py, pz]
        q = [qw, qx, qy, qz]
        ts.append(t)
        qs.append(q)
        stamps.append(stamp)

    assert len(stamps) > 0, 'There are no pose topics with the given name in the selected bag file or the number of published topics is zero.'
    qs = np.array(qs)
    ts = np.array(ts)
    stamps = np.array(stamps).reshape(-1,1)
    data = np.hstack([stamps, ts, qs])
    df = pd.DataFrame(data=data, columns= column_names)
    df.to_csv(os.path.join(output_dir, 'data.csv'), index = False)
    return df

def bag2JointStates(bag_file_path, output_dir, joint_state_topic):
    """
    bag2JointStates extracts the joint states published in a ROS bagfile and returns a pandas dataframe.

    :param bag_file_path: path to the rosbag file
    :param output_dir:    the output directory where the video and stamps should be stored
    :param joint_state_topic:   the name of the joint state topic published in the rosbag file
    :returns: None
    """

    bag = rosbag.Bag(bag_file_path, "r")
    column_names = ["#timestamp [ns]", 
                    "FR_HIP_q [rad]",
                    "FR_THIGH_q [rad]",
                    "FR_KNEE_q [rad]",
                    "FL_HIP_q [rad]",
                    "FL_THIGH_q [rad]",
                    "FL_KNEE_q [rad]",
                    "RR_HIP_q [rad]",
                    "RR_THIGH_q [rad]",
                    "RR_KNEE_q [rad]",
                    "RL_HIP_q [rad]",
                    "RL_THIGH_q [rad]",
                    "RL_KNEE_q [rad]",
                    "FR_HIP_qd [rad s^-1]",
                    "FR_THIGH_qd [rad s^-1]",
                    "FR_KNEE_qd [rad s^-1]",
                    "FL_HIP_qd [rad s^-1]",
                    "FL_THIGH_qd [rad s^-1]",
                    "FL_KNEE_qd [rad s^-1]",
                    "RR_HIP_qd [rad s^-1]",
                    "RR_THIGH_qd [rad s^-1]",
                    "RR_KNEE_qd [rad s^-1]",
                    "RL_HIP_qd [rad s^-1]",
                    "RL_THIGH_qd [rad s^-1]",
                    "RL_KNEE_qd [rad s^-1]",
                    "FR_HIP_tau [Nm]",
                    "FR_THIGH_tau [Nm]",
                    "FR_KNEE_tau [Nm]",
                    "FL_HIP_tau [Nm]",
                    "FL_THIGH_tau [Nm]",
                    "FL_KNEE_tau [Nm]",
                    "RR_HIP_tau [Nm]",
                    "RR_THIGH_tau [Nm]",
                    "RR_KNEE_tau [Nm]",
                    "RL_HIP_tau [Nm]",
                    "RL_THIGH_tau [Nm]",
                    "RL_KNEE_tau [Nm]",
                    ]
    q = []
    qd = []
    tau = []
    stamps = []
    for topic, msg, t in tqdm(bag.read_messages(topics=[joint_state_topic])):
        stamp = msg.header.stamp.to_nsec()
        q = msg.position
        qd = msg.velocity
        tau = msg.effort
        stamps.append(stamp)
    assert len(stamps) > 0, 'There are no joint state topics with the given name in the selected bag file or the number of published topics is zero.'
    q = np.array(q)
    qd = np.array(qd)
    tau = np.array(tau)
    stamps = np.array(stamps).reshape(-1,1)
    data = np.hstack([stamps, q, qd, tau])
    df = pd.DataFrame(data=data, columns= column_names)
    df.to_csv(os.path.join(output_dir, 'data.csv'), index = False)
    return df

def bag2FootContacts(bag_file_path, output_dir, foot_contact_topic):
    """
    bag2JointStates extracts the foot contacts published as joint state messages in a ROS bagfile and returns a pandas dataframe.

    :param bag_file_path: path to the rosbag file
    :param output_dir:    the output directory where the video and stamps should be stored
    :param foot_contact_topic:   the name of the joint state topic published in the rosbag file
    :returns: None
    """
    
    bag = rosbag.Bag(bag_file_path, "r")
    column_names = ["#timestamp [ns]", 
                    "FR_Contact [N]",
                    "FL_Contact [N]",
                    "RR_Contact [N]",
                    "RL_Contact [N]",
                    ]
    contacts = []
    stamps = [] 
    for topic, msg, t in tqdm(bag.read_messages(topics=[foot_contact_topic])):
        stamp = msg.header.stamp.to_nsec()
        fr = msg.states[0].effort
        fl = msg.states[1].effort
        rr = msg.states[2].effort
        rl = msg.states[3].effort
        contacts.append([fr, fl, rr, rl])
        stamps.append(stamp)
    assert len(stamps) > 0, 'There are no foot contact topics with the given name in the selected bag file or the number of published topics is zero.'
    contacts = np.array(contacts)
    stamps = np.array(stamps).reshape(-1,1)
    data = np.hstack([stamps, contacts])
    df = pd.DataFrame(data=data, columns= column_names)
    df.to_csv(os.path.join(output_dir, 'data.csv'), index = False)
    return df

