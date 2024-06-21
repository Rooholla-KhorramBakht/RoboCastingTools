from pyquaternion import Quaternion
import numpy as np
import cv2


def cvK2BulletP(K, w, h, near, far):
    """
    cvKtoPulletP converst the K interinsic matrix as calibrated using Opencv
    and ROS to the projection matrix used in openGL and Pybullet.

    :param K:  OpenCV 3x3 camera intrinsic matrix
    :param w:  Image width
    :param h:  Image height
    :param near:     The nearest objects to be included in the render
    :param far:      The furthest objects to be included in the render
    :return:   4x4 projection matrix as used in openGL and pybullet
    """
    f_x = K[0, 0]
    f_y = K[1, 1]
    c_x = K[0, 2]
    c_y = K[1, 2]
    A = (near + far) / (near - far)
    B = 2 * near * far / (near - far)

    projection_matrix = [
        [2 / w * f_x, 0, (w - 2 * c_x) / w, 0],
        [0, 2 / h * f_y, (2 * c_y - h) / h, 0],
        [0, 0, A, B],
        [0, 0, -1, 0],
    ]
    # The transpose is needed for respecting the array structure of the OpenGL
    return np.array(projection_matrix).T.reshape(16).tolist()


def cvPose2BulletView(q, t):
    """
    cvPose2BulletView gets orientation and position as used
    in ROS-TF and opencv and coverts it to the view matrix used
    in openGL and pyBullet.

    :param q: ROS orientation expressed as quaternion [qx, qy, qz, qw]
    :param t: ROS postion expressed as [tx, ty, tz]
    :return:  4x4 view matrix as used in pybullet and openGL

    """
    q = Quaternion([q[3], q[0], q[1], q[2]])
    R = q.rotation_matrix

    T = np.vstack([np.hstack([R, np.array(t).reshape(3, 1)]), np.array([0, 0, 0, 1])])
    # Convert opencv convention to python convention
    # By a 180 degrees rotation along X
    Tc = np.array([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]).reshape(
        4, 4
    )

    # pybullet pse is the inverse of the pose from the ROS-TF
    T = Tc @ np.linalg.inv(T)
    # The transpose is needed for respecting the array structure of the OpenGL
    viewMatrix = T.T.reshape(16)
    return viewMatrix


def merge_images(
    real_image, sim_image, masks, alpha, mask_ids=[2, 3, 4, 5, 6, 7, 8, 9, 10, 11]
):
    """
    Merges real and simulated images using provided masks.

    @param real_image: Real image captured by the robot
    @type real_image: numpy.ndarray of shape (height, width, 3)
    @param sim_image: Simulated image corresponding to the real image
    @type sim_image: numpy.ndarray of shape (height, width, 3)
    @param masks: List of binary masks specifying different objects in the scene
    @type masks: numpy.ndarray of shape (height, width, num_masks)
    @param alpha: Proportion parameter to control blending of real and simulated images
    @type alpha: float between 0 and 1
    @param mask_ids: List of integers specifying which masks to use for blending
    @type mask_ids: list of ints, default is [2,3,4,5,6,7, 8, 9, 10, 11]

    @return: Final blended image with cutouts from the simulated image added to the real image
    @rtype: numpy.ndarray of shape (height, width, 3)
    """
    sim_image = cv2.cvtColor(sim_image, cv2.COLOR_BGR2RGB)
    m = [(masks[..., 2] == i).astype(np.uint8) for i in mask_ids]
    mask = (sum(m) != 0).astype(np.uint8)
    # Make sure images are same size
    assert real_image.shape == sim_image.shape, "Images are not the same size"

    # Create a copy of real_image to add cutouts on
    output_image = real_image.copy()

    # Make sure mask is same size as images
    assert mask.shape[:2] == real_image.shape[:2], "Mask is not the same size as images"

    # Extract cutout from simulated image
    cutout = cv2.bitwise_and(sim_image, sim_image, mask=mask)
    cutout2 = cv2.bitwise_and(real_image, real_image, mask=1 - mask)
    cutout = cutout + cutout2

    # Add cutout to real image with a blend defined by alpha
    output_image = cv2.addWeighted(output_image, 1 - alpha, cutout, alpha, 0)

    # Return the final image
    return output_image
