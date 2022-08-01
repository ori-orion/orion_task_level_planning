from geometry_msgs.msg import Pose, Point;

import numpy as np;

from tf.transformations import euler_from_quaternion, quaternion_from_euler

def pose_to_xy_theta(pose:Pose):
    """ Function converts a pose to an (x,y,theta) triple.

    Args:
        pose: The pose msg

    Return:
        triple: A triple consisting of (x,y,theta)
    """
    x = pose.position.x
    y = pose.position.y

    quat = [pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w]

    (_, _, yaw) = euler_from_quaternion(quat)

    theta = yaw

    return (x,y,theta);


def distance_between_points(pos_1:Point, pos_2:Point):
    delta_x_sq = np.power(pos_2.x - pos_1.x, 2)
    delta_y_sq = np.power(pos_2.y - pos_1.y, 2)
    delta_z_sq = np.power(pos_2.z - pos_1.z, 2)

    return np.sqrt(delta_x_sq + delta_y_sq + delta_z_sq);
    
def distance_between_poses(pose_1:Pose, pose_2:Pose):
    """Given two poses, this finds the Euclidean distance between them. """

    pos_1 = pose_1.position
    pos_2 = pose_2.position

    return distance_between_points(pos_1, pos_2);
