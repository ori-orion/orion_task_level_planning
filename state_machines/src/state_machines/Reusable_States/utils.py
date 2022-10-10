from geometry_msgs.msg import Pose, Point, PoseStamped;

import numpy as np;

from tf.transformations import euler_from_quaternion, quaternion_from_euler

import rospy;

from strands_navigation_msgs.msg import TopologicalMap

from tmc_msgs.msg import TalkRequestAction, TalkRequestGoal, Voice

import actionlib


TASK_SUCCESS = 'task_success';
TASK_FAILURE = 'task_failure';
SUCCESS = 'success';
FAILURE = 'failure';
REPEAT_FAILURE = 'repeat_failure';


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

def get_closest_node(dest_pose):
    """ Get the closest node to a destination pose. Returns name and pose. """

    top_map = rospy.wait_for_message('/topological_map', TopologicalMap)

    nodes = top_map.nodes

    best_dist = float('inf')
    best_node_pose = (None, None)

    for node in nodes:
        new_dist = distance_between_poses(node.pose, dest_pose)
        if new_dist < best_dist:
            best_dist = new_dist
            best_node_pose = (node.name, node.pose)

    return best_node_pose

def get_point_magnitude(point:Point):
    return np.sqrt(point.x*point.x + point.y*point.y + point.z*point.z);

def call_talk_request_action_server(phrase : str):
    ''' Function to call the Toyota talk request action server

    '''
    action_goal = TalkRequestGoal()
    action_goal.data.language = Voice.kEnglish  # enum for value: 1
    action_goal.data.sentence = phrase

    rospy.loginfo("HSR speaking phrase: '{}'".format(phrase))
    speak_action_client = actionlib.SimpleActionClient('/talk_request_action',
                                    TalkRequestAction)

    speak_action_client.wait_for_server()
    speak_action_client.send_goal(action_goal)
    speak_action_client.wait_for_result()

    return

def attribute_to_sentence(attribute):
    # function to convert attributes like Wearing_Hat into pronouncable wearing hat
    atr = attribute.split('_')
    atr = [a.lower() for a in atr]
    return atr

def get_current_pose() -> Pose:
    pose:PoseStamped = rospy.wait_for_message('/global_pose', PoseStamped)
    return pose.pose;


# From semantic_mapping/.../utils.py
# For taking parameters from the ros parameter bit.
def dict_to_obj(dictionary:dict, objFillingOut):
    """
    The main idea here is that we may well want to convert an arbitrary dictionary to one of the ROS types
    we've created. This will do it.

    Inputs:
        dictionary:dict     The dictionary that we want to fill out the ROS message with. 
        objFillingOut       An empty ROS message to fill out.
    """

    attributes = objFillingOut.__dir__();
    for key in dictionary.keys():
        if (key in attributes):
            if isinstance(dictionary[key], dict):                
                dict_to_obj(dictionary[key], getattr(objFillingOut, key));
                continue;
            elif isinstance(dictionary[key], list):
                carry = [];
                for element in dictionary[key]:
                    if element is dict:
                        raise(Exception("The sub-class of a list is a dictionary. The type is not currently known."));
                    else:
                        carry.append(element);
                setattr(objFillingOut, key, carry);
                continue;
            else:
                setattr(objFillingOut, key, dictionary[key]);
    
    return objFillingOut;