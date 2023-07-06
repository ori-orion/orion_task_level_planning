import smach;
import numpy as np;
import math;

import rospy;
import actionlib
from geometry_msgs.msg import Pose, Point, PoseStamped, Quaternion;
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray, InteractiveMarker, InteractiveMarkerControl
from interactive_markers.interactive_marker_server import InteractiveMarkerServer

from tmc_msgs.msg import TalkRequestAction, TalkRequestGoal, Voice

import hsrb_interface;
import hsrb_interface.geometry as geometry
hsrb_interface.robot.enable_interactive();

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

# def get_closest_node(dest_pose):
#     """ Get the closest node to a destination pose. Returns name and pose. """

#     top_map = rospy.wait_for_message('/topological_map', TopologicalMap)

#     nodes = top_map.nodes

#     best_dist = float('inf')
#     best_node_pose = (None, None)

#     for node in nodes:
#         new_dist = distance_between_poses(node.pose, dest_pose)
#         if new_dist < best_dist:
#             best_dist = new_dist
#             best_node_pose = (node.name, node.pose)

#     return best_node_pose

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

# Returns the current location of the robot.
def get_current_pose() -> Pose:
    pose:PoseStamped = rospy.wait_for_message('/global_pose', PoseStamped)
    return pose.pose;

def point_to_numpy(point:Point) -> np.ndarray:
    return np.asarray([point.x, point.y, point.z], dtype=float);


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


def rot_mat_to_quaternion(rot_mat:np.ndarray):
    quat = Quaternion();

    quat.w = math.sqrt( 1 + rot_mat[0,0] + rot_mat[1,1] + rot_mat[2,2] )  / 2;
    quat.x = (rot_mat[2,1] - rot_mat[1,2])/(4*quat.w);
    quat.y = (rot_mat[0,2] - rot_mat[2,0])/(4*quat.w);
    quat.z = (rot_mat[1,0] - rot_mat[0,1])/(4*quat.w);

    return quat;

class RvizVisualisationManager:
    def __init__(self, 
        im_server:InteractiveMarkerServer, 
        colour_a, colour_r, colour_g, colour_b):

        # Interactive Marker server.
        self.im_server:InteractiveMarkerServer = im_server;

        # In the range [0,1]
        self.colour_a = colour_a;
        self.colour_r = colour_r;
        self.colour_g = colour_g;
        self.colour_b = colour_b;

        self.query_callback = None;

    def delete_object(self, id):
        """
        Deletes the rviz object with id.
        """
        self.im_server.erase(id);
        self.im_server.applyChanges();

    def handle_viz_input(self, input):
        """
        Handles the click callback for when the user clicks on one of the rviz boxes.
        """
        if (self.query_callback != None):
            pass;


    def add_object_arrow(
            self, 
            id:str, 
            position:Point, 
            direction_vec:np.ndarray, 
            size:Point, 
            obj_class:str,
            alpha_val=None):
            
        # print(direction_vec);
        def normalise(vec:np.ndarray):
            # print("\t", vec);
            return vec / np.linalg.norm(vec);

        direction_vec = np.copy( direction_vec );
        direction_vec *= -1;

        up_vec = np.asarray([0,0,1], dtype=np.float32);
        side = np.cross(up_vec, direction_vec);
        up_new = np.cross(side, direction_vec);
        
        up_vec = normalise(up_vec);
        side = normalise(side);
        direction_vec = normalise(direction_vec);

        rotation_mat = np.zeros((3,3));
        rotation_mat[:,0] = direction_vec;
        rotation_mat[:,1] = up_new;
        rotation_mat[:,2] = side;

        quaternion = rot_mat_to_quaternion(rotation_mat);
        pose = Pose();
        pose.position = position;
        pose.orientation = quaternion;

        self.add_object(id, pose, size, obj_class, marker_type=Marker.ARROW, alpha_val=alpha_val)

    def add_object(
            self, 
            id:str, 
            pose:Pose, 
            size:Point, 
            obj_class:str, 
            alpha_val=None,
            marker_type=Marker.CUBE):
        """
        Deals with the adding of an object to the visualisation server. 
        id                  - The id of the object (and the id that rviz will use).
        pose                - The pose of the object
        size                - The size of the object
        obj_class           - The label the object will be given in the visualisation.
        alpha_val
        marker_type
        """

        self.im_server.erase(id);

        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "map"
        int_marker.name = id;
        int_marker.description = obj_class;
        int_marker.pose = pose;

        box_marker = Marker();
        box_marker.type = marker_type;
        box_marker.pose.orientation.w = 1;
        
        box_marker.scale.x = size.x if size.x > 0.05 else 0.05;
        box_marker.scale.y = size.y if size.y > 0.05 else 0.05;
        box_marker.scale.z = size.z if size.z > 0.05 else 0.05;
        
        box_marker.color.r = self.colour_r;
        box_marker.color.g = self.colour_g;
        box_marker.color.b = self.colour_b;
        if alpha_val != None:
            box_marker.color.a = alpha_val;

        button_control = InteractiveMarkerControl()
        button_control.interaction_mode = InteractiveMarkerControl.BUTTON
        button_control.always_visible = True
        button_control.markers.append(box_marker);
        int_marker.controls.append(button_control);

        self.im_server.insert(int_marker, self.handle_viz_input)
        self.im_server.applyChanges();
        return


class SmachBaseClass(smach.State):
    
    JOINT_ARM_FLEX = 'arm_flex_joint';
    JOINT_ARM_LIFT = 'arm_lift_joint';
    JOINT_ARM_ROLL = 'arm_roll_joint';
    JOINT_HEAD_PAN = 'head_pan_joint';
    JOINT_HEAD_TILT = 'head_tilt_joint';
    JOINT_WRIST_FLEX = 'wrist_flex_joint';
    JOINT_WRIST_ROLL = 'wrist_roll_joint';

    
    def __init__(self, outcomes=None, input_keys=None, output_keys=None):
        if outcomes == None:
            outcomes = [SUCCESS];
        if input_keys == None:
            input_keys = [];
        if output_keys == None:
            output_keys = [];
        smach.State.__init__(
            self,
            outcomes=outcomes,
            input_keys=input_keys,
            output_keys=output_keys);
        
    def speak(self, phrase_speaking, wait_to_terminate=True):
        if not hasattr(self, "speak_action_client"):
            self.speak_action_client = actionlib.SimpleActionClient('/talk_request_action', TalkRequestAction)
        action_goal = TalkRequestGoal()
        action_goal.data.language = Voice.kEnglish  # enum for value: 1
        action_goal.data.sentence = phrase_speaking
        rospy.loginfo("HSR speaking phrase: '{}'".format(phrase_speaking))

        self.speak_action_client.wait_for_server()
        self.speak_action_client.send_goal(action_goal)
        if wait_to_terminate:
            self.speak_action_client.wait_for_result();
        
    def getRobotInterface(self):
        if not hasattr(self, "robot_local"):
            self.robot_local = hsrb_interface.Robot();
            self.whole_body = self.robot_local.try_get('whole_body');
            self.omni_base = self.robot_local.try_get('omni_base');
            self.gripper = self.robot_local.try_get('gripper');
            
    def getGripperDistance(self):
        self.getRobotInterface();
        return self.gripper.get_distance();
    
    def moveToNeutral(self):
        self.getRobotInterface();
        self.whole_body.move_to_neutral();
    def moveToGo(self):
        self.getRobotInterface();
        self.whole_body.move_to_go();