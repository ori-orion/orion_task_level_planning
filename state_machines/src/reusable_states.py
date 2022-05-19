#!/usr/bin/env python3
""" File containing reusable state machine states.

This file contains state machine tasks which are reusable across
multiple tasks.

Author: Charlie Street
Owner: Charlie Street

"""

import rospy
import smach
import actionlib
import time
import numpy as np
from smach import Concurrence
from tf.transformations import euler_from_quaternion
import tf

from orion_actions.msg import GiveObjectToOperatorGoal, \
    OpenDoorGoal, GiveObjectToOperatorGoal, GiveObjectToOperatorAction, \
        ReceiveObjectFromOperatorGoal, ReceiveObjectFromOperatorAction, PutObjectOnFloorGoal, \
            PutObjectOnSurfaceGoal, CheckForBarDrinksGoal, SpeakAndListenGoal, \
                HotwordListenGoal, PickUpObjectGoal, PickUpObjectAction, \
                    FollowGoal, OpenDrawerGoal, \
                        PlaceObjectRelativeGoal, PourIntoGoal, \
                            PointToObjectGoal, OpenFurnitureDoorGoal, \
                                PointingGoal, CloseDrawerGoal, \
                                    SpeakAndListenAction, SpeakAndListenGoal
from orion_door_pass.msg import DoorCheckGoal
from orion_actions.msg import DetectionArray, FaceDetectionArray, PoseDetectionArray
from orion_actions.msg import SOMObservation, Relation
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from actionlib_msgs.msg import GoalStatus
from tmc_msgs.msg import TalkRequestAction, TalkRequestGoal, Voice
from strands_navigation_msgs.srv import GetTaggedNodesResponse
from strands_navigation_msgs.msg import TopologicalMap
from strands_executive_msgs.msg import ExecutePolicyGoal, MdpDomainSpec

FAILURE_THRESHOLD = 3       # TODO - remove

# People
NAMES = ['Gemma', 'Acacia', 'Ollie', 'Nick', 'Hollie', 
          'Charlie', 'Matt', 'Daniele', 'Chris', 'Paul', 'Lars', 'John',
          'Michael', 'Matthew', 'Clarissa', 'Ricardo', 'Mia', 'Shu', 'Owen',
          'Jianeng', 'Kim', 'Liam', 'Kelvin', 'Benoit', 'Mark']

PRONOUNS = ['She/Her', 'He/Him', 'They/Them', 'Ze/Zir', 'Name']
GENDERS = ['Female', 'Male', 'Gender Fluid', 'Poly-Gender', 'Pangender', 'Agender']

# Commands
READY = ['ready']#['I am ready', 'ready', "let's go", "I'm ready"]

# Descriptors
COLOURS = ["Red", "Orange", "Yellow", "Green", "Blue", "Purple",
           "Black", "White", "Grey", "Brown", "Beige"]
RELATIONS = ['left', 'right', 'above', 'below', 'front', 'behind', 'near']
AR_MARKERS = {'bottle': 151}

# Objects & Things
#  Be careful of space/underscore representations of 'cleaning stuff'
OBJECT_CATEGORIES = ['cleaning stuff', 'containers', 'cutlery', 'drinks', 'food', 'fruits', 'snacks', 'tableware']  
FRUITS = ['apple', 'banana', 'orange', 'mango', 'strawberry', 'kiwi', 'plum',
          'nectarine'] # TODO: Fill in with the YCB benchmark
DRINKS = ['Coke', 'Beer', 'Water', 'Orange Juice', 'Champagne', 'Absinthe']
OBJECTS = ['potted plant', 'bottle', 'cup', 'cereal', 'bowl', 'cloth'] # TODO: YCB benchmark
OBJECTS += FRUITS + DRINKS
# TODO - other object categories from https://sites.google.com/diag.uniroma1.it/robocupathome-objects/home
# Parent               Number of children
# ===============================================
# Cleaning_stuff                37
# Containers                    17
# Cutlery                       15
# Drinks                        17
# Food                          22
# Fruits                        23
# Snacks                        26
# Tableware                     23


def pose_to_xy_theta(pose):
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

    return (x,y,theta)


def get_location_of_object(action_dict, obj_1, rel, obj_2):
    """ Function returns the location of an object in terms of a pose. 
    
    This function uses the semantic mapping to get the closest location of an
    object.

    Args:
        action_dict: our action dictionary to be able to use the services.
        obj_1: The first SOMObservation message
        rel: The Relation message between the objects
        obj_2: The second SOMObservation message
    
    Returns: 
        pose: The pose of the object

    """
    matches = action_dict['SOMQuery'](obj_1, rel, obj_2, Pose()).matches
    
    if len(matches) == 0:
        raise Exception("No matches found in Semantic Map")
    
    pose = matches[0].obj1.pose_estimate.most_likely_pose

    return pose


def distance_between_poses(pose_1, pose_2):
    """Given two poses, this finds the Euclidean distance between them. """

    pos_1 = pose_1.position
    pos_2 = pose_2.position

    delta_x_sq = np.power(pos_2.x - pos_1.x, 2)
    delta_y_sq = np.power(pos_2.y - pos_1.y, 2)
    delta_z_sq = np.power(pos_2.z - pos_1.z, 2)

    return np.sqrt(delta_x_sq + delta_y_sq + delta_z_sq)


def get_node_with_label(action_dict, label):
    """ Returns the name of the waypoint with a given label. """

    response = action_dict['GetTaggedNodes'](label)

    if response.nodes == []:
        return None
    else:
        return response.nodes[0]


def get_pose_of_node(waypoint):
    """ Gets the pose of a node in the topological map. """

    # Get the topological map
    top_map = rospy.wait_for_message('/topological_map', TopologicalMap)

    nodes = top_map.nodes

    for node in nodes:
        if node.name == waypoint:
            return node.pose

    return None


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


class SpeakState(smach.State):
    """ Smach state for the robot to speak a phrase.

    This class has the robot say something and return success.

    input_keys:
        phrase: What we want the robot to say
    """
    def __init__(self):
        smach.State.__init__(self, 
                                outcomes=['success'],
                                input_keys=['phrase'])
    
    def execute(self, userdata):
        action_goal = TalkRequestGoal()
        action_goal.data.language = Voice.kEnglish  # enum for value: 1
        action_goal.data.sentence = userdata.phrase

        rospy.loginfo("HSR speaking phrase: '{}'".format(userdata.phrase))
        speak_action_client = actionlib.SimpleActionClient('/talk_request_action', 
                                        TalkRequestAction)

        speak_action_client.wait_for_server()
        speak_action_client.send_goal(action_goal)
        speak_action_client.wait_for_result()

        # rospy.loginfo("Speaking complete")

        # Can only succeed
        return 'success'


class CreatePhraseAnnounceRetrievedItemToNamedOperatorState(smach.State):
    """ Smach state to create the phrase to announce the retreival of an item to a named operator

    This class always returns success.

    input_keys:
        operator_name: the name of the operator
        object_name: the name of the retrieved object
    output_keys:
        phrase: the returned phrase
    """
    def __init__(self):
        smach.State.__init__(self, 
                                outcomes=['success'],
                                input_keys=['operator_name', 'object_name'],
                                output_keys=['phrase'])
    
    def execute(self, userdata):
        userdata.phrase = "Hi, " + userdata.operator_name + ", I've brought you the " + userdata.object_name

        # Can only succeed
        return 'success'


class CreatePhraseAskForHelpPickupObjectState(smach.State):
    """ Smach state to create the phrase to ask for help to pick up an object

    This class always returns success.

    input_keys:
        object_name: the name of the object to be asked to be picked up
    output_keys:
        phrase: the returned phrase
    """
    def __init__(self):
        smach.State.__init__(self, 
                                outcomes=['success'],
                                input_keys=['object_name'],
                                output_keys=['phrase'])
    
    def execute(self, userdata):
        userdata.phrase = ("Can someone please help me pick up the " + userdata.object_name + 
                                " and say ready when they are ready?")

        # Can only succeed
        return 'success'


class HandoverObjectToOperatorState(smach.State):
    """ Smach state for handing a grasped object to an operator.

    This state hands over an object to the operator.
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['success','failure'])
    
    def execute(self, userdata):
        handover_goal = GiveObjectToOperatorGoal()

        give_object_to_operator_action_client = actionlib.SimpleActionClient('give_object_to_operator', 
                                         GiveObjectToOperatorAction)
        give_object_to_operator_action_client.wait_for_server()
        give_object_to_operator_action_client.send_goal(handover_goal)
        give_object_to_operator_action_client.wait_for_result()

        success = give_object_to_operator_action_client.get_result().result
        if success:
            return 'success'
        else:
            return 'failure'


class ReceiveObjectFromOperatorState(smach.State):
    """ Smach state for receiving an object from an operator.

    This state grasps an object currently held by an operator.
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['success','failure'])
    
    def execute(self, userdata):
        receive_goal = ReceiveObjectFromOperatorGoal()

        receive_object_from_operator_action_client = actionlib.SimpleActionClient('receive_object_from_operator',
                                         ReceiveObjectFromOperatorAction)
        receive_object_from_operator_action_client.wait_for_server()
        receive_object_from_operator_action_client.send_goal(receive_goal)
        receive_object_from_operator_action_client.wait_for_result()

        result = receive_object_from_operator_action_client.get_result()
        success = result.result
        if success:
            return 'success'
        else:
            return 'failure'


class GetRobotLocationState(smach.State):
    """ Smach state for getting the robot's current location.

    This state will get the robot's current location and return it in the userdata dict.
    """

    def __init__(self):
        smach.State.__init__(self, 
                                outcomes = ['stored'],
                                output_keys=['robot_location'])

    def execute(self, userdata):
        # Wait for one message on topic and then set as the location
        pose = rospy.wait_for_message('/global_pose', PoseStamped)
        userdata.robot_location = pose.pose
        rospy.loginfo(pose)
        return 'stored'


class SpeakAndListenState(smach.State):
    """ Smach state for speaking and then listening for a response.

    This state will calll the speak and listen action server, 
    to get the robot to say something, wait for a response, 
    parse the response, and return it as an output key.

    input_keys:
        question: the question to ask
        candidates: candidate sentences
        params: optional parameters for candidate sentences
        timeout: the timeout for listening
        number_of_failures: an external counter keeping track of the cumulative failure count (incremented in this state upon failure & reset upon success and repreat failure)
        failure_threshold: the number of cumulative failures required to return the repeat_failure outcome
    output_keys:
        operator_response: the recognised response text
        number_of_failures: the updated failure counter upon state exit
    """

    def __init__(self):
        smach.State.__init__(self, 
                                outcomes=['success','failure','repeat_failure'],
                                input_keys=['question', 'candidates','params','timeout','number_of_failures','failure_threshold'],
                                output_keys=['operator_response', 'number_of_failures'])
    
    def execute(self, userdata):
        speak_listen_goal = SpeakAndListenGoal()
        speak_listen_goal.question = userdata.question
        speak_listen_goal.candidates = userdata.candidates
        speak_listen_goal.params = userdata.params
        speak_listen_goal.timeout = userdata.timeout

        speak_listen_action_client = actionlib.SimpleActionClient('speak_and_listen', SpeakAndListenAction)
        speak_listen_action_client.wait_for_server()
        # rospy.loginfo("Pre sending goal");
        speak_listen_action_client.send_goal(speak_listen_goal)
        # rospy.loginfo("Pre wait for result");
        speak_listen_action_client.wait_for_result()
        # rospy.loginfo("Post wait for result");

        result = speak_listen_action_client.get_result()
        if result.succeeded:
            # todo - replace ugly global variable
            # self.global_store['last_response'] = result.answer
            userdata.operator_response = result.answer
            userdata.number_of_failures = 0
            return 'success'
        else:
            userdata.number_of_failures+= 1
            if userdata.number_of_failures >= userdata.failure_threshold:
                # reset number of failures because we've already triggered the repeat failure
                userdata.number_of_failures = 0
                return 'repeat_failure'
            return 'failure'


class SimpleNavigateState(smach.State):
    """ State for navigating directly to a location on the map.

    This state is given a pose and navigates there.

    input_keys:
        pose: pose for the robot to navigate to
        number_of_failures: an external counter keeping track of the cumulative failure count (incremented in this state upon failure & reset upon success and repreat failure)
        failure_threshold: the number of cumulative failures required to return the repeat_failure outcome
    output_keys:
        number_of_failures: the updated failure counter upon state exit
    """

    def __init__(self):
        smach.State.__init__(self, 
                                outcomes=['success', 'failure', 'repeat_failure'],
                                input_keys=['pose', 'number_of_failures', 'failure_threshold'],
                                output_keys=['number_of_failures'])
    
    def execute(self, userdata):
        # Navigating without top nav
        rospy.loginfo('Navigating without top nav')
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = userdata.pose
        rospy.loginfo(goal.target_pose.pose)

        navigate_action_client = actionlib.SimpleActionClient('move_base/move',  MoveBaseAction)
        navigate_action_client.wait_for_server()
        navigate_action_client.send_goal(goal)
        navigate_action_client.wait_for_result()
        status = navigate_action_client.get_state()
        navigate_action_client.cancel_all_goals()
        rospy.loginfo('status = ' + str(status))
        if status == GoalStatus.SUCCEEDED:
            userdata.number_of_failures = 0
            return 'success'
        else:
            userdata.number_of_failures += 1
            if userdata.number_of_failures >= userdata.failure_threshold:
                 # reset number of failures because we've already triggered the repeat failure outcome
                userdata.number_of_failures = 0
                return 'repeat_failure'
            return 'failure'


class PickUpObjectState(smach.State):
    """ State for picking up an object

    This state picks up an object specified by name.

    input_keys:
        object_name: the object to be picked up
        number_of_failures: an external counter keeping track of the cumulative failure count (incremented in this state upon failure & reset upon success and repreat failure)
        failure_threshold: the number of cumulative failures required to return the repeat_failure outcome
        ar_marker_ids: a dictionary of AR marker IDs, keyed by object name
    output_keys:
        number_of_failures: the updated failure counter upon state exit
    """

    def __init__(self):
        smach.State.__init__(self, 
                                outcomes=['success', 'failure', 'repeat_failure'],
                                input_keys=['object_name', 'number_of_failures', 'failure_threshold', 'ar_marker_ids'],
                                output_keys=['number_of_failures'])
    
    def execute(self, userdata):
        pick_up_goal = PickUpObjectGoal()
        pick_up_goal.goal_tf = userdata.object_name.replace(" ", "_")          # need to replace spaces with underscores for ROS TF tree look-up

        # check if we can see the tf in the tf tree - if not, check if we need to fall back on an ar_marker, otherwise trigger the failure outcome
        tf_listener = tf.TransformListener()
        rospy.sleep(2)  # wait 2 seconds for the tf listener to gather tf data
        frames = tf_listener.getFrameStrings()

        tf_name_from_tree = "NOT_FOUND" # the matched tf name from the tree
        
        found_by_name = False
        for frame in frames:
            # perform a sub-string search in the frame string so we find the 
            # frame we are looking for. Eg we find "potted_plant" in "potted_plant_1"
            if pick_up_goal.goal_tf in frame:
                found_by_name = True
                tf_name_from_tree = frame
                break

        if not found_by_name:
            if userdata.object_name in userdata.ar_marker_ids:
                ar_tf_string = 'ar_marker/' + str(userdata.ar_marker_ids[userdata.object_name])
                rospy.loginfo("Target TF '{}' not found in TF tree - using AR marker TF instead '{}'".format(pick_up_goal.goal_tf, ar_tf_string))
                
                found_by_ar_marker = False
                for frame in frames:
                    # perform a sub-string search in the frame string so we find the 
                    # frame we are looking for. Eg we find "potted_plant" in "potted_plant_1"
                    if ar_tf_string in frame:
                        found_by_ar_marker = True
                        break

                if not found_by_ar_marker:
                    rospy.loginfo("AR marker TF was not found in TF tree '{}'. PickUpObjectState will now return failure state".format(ar_tf_string))
                    userdata.number_of_failures += 1
                    if userdata.number_of_failures >= userdata.failure_threshold:
                        userdata.number_of_failures = 0
                        return 'repeat_failure'
                    else:
                        return 'failure'
            else:
                rospy.loginfo("Target TF '{}' not found in TF tree and no AR marker is known for object '{}'".format(pick_up_goal.goal_tf, userdata.object_name))
                rospy.loginfo("TF tree frames: '{}'".format(frames))
                rospy.loginfo("PickUpObjectState will now return failure state")
                userdata.number_of_failures += 1
                if userdata.number_of_failures >= userdata.failure_threshold:
                    userdata.number_of_failures = 0
                    return 'repeat_failure'
                else:
                    return 'failure'

        # continue
        pick_up_object_action_client = actionlib.SimpleActionClient('pick_up_object', PickUpObjectAction)
        pick_up_object_action_client.wait_for_server()

        pick_up_object_action_client.send_goal(pick_up_goal)
        pick_up_object_action_client.wait_for_result()

        result = pick_up_object_action_client.get_result().result

        if result:
            userdata.number_of_failures = 0
            return 'success'
        else:
            userdata.number_of_failures += 1
            if userdata.number_of_failures >= userdata.failure_threshold:
                userdata.number_of_failures = 0
                return 'repeat_failure'
            else:
                return 'failure'


###################### NEEDS REVIEWING #################################

# TODO - remove this once refactoring is complete
class ActionServiceState(smach.State):
    """ A subclass of Smach States which gives access to actions/services.

    This subclass of the default smach state takes in a dictionary of action
    service names and normal service names to actions/services. This allows
    them to be used straight away without having to wait for them to start up.
    This also has a global store which can contain useful information mid
    task.

    Attributes:
        action_dict: A dictionary from names of actions/services to a client
                     /service proxy for them.
        global_store: A dictionary of strings to objects.
    """
    def __init__(self, action_dict, global_store, outcomes):
        """ Overwriting the base class constructor to take new dictionary.

        Args:
            action_dict: The dictionary of names to actions.
            global_store: The dictionary from names to data
            outcomes: The outcomes, as in the base class.
        """
        self.action_dict = action_dict
        self.global_store = global_store
        super(ActionServiceState, self).__init__(outcomes=outcomes)
        # Need to set afterwards
        self._outcomes = outcomes

class SetNavGoalState(ActionServiceState):
    """ State for setting nav goal to something arbitrary defined by lambda. 
        DEPRECATED SINCE REFACTORING - NO LONGER NEEDED DUE TO REMOVAL OF GLOBAL VARIABLES
        TODO - REMOVE
    """

    def __init__(self, action_dict, global_store, function):
        """ function must have 0 parameters and must return the new nav goal """
        outcomes = ['SUCCESS']
        self.function = function
        super(SetNavGoalState, self).__init__(action_dict=action_dict,
                                              global_store=global_store,
                                              outcomes=outcomes)
    
    def execute(self, userdata):
        self.global_store['nav_location'] = self.function()
        return self._outcomes[0]


class NavToLocationState(ActionServiceState):
    """ State for setting nav goal and then navigating there. """
    
    def __init__(self, action_dict, global_store, function):
        """ Function must have 0 parameters and return new nav goal. """
        outcomes = ['SUCCESS', 'FAILURE', 'REPEAT_FAILURE']
        self.function = function
        super(SetNavGoalAndNavState, self).__init__(action_dict=action_dict,
                                                    global_store=global_store,
                                                    outcomes=outcomes)

        if 'nav_failure' not in self.global_store:
            self.global_store['nav_failure'] = 0

    def execute(self, userdata):
        dest_pose = self.function()
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = dest_pose
        self.action_dict['Navigate'].send_goal(goal)
        self.action_dict['Navigate'].wait_for_result()
        status = self.action_dict['Navigate'].get_state()
        self.action_dict['Navigate'].cancel_all_goals()
        rospy.loginfo('status = ' + str(status))
        if status == GoalStatus.SUCCEEDED:
            self.global_store['nav_failure'] = 0
            return self._outcomes[0]
        else:
            self.global_store['nav_failure'] += 1
            if self.global_store['nav_failure'] >= FAILURE_THRESHOLD:   # TODO - replace global variable with input userdata dictionary element
                return self._outcomes[2]
            return self._outcomes[1]


class SetPickupState(ActionServiceState):
    """ State for setting pick up to something arbitrary passed in. 
        
        DEPRECATED SINCE REFACTORING - NO LONGER NEEDED DUE TO REMOVAL OF GLOBAL VARIABLES
        TODO - REMOVE
    """

    def __init__(self, action_dict, global_store, obj):
        outcomes = ['SUCCESS']
        self.obj = obj
        super(SetPickupState, self).__init__(action_dict=action_dict,
                                             global_store=global_store,
                                             outcomes=outcomes)
    
    def execute(self, userdata):
        self.global_store['pick_up'] = self.obj
        return self._outcomes[0]


class SetPickupFuncState(ActionServiceState):
    """ State for setting pick up to something arbitrary defined by lambda. 

        DEPRECATED SINCE REFACTORING - NO LONGER NEEDED DUE TO REMOVAL OF GLOBAL VARIABLES
        TODO - REMOVE
    """

    def __init__(self, action_dict, global_store, func):
        outcomes = ['SUCCESS']
        self.func = func
        super(SetPickupFuncState, self).__init__(action_dict=action_dict,
                                             global_store=global_store,
                                             outcomes=outcomes)
    
    def execute(self, userdata):
        self.global_store['pick_up'] = self.func()
        return self._outcomes[0]


class OpenDrawerState(ActionServiceState):
    """ State for opening a drawer. """

    def __init__(self, action_dict, global_store):
        outcomes = ['SUCCESS', 'FAILURE']
        super(OpenDrawerState, self).__init__(action_dict=action_dict,
                                              global_store=global_store,
                                              outcomes=outcomes)
    
    def execute(self, userdata):
        drawer_goal = OpenDrawerGoal()
        drawer_goal.goal_tf = self.global_store['drawer_handle']
        self.action_dict['OpenDrawer'].send_goal(drawer_goal)
        self.action_dict['OpenDrawer'].wait_for_result()

        result = self.action_dict['OpenDrawer'].get_result().result

        if result:
            return self._outcomes[0]
        else:
            return self._outcomes[1]


class CloseDrawerState(ActionServiceState):
    """ State for closing a drawer. """

    def __init__(self, action_dict, global_store):
        outcomes = ['SUCCESS', 'FAILURE']
        super(CloseDrawerState, self).__init__(action_dict=action_dict,
                                              global_store=global_store,
                                              outcomes=outcomes)
    
    def execute(self, userdata):
        drawer_goal = CloseDrawerGoal()
        drawer_goal.goal_tf = self.global_store['drawer_handle']
        self.action_dict['CloseDrawer'].send_goal(drawer_goal)
        self.action_dict['CloseDrawer'].wait_for_result()

        result = self.action_dict['CloseDrawer'].get_result().result

        if result:
            return self._outcomes[0]
        else:
            return self._outcomes[1]


class PlaceObjectRelativeState(ActionServiceState):
    """ State for placing objects relative to something else. """

    def __init__(self, action_dict, global_store):
        outcomes = ['SUCCESS', 'FAILURE']
        super(PlaceObjectRelativeState, self).__init__(action_dict=action_dict,
                                                       global_store=
                                                       global_store,
                                                       outcomes=outcomes)
    
    def execute(self, userdata):
        """Relative position in global_store['rel_pos'] as a 4-tuple"""
        place_goal = PlaceObjectRelativeGoal()
        place_goal.goal_tf = self.global_store['rel_pos'][0]
        place_goal.x = self.global_store['rel_pos'][1]
        place_goal.y = self.global_store['rel_pos'][2]
        place_goal.z = self.global_store['rel_pos'][3]

        self.action_dict['PlaceObjectRelative'].send_goal(place_goal)
        self.action_dict['PlaceObjectRelative'].wait_for_result()

        result = self.action_dict['PlaceObjectRelative'].get_result().result

        if result:
            return self._outcomes[0]
        else:
            return self._outcomes[1] 


class PourIntoState(ActionServiceState):
    """ State for pouring something into a container. """

    def __init__(self, action_dict, global_store):
        outcomes = ['SUCCESS', 'FAILURE']
        super(PourIntoState, self).__init__(action_dict=action_dict,
                                            global_store=global_store,
                                            outcomes=outcomes)
    
    def execute(self, userdata):
        pour_goal = PourIntoGoal()
        pour_goal.goal_tf_frame = self.global_store['pour_into']   

        self.action_dict['PourInto'].send_goal(pour_goal)
        self.action_dict['PourInto'].wait_for_result()

        result = self.action_dict['PourInto'].get_result().result

        if result:
            return self._outcomes[0]
        else:
            return self._outcomes[1]  


class PointToObjectState(ActionServiceState):
    """ State for pointing at an object. """

    def __init__(self, action_dict, global_store):
        outcomes = ['SUCCESS', 'FAILURE']
        super(PointToObjectState, self).__init__(action_dict=action_dict,
                                                 global_store=global_store,
                                                 outcomes=outcomes)
    
    def execute(self, userdata):
        point_goal = PointToObjectGoal()
        point_goal.object_tf_frame = self.global_store['point_at']

        self.action_dict['PointToObject'].send_goal(point_goal)
        self.action_dict['PointToObject'].wait_for_result()

        result = self.action_dict['PointToObject'].get_result().result

        if result:
            return self._outcomes[0]
        else:
            return self._outcomes[1]

class CheckDoorIsOpenState(ActionServiceState):
    """ Smach state for robot to check if door is open. This is a common
        start signal for tasks.
    """
    def __init__(self, action_dict, global_store):
        outcomes = ['OPEN', 'CLOSED']
        super(CheckDoorIsOpenState, self).__init__(action_dict=action_dict,
                                                   global_store=global_store,
                                                   outcomes=outcomes)
    
    def execute(self, userdata):
        is_door_open_goal = DoorCheckGoal()
        is_door_open_goal.n_closed_door = 20 # Same as Bruno's code
        self.action_dict['IsDoorOpen'].send_goal(is_door_open_goal)
        self.action_dict['IsDoorOpen'].wait_for_result()

        # Boolean value returned
        is_door_open = self.action_dict['IsDoorOpen'].get_result().open
        if is_door_open:
            return self._outcomes[0]
        else:
            return self._outcomes[1]


class CheckAndOpenDoorState(ActionServiceState):
    """ Smach state for robot to check if door is open and open it if it isn't.

    This state checks if a door in front of the robot is open.
    """
    def __init__(self, action_dict, global_store):
        outcomes = ['SUCCESS', 'FAILURE']
        super(CheckAndOpenDoorState, self).__init__(action_dict=action_dict,
                                                    global_store=global_store,
                                                    outcomes=outcomes)
    
    def execute(self, userdata):
        is_door_open_goal = DoorCheckGoal()
        is_door_open_goal.n_closed_door = 20
        self.action_dict['IsDoorOpen'].send_goal(is_door_open_goal)
        self.action_dict['IsDoorOpen'].wait_for_result()

        # Boolean value returned
        is_door_open = self.action_dict['IsDoorOpen'].get_result().open
        if not is_door_open:
            door_goal = OpenDoorGoal()
            self.action_dict['OpenDoor'].send_goal(door_goal)
            self.action_dict['OpenDoor'].wait_for_result()

            door_success = self.action_dict['OpenDoor'].get_result().result
            if door_success:
                return self._outcomes[0]
            else:
                return self._outcomes[1]

        else:
            return self._outcomes[0]


class OpenFurnitureDoorState(ActionServiceState):
    """ Smach state to open furniture door. """
    def __init__(self, action_dict, global_store):
        outcomes = ['SUCCESS', 'FAILURE']
        super(OpenFurnitureDoorState, self).__init__(action_dict=action_dict,
                                                     global_store=global_store,
                                                     outcomes=outcomes)
    
    def execute(self, userdata):
        goal = OpenFurnitureDoorGoal()
        goal.goal_tf = self.global_store['furniture_door']
        self.action_dict['OpenFurnitureDoor'].send_goal(goal)
        self.action_dict['OpenFurnitureDoor'].wait_for_result()

        if self.action_dict['OpenFurnitureDoor'].get_result().result:
            return self._outcomes[0]
        else:
            return self._outcomes[1]

class PutObjectOnFloorState(ActionServiceState):
    """ Smach state for putting object on floor.

    This state puts an object held by the robot on the floor.
    """
    def __init__(self, action_dict, global_store):
        outcomes = ['SUCCESS', 'FAILURE']
        super(PutObjectOnFloorState, self).__init__(action_dict=action_dict,
                                                    global_store=global_store,
                                                    outcomes=outcomes)
    
    def execute(self, userdata):
        put_on_floor_goal = PutObjectOnFloorGoal()
        self.action_dict['PutObjectOnFloor'].send_goal(put_on_floor_goal)
        self.action_dict['PutObjectOnFloor'].wait_for_result()

        success = self.action_dict['PutObjectOnFloor'].get_result().result
        if success:
            return self._outcomes[0]
        else:
            return self._outcomes[1]


class PutObjectOnSurfaceState(ActionServiceState):
    """ Smach state for putting object on a surface in front of the robot.

    This state put an object held by the robot on a surface.
    """
    def __init__(self, action_dict, global_store):
        outcomes = ['SUCCESS', 'FAILURE']
        super(PutObjectOnSurfaceState, self).__init__(action_dict=action_dict,
                                                      global_store=global_store,
                                                      outcomes=outcomes)
    
    def execute(self, userdata):
        put_on_surface_goal = PutObjectOnSurfaceGoal()
        self.action_dict['PutObjectOnSurface'].send_goal(put_on_surface_goal)
        self.action_dict['PutObjectOnSurface'].wait_for_result()

        success = self.action_dict['PutObjectOnSurface'].get_result().result
        if success:
            return self._outcomes[0]
        else:
            return self._outcomes[1]


class CheckForBarDrinksState(ActionServiceState):
    """ Smach state for checking which drinks are currently available.

    This state checks which bar drinks are currently available, given
    we are already at the bar.
    """
    def __init__(self, action_dict, global_store):
        outcomes = ['IDENTIFIED']
        super(CheckForBarDrinksState, self).__init__(action_dict=action_dict,
                                                     global_store=global_store,
                                                     outcomes=outcomes)
    
    def execute(self, userdata):
        check_drinks_goal = CheckForBarDrinksGoal()
        self.action_dict['CheckForBarDrinks'].send_goal(check_drinks_goal)
        self.action_dict['CheckForBarDrinks'].wait_for_result()

        drinks = self.action_dict['CheckForBarDrinks'].get_result().drinks
        self.global_store['drinks'] = drinks
        return self._outcomes[0]


class SpeakAndHotwordState(ActionServiceState):
    """ Smach state for speaking and then listening for a hotword. """
    def __init__(self, action_dict, global_store, question, hotwords, timeout):
        """ Constructor initialises fields. 

        Args:
            action_dict: A dictionary of action clients
            global_store: All globally useful data
            question: The question to speak
            hotwords: A list of hotwords to detect
            timeout: The timeout for the hotword
        """
        outcomes = ['SUCCESS', 'FAILURE', 'REPEAT_FAILURE']
        self.question = question
        self.hotwords = hotwords
        self.timeout = timeout
        super(SpeakAndHotwordState, self).__init__(action_dict=action_dict,
                                                   global_store=global_store,
                                                   outcomes=outcomes)

        if 'speak_hotword_failure' not in self.global_store:
            self.global_store['speak_hotword_failure'] = 0

    def execute(self, userdata):

        try:
            speak_goal = TalkRequestGoal()
            speak_goal.data.language = Voice.kEnglish
            speak_goal.data.sentence = self.question
            self.action_dict['Speak'].send_goal(speak_goal)
            self.action_dict['Speak'].wait_for_result()

            hotword_goal = HotwordListenGoal()
            hotword_goal.hotwords = self.hotwords
            hotword_goal.timeout = self.timeout

            self.action_dict['HotwordListen'].send_goal(hotword_goal)
            self.action_dict['HotwordListen'].wait_for_result()

            success = self.action_dict['HotwordListen'].get_result().succeeded

            if success:
                self.global_store['speak_hotword_failure'] = 0
                return self._outcomes[0]
            else:
                self.global_store['speak_hotword_failure'] += 1
                if self.global_store['speak_hotword_failure'] >= FAILURE_THRESHOLD:     # TODO - replace global variable with input userdata dictionary element
                    return self._outcomes[2]
                else:
                    return self._outcomes[1]
        except:
            rospy.loginfo('SOMETHING WENT WRONG')
            return self._outcomes[2]

class HotwordListenState(ActionServiceState):
    """Smach state for listening for a hotword.

    This state listens for the occurrence of a hotword and returns the
    captured speech.
    """

    def __init__(self, action_dict, global_store, hotwords, timeout):
        outcomes = ['SUCCESS', 'FAILURE']
        self.timeout = timeout
        self.hotwords = hotwords
        super(HotwordListenState, self).__init__(action_dict=action_dict,
                                                 global_store=global_store,
                                                 outcomes=outcomes)
    
    def execute(self, userdata):
        hotword_goal = HotwordListenGoal()
        hotword_goal.hotwords = self.hotwords
        hotword_goal.timeout = self.timeout
        self.action_dict['HotwordListen'].send_goal(hotword_goal)

        goal_finished = False
        while not self.preempt_requested() and not goal_finished:
            timeout = rospy.Duration(secs=2)
            goal_finished = \
                self.action_dict['HotwordListen'].wait_for_result(timeout=
                                                                  timeout)
        
        if goal_finished:
            result = self.action_dict['HotwordListen'].get_result()

            if result.succeeded:
                return self._outcomes[0]
            else:
                return self._outcomes[1]
        else:
            self.action_dict['HotwordListen'].cancel_all_goals()
            return self._outcomes[1]


class PickUpPointedObject(ActionServiceState):
    """Smach state for detecting and picking up an object being pointed at.

    This state detects an object being pointed at by an operator and picks it
    up.
    """

    def __init__(self, action_dict, global_store):
        outcomes = ['SUCCESS', 'FAILURE']
        super(PickUpPointedObject, self).__init__(action_dict=action_dict,
                                                  global_store=global_store,
                                                  outcomes=outcomes)
    
    def execute(self, userdata):
        self.action_dict['GetPointedObject'].send_goal(PointingGoal())
        self.action_dict['GetPointedObject'].wait_for_result()
        
        result_point = self.action_dict['GetPointedObject'].get_result()
        if not result_point.is_present:
            return self._outcomes[1]
        
        objects = result_point.pointing_array[0].pointings.detections

        # specified to luggage!
        detected_obj = None
        options = OBJECTS    # TODO - replace global variable with input userdata dictionary element 
        for obj in objects:
            for option in options:
                if option in obj.label:
                    detected_obj = obj.label
                    break

        if detected_obj is None:
            return self._outcomes[1]

        pickup_goal = PickUpObjectGoal()
        pickup_goal.goal_tf = detected_obj


        self.action_dict['PickUpObject'].send_goal(pickup_goal)
        self.action_dict['PickUpObject'].wait_for_result()

        result = self.action_dict['PickUpObject'].get_result().result

        if result:
            return self._outcomes[0]
        else:
            return self._outcomes[1]


class OperatorDetectState(ActionServiceState):
    """ This state will detect/observe an operator and memorise them.

    This is a state for memorising an operator and memorising their information.
    Many tasks have an operator to follow etc.

        DEPRECATED SINCE REFACTORING - NO LONGER NEEDED DUE TO REMOVAL OF GLOBAL VARIABLES
                                     - HOWEVER, THE UN-USED SOM INTEGRATION MAY BE USEFUL TO RE-IMPLEMENT ELSEWHERE?
        TODO - REMOVE/MOVE SOM INTEGRATION ELSEWHERE
    """

    def __init__(self, action_dict, global_store):
        outcomes = ['SUCCESS', 'FAILURE']
        super(OperatorDetectState, self).__init__(action_dict=action_dict,
                                                  global_store=global_store,
                                                  outcomes=outcomes)
    
    def execute(self, userdata):
        self.global_store['operator_name'] = self.global_store['last_response']
        return self._outcomes[0]                        
        # RC: Given this early return statement, this state seems to have been down-scoped. 
        #     It now just sets the global variable, which is no longer used following the refactor. TODO - remove
        failed = 0
        operator = SOMObservation()

        if self.global_store['last_person'] is not None:
            operator.obj_id = self.global_store['last_person']

        operator.type = 'person'
        operator.task_role = 'operator'
        

        pose = rospy.wait_for_message('/global_pose', PoseStamped).pose
        operator.robot_pose = pose
        
        operator.room_name = self.action_dict['SOMGetRoom'](pose).room_name
        
        for name in NAMES:  # TODO - replace global variable with input userdata dictionary element
            if name in self.global_store['last_response']:
                operator.name = name
                break
        """
        try:
            person_msg = rospy.wait_for_message('/vision/pose_detections', 
                                                PoseDetectionArray, timeout=5)
            person = person_msg.detections[0]
            operator.shirt_colour = person.color
            rospy.loginfo("PERSON COLOUR: " + str(operator.shirt_colour))
        except:
            failed += 1

        try:
            listen = tf.TransformListener()
            tf_frame = 'person_' + operator.shirt_colour
            t = listen.getLatestCommonTime("map", tf_frame)
            (trans, rot) = listen.lookupTransform("map", tf_frame, t)
            pose = Pose()
            pose.position = trans
            pose.orientation = rot
            operator.pose_observation = pose

        except:
            failed += 1

        try:
            face_msg = rospy.wait_for_message('/vision/face_bbox_detections', 
                                              FaceDetectionArray, 
                                              timeout=5)
            
            face = face_msg.detections[0]
            operator.age = face.age
            operator.gender = face.gender
        except:
            failed += 1

        if failed >= 3:
            return self._outcomes[1]
        """
        result = self.action_dict['SOMObserve'](operator)
        if not result.result:
            return self._outcomes[1]
        else:
            self.global_store['operator'] = result.obj_id
            self.global_store['people_found'].append(result.obj_id)
            return self._outcomes[0]


class MemorisePersonState(ActionServiceState):
    """ State for memorising mates found. """

    def __init__(self, action_dict, global_store):
        outcomes = ['SUCCESS', 'FAILURE']
        super(MemorisePersonState, self).__init__(action_dict=action_dict,
                                                  global_store=global_store,
                                                  outcomes=outcomes)
    
    def execute(self, userdata):
        failed = 0
        person = SOMObservation()

        if self.global_store['last_person'] is not None:
            person.obj_id = self.global_store['last_person']

        person.type = 'person'

        pose = rospy.wait_for_message('/global_pose', PoseStamped).pose
        person.robot_pose = pose
        
        person.room_name = self.action_dict['SOMGetRoom'](pose).room_name

        for name in NAMES:  # TODO - replace global variable with input userdata dictionary element
            if name in self.global_store['last_response']:
                person.name = name
                break
        
        try:
            person_msg = rospy.wait_for_message('/vision/bbox_detections', 
                                                DetectionArray, timeout=5)
            for detection in person_msg.detections:
                if 'person' in detection.label.name:
                    person.shirt_colour = detection.colour
                    break
        except:
            failed += 1
        
        try:
            listen = tf.TransformListener()
            tf_frame = 'person_' + person.shirt_colour
            t = listen.getLatestCommonTime("map", tf_frame)
            (trans, rot) = listen.lookupTransform("map", tf_frame, t)
            pose = Pose()
            pose.position = trans
            pose.orientation = rot
            person.pose_observation = pose

        except:
            failed += 1

        try:
            face_msg = rospy.wait_for_message('/vision/face_bbox_detections', 
                                              FaceDetectionArray, 
                                              timeout=5)
            
            face = face_msg.detections[0]
            person.age = face.age
            person.gender = face.gender
        except:
            failed += 1

        if failed >= 3:
            return self._outcomes[1]

        result = self.action_dict['SOMObserve'](person)
        if not result.result:
            return self._outcomes[1]
        else:
            self.global_store['people_found'].append(result.obj_id)
            return self._outcomes[0]


#--- Code for following while listening for a hotword    
class FollowState(ActionServiceState):
    """ This state follows a person until it is preempted or fails. """

    def __init__(self, action_dict, global_store):
        outcomes = ['PREEMPTED', 'FAILURE', 'REPEAT_FAILURE']
        super(FollowState, self).__init__(action_dict=action_dict,
                                          global_store=global_store,
                                          outcomes=outcomes)

        if 'follow_failure' not in self.global_store:
            self.global_store['follow_failure'] = 0
    
    def execute(self, userdata):
        follow_goal = FollowGoal()

        obs = SOMObservation()
        obs.type = 'person'
        obs.task_role = 'operator'
        matches = self.action_dict['SOMQuery'](obs,Relation(),SOMObservation(), 
                                               Pose()).matches
        if len(matches) == 0:
            colour = 'green'
        else:
            op = matches[0].obj1
            colour = op.shirt_colour

        follow_goal.object_name = 'person_' + colour
        self.action_dict['Follow'].send_goal(follow_goal)
        print("COLOUR: " + colour)
        current_result = True
        while not self.preempt_requested():
            finished = self.action_dict['Follow'].wait_for_result(timeout=rospy.Duration(secs=1))
            if finished:
                current_result = self.action_dict['Follow'].get_result().succeeded
                break
        
        self.action_dict['Follow'].cancel_all_goals()
    
        if current_result == False:
            self.global_store['follow_failure'] += 1
            if self.global_store['follow_failure'] >= FAILURE_THRESHOLD:    # TODO - replace global variable with input userdata dictionary element
                return self._outcomes[2]
            return self._outcomes[1]
        else:
            self.global_store['follow_failure'] = 0
            return self._outcomes[0]


def follow_child_cb(outcome_map, global_store):
    """Executed whenever a child in the concurrent state is terminated."""
    if outcome_map['Hotword'] == 'SUCCESS':
        return True
    if outcome_map['Hotword'] == 'FAILURE':
        if 'follow_failure' in global_store:
            global_store['follow_failure'] += 1
        else:
            global_store['follow_faulure'] = 1
        return True
    if outcome_map['Follow'] == 'FAILURE':
        return True
    if outcome_map['Follow'] == 'REPEAT_FAILURE':
        return True
    
    return False

def follow_out_cb(outcome_map, global_store):
    if outcome_map['Hotword'] == 'SUCCESS':
        return 'SUCCESS'
    elif outcome_map['Follow'] == 'REPEAT_FAILURE':
        return 'REPEAT_FAILURE'
    elif ('follow_failure' in global_store and global_store['follow_failure'] >= FAILURE_THRESHOLD):    # TODO - replace global variable with input userdata dictionary element
        return 'REPEAT_FAILURE'
    else:
        return 'FAILURE'

def make_follow_hotword_state(action_dict, global_store):
    """ Creates a concurrent state which follows someone while listening.

    This concurrent state machine follows someone while waiting for a hot
    word to be spoken.
    """
    # TODO: Fix, this still isn't quite right
    con = Concurrence(outcomes=['SUCCESS', 'FAILURE', 'REPEAT_FAILURE'],
                      default_outcome='FAILURE',
                      child_termination_cb=(lambda om: (follow_child_cb(om, global_store))),
                      outcome_cb=(lambda om: follow_out_cb(om, global_store)))
    
    with con:
        Concurrence.add('Follow', FollowState(action_dict, global_store))
        Concurrence.add('Hotword', HotwordListenState(action_dict, 
                                                      global_store,
                                                      ['cancel'],
                                                      180))
    
    return con
# --- End of follow code

class NavigateState(ActionServiceState):
    """ State for navigating to location on map.

    This state is given a pose and navigates there.
        
        DEPRECATED SINCE REFACTORING - NO LONGER NEEDED DUE TO REMOVAL OF GLOBAL VARIABLES
        TODO - REMOVE
    """

    def __init__(self, action_dict, global_store):
        outcomes = ['SUCCESS', 'FAILURE', 'REPEAT_FAILURE']
        super(NavigateState, self).__init__(action_dict=action_dict,
                                            global_store=global_store,
                                            outcomes=outcomes)
        
        if 'nav_failure' not in self.global_store:
            self.global_store['nav_failure'] = 0
    
    def execute(self, userdata):
        dest_pose = self.global_store['nav_location']

        # Find closest node
        """(closest_node, wp_pose) = get_closest_node(dest_pose)

        current_pose = rospy.wait_for_message('/global_pose', PoseStamped).pose
        dist_to_wp = distance_between_poses(current_pose, wp_pose)
        dist_to_dest = distance_between_poses(current_pose, dest_pose)"""

        #if dist_to_wp < dist_to_dest: # Just go directly
        """rospy.loginfo('Using top nav to navigate to: ' + str(closest_node))
        ltl_task = 'F "' + closest_node + '"'
        policy_goal = ExecutePolicyGoal()
        policy_goal.spec = MdpDomainSpec()
        policy_goal.spec.ltl_task = ltl_task
        self.action_dict['ExecutePolicy'].send_goal(policy_goal)
        self.action_dict['ExecutePolicy'].wait_for_result()
        status = self.action_dict['ExecutePolicy'].get_state()
        self.action_dict['ExecutePolicy'].cancel_all_goals()
        if status != GoalStatus.SUCCEEDED: # If nav failed
            self.global_store['nav_failure'] += 1
            if self.global_store['nav_failure'] >= FAILURE_THRESHOLD:   # TODO - replace global variable with input userdata dictionary element
                return self._outcomes[2]
            return self._outcomes[1]"""

        # Navigating without top nav

        rospy.loginfo('Navigating without top nav')
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = dest_pose
        rospy.loginfo(goal.target_pose.pose)
        self.action_dict['Navigate'].send_goal(goal)
        self.action_dict['Navigate'].wait_for_result()
        status = self.action_dict['Navigate'].get_state()
        self.action_dict['Navigate'].cancel_all_goals()
        rospy.loginfo('status = ' + str(status))
        if status == GoalStatus.SUCCEEDED:
            self.global_store['nav_failure'] = 0
            return self._outcomes[0]
        else:
            self.global_store['nav_failure'] += 1
            if self.global_store['nav_failure'] >= FAILURE_THRESHOLD:   # TODO - replace global variable with input userdata dictionary element
                return self._outcomes[2]
            return self._outcomes[1]
