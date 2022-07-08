#!/usr/bin/env python3
""" File containing reusable state machine states.

This file contains state machine tasks which are reusable across
multiple tasks.

Author: Charlie Street
Owner: Ricardo Cannizzaro

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
                                    SpeakAndListenAction, Hotword
from orion_door_pass.msg import DoorCheckGoal, DoorCheckAction

from orion_actions.msg import DetectionArray, FaceDetectionArray, PoseDetectionArray
from orion_actions.msg import SOMObservation, Relation
from orion_actions.srv import SOMObserve
# new som system
from orion_actions.msg import SOMObject
from orion_actions.srv import SOMAddHumanObs, SOMAddHumanObsRequest, SOMQueryObjects, SOMQueryObjectsRequest, \
	SOMQueryHumans, SOMQueryHumansRequest, SOMQueryHumansResponse
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from actionlib_msgs.msg import GoalStatus
from tmc_msgs.msg import TalkRequestAction, TalkRequestGoal, Voice
from strands_navigation_msgs.srv import GetTaggedNodesResponse
from strands_navigation_msgs.msg import TopologicalMap
from strands_executive_msgs.msg import ExecutePolicyGoal, MdpDomainSpec
from ori_topological_navigation_msgs.msg import TraverseToNodeAction, TraverseToNodeGoal
from orion_face_recognition.msg import ActionServer_CapFaceAction, ActionServer_CapFaceGoal, \
	ActionServer_FindMatchAction, ActionServer_FindMatchGoal, ActionServer_FindAttrsAction, ActionServer_FindAttrsGoal, \
		ActionServer_ClearDatabaseAction, ActionServer_ClearDatabaseGoal

FAILURE_THRESHOLD = 3       # TODO - remove

# People
NAMES = ['Gemma', 'Acacia', 'Ollie', 'Nick', 'Hollie', 
          'Charlie', 'Matt', 'Daniele', 'Chris', 'Paul', 'Lars', 'John',
          'Michael', 'Matthew', 'Clarissa', 'Ricardo', 'Mia', 'Shu', 'Owen',
          'Jianeng', 'Kim', 'Liam', 'Kelvin', 'Benoit', 'Mark']

GENDERS = ['female', 'male', 'gender fluid', 'poly-gender', 'pangender', 'agender', 'non-binary', 'prefer not to say']
PRONOUNS = ['she her', 'he him', 'they them', 'prefer not to say']

# Commands
READY = ['ready']#['I am ready', 'ready', "let's go", "I'm ready"]

# Descriptors
COLOURS = ["Red", "Orange", "Yellow", "Green", "Blue", "Purple",
           "Black", "White", "Grey", "Brown", "Beige"]
RELATIONS = ['left', 'right', 'above', 'below', 'front', 'behind', 'near']
AR_MARKERS = {'bottle': 151}
FACE_ATTRIBUTES_EXCLUSION_LIST = ['Attractive',	'Bags_Under_Eyes',	'Bald',	'Big_Lips',	'Big_Nose', 'Blurry', 'Chubby',	'Double_Chin', 'Heavy_Makeup',	'Mouth_Slightly_Open',	'Narrow_Eyes',	'Pale_Skin',	'Pointy_Nose',	'Receding_Hairline', 'Smiling',  'Young']
#FACE_ATTRIBUTES_LIST = ['5_o_Clock_Shadow',	'Arched_Eyebrows',	'Attractive',	'Bags_Under_Eyes',	'Bald',	'Bangs',	'Big_Lips',	'Big_Nose',	'Black_Hair',	'Blond_Hair',	'Blurry',	'Brown_Hair',	'Bushy_Eyebrows',	'Chubby',	'Double_Chin',	'Eyeglasses',	'Goatee',	'Gray_Hair',	'Heavy_Makeup',	'High_Cheekbones',	'Male',	'Mouth_Slightly_Open',	'Mustache',	'Narrow_Eyes',	'No_Beard',	'Oval_Face',	'Pale_Skin',	'Pointy_Nose',	'Receding_Hairline',	'Rosy_Cheeks',	'Sideburns',	'Smiling',	'Straight_Hair',	'Wavy_Hair',	'Wearing_Earrings',	'Wearing_Hat',	'Wearing_Lipstick',	'Wearing_Necklace',	'Wearing_Necktie',	'Young']

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

def filter_face_attributes_by_exclusion(attributes):
	""" Filter the list of facial attributes by removing items in the exclusion list. """
	filtered_attributes = [attr for attr in attributes if attr not in FACE_ATTRIBUTES_EXCLUSION_LIST]

	return filtered_attributes

def get_most_recent_obj_from_som(class_=None):
    ''' Function to get the most recently observed object from the som (optionally filtered by class_ field)
    
    '''
    # call a query by class_ and sort results by most recent observation - return the newest one
    query = SOMQueryObjectsRequest()
    if(class_ is not None):
        query.query.class_ = class_
    
    rospy.wait_for_service('som/objects/basic_query')
    som_obj_query_service_client = rospy.ServiceProxy('som/objects/basic_query', SOMQueryObjects);

    # call the service
    response = som_obj_query_service_client(query);

    # process the service response
    if len(response.returns) == 0:
        return None

    # sort the object results by last updated field (most recent will appear first)
    returned_objects = response.returns
    returned_objects_sorted = sorted(returned_objects, key=lambda x: x.last_observed_at, reverse=True)
    most_recent_object = returned_objects_sorted[0]
    return most_recent_object

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

def positional_to_cardinal(num: int) -> str:
	# function to convert positional numbers to cardinal numbers. returns a string.
	mapping = {1:"first", 2: "second", 3:"third", 4:"fourth", 5:"fifth", 6:"sixth", 7:"seventh", 8:"eighth", 9:"nineth", 10:"tenth"}

	if num in mapping:
		return mapping[num]
	else:
		raise Exception("Input number is out of range for this function. Supported mapping: {}".format(mapping))

def create_learn_guest_sub_state_machine():
    
    # create the sub state machine
    sub_sm = smach.StateMachine(outcomes=['success', 'failure'],
                                input_keys=['guest_som_human_ids',
                                            'guest_som_obj_ids',
                                            'person_names'],
                                output_keys=['guest_som_human_ids',
                                            'guest_som_obj_ids'])

    # speech defaults
    sub_sm.userdata.speak_and_listen_params_empty = []
    sub_sm.userdata.speak_and_listen_timeout = 5
    sub_sm.userdata.speak_and_listen_failures = 0
    sub_sm.userdata.speak_and_listen_failure_threshold = 3

    # speaking to guests
    sub_sm.userdata.introduction_to_guest_phrase = "Hi, I'm Bam Bam, welcome to the party! I'm going to learn some information about you!"
    sub_sm.userdata.ask_name_phrase = "What is your name?"
    sub_sm.userdata.no_one_there_phrase = "Hmmm. I don't think anyone is there. It's time for me to move on."
    sub_sm.userdata.speech_recognition_failure_phrase = "I'm sorry but I did understand. Let's try that again."

    sub_sm.userdata.ask_gender_phrase = "What is your gender?"
    sub_sm.userdata.ask_gender_candidates = GENDERS

    sub_sm.userdata.ask_pronouns_phrase = "What are your preferred pronouns?"
    sub_sm.userdata.ask_pronouns_candidates = PRONOUNS

    # TODO - add age fields into SOM person observation data (and expand CreateGuestAttributesDict state)
    # sub_sm.userdata.ask_age_phrase = "How old are you?"                     # assume response is given in years
    # sub_sm.userdata.ask_age_candidates = [str(x) for x in range(1,101)]     # TODO - test recognition of numbers

    sub_sm.userdata.start_face_registration_phrase = "I am registering your face so I can tell the host what you look like. Please stand still."
    sub_sm.userdata.finish_face_registration_phrase = "I have finished registering your face. Thank you!"
    sub_sm.userdata.save_to_som_phrase = "I am saving your details to memory."
    sub_sm.userdata.farewell_guest = "Thank you. I need to go now. It was nice to meet you!"

    sub_sm.userdata.guest_name = ""
    sub_sm.userdata.guest_gender = ""
    sub_sm.userdata.guest_pronouns = ""
    sub_sm.userdata.guest_attributes = {}  # need to initialise it here because it needs to be an input into the CreateGuestAttributesDict state
    sub_sm.userdata.guest_face_attributes = {}  # need to initialise it here because it needs to be an input into the CreateGuestAttributesDict state

    # Open the container 
    with sub_sm:
        # introduction to guest
        smach.StateMachine.add('ANNOUNCE_GUEST_INTRO',
                                SpeakState(),
                                transitions={'success':'ASK_GUEST_NAME'},
                                remapping={'phrase':'introduction_to_guest_phrase'})
        
        # ask for guest's name
        smach.StateMachine.add('ASK_GUEST_NAME',
                               SpeakAndListenState(),
                                transitions={'success': 'ASK_GUEST_GENDER',
                                            'failure':'ANNOUNCE_MISSED_GUEST_NAME', 
                                            'repeat_failure':'ANNOUNCE_NO_ONE_THERE'},
                                remapping={'question':'ask_name_phrase',
                                            'operator_response': 'guest_name',
                                            'candidates':'person_names',
                                            'params':'speak_and_listen_params_empty',
                                            'timeout':'speak_and_listen_timeout',
                                            'number_of_failures': 'speak_and_listen_failures',
                                            'failure_threshold': 'speak_and_listen_failure_threshold'})
        
        # announce that we missed the name, and that we will try again
        smach.StateMachine.add('ANNOUNCE_MISSED_GUEST_NAME',
                                SpeakState(),
                                transitions={'success':'ASK_GUEST_NAME'},
                                remapping={'phrase':'speech_recognition_failure_phrase'})

        # announce that we think there is no-one there & end sub state machine
        smach.StateMachine.add('ANNOUNCE_NO_ONE_THERE',
                                SpeakState(),
                                transitions={'success':'failure'},
                                remapping={'phrase':'no_one_there_phrase'})
        
        # ask for guest's gender
        smach.StateMachine.add('ASK_GUEST_GENDER',
                               SpeakAndListenState(),
                                transitions={'success': 'ASK_GUEST_PRONOUNS',
                                            'failure':'ASK_GUEST_GENDER', 
                                            'repeat_failure':'ASK_GUEST_PRONOUNS'},
                                remapping={'question':'ask_gender_phrase',
                                            'operator_response': 'guest_gender',
                                            'candidates':'ask_gender_candidates',
                                            'params':'speak_and_listen_params_empty',
                                            'timeout':'speak_and_listen_timeout',
                                            'number_of_failures': 'speak_and_listen_failures',
                                            'failure_threshold': 'speak_and_listen_failure_threshold'})
        
        # ask for guest's pronouns
        smach.StateMachine.add('ASK_GUEST_PRONOUNS',
                               SpeakAndListenState(),
                                # transitions={'success': 'ANNOUNCE_GUEST_FACE_REGISTRATION_START', # TODO - change back
                                transitions={'success': 'CREATE_GUEST_ATTRIBUTES_DICT',
                                            'failure':'ASK_GUEST_PRONOUNS', 
                                            'repeat_failure':'ANNOUNCE_GUEST_FACE_REGISTRATION_START'},
                                remapping={'question':'ask_pronouns_phrase',
                                            'operator_response': 'guest_pronouns',
                                            'candidates':'ask_pronouns_candidates',
                                            'params':'speak_and_listen_params_empty',
                                            'timeout':'speak_and_listen_timeout',
                                            'number_of_failures': 'speak_and_listen_failures',
                                            'failure_threshold': 'speak_and_listen_failure_threshold'})
        
        # tell guest face registration is starting
        smach.StateMachine.add('ANNOUNCE_GUEST_FACE_REGISTRATION_START',
                                SpeakState(),
                                transitions={'success':'CAPTURE_GUEST_FACE'},
                                remapping={'phrase':'start_face_registration_phrase'})

        # capture guest's face
        smach.StateMachine.add('CAPTURE_GUEST_FACE',
                                RegisterFace(),
                                transitions={'success':'DETECT_OPERATOR_FACE_ATTRIBUTES_BY_DB',
                                            'failure':'DETECT_OPERATOR_FACE_ATTRIBUTES_BY_DB'},
                                remapping={'face_id':'guest_name'})
        
        # detect guest face attributes
        smach.StateMachine.add('DETECT_OPERATOR_FACE_ATTRIBUTES_BY_DB',
                                DetectFaceAttributes(),
                                transitions={'success':'ANNOUNCE_GUEST_FACE_REGISTRATION_FINISH'},
                                remapping={'face_id':'guest_name',
                                            'face_attributes':'guest_face_attributes',
                                            'num_attributes':'guest_num_attributes'  })
        
        # tell guest face registration is finished
        smach.StateMachine.add('ANNOUNCE_GUEST_FACE_REGISTRATION_FINISH',
                                SpeakState(),
                                transitions={'success':'ANNOUNCE_SAVE_GUEST_TO_SOM'},
                                remapping={'phrase':'finish_face_registration_phrase'})
        
        # tell guest we are saving their details
        smach.StateMachine.add('ANNOUNCE_SAVE_GUEST_TO_SOM',
                                SpeakState(),
                                transitions={'success':'CREATE_GUEST_ATTRIBUTES_DICT'},
                                remapping={'phrase':'save_to_som_phrase'})

        # create the guest_attributes dictionary
        smach.StateMachine.add('CREATE_GUEST_ATTRIBUTES_DICT',
                                CreateGuestAttributesDict(),
                                transitions={'success':'SAVE_GUEST_TO_SOM'},
                                remapping={'name':'guest_name','gender':'guest_gender','pronouns':'guest_pronouns','face_id':'guest_name','face_attributes':'guest_face_attributes',
                                        'guest_attributes':'guest_attributes'})

        # save the guest info to the SOM (requires at least one entry in SOM object DB with class_=='person')
        smach.StateMachine.add('SAVE_GUEST_TO_SOM',
                                SaveGuestToSOM(),
                                transitions={'success':'ANNOUNCE_GUEST_FAREWELL',
                                            'failure':'failure'},
                                remapping={'guest_attributes':'guest_attributes'})

        # farewell guest
        smach.StateMachine.add('ANNOUNCE_GUEST_FAREWELL',
                                SpeakState(),
                                transitions={'success':'success'},
                                remapping={'phrase':'farewell_guest'})
    
    return sub_sm

# TODO - create sub state machine for searching for humans
def create_search_for_guest_sub_state_machine():
    """ Smach sub state machine to search for guests (non-operator people)

    Returns 'success' if a non-operator person is found, `failure` otherwise.

    input_keys:
        nodes_not_searched: list of topological node ids to visit during search, in search order
        operator_uid: the operator unique id in the SOM object collection, used to ignore detections from the operator
        failure_threshold: number of allowed failed attempts for each topological navigation action
    output_keys:
        nodes_not_searched: list of topological node ids that were not visited during search (does not include the final node because there may be another person there)
        found_guest_uid: the uid of the found guest, if any
    """

    # gets called when ANY child state terminates
    def child_term_cb(outcome_map):
        # terminate all running states if CHECK_FOR_NEW_GUEST_SEEN finished with outcome 'success'
        if outcome_map['CHECK_FOR_NEW_GUEST_SEEN']:
            if outcome_map['CHECK_FOR_NEW_GUEST_SEEN'] == 'success':
                rospy.loginfo("Concurrence child_term_cb: outcome_map['CHECK_FOR_NEW_GUEST_SEEN'] == 'success'. Terminating")
                return True

        # terminate all running states if NAV_SUB finished with outcome 'failure'
        if outcome_map['NAV_SUB'] == 'failure':
            rospy.loginfo("Concurrence child_term_cb: outcome_map['NAV_SUB'] == 'failure'. Terminating")
            return True

        # in all other case, just keep running, don't terminate anything
        return False

    # gets called when ALL child states are terminated
    def out_cb(outcome_map):
        if outcome_map['CHECK_FOR_NEW_GUEST_SEEN']:
            if outcome_map['CHECK_FOR_NEW_GUEST_SEEN'] == 'success':
                return 'success'
        
        return 'failure'

    # creating the concurrence state machine
    sm_con = Concurrence(outcomes=['success', 'failure'],
                    default_outcome='success',
                    input_keys=['nodes_not_searched',
                                 'operator_uid',
                                 'failure_threshold'],
                    output_keys=['nodes_not_searched',
                                    'found_guest_uid'],
                    child_termination_cb = child_term_cb,
                    outcome_cb = out_cb)

    # Open the concurrence container
    with sm_con:
        sub_sm_nav = smach.StateMachine(outcomes=['failure', 'preempted'],
                                input_keys=['nodes_not_searched',
                                            'operator_uid',
                                            'failure_threshold'],
                                output_keys=['nodes_not_searched'])
        # Open the container 
        with sub_sm_nav:
            # nav to next top node
            smach.StateMachine.add('NAV_TO_NEXT_TOP_NODE',
                                    SearchForGuestNavToNextNode(),
                                    transitions={'searched':'NAV_TO_NEXT_TOP_NODE',
                                                'exhausted_search':'failure',
                                                'failure':'NAV_TO_NEXT_TOP_NODE',
                                                'preempted':'preempted'},
                                    remapping={'nodes_not_searched':'nodes_not_searched',
                                                'failure_threshold':'failure_threshold'})
        
        # Add states to the container
        smach.Concurrence.add('NAV_SUB', sub_sm_nav)
        smach.Concurrence.add('CHECK_FOR_NEW_GUEST_SEEN', CheckForNewGuestSeen())
        
    return sm_con

class SearchForGuestNavToNextNode(smach.State):
    """ Smach state to navigate the robot during the search for guests (non-operator people)

    Returns 'searched' if arrived at next node, 'exhausted_search' if no more nodes are available to visit, `failure` if navigation fails.

    input_keys:
        nodes_not_searched: list of topological node ids to visit during search, in search order
        failure_threshold: number of allowed failed attempts for each topological navigation action
    output_keys:
        nodes_not_searched: list of topological node ids that were not visited during search (does not include the final node because there may be another person there)
    """

    def __init__(self):
        smach.State.__init__(self,
                                outcomes=['searched', 'exhausted_search', 'failure', 'preempted'],
                                input_keys=['nodes_not_searched',
                                            'failure_threshold'],
                                output_keys=['nodes_not_searched'])

    def execute(self, userdata):
        # check if any nodes left to visit
        if not userdata.nodes_not_searched:
            rospy.loginfo('All nodes explored. Nothing to search next.')
            return 'exhausted_search'

        # create action server
        topological_navigate_action_client = actionlib.SimpleActionClient('traverse_to_node',  TraverseToNodeAction)
        topological_navigate_action_client.wait_for_server()

        # take the first node on the nodes not searched list
        node_id = userdata.nodes_not_searched[0]

        for attempt_num in range(userdata.failure_threshold):
            # Check for preempt
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'

            # create action goal and call action server
            goal = TraverseToNodeGoal(node_id=node_id)
            rospy.loginfo('Navigating with top nav to node "{}"'.format(node_id))

            topological_navigate_action_client.wait_for_server()
            topological_navigate_action_client.send_goal(goal)
            topological_navigate_action_client.wait_for_result()
            result = topological_navigate_action_client.get_result()

            rospy.loginfo('result = ' + str(result.success))

            # Process action result
            #   Note: result.success returns True if node_id was reached
            if result.success:
                break   # break out of attempts for-loop
            else:
                # attempt_num starts at 0, and we have just finished taking an attempt
                if (attempt_num + 1) >= userdata.failure_threshold:
                    rospy.logwarn('Navigating with top nav to node "{}" failed. Abandoning.'.format(node_id))
                    # now remove the node from the nodes_not_searched list, because the nav action failed
                    del userdata.nodes_not_searched[0]
                    return 'failure'
            # rospy.sleep(2)  # TODO - remove after testing
        # now remove the node from the nodes_not_searched list, because we have now searched it
        del userdata.nodes_not_searched[0]
        return 'searched'

class CheckForNewGuestSeen(smach.State):
    """ Smach state to check if we have seen a new guest (i.e., a guest we have not spoken to yet)

    Returns 'success' if a new guest is found, otherwise runs indefinitely. Needs to be preempted in a concurrency state.

    input_keys:
        
    output_keys:
        found_guest_uid: the uid of the found guest, if any
    """

    def __init__(self):
        smach.State.__init__(self,
                                outcomes=['success', 'preempted'],
                                input_keys=[],
                                output_keys=['found_guest_uid'])

    def execute(self, userdata):
        userdata.found_guest_uid = "not_set"   # initialise to empty to prepare for case where state gets preempted before new guest is found
                                        # (SMACH will complain if output key does not exist)
        
        # we sit in this loop forever, and only terminate with outcome 'success' if a new guest is found, or 'preempted' if preempted in concurrent state machine
        while True:
            # Check for preempt
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            
            # Check for new guest found
            if False:               # TODO - replace this with logic to look up the SOM and check if we found a new guest
                #userdata.found_guest_uid = <uid from SOM query result>     # TODO - save the uid of the found guest for output
                return 'success'
            else:
                rospy.loginfo("New guest not found yet")
                rospy.sleep(2)

class GetTime(smach.State):
    """ Smach state for current time using ROS clock.

    This state will get the current time and return it in the userdata dict.
    """

    def __init__(self):
        smach.State.__init__(self, 
                                outcomes = ['success'],
                                output_keys=['current_time'])

    def execute(self, userdata):
        # fetch the time and return
        now = rospy.Time.now()
        userdata.current_time = now
        rospy.loginfo("Retreived current time: %i sec, %i ns", now.secs, now.nsecs)
        return 'success'

class CreateGuestAttributesDict(smach.State):
    """ Smach state to build the guest attributes dictionary from userdata values.

    This state will return the built dictionary it in the userdata dict.
    """

    def __init__(self):
        smach.State.__init__(self, 
                                outcomes = ['success'],
                                input_keys=['guest_attributes','name','gender','pronouns','face_id','face_attributes'],
                                output_keys=['guest_attributes'])

    def execute(self, userdata):
        userdata.guest_attributes = {}
        userdata.guest_attributes["name"] = userdata.name
        userdata.guest_attributes["gender"] = userdata.gender
        userdata.guest_attributes["pronouns"] = userdata.pronouns
        userdata.guest_attributes["face_id"] = userdata.face_id
        userdata.guest_attributes["face_attributes"] = userdata.face_attributes
        
        # rospy.loginfo("Created guest_attributes dict: {}".format(userdata.guest_attributes))
        return 'success'

class ShouldIContinueGuestSearchState(smach.State):
    """ State determines whether we should continue or go back. 
    
    input_keys:
        max_search_duration: search duration (seconds)
        start_time: time we started the task (ros Time object)
        guest_som_obj_ids: the list of guest som object ids (only the length is used)
    output_keys:
    
    """

    def __init__(self):
        smach.State.__init__(self, 
                                outcomes = ['yes', 'no'],
                                input_keys=['max_search_duration',
                                            'expected_num_guests',
                                            'start_time',
                                            'guest_som_obj_ids'])
    
    def execute(self, userdata):
        # fetch the current time
        now = rospy.Time.now()
        time_elapsed = now - userdata.start_time

        # if time_elapsed > 210: # 3 and a half minutes - # TODO move to state machine
        if time_elapsed > rospy.Duration(userdata.max_search_duration):
            rospy.loginfo("Exceeded max search time ({}/{} sec) - stop searching!".format(time_elapsed.to_sec(), userdata.max_search_duration))
            return "no"
        
        num_guests_found = len(userdata.guest_som_obj_ids)
        if num_guests_found >= userdata.expected_num_guests:
            rospy.loginfo("Found all the guests ({}) - stop searching!".format(num_guests_found))
            return "no"
        
        rospy.loginfo("I'll keep searching! Time: {}/{} sec, Found: {}/{} guests".format(time_elapsed.to_sec(), userdata.max_search_duration, num_guests_found,userdata.expected_num_guests))
        return "yes"

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

class CreatePhraseStartSearchForPeopleState(smach.State):
    """ Smach state to create the phrase to announce the start of the search for people

    This class always returns success.

    input_keys:
        operator_name: the name of the operator
    output_keys:
        phrase: the returned phrase
    """
    def __init__(self):
        smach.State.__init__(self, 
                                outcomes=['success'],
                                input_keys=['operator_name'],
                                output_keys=['phrase'])
    
    def execute(self, userdata):
        userdata.phrase = "Ok, " + userdata.operator_name + ", I am now going to search for your friends. I'll be back soon!"

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

        if result is None:
            # action server failed
            userdata.number_of_failures+= 1
            if userdata.number_of_failures >= userdata.failure_threshold:
                # reset number of failures because we've already triggered the repeat failure
                userdata.number_of_failures = 0
                return 'repeat_failure'
            return 'failure'

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

#TODO - make a topological localisation node
#       Subscribe to /topological_location - topic type from ori_topological_navigation_msgs : TopologicalLocation to get closest_node_id and current_node_id strings (empty string if not at any)
#       watif for one message and then set variable.


class TopologicalNavigateState(smach.State):
    """ State for navigating along the topological map.

    This state is given a topological map ID and navigates there.

    input_keys:
        node_id: (string) the topological node for the robot to navigate to
        number_of_failures: an external counter keeping track of the cumulative failure count (incremented in this state upon failure & reset upon success and repreat failure)
        failure_threshold: the number of cumulative failures required to return the repeat_failure outcome
    output_keys:
        number_of_failures: the updated failure counter upon state exit
    """

    def __init__(self):
        smach.State.__init__(self, 
                                outcomes=['success', 'failure', 'repeat_failure'],
                                input_keys=['node_id', 'number_of_failures', 'failure_threshold'],
                                output_keys=['number_of_failures'])
    
    def execute(self, userdata):
        # Navigating with top nav
        rospy.loginfo('Navigating with top nav to node "{}"'.format(userdata.node_id))

        # create action goal and call action server
        goal = TraverseToNodeGoal(node_id=userdata.node_id)
       
        topological_navigate_action_client = actionlib.SimpleActionClient('traverse_to_node',  TraverseToNodeAction)
        topological_navigate_action_client.wait_for_server()
        topological_navigate_action_client.send_goal(goal)
        topological_navigate_action_client.wait_for_result()
        result = topological_navigate_action_client.get_result()

        rospy.loginfo('result = ' + str(result.success))

        # Process action result
        #   Note: result.success returns True if node_id was reached
        if result.success:
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

        matched_tf_from_tf_tree = "NOT_FOUND" # the matched tf name from the tree
        
        found_by_name = False
        for frame in frames:
            # perform a sub-string search in the frame string so we find the 
            # frame we are looking for. Eg we find "potted_plant" in "potted_plant_1"
            if pick_up_goal.goal_tf in frame:
                found_by_name = True
                matched_tf_from_tf_tree = frame
                pick_up_goal.goal_tf = matched_tf_from_tf_tree # need to update the goal_tf to the actual frame name (TODO - check why pickup action server no longer resolves this)
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
                        pick_up_goal.goal_tf = frame # need to update the goal_tf to the actual frame name (TODO - check why pickup action server no longer resolves this)
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
                    rospy.loginfo("PickUpObjectState found matching AR tf '{}' in tf-tree for task object {}".format(ar_tf_string, userdata.object_name))
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
        else:
            rospy.loginfo("PickUpObjectState found matching object tf '{}' in tf-tree for task object {}".format(matched_tf_from_tf_tree, userdata.object_name))


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


class CheckDoorIsOpenState(smach.State):
    """ State for robot to check if the door is open. TODO: THE ACTION SERVER NEEDS TESTING!

    This is a common start signal for tasks.

    input_keys:
        
    output_keys:
        
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['open', 'closed'])
    
    def execute(self, userdata):
        is_door_open_goal = DoorCheckGoal()
        is_door_open_goal.n_closed_door = 20 # Same as Bruno's code

        is_door_open_action_client = actionlib.SimpleActionClient('door_check',
                                                               DoorCheckAction)
        is_door_open_action_client.wait_for_server()
        is_door_open_action_client.send_goal(is_door_open_goal)
        is_door_open_action_client.wait_for_result()

        # Boolean value returned
        is_door_open = is_door_open_action_client.get_result().open
        if is_door_open:
            rospy.loginfo("Detected open door")
            return 'open'
        else:
            rospy.loginfo("Detected closed door")
            return 'closed'


class SaveOperatorToSOM(smach.State):
    """ State for robot to log the operator information as an observation in the SOM

    input_keys:
        operator_name: the operator's name
    output_keys:
        operator_som_human_id: the operator's UID in the SOM human collection, returned by SOM observation service call
        operator_som_obj_id: the operator's corresponding UID in the SOM object collection 
                             (assumed to be the most recently observed human-class object)
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure'],
                                input_keys=['operator_name'],
                                output_keys=['operator_som_human_id',
                                            'operator_som_obj_id'])
    
    def execute(self, userdata):
        
        # retreive the most recently observed human-class object in the SOM
        operator_som_obj = get_most_recent_obj_from_som(class_="person")
        if(operator_som_obj is None):
            # couldn't find any "person"-class objects in the SOM
            rospy.logwarn("Could not find any SOM object with class_ 'person' - state failed!")
            return 'failure'

        # create the service message
        operator_obs = SOMAddHumanObsRequest()

        operator_obs.adding.observed_at = rospy.Time.now()
        operator_obs.adding.object_uid  = operator_som_obj.UID
        operator_obs.adding.name = userdata.operator_name
        operator_obs.adding.task_role = 'operator'
        operator_obs.adding.obj_position = operator_som_obj.obj_position    # same as before
        
        # todo - check if used
        # pose = rospy.wait_for_message('/global_pose', PoseStamped).pose

        rospy.loginfo('Storing info for operator "{}" in SOM:\n\t{}'.format(operator_obs.adding.name, operator_obs.adding))

        rospy.wait_for_service('som/human_observations/input')
        som_human_obs_input_service_client = rospy.ServiceProxy('som/human_observations/input', SOMAddHumanObs)

        result = som_human_obs_input_service_client(operator_obs)

        rospy.loginfo('Operator "{}" successfully stored in SOM with human collection UID: {}'.format(operator_obs.adding.name, result.UID))
        userdata.operator_som_human_id = result.UID
        userdata.operator_som_obj_id = operator_obs.adding.object_uid
        return 'success'

class SaveGuestToSOM(smach.State):
    """ State for robot to log the guest information as an observation in the SOM,
        and update the ongoing list of som ids (both human ids and object ids)

    input_keys:
        guest_attributes: the guest's attributes (e.g., name), to be added to the SOM, represented as a dictionary with keys: ["name","age","gender","pronouns","facial_features"]
        guest_som_human_ids: list of UIDs in the SOM human collection, corresponding to the guests we've met so far
        guest_som_obj_ids: list of UIDs in the SOM object collection, corresponding to the guests we've met so far
    output_keys:
        guest_som_human_ids: as above
        guest_som_obj_ids: as above
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure'],
                                input_keys=['guest_attributes', 
                                            'guest_som_human_ids',
                                            'guest_som_obj_ids'],
                                output_keys=['guest_som_human_ids',
                                            'guest_som_obj_ids'])
    
    def execute(self, userdata):
        # retreive the most recently observed human-class object in the SOM
        guest_som_obj = get_most_recent_obj_from_som(class_="person")
        if(guest_som_obj is None):
            # couldn't find any "person"-class objects in the SOM
            rospy.logwarn("Could not find any SOM object with class_ 'person' - state failed!")
            return 'failure'
        
        # This check has been removed - now, if we have previously saved details for this person then it will override it in the SOM
        # if(guest_som_obj.UID in userdata.guest_som_obj_ids):
        #     # we have already saved details for this person
        #     # we haven't seen a new person since last time, 
        #     # so something has gone wrong. return failure
        #     rospy.logwarn("We have already saved the most recent SOM object with class_ 'person' - state failed! ")
        #     return 'failure'       

        rospy.loginfo("Found most recent SOM object with class_ 'person' with UID: {}".format(guest_som_obj.UID))    

        # create the service message
        guest_obs = SOMAddHumanObsRequest()

        guest_obs.adding.observed_at = rospy.Time.now()
        guest_obs.adding.object_uid  = guest_som_obj.UID
        guest_obs.adding.task_role = 'guest'
        guest_obs.adding.obj_position = guest_som_obj.obj_position    # same as before

        if("name" in userdata.guest_attributes):
            guest_obs.adding.name = userdata.guest_attributes["name"]
        if("gender" in userdata.guest_attributes):
            guest_obs.adding.gender = userdata.guest_attributes["gender"]
        if("pronouns" in userdata.guest_attributes):
            guest_obs.adding.pronouns = userdata.guest_attributes["pronouns"]
        if("face_id" in userdata.guest_attributes):
            guest_obs.adding.face_id = userdata.guest_attributes["face_id"]
        if("face_attributes" in userdata.guest_attributes):
            guest_obs.adding.face_attributes = userdata.guest_attributes["face_attributes"]

        # todo - check if used
        # pose = rospy.wait_for_message('/global_pose', PoseStamped).pose

        rospy.loginfo('Storing info for guest "{}" in SOM:\n{}'.format(guest_obs.adding.name, guest_obs.adding))

        rospy.wait_for_service('som/human_observations/input')
        som_human_obs_input_service_client = rospy.ServiceProxy('som/human_observations/input', SOMAddHumanObs)

        result = som_human_obs_input_service_client(guest_obs)

        rospy.loginfo('Guest "{}" successfully stored in SOM with human collection UID: {}'.format(guest_obs.adding.name, result.UID))
        userdata.guest_som_human_ids.append(result.UID)
        userdata.guest_som_obj_ids.append(guest_obs.adding.object_uid)
        return 'success'


class RegisterFace(smach.State):
	""" State for robot to register a new face using facial capture action server

	The action server detects faces from the robot RGB-D sensor's RGB camera image topic

	input_keys:
		face_id: (optional) the name or identifier of the person whose face is being registered
	output_keys:
		registered_face_id: the id assigned to the registered face. This will be the input face_id if given,
							else a unique sequential id generated by the facial capture action server
	"""

	def __init__(self):
		smach.State.__init__(self, outcomes=['success','failure'],
								input_keys=['face_id'],
								output_keys=['registered_face_id'])
		
	def execute(self, userdata):
		# if the face_id is not given, then set it to an empty string; 
		# the capface server will automatically generate one for us
		if("face_id" not in userdata):
			face_id = ""
		else:
			face_id = userdata["face_id"]

		# set action goal and call action server
		capface_goal = ActionServer_CapFaceGoal(face_id=face_id)
		
		capface_action_client = actionlib.SimpleActionClient('as_Capface', ActionServer_CapFaceAction)
		capface_action_client.wait_for_server()
		rospy.loginfo("Calling CapFace action server...")
		capface_action_client.send_goal(capface_goal)
		capface_action_client.wait_for_result()

		# process result
		found_face = capface_action_client.get_result().If_saved
		if not found_face:
			# no faces were found in the image
			rospy.logwarn("Capface action server failed. Did not find any faces in the image")
			return "failure"
		# success
		registered_face_id = capface_action_client.get_result().name
		userdata["registered_face_id"] = registered_face_id
		rospy.loginfo("Capface action server registered face with id: {}".format(registered_face_id))
		return "success"


class RecogniseFace(smach.State):
	""" State for robot to recognise a face and match it to a previously registered face,
		using the facial capture action server

	input_keys:
		min_score_threshold: (optional) the minimum score for a matched face to be considered a valid match
	output_keys:
		face_id: the id of the detected face, used in the facelib system / capface action server
		face_match_score: the match score of the recognised face
	"""

	def __init__(self):
		smach.State.__init__(self, outcomes=['success', 'failure'],
								input_keys=['min_score_threshold'],	
								output_keys=['face_id','face_match_score'])

	def execute(self, userdata):
		# use min_score_threshold, if set
		if "min_score_threshold" in userdata:
			min_score_threshold = userdata["min_score_threshold"]
		else:
			min_score_threshold = 0.1

		# set action goal and call action server
		findmatch_goal = ActionServer_FindMatchGoal()
		
		findmatch_action_client = actionlib.SimpleActionClient('as_Findmatch', ActionServer_FindMatchAction)
		findmatch_action_client.wait_for_server()
		rospy.loginfo("Calling face match action server...")
		findmatch_action_client.send_goal(findmatch_goal)
		findmatch_action_client.wait_for_result()

		# process result
		if(not findmatch_action_client.get_result().If_find):
			# face not found
			rospy.loginfo("Face FindMatch action server did not find any faces in the image")
			return "failure"
		# face was found
		matched_face_id = findmatch_action_client.get_result().face_id
		matched_face_score = findmatch_action_client.get_result().best_match_score
		# matched_face_file_name = findmatch_action_client.get_result().file_name
		if(matched_face_score < min_score_threshold):
			# no faces found with high enough score/confidence
			rospy.logwarn("Face FindMatch action server failed. Did not find any matches above min score threshold ({}); best score is '{}'.".format(min_score_threshold, matched_face_score))
			return "failure"
		# result was good!
		userdata["face_id"] = matched_face_id
		userdata["face_match_score"] = matched_face_score
		rospy.loginfo("Face FindMatch action server recognised face_id '{}', face_score: {}".format(matched_face_id, matched_face_score))
		return "success"

class DetectFaceAttributes(smach.State):
	""" State for the robot to detect face attributes
	
	Uses the FaceLib FindAttrs action server. 
	Always succeeds; an empty attributes list is not a failure.

	input_keys:
		face_id: (optional) If given, the action server will analyse the saved face for this face_id,
							otherwise it will analyse the face that currently appears in the image topic
	output_keys:
		face_attributes: A list of detected facial attributes, represented as strings. 
						 If a certain feature is detected, it will be present in this list.
						 Thus, features not in the list were not detected.
		num_attributes: The number of detected facial attributes
	"""

	def __init__(self):
		smach.State.__init__(self, outcomes=['success'],
								input_keys=['face_id'],
								output_keys=['face_attributes', 'num_attributes'])
		
	def execute(self, userdata):
		# if the face_id is not given, then set it to an empty string; 
		# the capface server will use the live camera topic feed (rather than the pre-registered face)
		if("face_id" not in userdata):
			face_id = ""
		else:
			face_id = userdata["face_id"]

		# set action goal and call action server
		find_face_attrs_goal = ActionServer_FindAttrsGoal(face_id=face_id)
		
		find_face_attrs_action_client = actionlib.SimpleActionClient('as_Findattrs', ActionServer_FindAttrsAction)
		find_face_attrs_action_client.wait_for_server()
		rospy.loginfo("Calling find face attributes action server...")
		find_face_attrs_action_client.send_goal(find_face_attrs_goal)
		find_face_attrs_action_client.wait_for_result()

		# process result
		face_attributes_raw = find_face_attrs_action_client.get_result().attrs
		num_attributes_raw = find_face_attrs_action_client.get_result().num_attrs
		face_attributes = filter_face_attributes_by_exclusion(face_attributes_raw) 		# remove unwanted labels
		num_attributes = len(face_attributes)
		userdata["face_attributes"] = face_attributes
		userdata["num_attributes"] = num_attributes
		rospy.loginfo("FaceAttributes action server registered {} face attibutes: \n\t{}".format(num_attributes, face_attributes))
		# rospy.loginfo("FaceAttributes action server registered {} face attibutes: \n\t{}\nFound {} unfiltered attributes: \n\t{}".format(num_attributes, face_attributes, num_attributes_raw, face_attributes_raw))
		return "success"

class ClearFaceDB(smach.State):
	""" State for the robot to clear the face database
	
	Uses the FaceLib as_Cleardatabase action server

	input_keys:

	output_keys:

	"""

	def __init__(self):
		smach.State.__init__(self, outcomes=['success'])
		
	def execute(self, userdata):
		# set action goal and call action server
		clear_face_db_goal = ActionServer_ClearDatabaseGoal()
		
		clear_face_db_action_client = actionlib.SimpleActionClient('as_Cleardatabase', ActionServer_ClearDatabaseAction)
		clear_face_db_action_client.wait_for_server()
		rospy.loginfo("Calling clear face db action server...")
		clear_face_db_action_client.send_goal(clear_face_db_goal)
		clear_face_db_action_client.wait_for_result()

		# process result
		# is_success = clear_face_db_action_client.get_result().Is_success - is this used? it's not essential atm

		rospy.loginfo("ClearFaceDatabase action server cleared the database")
		return "success"

# TODO - complete this state & test it
class AnnounceGuestDetailsToOperator(smach.State):
	""" State for the robot to give the operator info about mates
	
	Always succeeds.

	input_keys:
		guest_som_human_ids: TODO
		guest_som_obj_ids: TODO

	output_keys:
		
	"""

	def __init__(self):
		smach.State.__init__(self, outcomes=['success'],
								input_keys=['guest_som_human_ids', 'guest_som_obj_ids'])
		
	def execute(self, userdata):
		number_of_guests_found = min(len(userdata.guest_som_human_ids), len(userdata.guest_som_obj_ids))

		if number_of_guests_found == 0:
			talk_phrase = "I could not find any of your mates. Don't worry, I'm sure they will arrive soon!"
			call_talk_request_action_server(phrase=talk_phrase)
			return 'success'
		
		if number_of_guests_found > 0:
			talk_phrase = "I found {} of your mates! Let me tell you about them!".format(number_of_guests_found)
			call_talk_request_action_server(phrase=talk_phrase)
			for guest_num in range(number_of_guests_found):
				# Query human from human collection
				# call a query by class_ and sort results by most recent observation - return the newest one
				query = SOMQueryHumansRequest() 
				query.query.object_uid = userdata.guest_som_obj_ids[guest_num]
				
				rospy.wait_for_service('som/humans/basic_query')
				som_humans_query_service_client = rospy.ServiceProxy('som/humans/basic_query', SOMQueryHumans);

				# call the service
				response = som_humans_query_service_client(query);

				# if the SOM didn't return any records that match the object_uid, then skip it
				if len(response.returns) == 0:
					# let the operator know there was a problem
					talk_phrase = "Something has gone wrong and I cannot retreive information about person number {}".format(guest_num+1)
					call_talk_request_action_server(phrase=talk_phrase)
					continue

				# just take the first return because there should only ever be one human entry for a given object_uid
				human_record = response.returns[0]

				# build the string to tell the operator about the mate
				person_talk_phrase = ""
				if human_record.name:
					person_talk_phrase += "The {} person I met was {}.".format(positional_to_cardinal(guest_num+1), human_record.name)
				else:
					if guest_num == 0:
						person_talk_phrase += "I met the {} person.".format(positional_to_cardinal(guest_num+1))
					else:
						person_talk_phrase += "I met a {} person.".format(positional_to_cardinal(guest_num+1))

				# TODO - build in information about person location (maybe query the SOM to find out?)

				if not human_record.gender:
					pass
				else:
					if human_record.gender == "prefer not to say":
						pass
					else:
						person_talk_phrase += " They identify as {}.".format(human_record.gender)
				
				if not human_record.pronouns:
					pass
				else:
					if human_record.pronouns == "prefer not to say":
						pass
					else:
						person_talk_phrase += " Their pronouns are '{}'.".format(human_record.pronouns)
						
				if human_record.face_attributes:
					person_talk_phrase += " They have the following facial attributes: {}.".format(human_record.face_attributes)

				# speak the details for this person
				call_talk_request_action_server(phrase=person_talk_phrase)

			# wrap up
			talk_phrase = "That's everyone I met!"
			call_talk_request_action_server(phrase=talk_phrase)

		return 'success'

class WaitForHotwordState(smach.State):
    """ Smach state for waiting for the hotword detector to publish a detection message.

    Terminates with 'success' outcome if hotword detection message is received within the timeout (if used),
    otherwise 'failure'.
    
    input_keys:
		timeout: timeout time in seconds (set to None to wait indefinitely)
    """

    def __init__(self):
        smach.State.__init__(self, 
                                outcomes = ['success', 'failure'],
                                input_keys=['timeout'])

    def execute(self, userdata):
        # call_talk_request_action_server(phrase="I'm ready and waiting for the hotword")
        rospy.loginfo("Waiting for hotword...")
        try:
            # Wait for one message on topic
            hotword_msg = rospy.wait_for_message('/hotword', Hotword, timeout=userdata.timeout)
            rospy.loginfo("Hotword '{}' received at time: {}".format(hotword_msg.hotword, hotword_msg.stamp.to_sec()))
            # call_talk_request_action_server(phrase="Hotword received")
            return 'success'
        except rospy.ROSException as e:
            rospy.logwarn("Hotword not received within timeout")
            return 'failure'


# class GiveOperatorInfoState(ActionServiceState):
#     """ State for giving operator info about mates. """

#     def __init__(self, action_dict, global_store):
#         outcomes = ['SUCCESS']
#         super(GiveOperatorInfoState, self).__init__(action_dict=action_dict,
#                                                     global_store=global_store,
#                                                     outcomes=outcomes)
    
#     def execute(self, userdata):
#         obj1 = SOMObservation()
#         obj1.type = 'person'

#         matches = self.action_dict['SOMQuery'](obj1, Relation(), 
#                                                SOMObservation())

#         operator_name = ''
#         people_information = []

#         # Extract information from matches
#         for match in matches:
#             person = match.obj1
#             if person.task_role == 'operator':
#                 operator_name = person.name
#             else:
#                 person_info = {}
#                 person_info['name'] = person.name
#                 person_info['age'] = person.age
#                 person_info['gender'] = person.gender
#                 person_info['shirt_colour'] = person.shirt_colour
#                 person_info['room'].pose_estimate.most_likely_room
#                 people_information.append(person_info)
        
#         info_string = ("Hi " + operator_name + ", I have some people to tell " +
#                       "you about.")
        
#         for person in people_information:
#             person_string = ""
#             if person['room'] != '':
#                 person_string += " In the " + person['room'] + " I met "
#             else:
#                 person_string += " I met "
            
#             if person['name'] != '':
#                 person_string += person['name'] + ', '
#             else:
#                 person_string += 'someone, '
            
#             if person['age'] != 0:
#                 person_string += ('who I think is around the age of ' + 
#                                   str(person['age']) + ', ')
            
#             if person['gender'] != '':
#                 person_string += 'who is ' + person['gender'] + ', '
            
#             if person['shirt_colour'] != '':
#                 person_string += 'and who is wearing ' + person['shirt_colour']
            
#             if person_string[-2] == ',':
#                 person_string = person_string[0:-2] + '.'
#             else:
#                 person_string += '.'
#             info_string += person_string
#         goal = SpeakGoal()
#         goal.sentence = info_string
#         self.action_dict['Speak'].send_goal(goal)
#         self.action_dict['Speak'].wait_for_result()

#         return self._outcomes[0]

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

        TODO - 
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
        result = self.action_dict['SOMObserve'](operator) # change to SOM input service call
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
