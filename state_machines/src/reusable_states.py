#!/usr/bin/env python
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
    OpenDoorGoal, GiveObjectToOperatorGoal, \
        ReceiveObjectFromOperatorGoal, PutObjectOnFloorGoal, \
            PutObjectOnSurfaceGoal, CheckForBarDrinksGoal, SpeakAndListenGoal, \
                HotwordListenGoal, PickUpObjectGoal, \
                    FollowGoal, OpenDrawerGoal, \
                        PlaceObjectRelativeGoal, PourIntoGoal, \
                            PointToObjectGoal, OpenFurnitureDoorGoal, \
                                PointingGoal, CloseDrawerGoal
from orion_door_pass.msg import DoorCheckGoal
from orion_actions.msg import DetectionArray, FaceDetectionArray, PoseDetectionArray
from orion_actions.msg import SOMObservation, Relation
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from move_base_msgs.msg import MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from tmc_msgs.msg import TalkRequestGoal, Voice
from strands_navigation_msgs.srv import GetTaggedNodesResponse
from strands_navigation_msgs.msg import TopologicalMap
from strands_executive_msgs.msg import ExecutePolicyGoal, MdpDomainSpec

FAILURE_THRESHOLD = 3

# TODO: Update nearer the time!
NAMES = ['Alex', 'Charlie', 'Elizabeth', 'Francis', 'Jennifer', 'Linda',
         'Mary', 'Patricia', 'Robin', 'Skyler', 'James', 'John', 'Michael',
         'Robert', 'William', 'Mark', 'Chia-Man', 'Dennis', 'Matt', 'Shu', 
         'Mia', 'Tim', 'Oliver', 'Yizhe']

READY = ['ready']#['I am ready', 'ready', "let's go", "I'm ready"]
DRINKS = ['Coke', 'Beer', 'Water', 'Orange Juice', 'Champagne', 'Absinthe']
COLOURS = ["Red", "Orange", "Yellow", "Green", "Blue", "Purple",
           "Black", "White", "Grey", "Brown", "Beige"]

RELATIONS = ['left', 'right', 'above', 'below', 'front', 'behind', 'near']
OBJECTS = ['apple', 'banana', 'cereal', 'bowl', 'cloth'] # TODO: YCB benchmark
FRUITS = ['apple', 'banana', 'orange', 'mango', 'strawberry', 'kiwi', 'plum',
          'nectarine'] # TODO: Fill in with the YCB benchmark


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


class SpeakState(ActionServiceState):
    """ Smach state for the robot to say stuff.

    This class has the robot say something and return success if it has been
    said. Enough said...

    Attributes:
        phrase: What we want the robot to say
    """
    def __init__(self, action_dict, global_store, phrase):
        self.phrase = phrase
        outcomes = ['SUCCESS', 'FAILURE']
        super(SpeakState, self).__init__(action_dict=action_dict,
                                         global_store=global_store,
                                         outcomes=outcomes)
    
    def execute(self, userdata):
        action_goal = TalkRequestGoal()
        action_goal.data.language = Voice.kEnglish
        action_goal.data.sentence = self.phrase
        self.action_dict['Speak'].send_goal(action_goal)
        self.action_dict['Speak'].wait_for_result()

        # Can only succeed
        return self._outcomes[0]


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


class HandoverObjectToOperatorState(ActionServiceState):
    """ Smach state for handing a grasped object to an operator.

    This state hands over an object to the operator.
    """
    def __init__(self, action_dict, global_store):
        outcomes = ['SUCCESS', 'FAILURE']
        super(HandoverObjectToOperatorState, self).__init__(action_dict=
                                                            action_dict,
                                                            global_store=
                                                            global_store,
                                                            outcomes=
                                                            outcomes)
    
    def execute(self, userdata):
        handover_goal = GiveObjectToOperatorGoal()
        self.action_dict['GiveObjectToOperator'].send_goal(handover_goal)
        self.action_dict['GiveObjectToOperator'].wait_for_result()

        success = self.action_dict['GiveObjectToOperator'].get_result().result
        if success:
            return self._outcomes[0]
        else:
            return self._outcomes[1]
        

class ReceiveObjectFromOperatorState(ActionServiceState):
    """ Smach state for receiving an object from an operator.

    This state grasps an object currently held by an operator.
    """
    def __init__(self, action_dict, global_store):
        outcomes = ['SUCCESS', 'FAILURE']
        super(ReceiveObjectFromOperatorState, self).__init__(action_dict=
                                                             action_dict,
                                                             global_store=
                                                             global_store,
                                                             outcomes=
                                                             outcomes)

    def execute(self, userdata):
        receive_goal = ReceiveObjectFromOperatorGoal()
        self.action_dict['ReceiveObjectFromOperator'].send_goal(receive_goal)
        self.action_dict['ReceiveObjectFromOperator'].wait_for_result()

        result = self.action_dict['ReceiveObjectFromOperator'].get_result()
        success = result.result
        if success:
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


class GetRobotLocationState(ActionServiceState):
    """ Smach state for getting the robot's current location.

    This state will get the robot's current location and store it.
    """

    def __init__(self, action_dict, global_store):
        outcomes = ['STORED']
        super(GetRobotLocationState, self).__init__(action_dict=action_dict,
                                                    global_store=global_store,
                                                    outcomes=outcomes)
        
    def execute(self, userdata):
        # Wait for one message on topic and then set as the location
        pose = rospy.wait_for_message('/global_pose', PoseStamped)
        self.global_store['stored_location'] = pose.pose
        rospy.loginfo(pose)
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
                if self.global_store['speak_hotword_failure'] >= FAILURE_THRESHOLD:
                    return self._outcomes[2]
                else:
                    return self._outcomes[1]
        except:
            rospy.loginfo('SOMETHING WENT WRONG')
            return self._outcomes[2]



class SpeakAndListenState(ActionServiceState):
    """ Smach state for speaking and then listening for a response.

    This state will get the robot to say something and then wait for a 
    response.
    """

    def __init__(self, 
                 action_dict, 
                 global_store, 
                 question, 
                 candidates, 
                 params, 
                 timeout):
        """ Constructor initialises fields and calls super constructor.

        Args:
            action_dict: As in super class
            global_store: As in super class
            question: The question to ask
            candidates: Candidate sentences
            params: Optional parameters for candidate sentences
            timeout: The timeout for listening
        """

        outcomes = ['SUCCESS', 'FAILURE', 'REPEAT_FAILURE']
        self.question = question
        self.candidates = candidates
        self.params = params
        self.timeout = timeout
        super(SpeakAndListenState, self).__init__(action_dict=action_dict,
                                                  global_store=global_store,
                                                  outcomes=outcomes)
        
        if 'speak_listen_failure' not in self.global_store:
            self.global_store['speak_listen_failure'] = 0
    
    def execute(self, userdata):
        speak_listen_goal = SpeakAndListenGoal()
        speak_listen_goal.question = self.question
        speak_listen_goal.candidates = self.candidates
        speak_listen_goal.params = self.params
        speak_listen_goal.timeout = self.timeout

        self.action_dict['SpeakAndListen'].send_goal(speak_listen_goal)
        self.action_dict['SpeakAndListen'].wait_for_result()

        result = self.action_dict['SpeakAndListen'].get_result()
        if result.succeeded:
            self.global_store['last_response'] = result.answer
            self.global_store['speak_listen_failure'] = 0
            return self._outcomes[0]
        else:
            self.global_store['speak_listen_failure'] += 1
            if self.global_store['speak_listen_failure'] >= FAILURE_THRESHOLD:
                return self._outcomes[2]
            return self._outcomes[1]


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
        options = OBJECTS
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
    """

    def __init__(self, action_dict, global_store):
        outcomes = ['SUCCESS', 'FAILURE']
        super(OperatorDetectState, self).__init__(action_dict=action_dict,
                                                  global_store=global_store,
                                                  outcomes=outcomes)
    
    def execute(self, userdata):
        failed = 0
        operator = SOMObservation()

        if self.global_store['last_person'] is not None:
            operator.obj_id = self.global_store['last_person']

        operator.type = 'person'
        operator.task_role = 'operator'
        

        pose = rospy.wait_for_message('/global_pose', PoseStamped).pose
        operator.robot_pose = pose
        
        operator.room_name = self.action_dict['SOMGetRoom'](pose).room_name
        
        for name in NAMES:
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

        for name in NAMES:
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
            tf_frame = 'person_' + operator.shirt_colour
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
            if self.global_store['follow_failure'] >= FAILURE_THRESHOLD:
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
    elif ('follow_failure' in global_store and global_store['follow_failure'] >= FAILURE_THRESHOLD):
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
            if self.global_store['nav_failure'] >= FAILURE_THRESHOLD:
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
            if self.global_store['nav_failure'] >= FAILURE_THRESHOLD:
                return self._outcomes[2]
            return self._outcomes[1]


class PickUpObjectState(ActionServiceState):
    """ State for picking up an object.

    This state picks up an object specified in the global store.
    """

    def __init__(self, action_dict, global_store):
        outcomes = ['SUCCESS', 'FAILURE']
        super(PickUpObjectState, self).__init__(action_dict=action_dict,
                                                global_store=global_store,
                                                outcomes=outcomes)
    
    def execute(self, userdata):
        pick_up_goal = PickUpObjectGoal()
        pick_up_goal.goal_tf = self.global_store['pick_up']

        if pick_up_goal.goal_tf == 'potted plant':
            rospy.loginfo('POTTED PLANT')
            pick_up_goal.goal_tf = 'potted_plant'

        self.action_dict['PickUpObject'].send_goal(pick_up_goal)
        self.action_dict['PickUpObject'].wait_for_result()

        result = self.action_dict['PickUpObject'].get_result().result

        if result:
            return self._outcomes[0]
        else:
            return self._outcomes[1]


class SetNavGoalState(ActionServiceState):
    """ State for setting nav goal to something arbitrary defined by lambda. """

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
            if self.global_store['nav_failure'] >= FAILURE_THRESHOLD:
                return self._outcomes[2]
            return self._outcomes[1]


class SetPickupState(ActionServiceState):
    """ State for setting pick up to something arbitrary passed in. """

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
    """ State for setting pick up to something arbitrary defined by lambda. """

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
