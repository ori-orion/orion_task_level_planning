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
from smach import Concurrence

from orion_actions.msg import GiveObjectToOperatorGoal, \
    SpeakGoal, IsDoorOpenGoal, OpenDoorGoal, GiveObjectToOperatorGoal, \
        ReceiveObjectFromOperatorGoal, PutObjectOnFloorGoal, \
            PutObjectOnSurfaceGoal, CheckForBarDrinksGoal, SpeakAndListenGoal, \
                HotwordListenGoal, GetPointedObjectGoal, PickUpObjectGoal, \
                    NavigateGoal, FollowGoal, OpenBinLidGoal, OpenDrawerGoal, \
                        PlaceObjectRelativeGoal, PourIntoGoal, PointToObjectGoal

FAILURE_THRESHOLD = 3

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
        action_goal = SpeakGoal()
        action_goal.sentence = self.phrase
        self.action_dict['Speak'].send_goal(action_goal)
        self.action_dict['Speak'].wait_for_result()

        # Boolean value returned
        result = self.action_dict['Speak'].get_result().succeeded
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
        is_door_open_goal = IsDoorOpenGoal()
        self.action_dict['IsDoorOpen'].send_goal(is_door_open_goal)
        self.action_dict['IsDoorOpen'].wait_for_result()

        # Boolean value returned
        is_door_open = self.action_dict['IsDoorOpen'].get_result().is_open
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
        is_door_open_goal = IsDoorOpenGoal()
        self.action_dict['IsDoorOpen'].send_goal(is_door_open_goal)
        self.action_dict['IsDoorOpen'].wait_for_result()

        # Boolean value returned
        is_door_open = self.action_dict['IsDoorOpen'].get_result().is_open
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
        # TODO: Get location!
        # Should be a triple (x,y,theta)
        self.global_store['stored_location'] = 'DUMMY_LOCATION'


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

            del self.global_store['speak_listen_failure']
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

    def __init__(self, action_dict, global_store, timeout):
        outcomes = ['SUCCESS', 'FAILURE']
        self.timeout = timeout
        super(HotwordListenState, self).__init__(action_dict=action_dict,
                                                 global_store=global_store,
                                                 outcomes=outcomes)
    
    def execute(self, userdata):
        hotword_goal = HotwordListenGoal()
        hotword_goal.timeout = self.timeout
        self.action_dict['HotwordListen'].send_goal(hotword_goal)
        self.action_dict['HotwordListen'].wait_for_result()
        
        result = self.action_dict['HotwordListen'].get_result()

        if result.succeeded:
            self.global_store['last_response'] = result.answer
            return self._outcomes[0]
        else:
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
        self.action_dict['GetPointedObject'].send_goal(GetPointedObjectGoal())
        self.action_dict['GetPointedObject'].wait_for_result()
        
        obj = self.action_dict['getPointedObject'].get_result()

        pickup_goal = PickUpObjectGoal()
        pickup_goal.goal_tf = obj

        self.action_dict['PickUpObject'].send_goal(pickup_goal)
        self.action_dict['PickUpObject'].wait_for_result()

        result = self.action_dict['PickUpObject'].get_result().goal_complete

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
        # TODO: Fill in, this will likely do stuff with the semantic map
        # Make sure to store location too!
        pass


class MemorisePersonState(ActionServiceState):
    """ State for memorising mates found. """

    def __init__(self, action_dict, global_store):
        outcomes = ['SUCCESS', 'FAILURE']
        super(MemorisePersonState, self).__init__(action_dict=action_dict,
                                                  global_store=global_store,
                                                  outcomes=outcomes)
    
    def execute(self, userdata):
        # TODO: Fill in !
        # Should memorise like in operator detect but in list for this task
        # Should take drink information if possible/appropriate
        pass


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
        follow_goal.object_name = self.global_store['operator'] # TODO: Fix
        self.action_dict['Follow'].send_goal(follow_goal)

        current_result = True
        while not self.preempt_requested() and current_result != False:
            time.sleep(1)
            current_result = self.action_dict['Follow'].get_result().succeeded
        
        self.action_dict['Follow'].cancel_all_goals()
    
        if current_result == False:
            self.global_store['follow_failure'] += 1
            if self.global_store['follow_failure'] >= FAILURE_THRESHOLD:
                return self._outcomes[2]
            return self._outcomes[1]
        else:
            del self.global_store['follow_failure']
            return self._outcomes[0]


def follow_child_cb(outcome_map):
    """Executed whenever a child in the concurrent state is terminated."""
    if outcome_map['Hotword'] == 'SUCCESS':
        return True
    if outcome_map['Hotword'] == 'FAILURE':
        return True
    if outcome_map['Follow'] == 'FAILURE':
        return True
    if outcome_map['Follow'] == 'REPEAT_FAILURE':
        return True
    
    return False

def follow_out_cb(outcome_map):
    if outcome_map['Hotword'] == 'SUCCESS':
        return 'SUCCESS'
    elif outcome_map['Follow'] == 'REPEAT_FAILURE':
        return 'REPEAT_FAILURE'
    else:
        return 'FAILURE'

def make_follow_hotword_state(action_dict, global_store):
    """ Creates a concurrent state which follows someone while listening.

    This concurrent state machine follows someone while waiting for a hot
    word to be spoken.
    """
    con = Concurrence(outcomes=['SUCCESS', 'FAILURE', 'REPEAT_FAILURE'],
                      default_outcome='FAILURE',
                      child_termination_cb=follow_child_cb,
                      outcome_map=follow_out_cb)
    
    with con:
        Concurrence.add('Follow', FollowState(action_dict, global_store))
        Concurrence.add('Hotword', HotwordListenState(action_dict, 
                                                      global_store,
                                                      120))
    
    return con
# --- End of follow code

class NavigateState(ActionServiceState):
    """ State for navigating to location on map.

    This state is given a triple (x,y,theta) and navigates there.
    """

    def __init__(self, action_dict, global_store):
        outcomes = ['SUCCESS', 'FAILURE', 'REPEAT_FAILURE']
        super(NavigateState, self).__init__(action_dict=action_dict,
                                            global_store=global_store,
                                            outcomes=outcomes)
        
        if 'nav_failure' not in self.global_store:
            self.global_store['nav_failure'] = 0
    
    def execute(self, userdata):
        triple = self.global_store['nav_location']

        nav_goal = NavigateGoal()
        nav_goal.x = triple[0]
        nav_goal.y = triple[1]
        nav_goal.theta = triple[2]

        self.action_dict['Navigate'].send_goal(nav_goal)
        self.action_dict['Navigate'].wait_for_result()

        result = self.action_dict['Navigate'].get_result().succeeded

        if result:
            del self.global_store['nav_failure']
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

        self.action_dict['PickUpObject'].send_goal(pick_up_goal)
        self.action_dict['PickUpObject'].wait_for_result()

        result = self.action_dict['PickUpObject'].get_result().goal_complete

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


class SetPickupState(ActionServiceState):
    """ State for setting pick up to something arbitrary defined by lambda. """

    def __init__(self, action_dict, global_store, function):
        """ function must have 0 parameters and must return the new object. """
        outcomes = ['SUCCESS']
        self.function = function
        super(SetPickupState, self).__init__(action_dict=action_dict,
                                             global_store=global_store,
                                             outcomes=outcomes)
    
    def execute(self, userdata):
        self.global_store['pick_up'] = self.function()
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
