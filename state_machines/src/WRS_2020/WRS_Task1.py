#!/usr/bin/env python3
""" File for tidy up task for WRS 2020.

This file contains the state machine for task 1 of WRS 2020.
This involves detecting out of place objects in the environment and then
placing them back in the correct location based on the semantic map information.

Author: Mia Mijovic, Charlie Street
Owner: Mia Mijovic, Charlie Street

"""

import rospy
import smach
import actionlib

from reusable_states import * # pylint: disable=unused-wildcard-import
from set_up_clients import create_common_clients
from orion_actions.msg import SOMObservation, Relation, OpenDrawerAction,\
     CloseDrawerAction, PickUpObjectAction, PutObjectOnSurfaceAction
from orion_spin.msg import SpinGoal, SpinAction

import time


def speak(action_dict, phrase):
    """ Wrapper for getting the robot to talk. Good for debugging. """
    action_goal = TalkRequestGoal()
    action_goal.data.language = Voice.kEnglish
    action_goal.data.sentence = phrase
    action_dict['Speak'].send_goal(action_goal)
    action_dict['Speak'].wait_for_result()


class SearchForItemState(ActionServiceState):
    """ This state carries out a search for a misplaced item. """

    def __init__(self, action_dict, global_store):
        outcomes = ['OBJECT_FOUND', 'NO_OBJECTS']

        super(SearchForItemState, self).__init__(action_dict=action_dict,
                                                 global_store=global_store,
                                                 outcomes=outcomes)
    
    def execute(self, userdata):
        
        speak(self.action_dict, "I am now searching the room")

        # First spin
        self.action_dict['spin'].send_goal(SpinGoal())
        self.action_dict['spin'].wait_for_result()

        # TODO: Find a way to get the objects from the semantic map
        # Have it stored in  displaced_objects (name, loc, goal_loc, draw_num)
        # -1 draw_num for no drawer
        # ASK MARC
        speak(self.action_dict, "Charlie hasn't fixed this yet")
        return self._outcomes[1]


class GetItemLocationAndCheckState(ActionServiceState):
    "This state gets the default location of displaced item and checks the position"
    "of the drawers"

    def __init__(self, action_dict, global_store):
        outcomes = ['NO_ACTION_REQ', 'ACTION_REQUIRED', 'NO_OBJECTS']

        super(GetItemLocationAndCheck, self).__init__(action_dict=action_dict,
                                                      global_store = global_store,
                                                      outcomes=outcomes)

    def execute(self, userdata):
        if len(self.global_store['displaced_objects']) == 0:
            speak(self.action_dict, "My list of displaced objects is empty")
            return self._outcomes[2]

        new_item = self.global_store['displaced_objects'][0]
        self.global_store['displaced_objects'] = \
            self.global_store['displaced_objects'][1:]

        speak(self.action_dict, "I'm going to put away the " + str(new_item[0]))
        self.global_store['current_item'] = new_item
        self.global_store['pick_up'] = new_item[0]

        if new_item[3] == -1:
            self.global_store['drawer_handle'] = None
            self.global_store['nav_location'] = new_item[1]
            return self._outcomes[0]
        else:
            speak(self.action_dict, 'But I need to open a drawer first.')
            # TODO: Somehow deal with the different drawers here
            self.global_store['drawer_handle'] = new_item[3]
            self.global_store['nav_location'] = new_item[2]
            return self._outcomes[1]


class ShouldIContinueState(ActionServiceState):
    """ State determines whether we should continue or go back. """
    #Put after success when placing an object down

    def __init__(self, action_dict, global_store):
        outcomes = ['YES', 'NO']
        super(ShouldIContinueState, self).__init__(action_dict=action_dict,
                                                   global_store=global_store,
                                                   outcomes=outcomes)
    
    def execute(self, userdata):
        time_elapsed = time.time() - self.global_store['start_time']
        if time_elapsed > 540: # 9 minutes
            return self._outcomes[1]
        
        return self._outcomes[0]


class NeedToCloseState(ActionServiceState):
    """ Determines whether the robot needs to close a drawer. """

    def __init__(self, action_dict, global_store):
        outcomes = ['YES', 'NO']
        super(NeedToCloseState, self).__init__(action_dict=action_dict,
                                               global_store=global_store,
                                               outcomes=outcomes)
    
    def execute(self, userdata):

        if self.global_store['drawer_handle'] is None:
            return self._outcomes[1]
        else:
            return self._outcomes[0]


def create_state_machine(action_dict):
    """ This function builds the state machine for the Tidy Up task.

    The complete state machine for the tidy up task is returned
    from this function.

    Args:
        action_dict: The dictionary from action server client names
                     to action server clients

    Returns:
        sm: The complete state machine
    """

    # Initialise global store
    global_store = {}
    # (item name, current item location, item goal location, drawer number)
    # If drawer number is -1 this means we don't need to use a drawer
    global_store['current_item'] = ('None', Pose(), Pose(), -1)
    global_store['pick_up'] = None
    global_store['drawer_handle'] = None
    global_store['displaced_objects'] = []
    global_store['drawer_layout'] = [0, 0, 0]
    global_store['start_time'] = time.time()

    # Create the state machine
    sm = smach.StateMachine(outcomes=['TASK_SUCCESS', 'TASK_FAILURE'])

    with sm:

        # Navigate to the center of the room
        obj_1 = SOMObservation()
        # TODO: Get Marc to do add this to Semantic Map
        obj_1.type = 'tidy_up_pos_of_interest'
        fun = lambda: get_location_of_object(action_dict, obj_1, Relation(), 
                                             SOMObservation())
        smach.StateMachine.add('NavToCenterOfRoom',
                               NavToLocationState(action_dict,global_store,fun),
                               transitions={'SUCCESS': 'ShouldIContinue',
                                            'FAILURE': 'NavToCenterOfRoom',
                                            'REPEAT_FAILURE': 'TASK_FAILURE'})

        # Check if robot should continue
        smach.StateMachine.add('ShouldIContinue',
                               ShouldIContinueState(action_dict, global_store),
                               transitions={'YES':'GetLocAndCheck',
                                            'NO': 'TASK_SUCCESS'})
        
        # Get the items default location
        smach.StateMachine.add('GetLocAndCheck',
                               GetItemLocationAndCheckState(action_dict, 
                                                            global_store),
                               transitions={'NO_ACTION_REQ':'NavToObject',
                                            'ACTION_REQUIRED': 'NavToDrawer',
                                            'NO_OBJECTS':'SearchForItem'})

        #Exploring - Search for the next item
        smach.StateMachine.add('SearchForItem',
                                SearchForItemState(action_dict, global_store),
                                transitions = {'OBJECT_FOUND': 'GetLocAndCheck',
                                                'NO_OBJECTS': 'SearchForItem'})

        # Need to open a drawer, go to it
        smach.StateMachine.add('NavToDrawer',
                               NavigateState(action_dict, global_store),
                               transitions={'SUCCESS':'OpenDrawer',
                                            'FAILURE':'NavToDrawer',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})

        # Open the drawer
        # TODO: Which one?! Need to be able to decide
        smach.StateMachine.add('OpenDrawer',
                               OpenDrawerState(action_dict, global_store),
                               transitions={'SUCCESS':'NavToObject',
                                            'FAILURE':'TASK_FAILURE'})
        
        # Navigate to the object location (set in current_item)
        fun = lambda: global_store['current_item'][1]
        smach.StateMachine.add('NavToObject',
                               NavToLocationState(action_dict,global_store,fun),
                               transitions={'SUCCESS':'PickUpObject',
                                            'FAILURE':'NavToObject',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Pick up the object we wish to put away
        smach.StateMachine.add('PickUpObject',
                               PickUpObjectState(action_dict, global_store),
                               transitions={'SUCCESS':'NavToObjectLocation',
                                            'FAILURE':'TASK_FAILURE'})
        
        # Now set the nav back to the objects location and go there
        smach.StateMachine.add('NavToObjectLocation',
                               NavToLocationState(action_dict, global_store),
                               transitions={'SUCCESS':'PlaceObject',
                                            'FAILURE':'NavToObjectLocaion',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Place the object down
        smach.StateMachine.add('PlaceObject',
                               PutObjectOnSurfaceState(action_dict,global_store),
                               transitions={'SUCCESS':'NeedToClose',
                                            'FAILURE':'TASK_FAILURE'})
        
        # Did the robot leave a draw open?
        smach.StateMachine.add('NeedToClose',
                               NeedToCloseState(action_dict, global_store),
                               transitions={'YES':'CloseDrawer',
                                            'NO':'GetLocAndCheck'})
        
        # Close the drawer
        smach.StateMachine.add('CloseDrawer',
                               CloseDrawerState(action_dict, global_store),
                               transitions={'SUCCESS':'GetLocAndCheck',
                                            'FAILURE':'TASK_FAILURE'})

    return sm         


def create_clients():
    """ Creates the dictionary of action clients for the clean up task. """
    action_dict = create_common_clients()

    # Open drawer
    rospy.loginfo("Starting Open Drawer Client...")
    action_dict['OpenDrawer'] = actionlib.SimpleActionClient('open_drawer',
                                                            OpenDrawerAction)
    action_dict['OpenDrawer'].wait_for_server()
    rospy.loginfo("Open Drawer Client Created...")

    # Close drawer
    rospy.loginfo("Starting Close Drawer Client...")
    action_dict['CloseDrawer'] = actionlib.SimpleActionClient('close_drawer',
                                                            CloseDrawerAction)
    action_dict['CloseDrawer'].wait_for_server()
    rospy.loginfo("Close Drawer Client Created...")

    # Pick up objects
    rospy.loginfo('Starting Pick Up Object Client...')
    action_dict['PickUpObject'] = actionlib.SimpleActionClient('pick_up_object',
                                                             PickUpObjectAction)
    action_dict['PickUpObject'].wait_for_server()
    rospy.loginfo('Pick Up Object Client Created...')

    # Put objects on surface
    rospy.loginfo('Starting Put Object On Surface Client...')
    action_dict['PutObjectOnSurface'] = \
    actionlib.SimpleActionClient('put_object_on_surface',
                                 PutObjectOnSurfaceAction)
    action_dict['PutObjectOnSurface'].wait_for_server()
    rospy.loginfo('Put Object On Surface Client Created!')

    # Spin
    rospy.loginfo('Starting Spin Client')
    action_dict['spin'] = actionlib.SimpleActionClient('spin',SpinAction)
    action_dict['spin'].wait_for_server()
    rospy.loginfo('Spin Client Created')

    return action_dict


if __name__ == '__main__':
    sm = create_state_machine(create_clients())
    sm.execute()
