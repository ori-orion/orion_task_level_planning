#!/usr/bin/env python
""" Code for the Storing Groceries task.

This file contains the state machine code for the Storing Groceries task.

Author: Charlie Street
Owner: Charlie Street
"""

import rospy
import smach
import actionlib
import time

from reusable_states import * # pylint: disable=unused-wildcard-import
from set_up_clients import create_stage_1_clients
from orion_actions.msg import SOMObservation, Relation


class FindHandleState(ActionServiceState):
    """ State for finding the handle of a shelf in front of us. """
    
    def __init__(self, action_dict, global_store):
        outcomes = ['SUCCESS', 'FAILURE']
        super(FindHandleState, self).__init__(action_dict=action_dict,
                                              global_store=global_store,
                                              outcomes=outcomes)
    
    def execute(self, userdata):
        self.global_store['drawer_handle'] = 'cupboard'
        return self._outcomes[0]


class PickUpClosestItemState(ActionServiceState):
    """ State for picking up the closest item to us. """
    
    def __init__(self, action_dict, global_store):
        outcomes = ['SUCCESS', 'FAILURE']
        super(PickUpClosestItemState, self).__init__(action_dict=action_dict,
                                                     global_store=global_store,
                                                     outcomes=outcomes)
    
    def execute(self, userdata):
        # TODO: Fill In!
        # This should as well as picking up, store the info of the item too
        pass


class DecideItemPositionState(ActionServiceState):
    """ State for deciding the position of the item in our hand """
    
    def __init__(self, action_dict, global_store):
        outcomes = ['SUCCESS', 'FAILURE']
        super(DecideItemPositionState, self).__init__(action_dict=action_dict,
                                                      global_store=global_store,
                                                      outcomes=outcomes)
    
    def execute(self, userdata):
        # TODO: Fill In!
        # This should set the relative pos where the item should go
        # on the shelf, and also update our current state of the shelf
        # for future items being placed
        pass


class UpdateItemInfoState(ActionServiceState):
    """ State for updating the info of the object we hold given help. """
    
    def __init__(self, action_dict, global_store):
        outcomes = ['SUCCESS', 'FAILURE']
        super(UpdateItemInfoState, self).__init__(action_dict=action_dict,
                                                  global_store=global_store,
                                                  outcomes=outcomes)
    
    def execute(self, userdata):
        # TODO: Fill In!
        # This should parse the speech we have received to get the info
        # This should set the relative pos where the item should go
        # on the shelf, and also update our current state of the shelf
        # for future items being placed
        pass


def go_to_shelf(action_dict):
    """ Set nav goal to shelf. Announced pre-task. """
    obj = SOMObservation()
    obj.type = 'storing_groceries_point_of_interest_shelf'

    return get_location_of_object(action_dict, obj, 
                                  Relation(), SOMObservation())

def go_to_table(action_dict):
    """ Set nav goal to table. Announced pre-task. """
    obj = SOMObservation()
    obj.type = 'storing_groceries_point_of_interest_table'

    return get_location_of_object(action_dict, obj, 
                                  Relation(), SOMObservation())


def create_state_machine(action_dict):
    """ This function creates and returns the state machine for this task. """

    # Initialise global store
    global_store = {}

    # Create the state machine
    sm = smach.StateMachine(outcomes=['TASK_SUCCESS', 'TASK_FAILURE'])

    with sm:
        
        # Start talking
        phrase = "Hi, I'm Bam Bam and its time to store some groceries!"
        smach.StateMachine.add('StartTalking',
                               SpeakState(action_dict, global_store, phrase),
                               transitions={'SUCCESS':'WaitForDoor',
                                            'FAILURE':'WaitForDoor'})
        
        # Wait for the door to be opened
        smach.StateMachine.add('WaitForDoor',
                               CheckDoorIsOpenState(action_dict, global_store),
                               transitions={'OPEN':'SetNavToShelf',
                                            'CLOSED':'WaitForDoor'})
        
        # Set the navigation goal to the shelf
        func = lambda : go_to_shelf(action_dict)
        smach.StateMachine.add('SetNavToShelf',
                               SetNavGoalState(action_dict, global_store, func),
                               transitions={'SUCCESS':'NavToShelf'})
        
        # Navigate to the shelf
        smach.StateMachine.add('NavToShelf',
                               NavigateState(action_dict, global_store),
                               transitions={'SUCCESS':'FindShelfHandle',
                                            'FAILURE':'NavToShelf',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Find shelf handle
        smach.StateMachine.add('FindShelfHandle',
                               FindHandleState(action_dict, global_store),
                               transitions={'SUCCESS':'OpenShelf',
                                            'FAILURE':'AskForShelfHelp'})
    
        # Open the shelf
        smach.StateMachine.add('OpenShelf',
                               OpenDrawerState(action_dict, global_store),
                               transitions={'SUCCESS':'SetNavToTable',
                                            'FAILURE':'AskForShelfHelp'})
        
        # Ask for help with the shelf
        question = ("Can someone open the shelf for me please? If you can, " +
                   "please say when you have opened it.")
        smach.StateMachine.add('AskForShelfHelp',
                               SpeakAndListenState(action_dict,
                                                   global_store,
                                                   question,
                                                   ['Opened'],
                                                   [],
                                                   30),
                               transitions={'SUCCESS':'SetNavToTable',
                                            'FAILURE':'AskForShelfHelp',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Set the navigation to the table
        func = lambda: go_to_table(action_dict)
        smach.StateMachine.add('SetNavToTable',
                               SetNavGoalState(action_dict, global_store, func),
                               transitions={'SUCCESS':'NavToTable'})
        
        # Navigate to the table
        smach.StateMachine.add('NavToTable',
                               NavigateState(action_dict, global_store),
                               transitions={'SUCCESS':'PickUpClosestItem',
                                            'FAILURE':'NavToTable',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Pick up the closest item
        smach.StateMachine.add('PickUpClosestItem',
                               PickUpClosestItemState(action_dict,global_store),
                               transitions={'SUCCESS':'DecideItemPosition',
                                            'FAILURE':'AskForPickupHelp',
                                            'NO_ITEMS':'TASK_SUCCESS'})
        
        # Ask for help
        question = ("Could someone please put the item closest to me in my " +
                    "hand please?")
        smach.StateMachine.add('AskForPickupHelp',
                               SpeakAndListenState(action_dict,
                                                   global_store,
                                                   question,
                                                   ['I will'],
                                                   [],
                                                   30),
                               transitions={'SUCCESS':'ReceiveItem',
                                            'FAILURE':'AskForPickupHelp',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Receive the item from an operator
        smach.StateMachine.add('ReceiveItem',
                               ReceiveObjectFromOperatorState(action_dict,
                                                              global_store),
                               transitions={'SUCCESS':'DecideItemPosition',
                                            'FAILURE':'AskForPickupHelp'})
        
        # Decide the items position on the shelf
        smach.StateMachine.add('DecideItemPosition',
                               DecideItemPositionState(action_dict,
                                                       global_store),
                               transitions={'SUCCESS':'SetNavBackToShelf',
                                            'FAILURE':'AskForPosHelp'})
        

        # Ask for help with position
        question = ("I don't know where to put this object. Can someone " +
                    "tell me where I should put it?")
        smach.StateMachine.add('AskForPosHelp',
                               SpeakAndListenState(action_dict,
                                                   global_store,
                                                   question,
                                                   ['i will'],
                                                   [],
                                                   20),
                               transitions={'SUCCESS':'UpdateItemInfo',
                                            'FAILURE':'AskForPosHelp',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
            
        # Update item information
        smach.StateMachine.add('UpdateItemInfo',
                               UpdateItemInfoState(action_dict, global_store),
                               transitions={'SUCCESS':'SetNavBackToShelf',
                                            'FAILURE':'AskForPosHelp'})

        # Set navigation back to shelf
        func = lambda : go_to_shelf(action_dict)
        smach.StateMachine.add('SetNavBackToShelf',
                               SetNavGoalState(action_dict, global_store, func),
                               transitions={'SUCCESS':'NavBackToShelf'})
        
        # Go back to the shelf
        smach.StateMachine.add('NavBackToShelf',
                               NavigateState(action_dict, global_store),
                               transitions={'SUCCESS':'PlaceObject',
                                            'FAILURE':'NavBackToShelf',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Place object relatively on shelf
        smach.StateMachine.add('PlaceObject',
                               PlaceObjectRelativeState(action_dict, 
                                                        global_store),
                               transitions={'SUCCESS':'SetNavToTable',
                                            'FAILURE':'AskForPlacementHelp'})

        # Ask for help with placing the object
        question = ("I'm having trouble putting this object on the shelf. " +
                    "Could you help me please?")
        smach.StateMachine.add('AskForPlacementHelp',
                               SpeakAndListenState(action_dict,
                                                   global_store,
                                                   question,
                                                   ['I will'],
                                                   [],
                                                   20),
                               transitions={'SUCCESS':'HandoverObject',
                                            'FAILURE':'AskForPlacementHelp',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Handover object to operator
        smach.StateMachine.add('HandoverObject',
                               HandoverObjectToOperatorState(action_dict,
                                                             global_store),
                               transitions={'SUCCESS':'SetNavToTable',
                                            'FAILURE':'AskForPlacementHelp'})

    return sm

if __name__ == '__main__':
    action_dict = create_stage_1_clients(9)
    sm = create_state_machine(action_dict)
    sm.execute()