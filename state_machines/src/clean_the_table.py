#!/usr/bin/env python
""" Code for the Clean The Table Task.

This file contains the state machine code for the Clean The Table task.

Author: Charlie Street
Owner: Charlie Street
"""

import rospy
import smach
import actionlib
import time

from reusable_states import * # pylint: disable=unused-wildcard-import
from set_up_clients import create_stage_2_clients
from orion_actions.msg import SOMObservation, Relation

def go_to_dishwasher(action_dict):
    """ Gets location of dishwasher. """
    obj1 = SOMObservation()
    obj1.type = 'clean_the_table_point_of_interest_dishwasher'

    return get_location_of_object(action_dict, obj1, 
                                  Relation(), SOMObservation())


def go_to_table(action_dict):
    """ Gets location of table. """
    obj1 = SOMObservation()
    obj1.type = 'clean_the_table_point_of_interest_table'

    return get_location_of_object(action_dict, obj1, 
                                  Relation(), SOMObservation())


class DecideNextItemState(ActionServiceState):
    """ Decides the next item to put in the tray. """
    def __init__(self, action_dict, global_store):
        outcomes = ['ITEM', 'NONE_LEFT']
        super(DecideNextItemState, self).__init__(action_dict=action_dict,
                                                  global_store=global_store,
                                                  outcomes=outcomes)
    
    def execute(self, userdata):
        # TODO: Fill in! and organise how to do this
        self.global_store['rel_pos'] = ('tray', 0.0, 0.0, 0.2)
        return self._outcomes[1]


def create_state_machine(action_dict):
    """ This function creates and returns the state machine for this task. """

    # Initialise global_store
    global_store = {}
    global_store['drawer_handle'] = 'dishwasher'
    global_store['furniture_door'] = 'dishwasher'

    # Create the state machine
    sm = smach.StateMachine(outcomes=['TASK_SUCCESS', 'TASK_FAILURE'])

    with sm:
        
        # Store initial location
        smach.StateMachine.add('StoreLocation',
                               GetRobotLocationState(action_dict, global_store),
                               transitions={'STORED':'StartTalking'})
        
        # Start Talking
        phrase = "Hi, I'm Bam Bam, lets tidy up!"
        smach.StateMachine.add('StartTalking',
                               SpeakState(action_dict, global_store, phrase),
                               transitions={'SUCCESS':'SetNavToDishwasher',
                                            'FAILURE':'SetNavtoDishwasher'})
        
        # Set nav to dishwasher
        func = lambda : go_to_dishwasher(action_dict)
        smach.StateMachine.add('SetNavToDishwasher',
                               SetNavGoalState(action_dict, global_store, func),
                               transitions={'SUCCESS':'NavToDishwasher'})
        
        # Nav to dishwasher
        smach.StateMachine.add('NavToDishwasher',
                               NavigateState(action_dict, global_store),
                               transitions={'SUCCESS':'OpenDishwasher',
                                            'FAILURE':'NavToDishwasher',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Open dishwasher door
        smach.StateMachine.add('OpenDishwasher',
                               OpenFurnitureDoorState(action_dict,global_store),
                               transitions={'SUCCESS':'OpenRacks',
                                            'FAILURE':'AskForDishwasherHelp'})
        
        # Ask for help opening dishwasher
        question = ("Can someone open the dishwasher for me please and tell me"+
                   " when it is open?")
        smach.StateMachine.add('AskForDishwasherHelp',
                               SpeakAndListenState(action_dict, 
                                                   global_store,
                                                   question,
                                                   ['open'],
                                                   [],
                                                   30),
                               transitions={'SUCCESS':'OpenRacks',
                                            'FAILURE':'AskForDishwasherHelp',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Open dishwasher racks
        smach.StateMachine.add('OpenRacks',
                               OpenDrawerState(action_dict, global_store),
                               transitions={'SUCCESS':'SetPickUpTray',
                                            'FAILURE':'AskForRackHelp'})
        
        # Ask for help opening dishwasher racks
        question = ("Can someone open the dishwasher racks for me please " +
                    "and tell me when they are open?")
        smach.StateMachine.add('AskForRackHelp',
                               SpeakAndListenState(action_dict, 
                                                   global_store,
                                                   question,
                                                   ['open'],
                                                   [],
                                                   30),
                               transitions={'SUCCESS':'SetPickUpTray',
                                            'FAILURE':'AskForRackHelp',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # set pick up to tray
        smach.StateMachine.add('SetPickUpTray',
                               SetPickupState(action_dict, 
                                              global_store, 
                                              'tray'),
                               transitions={'SUCCESS':'PickUpTray'})
        
        # Pick up the tray
        smach.StateMachine.add('PickUpTray',
                               PickUpObjectState(action_dict, global_store),
                               transitions={'SUCCESS':'SetNavToTable',
                                            'FAILURE':'AskForTrayHelp'})
        
        # Ask for help picking up tray
        question = ("Can someone please hand me the dishwasher tray and " +
                    "let me know when they're ready to hand it over?")
        smach.StateMachine.add('AskForTrayHelp',
                               SpeakAndListenState(action_dict, 
                                                   global_store,
                                                   question,
                                                   READY,
                                                   [],
                                                   30),
                               transitions={'SUCCESS':'ReceiveTray',
                                            'FAILURE':'AskForTrayHelp',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Receive tray
        smach.StateMachine.add('ReceiveTray',
                               ReceiveObjectFromOperatorState(action_dict,
                                                              global_store),
                               transitions={'SUCCESS':'SetNavToTable',
                                            'FAILURE':'AskForTrayHelp'})
        
        # Set nav to table
        func = lambda : go_to_table(action_dict)
        smach.StateMachine.add('SetNavToTable',
                               SetNavGoalState(action_dict, global_store, func),
                               transitions={'SUCCESS':'NavToTable'})
        
        # Nav to table
        smach.StateMachine.add('NavToTable',
                               NavigateState(action_dict, global_store),
                               transitions={'SUCCESS':'PlaceTray',
                                            'FAILURE':'NavToTable',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Try placing tray down
        smach.StateMachine.add('PlaceTray',
                               PutObjectOnSurfaceState(action_dict, 
                                                       global_store),
                               transitions={'SUCCESS':'DecideNextItem',
                                            'FAILURE':'AskForTrayPlaceHelp'})
        
        # Ask for help picking up tray
        question = ("Can someone please take the tray from me? If so, can you" +
                    "let me know when you're ready to hand it over?")
        smach.StateMachine.add('AskForTrayPlaceHelp',
                               SpeakAndListenState(action_dict, 
                                                   global_store,
                                                   question,
                                                   READY,
                                                   [],
                                                   30),
                               transitions={'SUCCESS':'HandoverTray',
                                            'FAILURE':'AskForTrayPlaceHelp',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Handover tray
        smach.StateMachine.add('HandoverTray',
                               HandoverObjectToOperatorState(action_dict,
                                                             global_store),
                               transitions={'SUCCESS':'DecideNextItem',
                                            'FAILURE':'AskForTrayPlaceHelp'})
        
        # Decide next item
        smach.StateMachine.add('DecideNextItem',
                               DecideNextItemState(action_dict, global_store),
                               transitions={'ITEM':'PickupItem',
                                            'NONE_LEFT':'SetTakeTray'})
        
        # Pick up an item
        smach.StateMachine.add('PickupItem',
                               PickUpObjectState(action_dict, global_store),
                               transitions={'SUCCESS':'PlaceInTray',
                                            'FAILURE':'AskForHelpPickupItem'})
        
        # Ask for help picking up item
        question = ("Can someone please help me pick this up? If so, can you" +
                    "let me know when you're ready to hand it over?")
        smach.StateMachine.add('AskForHelpPickupItem',
                               SpeakAndListenState(action_dict, 
                                                   global_store,
                                                   question,
                                                   READY,
                                                   [],
                                                   30),
                               transitions={'SUCCESS':'ReceiveItem',
                                            'FAILURE':'AskForHelpPickupItem',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Receive item
        smach.StateMachine.add('ReceiveItem',
                               ReceiveObjectFromOperatorState(action_dict,
                                                              global_store),
                               transitions={'SUCCESS':'PlaceInTray',
                                            'FAILURE':'AskForHelpPickupItem'})
        
        # Place item in tray
        smach.StateMachine.add('PlaceInTray',
                               PlaceObjectRelativeState(action_dict, 
                                                        global_store),
                               transitions={'SUCCESS':'DecideNextItem',
                                            'FAILURE':'AskForHelpItemPlace'})
        
        # Ask for help putting item in tray
        question = ("Can someone please put this in the tray for me? If so, " +
                    "can you let me know when you're ready to hand over?")
        smach.StateMachine.add('AskForHelpItemPlace',
                               SpeakAndListenState(action_dict, 
                                                   global_store,
                                                   question,
                                                   READY,
                                                   [],
                                                   30),
                               transitions={'SUCCESS':'HandoverItem',
                                            'FAILURE':'AskForHelpItemPlace',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Handover item
        smach.StateMachine.add('HandoverItem',
                               HandoverObjectToOperatorState(action_dict,
                                                             global_store),
                               transitions={'SUCCESS':'DecideNextItem',
                                            'FAILURE':'AskForHelpItemPlace'})
        
        # Set pick up to tray
        smach.StateMachine.add('SetTakeTray',
                               SetPickupState(action_dict, global_store,'tray'),
                               transitions={'SUCCESS': 'TakeTray'})

        # Take the tray and run!
        smach.StateMachine.add('TakeTray',
                               PickUpObjectState(action_dict, global_store),
                               transitions={'SUCCESS':'SetNavBackToDishwasher',
                                            'FAILURE':'AskForHelpTakeTray'})
        
        # Ask for help picking up tray
        question = ("Can someone please help me pick up the tray? If so, " +
                    "can you let me know when you're ready to hand it over?")
        smach.StateMachine.add('AskForHelpTakeTray',
                               SpeakAndListenState(action_dict, 
                                                   global_store,
                                                   question,
                                                   READY,
                                                   [],
                                                   30),
                               transitions={'SUCCESS':'ReceiveTakeTray',
                                            'FAILURE':'AskForHelpTakeTray',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Receive tray
        smach.StateMachine.add('ReceiveTakeTray',
                               ReceiveObjectFromOperatorState(action_dict,
                                                              global_store),
                               transitions={'SUCCESS':'SetNavBackToDishwasher',
                                            'FAILURE':'AskForHelpTakeTray'})
        
        # Set nav back to dishwasher
        func = lambda : go_to_dishwasher(action_dict)
        smach.StateMachine.add('SetNavBackToDishwasher',
                               SetNavGoalState(action_dict, global_store, func),
                               transitions={'SUCCESS':'NavBackToDishwasher'})
        
        # Nav back to dishwasher
        smach.StateMachine.add('NavBackToDishwasher',
                               NavigateState(action_dict, global_store),
                               transitions={'SUCCESS':'PutTrayInDishwasher',
                                            'FAILURE':'NavBackToDishwasher',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Try to put tray back in dishwasher
        smach.StateMachine.add('PutTrayInDishwasher',
                               PutObjectOnSurfaceState(action_dict, 
                                                       global_store),
                               transitions={'SUCCESS':'FinishSpeak',
                                            'FAILURE':'AskForHelpTrayPlace'})
        
        # Ask for help putting tray in dishwasher
        question = ("Can someone please put the tray in the dishwasher? If " +
                    "so, can you let me know when you're ready to hand over?")
        smach.StateMachine.add('AskForHelpTrayPlace',
                               SpeakAndListenState(action_dict, 
                                                   global_store,
                                                   question,
                                                   READY,
                                                   [],
                                                   30),
                               transitions={'SUCCESS':'HandoverTrayToPlace',
                                            'FAILURE':'AskForHelpTrayPlace',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Handover tray
        smach.StateMachine.add('HandoverTrayToPlace',
                               HandoverObjectToOperatorState(action_dict,
                                                             global_store),
                               transitions={'SUCCESS':'FinishSpeak',
                                            'FAILURE':'AskForHelpTrayPlace'})
        
        # Finish speech
        phrase = "Looks like my job here is done! See ya!"
        smach.StateMachine.add('FinishSpeak',
                               SpeakState(action_dict, global_store, phrase),
                               transitions={'SUCCESS':'SetNavBackToStart',
                                            'FAILURE':'SetNavBackToStart'})
        
        # Set nav back to location
        func = lambda : global_store['stored_location']
        smach.StateMachine.add('SetNavBackToStart',
                               SetNavGoalState(action_dict, global_store, func),
                               transitions={'SUCCESS':'NavBackToStart'})
        
        # Nav back to start
        smach.StateMachine.add('NavBackToStart',
                               NavigateState(action_dict, global_store),
                               transitions={'SUCCESS':'TASK_SUCCESS',
                                            'FAILURE':'NavBackToStart',
                                            'TASK_FAILURE':'TASK_FAILURE'})

    return sm

if __name__ == '__main__':
    action_dict = create_stage_2_clients(1)
    sm = create_state_machine(action_dict)
    sm.execute()