#!/usr/bin/env python
""" File for carry my luggage task for stage 1 of RoboCup@home 2019.

This file contains the state machine for the carry my luggage task
of robocup@home 2019.

Author: Charlie Street
Owner: Charlie Street
"""

import rospy
import smach
import actionlib

from reusable_states import * # pylint: disable=unused-wildcard-import
from set_up_clients import create_stage_1_clients


def create_state_machine(action_dict):
    """ This function builds the state machine for the carry my luggage task.

    The complete state machine for the carry my luggage task is returned
    from this function.

    Args:
        action_dict: The dictionary from action server client names
                     to action server clients

    Returns:
        sm: The complete state machine
    """

    # Initialise global store
    global_store = {}

    # Create the state machine
    sm = smach.StateMachine(outcomes=['TASK_SUCCESS', 'TASK_FAILURE'])

    with sm:

        # Store initial location
        smach.StateMachine.add('StoreInitialLocation',
                               GetRobotLocationState(action_dict, global_store),
                               transitions={'STORED':'StartTalking'})
        
        # Start talking to the operator
        question = ("Hi, I'm Bam-Bam." + 
                   "Is there an operator nearby who needs my help? If so, " +
                   "please tell me your name and point at the luggage you " +
                   "want me to carry!")
        smach.StateMachine.add('StartTalking',
                               SpeakAndListenState(action_dict,
                                                   global_store,
                                                   question,
                                                   NAMES,
                                                   [],
                                                   20),
                               transitions={'SUCCESS': 'DetectOperator',
                                            'FAILURE':'StartTalking',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Detect and memorise the operator
        smach.StateMachine.add('DetectOperator',
                               OperatorDetectState(action_dict, global_store),
                               transitions={'SUCCESS': 'PickUpLuggage',
                                            'FAILURE': 'StartTalking'})
        
        # Find and pick up the luggage
        smach.StateMachine.add('PickUpLuggage',
                               PickUpPointedObject(action_dict, global_store),
                               transitions={'SUCCESS': 'ConfirmPickup',
                                            'FAILURE': 'AskForHelp'})
        
        # Talk to the operator pre-follow
        phrase = ("I've picked up the luggage. I'm ready to follow you " +
                 "whenever you're ready! When you want me to stop following, " +
                 "please say stop.")
        # Regardless of failure, keep pushing on
        smach.StateMachine.add('ConfirmPickup',
                               SpeakState(action_dict, global_store, phrase),
                               transitions={'SUCCESS':'Follow',
                                            'FAILURE':'Follow'})
        
        # Ask for help!
        question = ("It looks like I'm struggling to pick up your luggage. " + 
                   "Could you hand it to me please and let me know " +
                   "when you're ready to hand it over?")
        smach.StateMachine.add('AskForHelp',
                               SpeakAndListenState(action_dict, 
                                                   global_store,
                                                   question, 
                                                   READY,
                                                   [],
                                                   20),
                               transitions={'SUCCESS':'HandoverLuggage',
                                            'FAILURE':'AskForHelp',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})

        # If help offered, handover from operator
        smach.StateMachine.add('HandoverLuggage',
                               ReceiveObjectFromOperatorState(action_dict,
                                                              global_store),
                               transitions={'SUCCESS':'ConfirmHandover',
                                            'FAILURE':'TASK_FAILURE'})
        
        # Thank operator for handover
        phrase = ("Thank you so much! I'm ready to follow you " + 
                 "whenever you're ready! When you want me to stop following, " +
                 "please say my name.")
        smach.StateMachine.add('ConfirmHandover',
                               SpeakState(action_dict, global_store, phrase),
                               transitions={'SUCCESS': 'Follow',
                                            'FAILURE': 'Follow'})
        
        # Do following stuff here
        smach.StateMachine.add('Follow',
                                make_follow_hotword_state(action_dict, 
                                                          global_store),
                                transitions={'SUCCESS': 'ArrivalQuestion',
                                             'FAILURE': 'Follow',
                                             'REPEAT_FAILURE': 'TASK_FAILURE'})

        # Ask if OK for handover
        question = ("Yay, we've arrived! Please tell me when you're ready "
                    "for me to give you your luggage?")
        smach.StateMachine.add('ArrivalQuestion',
                               SpeakAndListenState(action_dict, 
                                                   global_store, 
                                                   question, 
                                                   READY,
                                                   [],
                                                   20),
                                transitions={'SUCCESS':'GiveLuggageBack',
                                             'FAILURE':'ArrivalQuestion',
                                             'REPEAT_FAILURE':'TASK_FAILURE'})

        # Hand back luggage
        smach.StateMachine.add('GiveLuggageBack',
                               HandoverObjectToOperatorState(action_dict,
                                                             global_store),
                               transitions={'SUCCESS': 'ThankOperator',
                                            'FAILURE': 'TASK_FAILURE'})

        # Thank the operator!
        phrase = ("It looks like my job here is done. Thank you for " +
                  "bearing with me, have a nice day!")
        smach.StateMachine.add('ThankOperator',
                               SpeakState(action_dict, global_store, phrase),
                               transitions={'SUCCESS':'SetNavToStart',
                                            'FAILURE':'SetNavToStart'})
        
        
        # Set Nav Back To Start
        func = lambda : global_store['stored_location']  
        smach.StateMachine.add('SetNavToStart',
                               SetNavGoalState(action_dict, global_store, func),
                               transitions={'SUCCESS':'NavToStart'}) 

        # Navigate back to the start!
        smach.StateMachine.add('NavToStart',
                               NavigateState(action_dict, global_store),
                               transitions={'SUCCESS': 'Finish',
                                            'FAILURE': 'NavToStart',
                                            'REPEAT_FAILURE': 'TASK_FAILURE'})

        # End the task
        phrase = "I've finished, woohoo!"
        smach.StateMachine.add('Finish',
                               SpeakState(action_dict, global_store, phrase),
                               transitions={'SUCCESS':'TASK_SUCCESS',
                                            'FAILURE':'TASK_FAILURE'})       

    return sm


if __name__ == '__main__':
    action_dict = create_stage_1_clients(1)
    sm = create_state_machine(action_dict)
    sm.execute()