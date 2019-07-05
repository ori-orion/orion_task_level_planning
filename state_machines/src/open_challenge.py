#!/usr/bin/env python
""" File for Open Challenge.

This file contains the state machine and states specific to the 
Open Challenge a.k.a. The Duck Task

Author: Charlie Street and Mia Mijovic
Owner: Charlie Street and Mia Mijovic
"""

import rospy
import smach
import actionlib

from reusable_states import * # pylint: disable=unused-wildcard-import
from set_up_clients import create_stage_1_clients
from orion_actions.msg import SOMObservation, Relation, PickUpBinBagGoal, \
    OpenBinLidGoal
from geometry_msgs.msg import Point, Quaternion

class OpenBinState(ActionServiceState):
    """ State for opening a bin lid."""

    def __init__(self, action_dict, global_store):
        outcomes = ['SUCCESS', 'FAILURE']
        super(OpenBinState, self).__init__(action_dict=action_dict, 
                                           global_store=global_store, 
                                           outcomes=outcomes)

    def execute(self, userdata):
        bin_lid_goal = OpenBinLidGoal()
        self.action_dict['OpenBinLid'].send_goal(bin_lid_goal)
        self.action_dict['OpenBinLid'].wait_for_result()

        result = self.action_dict['OpenBinLid'].get_result().result
        self.global_store['pick_up'] = 'garbage bag'
        if result:
            return self._outcomes[0]
        else:
            return self._outcomes[1]


def create_state_machine(action_dict):
    """ Creates the state machine to be used for the task.

    Args:
        action_dict: The dictionary from names to action clients for the task
    
    Returns:
        sm: The state machine to be executed
    """

    #Initialise global store
    global_store = {}
    global_store['people_found'] = []


    # Create the state machine
    sm = smach.StateMachine(outcomes=['TASK_SUCCESS', 'TASK_FAILURE'])


    with sm:

        # Opening speach
        phrase = "I'm here for the open challenge. I warn you, I like ducks"
        smach.StateMachine.add('OpenSpeach',
                                SpeakState(action_dict, global_store, phrase),
                                transitions={'SUCCESS':'IsDoorOpen',
                                             'FAILURE': 'IsDoorOpen'})

        # Check if door is open
        smach.StateMachine.add('IsDoorOpen', 
                               CheckDoorIsOpenState(action_dict, global_store),
                               transitions={'OPEN':'SetNavToTable',
                                            'CLOSED':'IsDoorOpen'})

        # Set navigation go to table
        func = lambda:None #TODO
        smach.StateMachine.add('SetNavToTable',
                                SetNavGoalState(action_dict, global_store, func),
                                transitions={'SUCCESS':'NavToTable'})

        # Navigate to the table
        smach.StateMachine.add('NavToTable',
                                NavigateState(action_dict, global_store),
                                transitions={'SUCCESS':'AskName',
                                            'FAILURE': 'NavToTable',
                                            'REPEAT_FAILURE': 'OldTownRoad'})

         # Start talking to the operator
        question = ("Hi, I'm Bam-Bam." + 
                   "Could you please" +
                   "tell me your name")
        smach.StateMachine.add('AskName',
                               SpeakAndListenState(action_dict,
                                                   global_store,
                                                   question,
                                                   NAMES,
                                                   [],
                                                   20),
                               transitions={'SUCCESS': 'DetectOperator',
                                            'FAILURE':'AskName',
                                            'REPEAT_FAILURE':'OldTownRoad'})
        
        # Detect and memorise the operator
        smach.StateMachine.add('DetectOperator',
                               OperatorDetectState(action_dict, global_store),
                               transitions={'SUCCESS': 'PleaseShowObj',
                                            'FAILURE': 'AskName'})

        # Opening speach
        phrase = "Operator, please, could you show me some objects?"
        smach.StateMachine.add('PleaseShowObj',
                                SpeakState(action_dict, global_store, phrase),
                                transitions={'SUCCESS':'DetectObj',
                                             'FAILURE': 'DetectObj'})

        # Detection of objects TODO
        smach.StateMachine.add('DetectObj',
                               DetectObjectsState(action_dict, global_store),
                               transitions={'SUCCESS':'ReciteObj',
                                            'FAILURE':'PleaseShowObj'})

        # Recite the objects TODO
        smach.StateMachine.add('ReciteObj',
                                ReciteObjectsState(action_dict, global_store),
                                transitions={'SUCCESS':'DuckHotWord',
                                             'FAILURE':'DuckHotWord'})

        # Duck Hot Word
        smach.StateMachine.add('DuckHotWord',
                                HotwordListenState(action_dict, global_store,
                                                    ['duck'], 20),
                                transitions={'SUCCESS':'PlaySong',
                                             'FAILURE':'DuckHotWord'})

        #Play Duck Song TODO
        smach.StateMachine.add('PlaySong',
                               DuckSongState(action_dict, global_store),
                               transitions={'SUCCESS':'WhatNext'})

        # Ask what next?
        question = ("What object would you like me to put in the bin?")
        smach.StateMachine.add('WhatNext',
                               SpeakAndListenState(action_dict,
                                                   global_store,
                                                   question,
                                                   ['first', 'second', 'third'],
                                                   [],
                                                   20),
                               transitions={'SUCCESS': 'SpeakCool',
                                            'FAILURE':'WhatNext',
                                            'REPEAT_FAILURE':'OldTownRoad'})

        # Cool!
        phrase = "Cool! See you shortly"
        smach.StateMachine.add('SpeakCool',
                                SpeakState(action_dict, global_store, phrase),
                                transitions={'SUCCESS':'SetNavToBin',
                                            'FAILURE': 'SetNavToBin'})

        # Set nav to bin
        func = lambda:None #TODO
        smach.StateMachine.add('SetNavToBin',
                                SetNavGoalState(action_dict, global_store, func),
                                transitions={'SUCCESS':'NavToBin'})

        # Navigate to bin
        smach.StateMachine.add('NavToBin',
                                NavigateState(action_dict, global_store),
                                transitions={'SUCCESS':'OpenBinLid',
                                            'FAILURE': 'NavToBin',
                                            'REPEAT_FAILURE': 'OldTownRoad'})

        # Open Bin Lid
        smach.StateMachine.add('OpenBinLid',
                               OpenBinState(action_dict, global_store),
                               transitions={'SUCCESS':'SetNavBackToTable',
                                            'FAILURE':'SetNavBackToTable'})

        # Set Nav Back To Table
        func = lambda:None #TODO
        smach.StateMachine.add('SetNavBackToTable',
                                SetNavGoalState(action_dict, global_store, func),
                                transitions={'SUCCESS':'NavBackToTable'})

        # Nav Back To Table
        smach.StateMachine.add('NavBackToTable',
                                NavigateState(action_dict, global_store),
                                transitions={'SUCCESS':'SetPickUp',
                                            'FAILURE': 'NavBackToTable',
                                            'REPEAT_FAILURE': 'OldTownRoad'})

        # Set Pick Up
        obj = None # TODO
        smach.StateMachine.add('SetPickUp',
                               SetPickupState(action_dict, global_store, obj),
                               transitions={'SUCCESS':'PickUp'})

        # Pick Up
        smach.StateMachine.add('PickUp',
                               PickUpObjectState(action_dict, global_store),
                               transitions={'SUCCESS':'SetNavBackToBin',
                                            'FAILURE':'SetNavBackToBin'})

        # Set Nav Back To Bin
        func = lambda:None #TODO
        smach.StateMachine.add('SetNavBackToBin',
                                SetNavGoalState(action_dict, global_store, func),
                                transitions={'SUCCESS':'NavBackToBin'})

        # Nav Back To Bin
        smach.StateMachine.add('NavBackToBin',
                                NavigateState(action_dict, global_store),
                                transitions={'SUCCESS':'PutInBin',
                                            'FAILURE': 'NavBackToBin',
                                            'REPEAT_FAILURE': 'OldTownRoad'})

        # Put in Bin TODO
        smach.StateMachine.add('PutInBin',
                               PutInBinState(action_dict, global_store),
                               transitions={'SUCCESS':'SetNavToDoor',
                                            'FAILURE':'SetNavToDoor'})

        # Set Nav to Door
        func = lambda:None #TODO
        smach.StateMachine.add('SetNavToDoor',
                                SetNavGoalState(action_dict, global_store, func),
                                transitions={'SUCCESS':'NavToDoor'})

        # Nav To Door
        smach.StateMachine.add('NavToDoor',
                                NavigateState(action_dict, global_store),
                                transitions={'SUCCESS':'OpenDoor',
                                            'FAILURE': 'NavToDoor',
                                            'REPEAT_FAILURE': 'OldTownRoad'})

        # Open Door TODO
        smach.StateMachine.add('OpenDoor',
                               OpenDoorState(action_dict, global_store),
                               transitions={'SUCCESS':'SetNavOut',
                                            'FAILURE':'OldTownRoad'})
        
        # Set Nav Out
        func = lambda:None #TODO
        smach.StateMachine.add('SetNavOut',
                                SetNavGoalState(action_dict, global_store, func),
                                transitions={'SUCCESS':'NavOut'})

        # Nav To Door
        smach.StateMachine.add('NavOut',
                                NavigateState(action_dict, global_store),
                                transitions={'SUCCESS':'OldTownRoad',
                                            'FAILURE': 'NavToDoor',
                                            'REPEAT_FAILURE': 'OldTownRoad'})

        #I've got my horses in the bacc TODO
        smach.StateMachine.add('OldTownRoad',
                               OldTownState(action_dict, global_store),
                               transitions={'SUCCESS':'TASK_SUCCESS'})
        

