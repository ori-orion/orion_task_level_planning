#!/usr/bin/env python3
""" File for Open Challenge.

This file contains the state machine and states specific to the 
Open Challenge a.k.a. The Duck Task

Author: Charlie Street and Mia Mijovic
Owner: Charlie Street and Mia Mijovic
"""

import rospy
import smach
import actionlib
import time

from pygame import mixer

from orion_task_level_planning.state_machines.src.state_machines.reusable_states_deprecated import * # pylint: disable=unused-wildcard-import
from set_up_clients import create_open_clients
from orion_actions.msg import SOMObservation, Relation, PickUpBinBagGoal, \
    OpenBinLidGoal, PutObjectInBinGoal, MemorizeGoal
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


class DuckSongState(ActionServiceState):

    def __init__(self, action_dict, global_store):
        outcomes = ['SUCCESS']
        super(DuckSongState, self).__init__(action_dict=action_dict, 
                                            global_store=global_store, 
                                            outcomes=outcomes)
    
    def execute(self, userdata):
        mixer.init()
        mixer.music.load('../sound/birdie_song.mp3')
        mixer.music.play()
        time.sleep(20)
        mixer.music.stop()

        return self._outcomes[0]


class DetectObjectsState(ActionServiceState):

    def __init__(self, action_dict, global_store):
        outcomes = ['SUCCESS', 'FAILURE']
        super(DetectObjectsState, self).__init__(action_dict=action_dict, 
                                                 global_store=global_store, 
                                                 outcomes=outcomes)
    
    def execute(self, userdata):
        
        goal = MemorizeGoal()
        self.action_dict['Memorize'].send_goal(goal)
        self.action_dict['Memorize'].wait_for_result()
        self.global_store['shown_objects'] = self.action_dict['Memorize'].get_result().objects

        return self._outcomes[0]


class ReciteObjectsState(ActionServiceState):

    def __init__(self, action_dict, global_store):
        outcomes = ['SUCCESS', 'FAILURE']
        super(ReciteObjectsState, self).__init__(action_dict=action_dict, 
                                                 global_store=global_store, 
                                                 outcomes=outcomes)
    
    def execute(self, userdata):
        objects = self.global_store['shown_objects']

        sentence = ('You showed me the ' + str(objects[0]) + ', then the ' + 
                    str(objects[1]) + ' and finally the ' + str(objects[2]))
        
        action_goal = TalkRequestGoal()
        action_goal.data.language = Voice.kEnglish
        action_goal.data.sentence = sentence
        self.action_dict['Speak'].send_goal(action_goal)
        self.action_dict['Speak'].wait_for_result()

        return self._outcomes[0]


class OldTownState(ActionServiceState):

    def __init__(self, action_dict, global_store):
        outcomes = ['SUCCESS']
        super(OldTownState, self).__init__(action_dict=action_dict, 
                                            global_store=global_store, 
                                            outcomes=outcomes)
    
    def execute(self, userdata):
        mixer.init()
        mixer.music.load('../sound/old_town_road.mp3')
        mixer.music.play()
        time.sleep(45)
        mixer.music.stop()

        return self._outcomes[0]


class GetChosenItemState(ActionServiceState):

    def __init__(self, action_dict, global_store):
        outcomes = ['SUCCESS']
        super(GetChosenItemState, self).__init__(action_dict=action_dict, 
                                                 global_store=global_store, 
                                                 outcomes=outcomes)
    

    def execute(self, userdata):
        response = self.global_store['last_response']

        if response == 'first':
            self.global_store['pick_up'] = self.global_store['shown_objects'][0]
        elif response == 'second':
            self.global_store['pick_up'] = self.global_store['shown_objects'][1]
        elif response == 'third':
            self.global_store['pick_up'] = self.global_store['shown_objects'][2]


class PutInBinState(ActionServiceState):

    def __init__(self, action_dict, global_store):
        outcomes = ['SUCCESS', 'FAILURE']
        super(PutInBinState, self).__init__(action_dict=action_dict, 
                                                 global_store=global_store, 
                                                 outcomes=outcomes)
    
    def execute(self, userdata):

        goal = PutObjectInBinGoal()
        self.action_dict['PutObjectInBin'].send_goal(goal)
        self.action_dict['PutObjectInBin'].wait_for_result()

        result = self.action_dict['PutObjectInBin'].get_result().result

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
    global_store['shown_objects'] = []
    global_store['last_response'] = None

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
        func = lambda: table_pose
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
                               transitions={'SUCCESS': 'PleaseShowObj',
                                            'FAILURE':'AskName',
                                            'REPEAT_FAILURE':'OldTownRoad'})
        

        # Opening speach
        phrase = "Operator, please, could you show me some objects?"
        smach.StateMachine.add('PleaseShowObj',
                                SpeakState(action_dict, global_store, phrase),
                                transitions={'SUCCESS':'DetectObj',
                                             'FAILURE': 'DetectObj'})

        # Detection of objects
        smach.StateMachine.add('DetectObj',
                               DetectObjectsState(action_dict, global_store),
                               transitions={'SUCCESS':'ReciteObj',
                                            'FAILURE':'PleaseShowObj'})

        # Recite the objects
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

        #Play Duck Song
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
        bin_pose = Pose()
        bin_pose.position = Point(3.52,-14.75,0.0)
        bin_pose.orientation = Quaternion(0.0, 0.0, 0.952, -0.304)
        func = lambda: bin_pose
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
        table_pose = Pose()
        table_pose.position = Point(3.67,-13.5,0.0)
        table_pose.orientation = Quaternion(0.99958, 0.00035, 0.01352, 0.02574)
        func = lambda: table_pose
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
        smach.StateMachine.add('SetPickUp',
                               GetChosenItemState(action_dict, global_store),
                               transitions={'SUCCESS':'PickUp'})

        # Pick Up
        smach.StateMachine.add('PickUp',
                               PickUpObjectState(action_dict, global_store),
                               transitions={'SUCCESS':'SetNavBackToBin',
                                            'FAILURE':'SetNavBackToBin'})

        # Set Nav Back To Bin
        func = lambda: bin_pose
        smach.StateMachine.add('SetNavBackToBin',
                                SetNavGoalState(action_dict, global_store, func),
                                transitions={'SUCCESS':'NavBackToBin'})

        # Nav Back To Bin
        smach.StateMachine.add('NavBackToBin',
                                NavigateState(action_dict, global_store),
                                transitions={'SUCCESS':'PutInBin',
                                            'FAILURE': 'NavBackToBin',
                                            'REPEAT_FAILURE': 'OldTownRoad'})

        # Put in Bin
        smach.StateMachine.add('PutInBin',
                               PutInBinState(action_dict, global_store),
                               transitions={'SUCCESS':'SetNavToDoor',
                                            'FAILURE':'SetNavToDoor'})

        # Set Nav to Door
        door_pose = Pose()
        door_pose.position = Point(3.42, -18.7, 0.0)
        door_pose.orientation = Quaternion(0.99984,0.00016,0.01365,0.01187)
        func = lambda: door_pose
        smach.StateMachine.add('SetNavToDoor',
                                SetNavGoalState(action_dict, global_store, func),
                                transitions={'SUCCESS':'NavToDoor'})

        # Nav To Door
        smach.StateMachine.add('NavToDoor',
                                NavigateState(action_dict, global_store),
                                transitions={'SUCCESS':'OpenDoor',
                                            'FAILURE': 'NavToDoor',
                                            'REPEAT_FAILURE': 'OldTownRoad'})

        # Open Door
        smach.StateMachine.add('OpenDoor',
                               CheckAndOpenDoorState(action_dict, global_store),
                               transitions={'SUCCESS':'SetNavOut',
                                            'FAILURE':'OldTownRoad'})
        
        # Set Nav Out
        out_pose = Pose()
        out_pose.position = Point(3.19, -20.0, 0.0)
        out_pose.orientation = Quaternion(0.0,0.0,0.0,0.0)
        func = lambda: out_pose
        smach.StateMachine.add('SetNavOut',
                                SetNavGoalState(action_dict, global_store, func),
                                transitions={'SUCCESS':'NavOut'})

        # Nav To Door
        smach.StateMachine.add('NavOut',
                                NavigateState(action_dict, global_store),
                                transitions={'SUCCESS':'OldTownRoad',
                                            'FAILURE': 'NavToDoor',
                                            'REPEAT_FAILURE': 'OldTownRoad'})

        #I've got my horses in the bacc
        smach.StateMachine.add('OldTownRoad',
                               OldTownState(action_dict, global_store),
                               transitions={'SUCCESS':'TASK_SUCCESS'})
        

if __name__ == '__main__':
    action_dict = create_open_clients()
    sm = create_state_machine(action_dict)
    sm.execute()