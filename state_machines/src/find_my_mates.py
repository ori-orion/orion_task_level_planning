#!/usr/bin/env python
""" Code for the find my mate task.

This file contains code for the find my mate task, including the
state machine itself.

Author: Charlie Street
Owner: Charlie Street

"""

import rospy
import smach
import actionlib

from reusable_states import * # pylint: disable=unused-wildcard-import

import time


class LookForPeopleState(ActionServiceState):
    """ State for searching for friends of the operator. """

    def __init__(self, action_dict, global_store):
        outcomes = ['PERSON_FOUND', 'NOBODY_FOUND']
        super(LookForPeopleState, self).__init__(action_dict=action_dict,
                                                 global_store=global_store,
                                                 outcomes=outcomes)
        
    def execute(self, userdata):
        # TODO: Fill in!
        # Should explore, looking for people who aren't the operator
        # And haven't been found already
        pass


class GiveOperatorInfoState(ActionServiceState):
    """ State for giving operator info about mates. """

    def __init__(self, action_dict, global_store):
        outcomes = ['SUCCESS']
        super(GiveOperatorInfoState, self).__init__(action_dict=action_dict,
                                                    global_store=global_store,
                                                    outcomes=outcomes)
    
    def execute(self, userdata):
        # TODO: Fill in!
        # Take information of memorised people and form speech from it.
        # Then say it!
        return self._outcomes[0]


class ShouldIContinueState(ActionServiceState):
    """ State determines whether we should continue or go back. """

    def __init__(self, action_dict, global_store):
        outcomes = ['YES', 'NO']
        super(ShouldIContinueState, self).__init__(action_dict=action_dict,
                                                   global_store=global_store,
                                                   outcomes=outcomes)
    
    def execute(self, userdata):
        time_elapsed = time.time() - self.global_store['start_time']
        if time_elapsed > 210: # 3 and a half minutes
            return self._outcomes[1]
        elif len(self.global_store['people_found']) == 3:
            return self._outcomes[1]
        
        return self._outcomes[0]


def create_state_machine(action_dict):
    """ This function creates and returns the state machine for the task. """

    # Initialise the global store
    global_store = {}
    global_store['start_time'] = time.time()
    global_store['people_found'] = []
    # TODO: Set instruction point as initial nav goal

    # Create the state machine
    sm = smach.StateMachine(outcomes=['TASK_SUCCESS', 'TASK_FAILURE'])

    with sm:
        
        # Start by stating intentions
        phrase = "Hi, I'm Bam Bam and I'm here to find some mates!"
        smach.StateMachine.add('StartSpeak',
                               SpeakState(action_dict, global_store, phrase),
                               transitions={'SUCCESS':'CheckDoorOpen',
                                            'FAILURE':'CheckDoorOpen'})
        
        # Now wait for the door to be opened
        smach.StateMachine.add('CheckDoorOpen',
                               CheckDoorIsOpenState(action_dict, global_store),
                               transitions={'OPEN':'NavToInstructionPoint',
                                            'CLOSED':'CheckDoorOpen'})
        
        # Now navigate to the operator
        smach.StateMachine.add('NavToInstructionPoint',
                               NavigateState(action_dict, global_store),
                               transitions={'SUCCESS':'AskForOperator',
                                            'FAILURE':'NavToInstructionPoint',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Ask for an operator
        question = ("Hi, nice to meet you! Are you the operator who is looking "+
                   "for their friends!")
        smach.StateMachine.add('AskForOperator',
                               SpeakAndListenState(action_dict,
                                                   global_store,
                                                   question,
                                                   ['Find my mates'],
                                                   [],
                                                   30),
                               transitions={'SUCCESS':'MemoriseOperator',
                                            'FAILURE':'AskForOperator',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})

        # Memorise operator
        smach.StateMachine.add('MemoriseOperator',
                               OperatorDetectState(action_dict, global_store),
                               transitions={'SUCCESS':'OnMyWay',
                                            'FAILURE':'AskForOperator'})

        # Head on the way
        phrase = ("Right, I'm off to find your mates! Don't worry, I never " +
                 "forget a friendly face! In the words of Arnold Schwarzenegger"
                 + ", I'll be back!")
        smach.StateMachine.add('OnMyWay',
                               SpeakState(action_dict, global_store, phrase),
                               transitions={'SUCCESS':'LookForPeople',
                                            'FAILURE':'LookForPeople'})
        
        # Start looking for people
        smach.StateMachine.add('LookForPeople',
                               LookForPeopleState(action_dict, global_store),
                               transitions={'PERSON_FOUND':'TalkToPerson',
                                            'NOBODY_FOUND':'SetOpDestination'})
        
        # Get information from person
        question = ("Hi, I'm Bam Bam, nice to meet you! Would you mind telling "
                   + "me a little about yourself?")
        smach.StateMachine.add('TalkToPerson',
                               SpeakAndListenState(action_dict,
                                                   global_store,
                                                   question,
                                                   ['My name is'],
                                                   [],
                                                   30),
                               transitions={'SUCCESS':'MemorisePerson',
                                            'FAILURE':'TalkToPerson',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # memorise the person
        smach.StateMachine.add('MemorisePerson',
                               MemorisePersonState(action_dict, global_store),
                               transitions={'SUCCESS':'ThankYou',
                                            'FAILURE':'TalkToPerson'})
        
        # Thank the person
        phrase = ("Thank you, I think I got all that. I need to go now, my " +
                 "home planet needs me! Beep boop")
        smach.StateMachine.add('ThankYou',
                               SpeakState(action_dict, global_store, phrase),
                               transitions={'SUCCESS':'ShouldIContinue',
                                            'FAILURE':'ShouldIContinue'})
        
        # Check whether I should continue
        smach.StateMachine.add('ShouldIContinue',
                               ShouldIContinueState(action_dict, global_store),
                               transitions={'YES':'LookForPeople',
                                            'NO':'SetOpDestination'})
        
        # Set operator destination
        function = lambda : None # TODO: Make correct!
        smach.StateMachine.add('SetOpDestination',
                               SetNavGoalState(action_dict, 
                                               global_store, 
                                               function),
                               transitions={'SUCCESS':'BackToOp'})
        
        # Navigate back to the operator
        smach.StateMachine.add('BackToOp',
                               NavigateState(action_dict, global_store),
                               transitions={'SUCCESS':'GiveOperatorInfo',
                                            'FAILURE':'BackToOp',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Give the operator information
        smach.StateMachine.add('GiveOperatorInfo',
                               GiveOperatorInfoState(action_dict, global_store),
                               transitions={'SUCCESS':'ThankOp'})
        
        # Thank the operator
        smach.StateMachine.add('ThankOp',
                               SpeakState(action_dict, global_store, phrase),
                               transitions={'SUCCESS':'TASK_SUCCESS',
                                            'FAILURE':'TASK_SUCCESS'})

    return sm


if __name__ == '__main__':
    action_dict = {} # TODO: Sort out!
    sm = create_state_machine(action_dict)
    sm.execute()