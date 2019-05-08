#!/usr/bin/env python
""" Code for the Receptionist task.

This file contains the state machine and other code specific to the
Receptionist task.

Author: Charlie Street
Owner: Charlie Street
"""

import rospy
import smach
import actionlib

from reusable_states import * # pylint: disable=unused-wildcard-import


class DetectDoorKnockState(ActionServiceState):
    """ A state to detect a door knock in the vicinity of the robot. """

    def __init__(self, action_dict, global_store, timeout):
        outcomes = ['DETECTED', 'TIMEOUT']
        self.timeout = timeout
        super(DetectDoorKnockState, self).__init__(action_dict=action_dict,
                                                   global_store=global_store,
                                                   outcomes=outcomes)

    def execute(self, userdata):
        # TODO: Fill in!
        # Listen for a door knock, and return if heard. Otherwise timeout.
        pass


class LookForOldestGuestState(ActionServiceState):
    """ A state to look for the oldest guest in the robots vicinity. """

    def __init__(self, action_dict, global_store):
        outcomes = ['SUCCESS', 'FAILURE']
        super(LookForOldestGuestState, self).__init__(action_dict=action_dict,
                                                      global_store=global_store,
                                                      outcomes=outcomes)

    def execute(self, userdata):
        # TODO: Fill in!
        # Look in robots vicinity for oldest guest and set nav to them
        # Note there may only be one guest
        pass


class UpdateWithDrinkState(ActionServiceState):
    """ Adds the drink information to the last person met. """
    def __init__(self, action_dict, global_store):
        outcomes = ['SUCCESS']
        super(UpdateWithDrinkState, self).__init__(action_dict=action_dict,
                                                   global_store=global_store,
                                                   outcomes=outcomes)

    def execute(self, userdata):
        # TODO: Fill in!
        # Update last person memorised with drink info in latest response
        pass


class IntroduceGuestState(ActionServiceState):
    """ Introduces a guest to other guests. """
    def __init__(self, action_dict, global_store):
        outcomes = ['SUCCESS']
        super(IntroduceGuestState, self).__init__(action_dict=action_dict,
                                                  global_store=global_store,
                                                  outcomes=outcomes)

    def execute(self, userdata):
        # TODO: Fill in!
        # Form string to speak which introduces the guest
        pass


class DecideSeatingPlanState(ActionServiceState):
    """ Updates seating plan and speaks if people should swap around """
    def __init__(self, action_dict, global_store):
        outcomes = ['SUCCESS']
        super(DecideSeatingPlanState, self).__init__(action_dict=action_dict,
                                                     global_store=global_store,
                                                     outcomes=outcomes)

    def execute(self, userdata):
        # TODO: Fill in!
        # Update internal seating plan, determine if oldest guest isn't sat
        # on sofa
        pass


class FindPersonState(ActionServiceState):
    """ A state to determine who to introduce the next. """

    def __init__(self, action_dict, global_store):
        outcomes = ['PERSON_FOUND', 'NO_PEOPLE_LEFT']
        super(FindPersonState, self).__init__(action_dict=action_dict,
                                              global_store=global_store,
                                              outcomes=outcomes)
        
    def execute(self, userdata):
        # TODO: Fill in!
        # Use global store information to decide who to introduce next
        # Set a flag for this and set the point location to them.
        # The last time this is called, we should introduce the new guest to 
        # everyone at once
        pass


def create_state_machine(action_dict):
    """ File creates and returns state machine for receptionist task. """

    # Initialise global store
    global_store = {}

    # Create the state machine
    sm = smach.StateMachine(outcomes=['TASK_SUCCESS', 'TASK_FAILURE'])

    with sm:
        
        # Start talking
        phrase = "Hi, I'm Bam Bam, and I'm here to be a receptionist!"
        smach.StateMachine.add('StartTalking',
                               SpeakState(action_dict, global_store, phrase),
                               transitions={'SUCCESS':'DetectDoorKnock',
                                            'FAILURE':'DetectDoorKnock'})
        
        # Detect a door knock
        smach.StateMachine.add('DetectDoorKnock',
                               DetectDoorKnockState(action_dict, 
                                                    global_store,
                                                    30),
                               transitions={'DETECTED':'SetNavToDoor',
                                            'TIMEOUT':'SpeakNoDoor'})
                                        
        # Announce no knock heard
        phrase = ("I haven't heard a door knock, but I have a suspicion there "+
                  "might be some people waiting for me.")
        smach.StateMachine.add('SpeakNoDoor',
                               SpeakState(action_dict, global_store, phrase),
                               transitions={'SUCCESS':'SetNavToDoor',
                                            'FAILURE':'SetNavToDoor'})
        
        # Set nav goal to door
        func = lambda: None # TODO: Fix!
        smach.StateMachine.add('SetNavToDoor',
                               SetNavGoalState(action_dict, global_store, func),
                               transitions={'SUCCESS':'NavToDoor'})
        
        # Navigate to the door
        smach.StateMachine.add('NavToDoor',
                               NavigateState(action_dict, global_store),
                               transitions={'SUCCESS':'CheckAndOpenDoor',
                                            'FAILURE':'NavToDoor',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Check and open the door
        smach.StateMachine.add('CheckAndOpenDoor',
                               CheckAndOpenDoorState(action_dict, global_store),
                               transitions={'SUCCESS':'LookForOldestGuest',
                                            'FAILURE':'AskForDoorHelp'})
        
        # Ask for help opening the door
        question = ("I'm having trouble opening this door. Could someone " + 
                    "please help me open it and let me know when they have?")
        smach.StateMachine.add('AskForDoorHelp',
                               SpeakAndListenState(action_dict,
                                                   global_store,
                                                   question,
                                                   ['door is open'],
                                                   [],
                                                   20),
                               transitions={'SUCCESS':'CheckDoorOpen',
                                            'FAILURE':'AskForDoorHelp',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Check door is now open
        smach.StateMachine.add('CheckDoorOpen',
                               CheckDoorIsOpenState(action_dict, global_store),
                               transitions={'SUCCESS':'LookForOldestGuest',
                                            'FAILURE':'AskForDoorHelp'})
        
        # Look for oldest guest (both could arrive at once)
        smach.StateMachine.add('LookForOldestGuest',
                               LookForOldestGuestState(action_dict, 
                                                       global_store),
                               transitions={'SUCCESS':'NavToGuest',
                                            'FAILURE':'TASK_FAILURE'})
        
        # Navigate to the guest
        smach.StateMachine.add('NavToGuest',
                               NavigateState(action_dict, global_store),
                               transitions={'SUCCESS':'AskName',
                                            'FAILURE':'NavToGuest',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        

        # Ask the guests name
        question = ("Hi, I'm Bam Bam, nice to meet you. What's your name?")
        smach.StateMachine.add('AskName',
                               SpeakAndListenState(action_dict,
                                                   global_store,
                                                   question,
                                                   ['my name is'],
                                                   [],
                                                   30),
                               transitions={'SUCCESS':'MemorisePerson',
                                            'FAILURE':'AskName',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Memorise the person
        smach.StateMachine.add('MemorisePerson',
                                MemorisePersonState(action_dict, global_store),
                                transitions={'SUCCESS':'AskDrink',
                                             'FAILURE':'TASK_FAILURE'})

        # Ask their preferred drink
        question = ("Cool! And what is your preferred drink?")
        smach.StateMachine.add('AskDrink',
                               SpeakAndListenState(action_dict,
                                                   global_store,
                                                   question,
                                                   ['I like drinking'],
                                                   ['wine', 'beer', 'coke'],
                                                   20),
                               transitions={'SUCCESS':'UpdateWithDrink',
                                            'FAILURE':'AskDrink',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Update with drink
        smach.StateMachine.add('UpdateWithDrink',
                               UpdateWithDrinkState(action_dict, global_store),
                               transitions={'SUCCESS':'SpeakFollowMe'})
        
        # Start person following robot
        phrase = ("Sweet! Please follow me and I'll introduce you to the " + 
                  "other guests.")
        smach.StateMachine.add('SpeakFollowMe',
                               SpeakState(action_dict, global_store, func),
                               transitions={'SUCCESS':'SetNavToSofa',
                                            'FAILURE':'SpeakFollowMe'})
        
        # Set nav goal to sofa in living room
        func = lambda : None # TODO: Fix!
        smach.StateMachine.add('SetNavToSofa',
                               SetNavGoalState(action_dict, global_store, func),
                               transitions={'SUCCESS':'NavToSofa'})
        
        # Navigate to the sofa
        smach.StateMachine.add('NavToSofa',
                               NavigateState(action_dict, global_store),
                               transitions={'SUCCESS':'FindPersonToIntroduce',
                                            'FAILURE':'NavToSofa',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        

        # Find the next person to introduce
        smach.StateMachine.add('FindPersonToIntroduce',
                               FindPersonState(action_dict, global_store),
                               transitions={'PERSON_FOUND':'PointToGuest',
                                            'NO_PEOPLE_LEFT':
                                            'DecideSeatingPlan'})
                    
        # Point to guest
        smach.StateMachine.add('PointToGuest',
                               PointToObjectState(action_dict, global_store),
                               transitions={'SUCCESS':'IntroduceGuest',
                                            'FAILURE':'FindPersonToIntroduce'})
        
        # Introduce guest
        smach.StateMachine.add('IntroduceGuest',
                               IntroduceGuestState(action_dict, global_store),
                               transitions={'SUCCESS':'FindPersonToIntroduce'})
        
        # Update the seating plan
        smach.StateMachine.add('DecideSeatingPlan',
                               DecideSeatingPlanState(action_dict, 
                                                      global_store),
                               transitions={'SUCCESS':'SetNavToDoor'})
        
        
        
    
    return sm


if __name__ == '__main__':
    action_dict = {} # TODO: Fix!
    sm = create_state_machine(action_dict)
    sm.execute()

