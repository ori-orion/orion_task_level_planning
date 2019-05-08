#!/usr/bin/env python
""" Code for the farewell task.

This file contains the state machine for the farewell task, as well
as any related code specific to this task.

Author: Charlie Street
Owner: Charlie Street
"""

import rospy
import smach
import actionlib

from reusable_states import * # pylint: disable=unused-wildcard-import

class FindLeaverState(ActionServiceState):
    """ A state to find someone wanting to leave. """

    def __init__(self, action_dict, global_store):
        outcomes = ['FOUND_LEAVER', 'NONE_LEFT']
        super(FindLeaverState, self).__init__(action_dict=action_dict,
                                              global_store=global_store,
                                              outcomes=outcomes)
        
    def execute(self, userdata):
        # TODO: Fill in!
        # should find someone wanting to leave and set nav goal to them
        pass


class FindHighVizPersonState(ActionServiceState):
    """ A state to navigate towards someone in a high viz vest. """

    def __init__(self, action_dict, global_store):
        outcomes = ['SUCCESS', 'FAILURE']
        super(FindHighVizPersonState, self).__init__(action_dict=action_dict,
                                                     global_store=global_store,
                                                     outcomes=outcomes)
        
    def execute(self, userdata):
        # TODO: Fill in!
        # should navigate towards someone wearing a high viz jacket etc.
        pass


class UpdateCoatInfoState(ActionServiceState):
    """ State updates persons coat information. """

    def __init__(self, action_dict, global_store):
        outcomes = ['SUCCESS']
        super(UpdateCoatInfoState, self).__init__(action_dict=action_dict,
                                                  global_store=global_store,
                                                  outcomes=outcomes)
    
    def execute(self, userdata):
        # TODO: Fill in!
        # Should update persons info with coat colour based on last response
        # Should also set pick up to this
        pass


def create_state_machine(action_dict):
    """ Function creates and returns the state machine for this task. """

    # Initialise global store
    global_store = {}

    # Create state machine
    sm = smach.StateMachine(outcomes=['TASK_SUCCESS', 'TASK_FAILURE'])

    with sm:
        
        # Start talking
        phrase = "It's always sad to say goodbye, but people need to go home!"
        smach.StateMachine.add('StartTalking',
                               SpeakState(action_dict, global_store, phrase),
                               transitions={'SUCCESS':'WaitForDoor',
                                            'FAILURE':'WaitForDoor'})
        
        # Wait for the door to be opened
        smach.StateMachine.add('WaitForDoor',
                               CheckDoorIsOpenState(action_dict, global_store),
                               transitions={'OPEN':'SetNavToLivingRoom',
                                            'CLOSED':'WaitForDoor'})
        
        # Set nav goal to living room
        func = lambda : None # TODO: Fix
        smach.StateMachine.add('SetNavToLivingRoom',
                               SetNavGoalState(action_dict, global_store, func),
                               transitions={'SUCCESS':'NavToLivingRoom'})
        
        # Navigate to living room
        smach.StateMachine.add('NavToLivingRoom',
                               NavigateState(action_dict, global_store),
                               transitions={'SUCCESS':'AskWhosLeaving',
                                            'FAILURE':'NavToLivingRoom',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Speak who wants to leave
        phrase = "Hi everyone, who wants to go home?"
        smach.StateMachine.add('AskWhosLeaving',
                               SpeakState(action_dict, global_store, phrase),
                               transitions={'SUCCESS':'FindPersonToGo',
                                            'FAILURE':'AskWhosLeaving'})

        # Find person to leave
        smach.StateMachine.add('FindPersonToGo',
                               FindLeaverState(action_dict, global_store),
                               transitions={'FOUND_LEAVER': 'NavToLeaver',
                                            'NONE_LEFT':'TASK_SUCCESS'})
        
        # Navigate to person
        smach.StateMachine.add('NavToLeaver',
                               NavigateState(action_dict, global_store),
                               transitions={'SUCCESS':'AskName',
                                            'FAILURE':'NavToLeaver',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Ask for the persons name
        question = "Hi, I'm Bam Bam. What's your name?"
        smach.StateMachine.add('AskName',
                               SpeakAndListenState(action_dict,
                                                   global_store,
                                                   question,
                                                   ['my name is'],
                                                   ['kyle'],
                                                   20),
                               transitions={'SUCCESS':'MemorisePerson',
                                            'FAILURE':'AskName',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Memorise the person
        smach.StateMachine.add('MemorisePerson',
                               MemorisePersonState(action_dict, global_store),
                               transitions={'SUCCESS':'AskCoat',
                                            'FAILURE':'AskName'})
        
        # Ask about the persons coat
        question = "Cool! And what colour is your coat?"
        smach.StateMachine.add('AskCoat',
                               SpeakAndListenState(action_dict,
                                                   global_store,
                                                   question,
                                                   ['my coat is'],
                                                   ['red'],
                                                   20),
                               transitions={'SUCCESS':'UpdateCoatInfo',
                                            'FAILURE':'AskCoat',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
            
        # Update the coat information
        smach.StateMachine.add('UpdateCoatInfo',
                               UpdateCoatInfoState(action_dict, global_store),
                               transitions={'SUCCESS':'SetNavToCoatRack'})

        # Set nav goal to coat rack
        func = lambda : None # TODO: Fix!
        smach.StateMachine.add('SetNavToCoatRack',
                               SetNavGoalState(action_dict, global_store, func),
                               transitions={'SUCCESS':'NavToCoatRack'})

        # Nav to coat rack
        smach.StateMachine.add('NavToCoatRack',
                               NavigateState(action_dict, global_store),
                               transitions={'SUCCESS':'PickUpCoat',
                                            'FAILURE':'NavToCoatRack',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})

        # Try to pick up coat
        smach.StateMachine.add('PickUpCoat',
                               PickUpObjectState(action_dict, global_store),
                               transitions={'SUCCESS':'SetNavToPerson',
                                            'FAILURE':'AskForCoatHelp'})

        # Ask for help with coat
        question = ("I can't seem to pick up this coat I'm trying to pick up." +
                   " Can someone help me and tell me when they're ready to " +
                   "hand it to me please?")    
        smach.StateMachine.add('AskForCoatHelp',
                               SpeakAndListenState(action_dict, 
                                                   global_store,
                                                   question,
                                                   ['ready'],
                                                   [],
                                                   20),
                               transitions={'SUCCESS':'ReceiveCoat',
                                            'FAILURE':'AskForCoatHelp',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})


        # Receive coat
        smach.StateMachine.add('ReceiveCoat',
                               ReceiveObjectFromOperatorState(action_dict,
                                                              global_store),
                               transitions={'SUCCESS':'SetNavToPerson',
                                            'FAILURE':'AskForCoatHelp'})

        # Set nav to person
        func = lambda: None # TODO: Fix!
        smach.StateMachine.add('SetNavToPerson',
                               SetNavGoalState(action_dict, global_store, func),
                               transitions={'SUCCESS':'NavToPerson'})
        
        # Navigate to person
        smach.StateMachine.add('NavToPerson',
                               NavigateState(action_dict, global_store),
                               transitions={'SUCCESS':'RequestCoatHandover',
                                            'FAILURE':'NavToPerson',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Ask if person wants coat
        question = ("I'm back. Let me know when you're ready to receive your " +
                   "and I'll hand it to you.")
        smach.StateMachine.add('RequestCoatHandover',
                               SpeakAndListenState(action_dict,
                                                   global_store,
                                                   question,
                                                   ['ready'],
                                                   [],
                                                   20),
                               transitions={'SUCCESS':'HandoverCoat',
                                            'FAILURE':'RequestCoatHandover',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Handover coat
        smach.StateMachine.add('HandoverCoat',
                               HandoverObjectToOperatorState(action_dict,
                                                             global_store),
                               transitions={'SUCCESS':'SpeakToFollowToDoor',
                                            'FAILURE':'RequestCoatHandover'})
        
        # Request guest to follow robot to door
        question = ("Please follow me to the door. Let me know when you're " +
                   "ready.")
        smach.StateMachine.add('SpeakToFollowToDoor',
                               SpeakAndListenState(action_dict,
                                                   global_store,
                                                   question,
                                                   ['ready'],
                                                   [],
                                                   20),
                               transitions={'SUCCESS':'SetNavToDoor',
                                            'FAILURE':'SpeakToFollowToDoor',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # set nav to door
        func = lambda: None # TODO: Fix!
        smach.StateMachine.add('SetNavToDoor',
                               SetNavGoalState(action_dict, global_store, func),
                               transitions={'SUCCESS':'NavToDoor'})
        
        # Navigate to door
        smach.StateMachine.add('NavToDoor',
                               NavigateState(action_dict, global_store),
                               transitions={'SUCCESS':'SpeakAboutUmbrella',
                                            'FAILURE':'NavToDoor',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Alert about umbrella
        phrase = "I'm going to go and get the umbrella. Be right back!"
        smach.StateMachine.add('SpeakAboutUmbrella',
                               SpeakState(action_dict, global_store, phrase),
                               transitions={'SUCCESS':'SetNavToUmbrella',
                                            'FAILURE':'SetNavToUmbrella'})
        
        # Set nav to umbrella
        func = lambda: None # TODO: Fix!
        smach.StateMachine.add('SetNavToUmbrella',
                               SetNavGoalState(action_dict, global_store, func),
                               transitions={'SUCCESS':'NavToUmbrella'})
        
        # Navigate to umbrella
        smach.StateMachine.add('NavToUmbrella',
                               NavigateState(action_dict, global_store),
                               transitions={'SUCCESS':'SetPickUpUmbrella',
                                            'FAILURE':'NavToUmbrella',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # set pick up to umbrella
        func = lambda: None # TODO: Fix!
        smach.StateMachine.add('SetPickUpUmbrella',
                               SetPickupState(action_dict, global_store, func),
                               transitions={'SUCCESS':'PickUpUmbrella'})

        # Pick up umbrella
        smach.StateMachine.add('PickUpUmbrella',
                                PickUpObjectState(action_dict, global_store),
                                transitions={'SUCCESS':'SetNavGoalBackToDoor',
                                             'FAILURE':'AskForUmbrellaHelp'})
        
        # Ask for help with umbrella
        question = ("Can someone please pass me the umbrella? If so, can " +
                    "you tell me when you're ready?")
        smach.StateMachine.add('AskForUmbrellaHelp',
                               SpeakAndListenState(action_dict,
                                                   global_store,
                                                   question,
                                                   ['ready'],
                                                   [],
                                                   20),
                               transitions={'SUCCESS':'HandoverUmbrella',
                                            'FAILURE':'AskForUmbrellaHelp',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Receive umbrella
        smach.StateMachine.add('HandoverUmbrella',
                               ReceiveObjectFromOperatorState(action_dict,
                                                              global_store),
                               transitions={'SUCCESS':'SetNavGoalBackToDoor',
                                            'FAILURE':'AskForUmbrellaHelp'})
        
        # Set Nav back to door
        func = lambda: None # TODO: Fix!
        smach.StateMachine.add('SetNavGoalBackToDoor',
                               SetNavGoalState(action_dict, global_store, func),
                               transitions={'SUCCESS':'NavBackToDoor'})
        
        # Nav back to door
        smach.StateMachine.add('NavBackToDoor',
                               NavigateState(action_dict, global_store),
                               transitions={'SUCCESS':'PrepareForCab',
                                            'FAILURE':'NavBackToDoor',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Prepare guest for following to cab
        question = ("I'm back! Let me know when you're ready and you can " +
                   "follow me to the cab!")
        smach.StateMachine.add('PrepareForCab',
                               SpeakAndListenState(action_dict,
                                                   global_store,
                                                   question,
                                                   ['ready'],
                                                   [],
                                                   20),
                               transitions={'SUCCESS':'ToHighViz',
                                            'FAILURE':'PrepareForCab',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Find High viz cab person
        smach.StateMachine.add('ToHighViz',
                               FindHighVizPersonState(action_dict, 
                                                      global_store),
                               transitions={'SUCCESS':'ThankGuest',
                                            'FAILURE':'SetNavToLivingRoom'})
        
        # Thank guest
        phrase = "I hope you had a nice time. Safe journey home!"
        smach.StateMachine.add('ThankGuest',
                               SpeakState(action_dict, global_store, phrase),
                               transitions={'SUCCESS':'SetNavBackToUmbrella',
                                            'FAILURE':'SetNavBackToUmbrella'})
        
        # Go to put the umbrella back
        func = lambda: None # TODO: Fix!
        smach.StateMachine.add('SetNavBackToUmbrella',
                               SetNavGoalState(action_dict, global_store, func),
                               transitions={'SUCCESS':'NavBackToUmnbrella'})
        
        # Navigate back to the umbrellas default location
        smach.StateMachine.add('NavBackToUmbrella',
                               NavigateState(action_dict, global_store),
                               transitions={'SUCCESS':'PutUmbrellaBack',
                                            'FAILURE':'NavBackToUmbrella',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Try to put the umbrella down
        smach.StateMachine.add('PutUmbrellaBack',
                               PutObjectOnSurfaceState(action_dict, 
                                                       global_store),
                               transitions={'SUCCESS':'SetNavToLivingRoom',
                                            'FAILURE':'AskForHelpUmbrellaBack'})
        
        # Ask for help putting umbrella back
        question = ("I'm having trouble putting the umbrella back. Can " +
                    "someone please help me? If so, can you tell me when " +
                    "you're ready for me to hand it to you?")
        smach.StateMachine.add('AskForHelpUmbrellaBack',
                               SpeakAndListenState(action_dict,
                                                   global_store,
                                                   question,
                                                   ['ready'],
                                                   [],
                                                   20),
                               transitions={'SUCCESS':'HandUmbrellaBack',
                                            'FAILURE':'AskForHelpUmbrellaBack',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Handover umbrella
        smach.StateMachine.add('HandUmbrellaBack',
                               HandoverObjectToOperatorState(action_dict,
                                                             global_store),
                               transitions={'SUCCESS':'SetNavToLivingRoom',
                                            'FAILURE':'AskForHelpUmbrellaBack'})            

    return sm


if __name__ == '__main__':
    action_dict = {} # TODO: Sort out!
    sm = create_state_machine(action_dict)
    sm.execute()