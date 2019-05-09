#!/usr/bin/env python
""" Smach state machine for the clean up task.

This file contains any code specific to the clean up task, including
the state machine itself.

Author Charlie Street
Owner: Charlie Street
"""

import rospy
import smach
import actionlib

from reusable_states import * # pylint: disable=unused-wildcard-import
from set_up_clients import create_stage_1_clients
from orion_actions.msg import SOMObservation, Relation


class SearchForItemState(ActionServiceState):
    """ This state carries out a search for a misplaced item. """

    def __init__(self, action_dict, global_store):
        outcomes = ['OBJECT_FOUND', 'NO_OBJECTS']

        super(SearchForItemState, self).__init__(action_dict=action_dict,
                                                 global_store=global_store,
                                                 outcomes=outcomes)
    
    def execute(self, userdata):
        # TODO: Fill in!
        # Should update tf frame for pick up and also store information
        # About object in grasp so we can search its default location
        pass


class GetItemLocationState(ActionServiceState):
    """ This state gets the default location of the item being grasped. """

    def __init__(self, action_dict, global_store):
        outcomes = ['SUCCESS', 'FAILURE']

        super(GetItemLocationState, self).__init__(action_dict=action_dict,
                                                   global_store=global_store,
                                                   outcomes=outcomes)
    
    def execute(self, userdata):
        item = self.global_store['pick_up']

        obj1 = SOMObservation()
        obj1.type = item
        
        rel = Relation()
        obj2 = SOMObservation()

        try:
            self.global_store['nav_location'] = \
                get_location_of_object(self.action_dict, obj1, rel, obj2)
            return self._outcomes[0]
        except:
            return self._outcomes[1]


class UpdateItemLocationState(ActionServiceState):
    """ This state updates item location based on speech. """

    def __init__(self, action_dict, global_store):
        outcomes = ['SUCCESS', 'FAILURE']

        super(UpdateItemLocationState, self).__init__(action_dict=action_dict,
                                                      global_store=global_store,
                                                      outcomes=outcomes)
    
    def execute(self, userdata):
        # TODO: Fill in!
        # Should set navigation goal in global store and parse speech with soma
        pass


def create_state_machine(action_dict):
    """ Creates and returns the smach state machine for this task. """

    # Initialise the global store
    global_store = {}

    # Create the state machine
    sm = smach.StateMachine(outcomes=['TASK_SUCCESS', 'TASK_FAILURE'])

    with sm:

        # State intentions at start
        phrase = "Here I go! I'm gonna try to open the door then clean up!"
        smach.StateMachine.add('StartSpeak',
                               SpeakState(action_dict, global_store, phrase),
                               transitions={'SUCCESS':'OpenDoor',
                                            'FAILURE':'OpenDoor'})

        # Open the door autonomously
        smach.StateMachine.add('OpenDoor',
                               CheckAndOpenDoorState(action_dict, global_store),
                               transitions={'SUCCESS':'SearchForItemSpeech',
                                            'FAILURE':'AskForDoorHelp'})
        
        # Ask For help opening the door
        question = ("I can't open the door. Can someone please help me and " + 
                   "let me know?")
        smach.StateMachine.add('AskForDoorHelp',
                               SpeakAndListenState(action_dict,
                                                   global_store,
                                                   question,
                                                   ['I will'],
                                                   [],
                                                   20),
                               transitions={'SUCCESS':'CheckDoorOpen',
                                            'FAILURE':'AskForDoorHelp',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Check the door has been opened
        smach.StateMachine.add('CheckDoorOpen',
                               CheckDoorIsOpenState(action_dict, global_store),
                               transitions={'SUCCESS':'SearchForItemSpeech',
                                            'FAILURE':'AskForDoorHelp'})
        
        # Speak the next intention
        smach.StateMachine.add('SearchForItemSpeech',
                               SpeakState(action_dict, global_store, phrase),
                               transitions={'SUCCESS':'SearchForItem',
                                            'FAILURE':'SearchForItem'})
        
        # Search for the next item
        smach.StateMachine.add('SearchForItem',
                               SearchForItemState(action_dict, global_store),
                               transitions={'OBJECT_FOUND':'GraspItem',
                                            'NO_OBJECTS':'TASK_SUCCESS'})
        
        # Pick up the object
        smach.StateMachine.add('GraspItem',
                               PickUpObjectState(action_dict, global_store),
                               transitions={'SUCCESS':'GetItemLocation',
                                            'Failure':'AskForHelpGrasping'})
        
        # Ask for help if can't grasp object
        question = ("I can't pick up this object. Can someone help me please " +
                    "and let me know once they have?")
        smach.StateMachine.add('AskForHelpGrasping',
                               SpeakAndListenState(action_dict,
                                                   global_store,
                                                   question,
                                                   ["Done"],
                                                   [],
                                                   20),
                               transitions={'SUCCESS':'ReceiveObject',
                                            'FAILURE':'AskForHelpGrasping',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Receive object from operator
        smach.StateMachine.add('ReceiveObject',
                               ReceiveObjectFromOperatorState(action_dict,
                                                              global_store),
                               transitions={'SUCCESS':'GetItemLocation',
                                            'FAILURE':'AskForHelpGrasping'})
        
        # Get the items default location
        smach.StateMachine.add('GetItemLocation',
                               GetItemLocationState(action_dict, global_store),
                               transitions={'SUCCESS':'NavToObjectLocation',
                                            'FAILURE':'AskForHelpLocation'})
        
        # Ask for help if can't find item location
        question = ("Can someone please tell me where to place the object I am"+
                   " holding? Thank you.")
        smach.StateMachine.add('AskForHelpLocation',
                               SpeakAndListenState(action_dict,
                                                   global_store,
                                                   question,
                                                   ['The item goes here'],
                                                   [],
                                                   30),
                               transitions={'SUCCESS':'UpdateItemLocation',
                                            'FAILURE':'AskForHelpLocation',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Update the items location
        smach.StateMachine.add('UpdateItemLocation',
                               UpdateItemLocationState(action_dict, 
                                                       global_store),
                               transitions={'SUCCESS':'NavToObjectLocation',
                                            'FAILURE':'AskForHelpLocation'})
        
        # Navigate to the objects default location (or the bin)
        smach.StateMachine.add('NavToObjectLocation',
                               NavigateState(action_dict, global_store),
                               transitions={'SUCCESS':'PlaceObject',
                                            'FAILURE':'NavToObjectLocation',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Place the object down
        smach.StateMachine.add('PlaceObject',
                               PutObjectOnSurfaceState(action_dict, 
                                                       global_store),
                               transitions={'SUCCESS':'SearchForItem',
                                            'FAILURE':'AskForHelpPlacing'})
        
        # Ask for help Placing
        question = ("Could someone please take the item I'm holding and place " +
                   "it on the surface in front of me and let me know when " +
                   "they have? Thank you.")
        smach.StateMachine.add('AskForHelpPlacing',
                               SpeakAndListenState(action_dict,
                                                   global_store,
                                                   question,
                                                   ['Its placed'],
                                                   [],
                                                   30),
                               transitions={'SUCCESS':'HandoverObject',
                                            'FAILURE':'AskForHelpPlacing',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Handover the object
        smach.StateMachine.add('HandoverObject',
                               HandoverObjectToOperatorState(action_dict,
                                                             global_store),
                               transitions={'SUCCESS':'SearchForItem',
                                            'FAILURE':'AskForHelpPlacing'})
    
    return sm
            

if __name__ == '__main__':
    action_dict = create_stage_1_clients(2)
    sm = create_state_machine(action_dict)
    sm.execute()