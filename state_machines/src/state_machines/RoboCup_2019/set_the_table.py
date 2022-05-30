#!/usr/bin/env python3
""" Code for the Set The Table task.

This file contains the state machine code for the Set The Table task.

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

def go_to_cupboard(action_dict):
    """ Gets location of cupboard. """
    obj1 = SOMObservation()
    obj1.type = 'set_the_table_point_of_interest_cupboard'

    return get_location_of_object(action_dict, obj1, 
                                  Relation(), SOMObservation())


def go_to_table(action_dict):
    """ Gets location of table. """
    obj1 = SOMObservation()
    obj1.type = 'set_the_table_point_of_interest_table'

    return get_location_of_object(action_dict, obj1, 
                                  Relation(), SOMObservation())


class ObserveCupboardState(ActionServiceState):
    """ State observes the cupboard and what is in it. """
    def __init__(self, action_dict, global_store):
        outcomes = ['SUCCESS']
        super(ObserveCupboardState, self).__init__(action_dict=action_dict,
                                                   global_store=global_store,
                                                   outcomes=outcomes)
        
    def execute(self, userdata):
        # TODO: Fill in!
        # Should set a list of observed items, make sure placemat, then dish!
        # This should give an ordered list of items to place
        # Should also populate semantic map
        # Should fill in to_place in global_store
        items = [] # TODO: Items should be the list of items observed

        if 'placemat' in items: # TODO: Spelling
            self.global_store['to_place'].append('placemat')
        
        if 'plate' in items or 'dish' in items:
            item = None
            if 'plate' in items:
                item = 'plate'
            else:
                item = 'dish'
            self.global_store['to_place'].append(item)
        
        if 'bowl' in items:
            self.global_store['to_place'].append('bowl')

        if 'knife' in items:
            self.global_store['to_place'].append('knife')

        if 'spoon' in items:
            self.global_store['to_place'].append('spoon')
        
        if 'fork' in items:
            self.global_store['to_place'].append('fork')

        if 'cloth' in items or 'napkin' in items:
            item = None
            if 'cloth' in items:
                item = 'cloth'
            else:
                item = 'napkin'
            self.global_store['to_place'].append(item)

        if 'cup' in items or 'mug' in items:
            item = None
            if 'cup' in items:
                item = 'cup'
            else:
                item = 'mug'
            self.global_store['to_place'].append(item)

        return self._outcomes[0]


class ChooseItemState(ActionServiceState):
    """ State chooses the next item to pick up. """
    def __init__(self, action_dict, global_store):
        outcomes = ['PLACEMAT', 'ITEM', 'NONE_LEFT']
        super(ChooseItemState, self).__init__(action_dict=action_dict,
                                              global_store=global_store,
                                              outcomes=outcomes)
    
    def execute(self, userdata):

        next_item_index = self.global_store['next_item']
        items = self.global_store['to_place']

        if next_item_index >= len(items): # We're done!
            return self._outcomes[2]
        
        new_item = items[next_item_index]
        self.global_store['pick_up'] = new_item

        if new_item == 'placemat': # TODO: Check spelling
            return self._outcomes[0]
        elif (new_item == 'dish' or new_item == 'plate'):
            self.global_store['dish'] = new_item
            self.global_store['rel_pos'] = ('placemat', 0, 0, 0.1)

        elif new_item == 'bowl':
            self.global_store['rel_pos'] = (self.global_store['dish'],0,0,0.1)

        elif new_item == 'fork':
            self.global_store['rel_pos'] = (self.global_store['dish'],0,0.1,0)

        elif new_item == 'spoon':
            self.global_store['rel_pos'] = (self.global_store['dish'],0,-0.2,0)

        elif new_item == 'knife':
            self.global_store['rel_pos'] = (self.global_store['dish'],0,-0.1,0)

        elif new_item == 'cup' or new_item == 'mug':
            self.global_store['rel_pos'] = (self.global_store['dish'],
                                            0.15,-0.15,0)

        elif new_item == 'cloth' or new_item == 'napkin':
            self.global_store['rel_pos'] = (self.global_store['dish'],0,0.2,0)

        else: # Default: put left of bowl
            self.global_store['rel_pos'] = (self.global_store['dish'], 
                                            0, 0.2, 0)

        return self._outcomes[1]


def create_state_machine(action_dict):
    """ This function creates and returns the state machine for this task. """

    # Initialise global_store
    global_store = {}
    global_store['to_place'] = []
    global_store['drawer_handle'] = 'cupboard' # TODO: Sort out!
    global_store['next_item'] = 0
    global_store['furniture_door'] = 'cupboard'
    global_store['dish'] = None

    # Create the state machine
    sm = smach.StateMachine(outcomes=['TASK_SUCCESS', 'TASK_FAILURE'])

    with sm:
        
        # Get the robot to start talking
        phrase = "Hi, I'm Bam Bam and I'm here to set the table!"
        smach.StateMachine.add('StartTalking',
                               SpeakState(action_dict, global_store, phrase),
                               transitions={'SUCCESS':'SetNavToCupboard',
                                            'FAILURE':'SetNavToCupboard'})
                    
        # Set nav to cupboard
        func = lambda: go_to_cupboard(action_dict)
        smach.StateMachine.add('SetNavToCupboard',
                               SetNavGoalState(action_dict, global_store, func),
                               transitions={'SUCCESS':'NavToCupboard'})
        
        # Navigate to cupboard
        smach.StateMachine.add('NavToCupboard',
                               NavigateState(action_dict, global_store),
                               transitions={'SUCCESS':'OpenCupboard',
                                            'FAILURE':'NavToCupboard',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Open the cupboard
        smach.StateMachine.add('OpenCupboard',
                               OpenFurnitureDoorState(action_dict, 
                                                      global_store),
                               transitions={'SUCCESS':'ObserveCupboard',
                                            'FAILURE':'AskForHelpCupboard'})
        
        # If can't open cupboard
        question = ("Can someone open the cupboard for me and " +
                   "say my name when they have?")
        smach.StateMachine.add('AskForHelpCupboard',
                               SpeakState(action_dict, global_store, question),
                               transitions={'SUCCESS':'WaitTillOpen',
                                            'FAILURE':'WaitTillOpen'})
        
        # Wait till name said
        smach.StateMachine.add('WaitTillOpen',
                               HotwordListenState(action_dict, global_store, ['bambam'],30),
                               transitions={'SUCCESS':'ObserveCupboard',
                                            'FAILURE':'TASK_FAILURE'})
        
        # Observe Cupboard
        smach.StateMachine.add('ObserveCupboard',
                               ObserveCupboardState(action_dict, global_store),
                               transitions={'SUCCESS':'ChooseNextItem'})
        
        # Choose Item
        smach.StateMachine.add('ChooseNextItem',
                               ChooseItemState(action_dict, global_store),
                               transitions={'PLACEMAT':'PickUpPlacemat',
                                            'ITEM':'PickUpItem',
                                            'NONE_LEFT':'Finish'})
        
        # If no items left, finish
        phrase = "Your table is set for dinner. Bon appetite!"
        smach.StateMachine.add('Finish',
                               SpeakState(action_dict, global_store, phrase),
                               transitions={'SUCCESS':'TASK_SUCCESS',
                                            'FAILURE':'TASK_SUCCESS'})

        # Pick up the placemat
        smach.StateMachine.add('PickUpPlacemat',
                               PickUpObjectState(action_dict, global_store),
                               transitions={'SUCCESS':'SetNavToTable',
                                            'FAILURE':'AskForPlacematHelp'})
        
        # If failed, ask for help
        question = ("I can't pick up the placemat. Can you hand it to me " +
                   "please? Let me know when you're ready to hand over!")
        smach.StateMachine.add('AskForPlacematHelp',
                               SpeakAndHotwordState(action_dict,
                                                   global_store,
                                                   question,
                                                   ['ready'],
                                                   30),
                               transitions={'SUCCESS':'ReceivePlacemat',
                                            'FAILURE':'AskForPlacematHelp',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Receive Placemat
        smach.StateMachine.add('ReceivePlacemat',
                               ReceiveObjectFromOperatorState(action_dict,
                                                              global_store),
                               transitions={'SUCCESS':'SetNavToTable',
                                            'FAILURE':'AskForPlacematHelp'})
        
        # Set nav To table
        func = lambda : go_to_table(action_dict)
        smach.StateMachine.add('SetNavToTable',
                               SetNavGoalState(action_dict, global_store, func),
                               transitions={'SUCCESS':'NavToTable'})
        
        # Navigate to table
        smach.StateMachine.add('NavToTable',
                               NavigateState(action_dict, global_store),
                               transitions={'SUCCESS':'PlacePlacemat',
                                            'FAILURE':'NavToTable',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Place down placemat
        smach.StateMachine.add('PlacePlacemat',
                               PutObjectOnSurfaceState(action_dict, 
                                                       global_store),
                               transitions={'SUCCESS':'SetNavBackToCupboard',
                                            'FAILURE':'AskForPlacingMatHelp'})
        
        # Get help placing place mat
        question = ("I'm struggling to put this place mat down. Can you take " +
                   "from me and put it down on the table for me? Let me know " +
                   "when you're ready for me to hand it to you.")
        smach.StateMachine.add('AskForPlacingMatHelp',
                               SpeakAndHotwordState(action_dict,
                                                   global_store,
                                                   question,
                                                   ['ready'],
                                                   20),
                               transitions={'SUCCESS':'GivePlacemat',
                                            'FAILURE':'AskForPlacingMatHelp',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Give placemat to person
        smach.StateMachine.add('GivePlacemat',
                               HandoverObjectToOperatorState(action_dict,
                                                             global_store),
                               transitions={'SUCCESS':'SetNavBackToCupboard',
                                            'FAILURE':'AskForPlacingMatHelp'})
        
        # Go back to the cupboard
        func = lambda : go_to_cupboard(action_dict)
        smach.StateMachine.add('SetNavBackToCupboard',
                               SetNavGoalState(action_dict, global_store, func),
                               transitions={'SUCCESS':'NavBackToCupboard'})
        
        # Nav back to cupboard
        smach.StateMachine.add('NavBackToCupboard',
                               NavigateState(action_dict, global_store),
                               transitions={'SUCCESS':'ChooseNextItem',
                                            'FAILURE':'NavBackToCupboard',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})


        # Pick up object
        smach.StateMachine.add('PickUpItem',
                               PickUpObjectState(action_dict, global_store),
                               transitions={'SUCCESS':'SetNavToTable',
                                            'FAILURE':'AskForItemHelp'})
        
        # If failed, ask for help
        question = ("I can't pick up this item. Can you hand it to me " +
                   "please? Let me know when you're ready to hand over!")
        smach.StateMachine.add('AskForItemHelp',
                               SpeakAndHotwordState(action_dict,
                                                   global_store,
                                                   question,
                                                   ['ready'],
                                                   30),
                               transitions={'SUCCESS':'ReceiveItem',
                                            'FAILURE':'AskForItemHelp',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Receive item
        smach.StateMachine.add('ReceiveItem',
                               ReceiveObjectFromOperatorState(action_dict,
                                                              global_store),
                               transitions={'SUCCESS':'SetNavToTableItem',
                                            'FAILURE':'AskForItemHelp'})

         # Set nav To table
        func = lambda : go_to_table(action_dict)
        smach.StateMachine.add('SetNavToTableItem',
                               SetNavGoalState(action_dict, global_store, func),
                               transitions={'SUCCESS':'NavToTableItem'})
        
        # Navigate to table
        smach.StateMachine.add('NavToTableItem',
                               NavigateState(action_dict, global_store),
                               transitions={'SUCCESS':'PlaceItem',
                                            'FAILURE':'NavToTableItem',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})

        # Place down placemat
        smach.StateMachine.add('PlaceItem',
                               PlaceObjectRelativeState(action_dict, 
                                                       global_store),
                               transitions={'SUCCESS':'SetNavBackToCupboard',
                                            'FAILURE':'AskForPlacingItemHelp'})
        
        # Get help placing place mat
        question = ("I'm struggling to put this item down. Can you take " +
                   "from me and put it down in the right place for me? " +
                   "Let me know when you're ready for me to hand it to you.")
        smach.StateMachine.add('AskForPlacingItemHelp',
                               SpeakAndHotwordState(action_dict,
                                                   global_store,
                                                   question,
                                                   ['ready'],
                                                   20),
                               transitions={'SUCCESS':'GiveItem',
                                            'FAILURE':'AskForPlacingItemHelp',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Give itemto person
        smach.StateMachine.add('GiveItem',
                               HandoverObjectToOperatorState(action_dict,
                                                             global_store),
                               transitions={'SUCCESS':'SetNavBackToCupboard',
                                            'FAILURE':'AskForPlacingItemHelp'})

    return sm

if __name__ == '__main__':
    action_dict = create_stage_2_clients(5)
    sm = create_state_machine(action_dict)
    sm.execute()