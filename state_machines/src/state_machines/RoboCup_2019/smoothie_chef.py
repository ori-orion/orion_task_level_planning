#!/usr/bin/env python3
""" Code for the Smoothie Chef task.

This file contains the state machine code for the Smoothie Chef task.

Author: Charlie Street
Owner: Charlie Street
"""

import rospy
import smach
import actionlib
import time

from orion_task_level_planning.state_machines.src.state_machines.reusable_states_deprecated import * # pylint: disable=unused-wildcard-import
from set_up_clients import create_stage_2_clients
from orion_actions.msg import SOMObservation, Relation

def go_to_kitchen_counter(action_dict):
    """ Gets location of kitchen counter. """
    obj1 = SOMObservation()
    obj1.type = 'smoothie_chef_point_of_interest'

    return get_location_of_object(action_dict, obj1, 
                                  Relation(), SOMObservation())


class MemoriseRecipeState(ActionServiceState):
    """ This state watches the operator make a smoothie. 
        Returns the sequence of fruits."""

    def __init__(self, action_dict, global_store):
        outcomes = ['SUCCESS']
        super(MemoriseRecipeState, self).__init__(action_dict=action_dict,
                                                  global_store=global_store,
                                                  outcomes=outcomes)
        
    def execute(self, userdata):

        # TODO : Call action server and get response
        return self._outcomes[0]


class ChooseIngredientState(ActionServiceState):
    """ This state chooses the next ingredient to add to the smoothie. """

    def __init__(self, action_dict, global_store):
        outcomes = ['FRUIT','SUGAR','MILK','NO_MORE','DONT_KNOW']
        super(ChooseIngredientState, self).__init__(action_dict=action_dict,
                                                    global_store=global_store,
                                                    outcomes=outcomes)
        self.asked_for_help = False
        self.global_store['rel_pos'] = ('blender', 0.0, 0.0, 0.2) # TODO: Check
        self.global_store['pour_into'] = 'blender'
    
    def execute(self, userdata):
        
        if self.global_store['next_item'] < 3:
            fruits = self.global_store['recipe']
            if len(fruits) != 3: # Bad information
                if not self.asked_for_help:
                    return self._outcomes[4]
                else:
                    self.global_store['pick_up'] = \
                        self.global_store['last_response']
                    self.global_store['next_item'] += 1
                    return self._outcomes[0]
            else:
                self.global_store['pick_up'] = \
                    fruits[self.global_store['next_item']]
                self.global_store['next_item'] += 1
                return self._outcomes[0]

        elif self.global_store['next_item'] == 3: # sugar
            self.global_store['pick_up'] = 'spoon'
            self.global_store['next_item'] += 1
            return self._outcomes[1]

        elif self.global_store['next_item'] == 4: # milk
            self.global_store['pick_up'] = 'milk'
            self.global_store['next_item'] += 1
            return self._outcomes[2]
        else: # Done
            return self._outcomes[3]


class PourSugarState(ActionServiceState):

    def __init__(self, action_dict, global_store):
        outcomes = ['SUCCESS', 'FAILURE']
        super(PourSugarState, self).__init__(action_dict=action_dict,
                                             global_store=global_store,
                                             outcomes=outcomes)
    
    def execute(self, userdata):
        # TODO: Fill in!
        return self._outcomes[1]


def create_state_machine(action_dict):
    """ This function creates and returns the state machine for this task. """

    # Initialise global store
    global_store = {}
    global_store['next_item'] = 0
    global_store['recipe'] = []

    # Create the state machine
    sm = smach.StateMachine(outcomes=['TASK_SUCCESS', 'TASK_FAILURE'])

    with sm:
        
        # Get the robot to start talking
        phrase = "Hi, I'm Bam Bam, lets make some smoothies!"
        smach.StateMachine.add('StartTalking',
                               SpeakState(action_dict, global_store, phrase),
                               transitions={'SUCCESS':'WaitForDoor',
                                            'FAILURE':'WaitForDoor'})

        # Wait for the door to open
        smach.StateMachine.add('WaitForDoor',
                               CheckDoorIsOpenState(action_dict, global_store),
                               transitions={'OPEN':'SetNavToKitchen',
                                            'CLOSED':'WaitForDoor'})
        
        # Set nav to kitchen counter 
        func = lambda : go_to_kitchen_counter(action_dict)
        smach.StateMachine.add('SetNavToKitchen',
                               SetNavGoalState(action_dict, global_store, func),
                               transitions={'SUCCESS':'NavToKitchen'})

        # Navigate to kitchen counter
        smach.StateMachine.add('NavToKitchen',
                               NavigateState(action_dict, global_store),
                               transitions={'SUCCESS':'AskForDemo',
                                            'FAILURE':'NavToKitchen',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})

        # Ask for a demo of the smoothie recipe
        phrase = ("Hello, please could you show me how to make a smoothie? " +
                 "Please say my name, Bam Bam, when you're finished.")
        smach.StateMachine.add('AskForDemo',
                               SpeakState(action_dict, global_store, phrase),
                               transitions={'SUCCESS':'Demo',
                                            'FAILURE':'Demo'})
            
        # Watch the demo
        smach.StateMachine.add('Demo',
                               MemoriseRecipeState(action_dict, global_store),
                               transitions={'SUCCESS':'WaitForName'})
        
        # wait for name
        smach.StateMachine.add('WaitForName',
                               HotwordListenState(action_dict, 
                                                  global_store, ['bambam'], 120),
                               transitions={'SUCCESS': 'ChooseIngredient',
                                            'FAILURE': 'WaitForName'})
        
        # Choose next ingredient
        smach.StateMachine.add('ChooseIngredient',
                               ChooseIngredientState(action_dict, global_store),
                               transitions={'FRUIT': 'PickUpFruit',
                                            'SUGAR': 'PickUpSpoon',
                                            'MILK': 'PickUpMilk',
                                            'NO_MORE': 'SpeakFinish',
                                            'DONT_KNOW':'AskForIngredientHelp'})

        # If no more items
        phrase = ("Looks like I've just made a delicious smoothie! " + 
                 "Thanks for showing me how, good bye!")
        smach.StateMachine.add('SpeakFinish',
                               SpeakState(action_dict, global_store, phrase),
                               transitions={'SUCCESS':'TASK_SUCCESS',
                                            'FAILURE':'TASK_SUCCESS'})
        
        # If don't know
        phrase = ("I'm not sure what I should be adding next, could you tell " +
                  "what I should add please?")
        smach.StateMachine.add('AskForIngredientHelp',
                               SpeakAndListenState(action_dict,
                                                   global_store,
                                                   phrase,
                                                   FRUITS + ['sugar', 'milk'],
                                                   [],
                                                   20),
                               transitions={'SUCCESS':'ChooseIngredient',
                                            'FAILURE':'AskForIngredientHelp',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # If fruit, pick up
        smach.StateMachine.add('PickUpFruit',
                               PickUpObjectState(action_dict, global_store),
                               transitions={'SUCCESS':'PlaceFruitInBlender',
                                            'FAILURE':'AskForFruitPickHelp'})
        
        # get help with fruit
        question = ("I can't seem to pick up this fruit, could you please help"+
                   " me and let me know when you're ready to hand it over?")
        smach.StateMachine.add('AskForFruitPickHelp',
                               SpeakAndHotwordState(action_dict,
                                                   global_store,
                                                   question,
                                                   ['ready'],
                                                   20),
                               transitions={'SUCCESS':'ReceiveFruit',
                                            'FAILURE':'AskForFruitPickHelp',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # receive fruit
        smach.StateMachine.add('ReceiveFruit',
                               ReceiveObjectFromOperatorState(action_dict,
                                                              global_store),
                               transitions={'SUCCESS':'PlaceFruitInBlender',
                                            'FAILURE':'AskForFruitPickHelp'})
                                        

        # Place Fruit In Blender
        smach.StateMachine.add('PlaceFruitInBlender',
                               PlaceObjectRelativeState(action_dict, 
                                                        global_store),
                               transitions={'SUCCESS':'ChooseIngredient',
                                            'FAILURE':'AskForFruitPlaceHelp'})
        
        # Ask for fruit based help
        question = ("I can't get this fruit in the blender. Can you help me " +
                   "please and let me know when you're ready to receive " +
                   "the fruit?")
        smach.StateMachine.add('AskForFruitPlaceHelp',
                               SpeakAndHotwordState(action_dict,
                                                   global_store,
                                                   question,
                                                   ['ready'],
                                                   20),
                               transitions={'SUCCESS':'HandoverFruit',
                                            'FAILURE':'AskForFruitPlaceHelp',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Handover fruit to operator
        smach.StateMachine.add('HandoverFruit',
                               HandoverObjectToOperatorState(action_dict, 
                                                             global_store),
                               transitions={'SUCCESS':'ChooseIngredient',
                                            'FAILURE':'AskForFruitPlaceHelp'})
        
        # If spoon/sugar
        smach.StateMachine.add('PickUpSpoon',
                               PickUpObjectState(action_dict, global_store),
                               transitions={'SUCCESS':'PourSugar',
                                            'FAILURE':'AskForSpoonHelp'})
        

        # get help with spoon
        question = ("I can't seem to pick up this spoon, could you please help"+
                   " me and let me know when you're ready to hand it over?")
        smach.StateMachine.add('AskForSpoonHelp',
                               SpeakAndHotwordState(action_dict,
                                                   global_store,
                                                   question,
                                                   ['ready'],
                                                   20),
                               transitions={'SUCCESS':'ReceiveSpoon',
                                            'FAILURE':'AskForSpoonHelp',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # receive spoon
        smach.StateMachine.add('ReceiveSpoon',
                               ReceiveObjectFromOperatorState(action_dict,
                                                              global_store),
                               transitions={'SUCCESS':'PourSugar',
                                            'FAILURE':'AskForSpoonHelp'})

        # Pour some sugar on me...
        smach.StateMachine.add('PourSugar',
                               PourSugarState(action_dict, global_store),
                               transitions={'SUCCESS':'ChooseIngredient',
                                            'FAILURE':'AskForSugarHelp'})
        
        # Ask for sugar based help
        question = ("I can't get this sugar in the blender. Can you do it for "+
                   "me please and let me know when you're ready so I can give "+
                   "you the spoon?")
        smach.StateMachine.add('AskForSugarHelp',
                               SpeakAndListenState(action_dict,
                                                   global_store,
                                                   question,
                                                   ['Done'],
                                                   [],
                                                   20),
                               transitions={'SUCCESS':'HandoverSpoon',
                                            'FAILURE':'AskForSugarHelp',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Handover the spoon
        smach.StateMachine.add('HandoverSpoon',
                               HandoverObjectToOperatorState(action_dict, 
                                                             global_store),
                               transitions={'SUCCESS':'ChooseIngredient',
                                            'FAILURE':'AskForSugarHelp'})

        # Pick up the milk
        smach.StateMachine.add('PickUpMilk',
                               PickUpObjectState(action_dict, global_store),
                               transitions={'SUCCESS':'PourMilk',
                                            'FAILURE':'AskForMilkHelp'})
        
         # get help with milk
        question = ("I can't seem to pick up this milk, could you please help"+
                   " me and let me know when you're ready to hand it over?")
        smach.StateMachine.add('AskForMilkHelp',
                               SpeakAndListenState(action_dict,
                                                   global_store,
                                                   question,
                                                   READY,
                                                   [],
                                                   20),
                               transitions={'SUCCESS':'ReceiveMilk',
                                            'FAILURE':'AskForMilkHelp',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # receive milk
        smach.StateMachine.add('ReceiveMilk',
                               ReceiveObjectFromOperatorState(action_dict,
                                                              global_store),
                               transitions={'SUCCESS':'PourMilk',
                                            'FAILURE':'AskForMilkHelp'})
        
        # Pour milk
        smach.StateMachine.add('PourMilk',
                               PourIntoState(action_dict, global_store),
                               transitions={'SUCCESS':'ChooseIngredient',
                                            'FAILURE':'AskForMilkPourHelp'})

        # Ask for milk based help
        question = ("I can't pour this milk in the blender. Can you help me " +
                   "please and let me know when you're ready to receive " +
                   "the milk?")
        smach.StateMachine.add('AskForMilkPourHelp',
                               SpeakAndHotwordState(action_dict,
                                                   global_store,
                                                   question,
                                                   ['ready'],
                                                   20),
                               transitions={'SUCCESS':'HandoverMilk',
                                            'FAILURE':'AskForMilkPourHelp',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Handover milk to operator
        smach.StateMachine.add('HandoverMilk',
                               HandoverObjectToOperatorState(action_dict, 
                                                             global_store),
                               transitions={'SUCCESS':'ChooseIngredient',
                                            'FAILURE':'AskForMilkPourHelp'})

    return sm

if __name__ == '__main__':
    action_dict = create_stage_2_clients(7)
    sm = create_state_machine(action_dict)
    sm.execute()
