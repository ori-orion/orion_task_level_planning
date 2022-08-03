#!/usr/bin/env python3
""" Code for the serve the breakfast task.

This file contains the state machine code for the serve the breakfast task.

Author: Charlie Street
Owner: Charlie Street
"""

import rospy
import smach
import actionlib

from orion_task_level_planning.state_machines.src.state_machines.reusable_states_deprecated import * # pylint: disable=unused-wildcard-import
from set_up_clients import create_stage_1_clients
from orion_actions.msg import SOMObservation, Relation


class ChooseNextItemState(ActionServiceState):
    """ State for choosing the next item to grab. """

    def __init__(self, action_dict, global_store):
        outcomes = ['ITEM_CHOSEN', 'NONE_LEFT']
        super(ChooseNextItemState, self).__init__(action_dict=action_dict,
                                                  global_store=global_store,
                                                  outcomes=outcomes)
    
    def execute(self, userdata):
        if self.global_store['next_item'] >= len(self.global_store['items']):
            return self._outcomes[1]
        else:
            item = self.global_store['items'][self.global_store['next_item']]
            self.global_store['current_item'] = item
            self.global_store['next_item'] += 1
            self.global_store['pick_up'] = item
            return self._outcomes[0]


class ItemDecisionState(ActionServiceState):
    """ State for choosing what we should do with the grasped object. """

    def __init__(self, action_dict, global_store):
        outcomes = ['BOWL', 'POUR', 'SPOON']
        super(ItemDecisionState, self).__init__(action_dict=action_dict,
                                                global_store=global_store,
                                                outcomes=outcomes)
    
    def execute(self, userdata):
        if self.global_store['current_item'] == 'bowl':
            return self._outcomes[0]
        elif self.global_store['current_item'] == 'spoon':
            self.global_store['rel_pos'] = ('bowl', 0.0, 0.2, 0.0)
            return self._outcomes[2]
        else: # Pour relative to the bowl
            self.global_store['pour_into'] = 'bowl'
            return self._outcomes[1]

def set_nav_goal_to_kitchen_item(item, action_dict):
    """ Function acts as a wrapper for get_location_of_object.

    Args:
        item: The string of the object
        action_dict: The dictionary giving access to services
    """

    obj1 = SOMObservation()
    obj1.type = item
    obj1.room_name = 'kitchen'

    rel = Relation()
    obj2 = SOMObservation()
    
    return get_location_of_object(action_dict, obj1, rel, obj2)
    

def create_state_machine(action_dict):
    """ Function creates and returns the state machine for the task. """

    # Initialise the global store
    global_store = {}
    global_store['items'] = ['bowl', 'cereal', 'spoon', 'milk']
    global_store['next_item'] = 0
    global_store['current_item'] = None

    # Create the state machine
    sm = smach.StateMachine(outcomes=['TASK_SUCCESS', 'TASK_FAILURE'])

    with sm:

        # Get the robot to start talking        
        phrase = "Hi, I'm Bam Bam, lets store some groceries!"
        smach.StateMachine.add('StartTalking',
                               SpeakState(action_dict, global_store, phrase),
                               transitions={'SUCCESS':'WaitForDoor',
                                            'FAILURE':'WaitForDoor'})
        
        # Wait for door to open
        smach.StateMachine.add('WaitForDoor',
                               CheckDoorIsOpenState(action_dict, global_store),
                               transitions={'OPEN':'SetNavToKitchen',
                                            'CLOSED':'WaitForDoor'})
                    
        # Set nav goal to kitchen
        func = lambda : (get_pose_of_node(get_node_with_label(action_dict, 
                                                              'kitchen')))
        smach.StateMachine.add('SetNavToKitchen',
                               SetNavGoalState(action_dict, global_store, func),
                               transitions={'SUCCESS':'NavToKitchen'})
        
        # Navigate to kitchen
        smach.StateMachine.add('NavToKitchen',
                               NavigateState(action_dict, global_store),
                               transitions={'SUCCESS':'ChooseNextItem',
                                            'FAILURE':'NavToKitchen',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Find next item
        smach.StateMachine.add('ChooseNextItem',
                               ChooseNextItemState(action_dict, global_store),
                               transitions={'ITEM_CHOSEN':'SetNavToItem',
                                            'NONE_LEFT':'TASK_SUCCESS'})
        
        # Set nav goal to next item
        func = lambda: set_nav_goal_to_kitchen_item(global_store['current_item']
                                                    ,action_dict)
        smach.StateMachine.add('SetNavToItem',
                               SetNavGoalState(action_dict, global_store, func),
                               transitions={'SUCCESS':'NavToItem'})
        
        # Navigate to the item
        smach.StateMachine.add('NavToItem',
                               NavigateState(action_dict, global_store),
                               transitions={'SUCCESS':'GraspItem',
                                            'FAILURE':'NavToItem',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Pick Up the item
        smach.StateMachine.add('GraspItem',
                               PickUpObjectState(action_dict, global_store),
                               transitions={'SUCCESS':'SetNavToSurface',
                                            'FAILURE':'AskForGraspHelp'})
        
        # Ask for help picking up object
        question = ("I can't seem to pick up this item. Can you help me please?"
                     + " If you can, please tell me when you're ready " + 
                     " to hand it to me.")
        smach.StateMachine.add('AskForGraspHelp',
                               SpeakAndHotwordState(action_dict,
                                                   global_store,
                                                   question,
                                                   ['ready'],
                                                   20),
                               transitions={'SUCCESS':'ReceiveObject',
                                            'FAILURE':'AskForGraspHelp',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Receive object
        smach.StateMachine.add('ReceiveObject',
                               ReceiveObjectFromOperatorState(action_dict,
                                                              global_store),
                               transitions={'SUCCESS':'SetNavToSurface',
                                            'FAILURE':'AskForGraspHelp'})
        
        # Set nav goal to surface
        func = lambda : set_nav_goal_to_kitchen_item('table', action_dict)
        smach.StateMachine.add('SetNavToSurface',
                               SetNavGoalState(action_dict, global_store, func),
                               transitions={'SUCCESS':'NavToSurface'})
        

        # Navigate to surface
        smach.StateMachine.add('NavToSurface',
                               NavigateState(action_dict, global_store),
                               transitions={'SUCCESS':'ItemDecision',
                                            'FAILURE':'NavToSurface',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Decide what to do with object
        smach.StateMachine.add('ItemDecision',
                               ItemDecisionState(action_dict, global_store),
                               transitions={'BOWL':'PlaceBowl',
                                            'POUR':'PourObject',
                                            'SPOON':'PlaceNearBowl'})
        
        # If bowl
        smach.StateMachine.add('PlaceBowl',
                              PutObjectOnSurfaceState(action_dict, 
                                                      global_store),
                              transitions={'SUCCESS':'ChooseNextItem',
                                           'FAILURE':'AskForHelpBowl'})
        
        # Ask for help with bowl
        question = ("I appear to be struggling to put the bowl down. " +
                    "Can someone help me? If you can, tell me when you're " +
                    "ready for me to pass you the bowl.")
        smach.StateMachine.add('AskForHelpBowl',
                               SpeakAndHotwordState(action_dict,
                                                   global_store,
                                                   question,
                                                   ['ready'],
                                                   20),
                               transitions={'SUCCESS':'HandoverBowl',
                                            'FAILURE':'AskForHelpBowl',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        

        # Handover bowl
        smach.StateMachine.add('HandoverBowl',
                               HandoverObjectToOperatorState(action_dict,
                                                             global_store),
                               transitions={'SUCCESS':'ChooseNextItem',
                                            'FAILURE':'AskForHelpBowl'})
        
        # Pour object
        smach.StateMachine.add('PourObject',
                               PourIntoState(action_dict, global_store),
                               transitions={'SUCCESS':'ChooseNextItem',
                                            'FAILURE':'AskForHelpPour'})
        
        # Ask For help pouring
        question = ("I appear to be struggling to pour this." +
                    "Can someone help me? If you can, tell me when you're " +
                    "ready for me to pass it to you.")
        smach.StateMachine.add('AskForHelpPour',
                               SpeakAndHotwordState(action_dict,
                                                   global_store,
                                                   question,
                                                   ['ready'],
                                                   20),
                               transitions={'SUCCESS':'HandoverPourable',
                                            'FAILURE':'AskForHelpPour',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})

        # Hand over pourable thing
        smach.StateMachine.add('HandoverPourable',
                               HandoverObjectToOperatorState(action_dict,
                                                             global_store),
                               transitions={'SUCCESS':'ChooseNextItem',
                                            'FAILURE':'AskForHelpPour'})   

        # Place spoon near bowl
        smach.StateMachine.add('PlaceNearBowl',
                               PlaceObjectRelativeState(action_dict, 
                                                        global_store),
                               transitions={'SUCCESS':'ChooseNextItem',
                                            'FAILURE':'AskForHelpSpoon'})  
                 
        # Ask For help with spoon
        question = ("I appear to be struggling to pour the spoon." +
                    "Can someone help me? If you can, tell me when you're " +
                    "ready for me to pass you the spoon.")
        smach.StateMachine.add('AskForHelpSpoon',
                               SpeakAndHotwordState(action_dict,
                                                   global_store,
                                                   question,
                                                   ['ready'],
                                                   20),
                               transitions={'SUCCESS':'HandoverSpoon',
                                            'FAILURE':'AskForHelpSpoon',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})

        # Hand over spoon
        smach.StateMachine.add('HandoverSpoon',
                               HandoverObjectToOperatorState(action_dict,
                                                             global_store),
                               transitions={'SUCCESS':'ChooseNextItem',
                                            'FAILURE':'AskForHelpSpoon'}) 
    
    return sm


if __name__ == '__main__':
    action_dict = create_stage_1_clients(8)
    sm = create_state_machine(action_dict)
    sm.execute()