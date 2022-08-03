#!/usr/bin/env python3
""" Code for the Storing Groceries task.

This file contains the state machine code for the Storing Groceries task.

Author: Charlie Street
Owner: Charlie Street
"""

import rospy
import smach
import actionlib
import numpy as np
import time

from orion_task_level_planning.state_machines.src.state_machines.reusable_states_deprecated import * # pylint: disable=unused-wildcard-import
from set_up_clients import create_stage_1_clients
from orion_actions.msg import SOMObservation, Relation
from orion_actions.msg import GetClosestObjectNameGoal


class FindHandleState(ActionServiceState):
    """ State for finding the handle of a shelf in front of us. """
    
    def __init__(self, action_dict, global_store):
        outcomes = ['SUCCESS', 'FAILURE']
        super(FindHandleState, self).__init__(action_dict=action_dict,
                                              global_store=global_store,
                                              outcomes=outcomes)
    
    def execute(self, userdata):
        self.global_store['drawer_handle'] = 'cupboard'
        return self._outcomes[0]


class PickUpClosestItemState(ActionServiceState):
    """ State for picking up the closest item to us. """
    
    def __init__(self, action_dict, global_store):
        outcomes = ['SUCCESS', 'FAILURE', 'NO_ITEMS']
        super(PickUpClosestItemState, self).__init__(action_dict=action_dict,
                                                     global_store=global_store,
                                                     outcomes=outcomes)
    
    def execute(self, userdata):
        
        goal = GetClosestObjectNameGoal()       # todo - what is this closest to?
        self.action_dict['GetClosestObjectName'].send_goal(goal)
        self.action_dict['GetClosestObjectName'].wait_for_result()
        obj = self.action_dict['GetClosestObjectName'].get_result().object

        if obj == '':
            return self._outcomes[2]

        self.global_store['pick_up'] = obj

        pick_up_goal = PickUpObjectGoal()
        pick_up_goal.goal_tf = self.global_store['pick_up']

        self.action_dict['PickUpObject'].send_goal(pick_up_goal)
        self.action_dict['PickUpObject'].wait_for_result()

        result = self.action_dict['PickUpObject'].get_result().goal_complete

        if result:
            return self._outcomes[0]
        else:
            return self._outcomes[1]


class DecideItemPositionState(ActionServiceState):
    """ State for deciding the position of the item in our hand """
    
    def __init__(self, action_dict, global_store):
        outcomes = ['SUCCESS', 'FAILURE']
        super(DecideItemPositionState, self).__init__(action_dict=action_dict,
                                                      global_store=global_store,
                                                      outcomes=outcomes)
    
    def execute(self, userdata):

        # TODO - Matthew Munks will combine the query to check for if in shelf and of type of category
        # category set in SOM when logging the perception output
        # 2) replace logic for deciding position on shelf to put object
        #   - query SOM to find region which contains similar class (eg shelf where drinks are)
        #   - do some logic to find out, between the shelf region bounds, where free space is, so we can decide where to place this object
        #   - check that there is free space in the collision map to place the object in (of certain size - either from SOM entry or assumed width)
        
        # Get objects near the cupboard
        rel = Relation()
        rel.near = True
        cupboard = SOMObservation()
        cupboard.type = 'cupboard'      #TODO - change .type to .class_
        pose = rospy.wait_for_message('/global_pose', PoseStamped)
        pose = pose.pose

        matches = self.action_dict['SOMQuery'](SOMObservation(), rel, 
                                               cupboard, pose)
        matches = matches.matches

        # Find the most similar item
        closest_item = None
        obj_loc = None
        sim = float('inf')
        link = None

        for match in matches:
            obj_type = match.obj1.type

            obj1 = SOMObservation(type=self.global_store['pick_up'])
            obj2 = SOMObservation(type=obj_type)

            sim_result = self.action_dict['SOMCheckSimilarity'](obj1, obj2)
            current_sim = sim_result.similarity
            common_type = sim_result.common_type

            if current_sim < sim:
                closest_item = obj_type
                sim = current_sim
                link = common_type
                obj_loc = match.obj1.pose_estimate.most_likely_pose

        if closest_item == None:
            return self._outcomes[1] # TODO - change to string literal

        # Find the leftmost and rightmost items in the cupboard
        rel_left = Relation()
        rel_left.near = True
        rel_left.left_most = True
        left_result = self.action_dict['SOMQuery'](SOMObservation(), rel_left,
                                                   cupboard, pose)
        left_result = left_result.matches[0].obj1
        left_pose = left_result.pose_estimate.most_likely_pose

        rel_right = Relation()
        rel_right.near = True
        rel_right.right_most = True
        right_result = self.action_dict['SOMQuery'](SOMObservation(), rel_right,
                                                    cupboard, pose)
        
        right_result = right_result.matches[0].obj1
        right_pose = right_result.pose_estimate.most_likely_pose

        # Decide rel pos
        left_to_right = (right_pose.position.x - left_pose.position.x, 
                         right_pose.position.y - left_pose.position.y)
        mag = np.sqrt(np.power(left_to_right[0], 2), 
                      np.power(left_to_right[1], 2))
        left_to_right = (left_to_right[0]/mag, left_to_right[1]/mag)
        
        distance_to_left = np.sqrt(np.power(left_pose.position.x - \
                                          obj_loc.position.x ,2) + 
                                          np.power(left_pose.position.y - \
                                                 obj_loc.position.y,2) +
                                                 np.power(left_pose.position.z-\
                                                        obj_loc.position.z, 2))
                    
        distance_to_right = np.sqrt(np.power(right_pose.position.x - \
                                          obj_loc.position.x ,2) + 
                                          np.power(right_pose.position.y - \
                                                 obj_loc.position.y,2) +
                                                 np.power(right_pose.position.z-\
                                                        obj_loc.position.z, 2))

        if distance_to_left < distance_to_right: # Place to right of object
            rel_pos = (closest_item,left_to_right[0]*0.2, 
                       left_to_right[1]*0.2,0.0)
        else: # Place to left of object
            rel_pos = (closest_item,left_to_right[0]*-0.2, 
                       left_to_right[1]*-0.2,0.0)

        self.global_store['rel_pos'] = rel_pos

        # Speak to state intentions
        obj = self.global_store['pick_up']
        obj.replace('_', ' ')
        closest = closest_item.replace('_', ' ')
        link = link.replace('_', ' ')
        sentence = ('I will put the ' + obj + ' next to the ' + closest + 
                    ' as they are both ' + link)
        speak_goal = TalkRequestGoal()
        speak_goal.data.language = Voice.kEnglish
        speak_goal.data.sentence = sentence
        self.action_dict['Speak'].send_goal(speak_goal)
        self.action_dict['Speak'].wait_for_result()

        return self._outcomes[0]


class UpdateItemInfoState(ActionServiceState):
    """ State for updating the info of the object we hold given help. """
    
    def __init__(self, action_dict, global_store):
        outcomes = ['SUCCESS', 'FAILURE']
        super(UpdateItemInfoState, self).__init__(action_dict=action_dict,
                                                  global_store=global_store,
                                                  outcomes=outcomes)
    
    def execute(self, userdata):
        # operator voice response in the form: <relation> of <object>
        # eg: left of bottle
        response = self.global_store['last_response']   

        relation = None
        for rel in RELATIONS:
            if rel in response:
                relation = rel
                break
        
        if relation is None:
            return self._outcomes[1]

        obj = None
        for o in OBJECTS:
            if o in response:
                obj = o
                break
        
        if obj is None:
            return self._outcomes[1]
        
        obj = obj.replace(' ', '_') # For tf frames

        x = 0
        y = 0
        z = 0

        if relation == 'left':
            y = 0.2
        elif relation == 'right':
            y = -0.2
        elif relation == 'above':
            z = 0.3
        elif relation == 'below':
            z = -0.3
        elif relation == 'front':
            x = -0.2
        elif relation == 'behind':
            x = 0.2
        elif relation == 'near':
            y = 0.2
        else:
            return self._outcomes[1]
        
        self.global_store['rel_pos'] = (obj,x,y,z)


def go_to_shelf(action_dict):
    """ Set nav goal to shelf. Announced pre-task. """
    obj = SOMObservation()
    obj.type = 'storing_groceries_point_of_interest_shelf'

    return get_location_of_object(action_dict, obj, 
                                  Relation(), SOMObservation())

def go_to_table(action_dict):
    """ Set nav goal to table. Announced pre-task. """
    obj = SOMObservation()
    obj.type = 'storing_groceries_point_of_interest_table'

    return get_location_of_object(action_dict, obj, 
                                  Relation(), SOMObservation())


def create_state_machine(action_dict):
    """ This function creates and returns the state machine for this task. """

    # Initialise global store
    global_store = {}

    # Create the state machine
    sm = smach.StateMachine(outcomes=['TASK_SUCCESS', 'TASK_FAILURE'])

    with sm:
        
        # Start talking
        phrase = "Hi, I'm Bam Bam and its time to store some groceries!"
        smach.StateMachine.add('StartTalking',
                               SpeakState(action_dict, global_store, phrase),
                               transitions={'SUCCESS':'WaitForDoor',
                                            'FAILURE':'WaitForDoor'})
        
        # Wait for the door to be opened
        smach.StateMachine.add('WaitForDoor',
                               CheckDoorIsOpenState(action_dict, global_store),
                               transitions={'OPEN':'SetNavToShelf',
                                            'CLOSED':'WaitForDoor'})
        
        # Set the navigation goal to the shelf
        func = lambda : go_to_shelf(action_dict)
        smach.StateMachine.add('SetNavToShelf',
                               SetNavGoalState(action_dict, global_store, func),
                               transitions={'SUCCESS':'NavToShelf'})
        
        # Navigate to the shelf
        smach.StateMachine.add('NavToShelf',
                               NavigateState(action_dict, global_store),
                               transitions={'SUCCESS':'FindShelfHandle',
                                            'FAILURE':'NavToShelf',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Find shelf handle
        smach.StateMachine.add('FindShelfHandle',
                               FindHandleState(action_dict, global_store),
                               transitions={'SUCCESS':'OpenShelf',
                                            'FAILURE':'AskForShelfHelp'})
    
        # Open the shelf
        smach.StateMachine.add('OpenShelf',
                               OpenDrawerState(action_dict, global_store),
                               transitions={'SUCCESS':'SetNavToTable',
                                            'FAILURE':'AskForShelfHelp'})
        
        # Ask for help with the shelf
        question = ("Can someone open the shelf for me please? If you can, " +
                   "please say when you have opened it.")
        smach.StateMachine.add('AskForShelfHelp',
                               SpeakAndListenState(action_dict,
                                                   global_store,
                                                   question,
                                                   ['Opened', 'I have'],
                                                   [],
                                                   30),
                               transitions={'SUCCESS':'SetNavToTable',
                                            'FAILURE':'AskForShelfHelp',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Set the navigation to the table
        func = lambda: go_to_table(action_dict)
        smach.StateMachine.add('SetNavToTable',
                               SetNavGoalState(action_dict, global_store, func),
                               transitions={'SUCCESS':'NavToTable'})
        
        # Navigate to the table
        smach.StateMachine.add('NavToTable',
                               NavigateState(action_dict, global_store),
                               transitions={'SUCCESS':'PickUpClosestItem',
                                            'FAILURE':'NavToTable',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Pick up the closest item
        smach.StateMachine.add('PickUpClosestItem',
                               PickUpClosestItemState(action_dict,global_store),
                               transitions={'SUCCESS':'DecideItemPosition',
                                            'FAILURE':'AskForPickupHelp',
                                            'NO_ITEMS':'TASK_SUCCESS'})
        
        # Ask for help
        question = ("Could someone please get the item closest to me and " +
                    "tell me when you're ready to hand it to me?")
        smach.StateMachine.add('AskForPickupHelp',
                               SpeakAndHotwordState(action_dict,
                                                   global_store,
                                                   question,
                                                   ['ready'],
                                                   30),
                               transitions={'SUCCESS':'ReceiveItem',
                                            'FAILURE':'AskForPickupHelp',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Receive the item from an operator
        smach.StateMachine.add('ReceiveItem',
                               ReceiveObjectFromOperatorState(action_dict,
                                                              global_store),
                               transitions={'SUCCESS':'DecideItemPosition',
                                            'FAILURE':'AskForPickupHelp'})
        
        # Decide the items position on the shelf
        smach.StateMachine.add('DecideItemPosition',
                               DecideItemPositionState(action_dict,
                                                       global_store),
                               transitions={'SUCCESS':'SetNavBackToShelf',
                                            'FAILURE':'AskForPosHelp'})
        

        # Ask for help with position
        question = ("I don't know where to put this object. Can someone " +
                    "tell me where I should put it?")
        smach.StateMachine.add('AskForPosHelp',
                               SpeakAndListenState(action_dict,
                                                   global_store,
                                                   question,
                                                   map((lambda x: x+' <param>'),
                                                   RELATIONS),
                                                   OBJECTS,
                                                   20),
                               transitions={'SUCCESS':'UpdateItemInfo',
                                            'FAILURE':'AskForPosHelp',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
            
        # Update item information
        smach.StateMachine.add('UpdateItemInfo',
                               UpdateItemInfoState(action_dict, global_store),
                               transitions={'SUCCESS':'SetNavBackToShelf',
                                            'FAILURE':'AskForPosHelp'})

        # Set navigation back to shelf
        func = lambda : go_to_shelf(action_dict)
        smach.StateMachine.add('SetNavBackToShelf',
                               SetNavGoalState(action_dict, global_store, func),
                               transitions={'SUCCESS':'NavBackToShelf'})
        
        # Go back to the shelf
        smach.StateMachine.add('NavBackToShelf',
                               NavigateState(action_dict, global_store),
                               transitions={'SUCCESS':'PlaceObject',
                                            'FAILURE':'NavBackToShelf',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Place object relatively on shelf
        smach.StateMachine.add('PlaceObject',
                               PlaceObjectRelativeState(action_dict, 
                                                        global_store),
                               transitions={'SUCCESS':'SetNavToTable',
                                            'FAILURE':'AskForPlacementHelp'})

        # Ask for help with placing the object
        question = ("I'm having trouble putting this object on the shelf. " +
                    "Could you help me please?")
        smach.StateMachine.add('AskForPlacementHelp',
                               SpeakAndListenState(action_dict,
                                                   global_store,
                                                   question,
                                                   ['I will', 'yes'],
                                                   [],
                                                   20),
                               transitions={'SUCCESS':'HandoverObject',
                                            'FAILURE':'AskForPlacementHelp',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Handover object to operator
        smach.StateMachine.add('HandoverObject',
                               HandoverObjectToOperatorState(action_dict,
                                                             global_store),
                               transitions={'SUCCESS':'SetNavToTable',
                                            'FAILURE':'AskForPlacementHelp'})

    return sm

if __name__ == '__main__':
    action_dict = create_stage_1_clients(9)
    sm = create_state_machine(action_dict)
    sm.execute()