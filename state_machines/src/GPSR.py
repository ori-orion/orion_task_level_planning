#!/usr/bin/env python
""" File for main state machine of GPSR task.

This file contains the code for the main part of the
GPSR task. The grammar stuff is dealt with elsewhere.

Author: Charlie Street
Owner: Charlie Street
"""

import rospy
import smach
import actionlib
import time

from reusable_states import * # pylint: disable=unused-wildcard-import
from set_up_clients import create_stage_1_clients
from orion_actions.msg import SOMObservation, Relation
from GPSR_parser import GPSRParser
from GPSR_utils import parse_string


class ExecuteTaskState(ActionServiceState):
    """ State to execute the task given by the operator. """
    
    def __init__(self, action_dict, global_store):
        outcomes = ['CONTINUE', 'STOP']
        super(ExecuteTaskState, self).__init__(action_dict=action_dict,
                                               global_store=global_store,
                                               outcomes=outcomes)
    
    def execute(self, userdata):
        
        parser = GPSRParser()
        task_string = self.global_store['last_response']

        # Get the state machine by parsing the string
        sm = parse_string(parser, task_string)

        # If something has gone wrong with the parsing
        if not isinstance(sm, smach.StateMachine):
            return self._outcomes[1]
        
        # Execute the state machine
        outcome = sm.execute()

        # If we've failed then stop so we don't make things worse
        if outcome == 'TASK_FAILURE':
            return self._outcomes[1]

        self.global_store['tasks_completed'] += 1
        if self.global_store['tasks_completed'] == 3:
            return self._outcomes[1]
        elif time.time() - self.global_store['start_time'] >= 240:
            return self._outcomes[0]


def set_nav_goal_to_instruction_point(action_dict):
    """ A function to return the location of the instruction point. """
    obj1 = SOMObservation()
    obj1.type = 'GPSR_point_of_interest'

    return get_location_of_object(action_dict, obj1, 
                                  Relation(), SOMObservation())


def create_state_machine(action_dict):
    """ Function creates and returns the state machine for the GPSR task. """

    # Initialise global store
    global_store = {}
    global_store['start_time'] = time.time()
    global_store['tasks_completed'] = 0

    sm = smach.StateMachine(outcomes=['TASK_SUCCESS', 'TASK_FAILURE'])

    with sm:

        # Set nav point to instruction point
        func = lambda: set_nav_goal_to_instruction_point(action_dict)
        smach.StateMachine.add('SetNavToInstructionPoint',
                               SetNavGoalState(action_dict, global_store, func),
                               transitions={'SUCCESS':'StoreInitialLocation'})

        # Store initial location
        smach.StateMachine.add('StoreInitialLocation',
                               GetRobotLocationState(action_dict, global_store),
                               transitions={'STORED':'StartSpeak'})


        # Start talking
        phrase = "I can't wait to get going with this task!"
        smach.StateMachine.add('StartSpeak',
                               SpeakState(action_dict, global_store, phrase),
                               transitions={'SUCCESS':'WaitForDoor',
                                            'FAILURE':'WaitForDoor'})

        # Wait for the door to be opened
        smach.StateMachine.add('WaitForDoor',
                               CheckDoorIsOpenState(action_dict, global_store),
                               transitions={'OPEN':'NavToInstructionPoint',
                                            'CLOSED':'WaitForDoor'})
        
        # Navigate to the instruction point
        smach.StateMachine.add('NavToInstructionPoint',
                               NavigateState(action_dict, global_store),
                               transitions={'SUCCESS':'AskForTask',
                                            'FAILURE':'NavToInstructionPoint',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Ask for a task
        question = "Hi there, do you have a task for me?"
        smach.StateMachine.add('AskForTask',
                               SpeakAndListenState(action_dict,
                                                   global_store,
                                                   question,
                                                   ['Do this'],
                                                   [],
                                                   30),
                               transitions={'SUCCESS':'ExecuteTask',
                                            'FAILURE':'AskForTask',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Execute that task
        smach.StateMachine.add('ExecuteTask',
                               ExecuteTaskState(action_dict, global_store),
                               transitions={'CONTINUE':'NavToInstructionPoint',
                                            'STOP':'SetNavOutOfArena'})
        
        # Set nav out of arena
        func = lambda: global_store['stored_location']
        smach.StateMachine.add('SetNavOutOfArena',
                               SetNavGoalState(action_dict, global_store, func),
                               transitions={'SUCCESS':'NavOutOfArena'})

        # Navigate out of arena
        smach.StateMachine.add('NavOutOfArena',
                               NavigateState(action_dict, global_store),
                               transitions={'SUCCESS':'TASK_SUCCESS',
                                            'FAILURE':'NavOutOfArena',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
         

    return sm


if __name__ == '__main__':
    action_dict = create_stage_1_clients(5)
    sm = create_state_machine(action_dict)
    sm.execute()