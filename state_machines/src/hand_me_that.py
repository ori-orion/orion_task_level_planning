#!/usr/bin/env python
""" Code for the Hand Me That Task.

This file contains the state machine code for the Hand Me That task.

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


class DetectPointedObjectsState(ActionServiceState):
    """ Detects Pointed objects in vicinity of point. """
    def __init__(self, action_dict, global_store):
        outcomes = ['SUCCESS', 'FAILURE']
        super(DetectPointedObjectsState, self).__init__(action_dict=action_dict,
                                                        global_store=
                                                        global_store,
                                                        outcomes=outcomes)

    def execute(self, userdata):
        self.action_dict['GetPointedObject'].send_goal(PointingGoal())
        self.action_dict['GetPointedObject'].wait_for_result()
        
        result_point = self.action_dict['GetPointedObject'].get_result()
        if not result_point.is_present:
            return self._outcomes[1]
        
        objects = result_point.pointing_array[0].pointings.detections
        self.global_store['pointed_objects'] = objects

        return self._outcomes[0] 


class NextQuestionState(ActionServiceState):
    """ Determines what the next action by the robot should be and does it. """
    def __init__(self, action_dict, global_store):
        """ The outcomes mean the following:
            * CORRECT: Guessed the object and is correct
            * INCORRECT: Guessed the object and is incorrect
            * ANSWERED: Asked a question and got an answer
            * GIVE_UP: Gave up with an object
            * FINISHED: Finished all 5 objects
        """
        outcomes = ['CORRECT', 'INCORRECT', 'ANSWERED', 'GIVE_UP', 'FINISHED']
        super(NextQuestionState, self).__init__(action_dict=action_dict,
                                                global_store=global_store,
                                                outcomes=outcomes)
    
    def execute(self, userdata):
        # TODO: Fill in!
        # Should do the following:
        # Look at the list of pointed objects
        # If only one possibility, say that this is the object
        # Speak and listen, yes and no,
        # If yes, correct (reset pointed objects), if no, incorrect
        # If multiple possibilities, generate question and ask it
        # Get yes or no response.
        # If too many guesses, give up
        # If all 5 objects done, return finished
        return self._outcomes[0]



def create_state_machine(action_dict):
    """ This function creates and returns the state machine for this task. """

    # Initialise global_store
    global_store = {}
    global_store['pointed_objects'] = None
    global_store['question'] = []

    # Create the state machine
    sm = smach.StateMachine(outcomes=['TASK_SUCCESS', 'TASK_FAILURE'])

    with sm:

        # Get name off the operator
        question = "Hi, I'm Bam Bam, what is your name?"
        smach.StateMachine.add('StartTalking',
                               SpeakAndListenState(action_dict,
                                                   global_store,
                                                   question,
                                                   NAMES,
                                                   [],
                                                   20),
                               transitions={'SUCCESS': 'DetectOperator',
                                            'FAILURE': 'StartTalking',
                                            'REPEAT_FAILURE': 'TASK_FAILURE'})
        
        # Detect and memorise the operator
        smach.StateMachine.add('DetectOperator',
                               OperatorDetectState(action_dict, global_store),
                               transitions={'SUCCESS': 'WhatDoYouWant',
                                            'FAILURE': 'StartTalking'})
        
        # Ask what does the operator want
        phrase = ("What do you need? I will follow you to what you want. " +
                 "Please say Bam Bam when you have arrived")
        smach.StateMachine.add('WhatDoYouWant',
                               SpeakState(action_dict, global_store, phrase),
                               transitions={'SUCCESS':'Follow',
                                            'FAILURE':'Follow'})
        
        # Follow the operator until they arrive
        smach.StateMachine.add('Follow',
                               make_follow_hotword_state(action_dict,
                                                         global_store),
                               transitions={'SUCCESS': 'AskToPoint',
                                            'FAILURE': 'Follow',
                                            'REPEAT_FAILURE': 'TASK_FAILURE'})
        
        # Ask the operator to point to the object
        phrase = "Please point at the object you need."
        smach.StateMachine.add('AskToPoint',
                               SpeakState(action_dict, global_store, phrase),
                               transitions={'SUCCESS': 'DetectPointedObjects',
                                            'FAILURE': 'DetectPointedObjects'})
        
        # Detect Pointed Objects
        smach.StateMachine.add('DetectPointedObjects',
                               DetectPointedObjectsState(action_dict, 
                                                         global_store),
                               transitions={'SUCCESS': 'NextQuestion',
                                            'FAILURE': 'AskToPoint'})

        smach.StateMachine.add('NextQuestion',
                               NextQuestionState(action_dict, global_store),
                               transitions={'CORRECT': 'WhatDoYouWant',
                                            'INCORRECT': 'NextQuestion',
                                            'ANSWERED': 'NextQuestion',
                                            'GIVE_UP': 'WhatDoYouWant',
                                            'FINISHED': 'TASK_SUCCESS'})
    
    return sm


if __name__ == '__main__':
    action_dict = create_stage_2_clients(4)
    sm = create_state_machine(action_dict)
    sm.execute()