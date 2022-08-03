#!/usr/bin/env python3
""" Code for the Hand Me That Task.

This file contains the state machine code for the Hand Me That task.

Author: Charlie Street
Owner: Charlie Street
"""

import rospy
import smach
import actionlib
import time
import random

from orion_task_level_planning.state_machines.src.state_machines.reusable_states_deprecated import * # pylint: disable=unused-wildcard-import
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
        self.global_store['remaining_objects'] = objects

        return self._outcomes[0] 


def get_question(objects, already_asked):
    """ Gets a question to ask person and splits up objects into yes/no. """

    if already_asked == 0: # tall
        question = "Is the object relatively tall?"
        avg_height = 0.0
        for obj in objects:
            avg_height += obj.height 
        avg_height /= len(objects)

        yes = []
        no = []
        for obj in objects:
            if obj.height > avg_height:
                yes.append(obj)
            else:
                no.append(obj)
        return question, yes, no

    elif already_asked == 1: # wide
        question = "Is the object relatively wide?"
        avg_width = 0.0
        for obj in objects:
            avg_width += obj.width 
        avg_width /= len(objects)

        yes = []
        no = []
        for obj in objects:
            if obj.width > avg_width:
                yes.append(obj)
            else:
                no.append(obj)
        return question, yes, no

    elif already_asked == 2: # left
        question = "Is the object more to my left?"
        avg_pos = 0.0
        for obj in objects:
            avg_pos += obj.y
        avg_pos /= len(objects)
        yes = []
        no = []
        for obj in objects:
            if obj.y > avg_pos:
                yes.append(obj)
            else:
                no.append(obj)
        return question, yes, no

    elif already_asked == 3: # right
        question = "Is the object more to my right?"
        avg_pos = 0.0
        for obj in objects:
            avg_pos += obj.y
        avg_pos /= len(objects)
        yes = []
        no = []
        for obj in objects:
            if obj.y < avg_pos:
                yes.append(obj)
            else:
                no.append(obj)
        return question, yes, no
    elif already_asked < 6: # colour (ask a few times)
        colours = []
        for obj in objects:
            colours.append(obj.color)
        least_common = min(set(colours), key = colours.count) 

        question = "Is the object " + least_common

        yes = []
        no = []
        for obj in objects:
            if obj.color == least_common:
                yes.append(obj)
            else:
                no.append(obj)
        
        return question, yes, no
        
    else:
        return None, None, None


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
        self.questions_right = 0
        self.wrong_guesses = 0
        self.already_asked = 0
    
    def execute(self, userdata):

        remaining_objects = self.global_store['remaining_objects']

        if len(remaining_objects) == 1: # Robot knows what it is!

            guess = remaining_objects[0].label
            guess = guess.replace('_', ' ')

            question = 'You are pointing at an ' + guess + '. Am I correct?'

            answered = False

            while not answered:
                speak_goal = SpeakAndListenGoal()
                speak_goal.question = question
                speak_goal.candidates = ['yes', 'no']
                speak_goal.params = []
                speak_goal.timeout = 25

                self.action_dict['SpeakAndListen'].send_goal(speak_goal)
                self.action_dict['SpeakAndListen'].wait_for_result()

                result = self.action_dict['SpeakAndListen'].get_result()
                if result.succeeded:
                    self.global_store['last_response'] = result.answer
                    answered = True

            if self.global_store['last_response'] == 'yes':
                self.questions_right += 1
                self.global_store['remaining_objects'] = []
                self.global_store['pointed_objects'] = []
                self.already_asked = 0

                speak_goal = TalkRequestGoal()
                speak_goal.data.language = Voice.kEnglish
                speak_goal.data.sentence = 'Yay, I got it right!'
                self.action_dict['Speak'].send_goal(speak_goal)
                self.action_dict['Speak'].wait_for_result()

                if self.questions_right == 5:
                    return self._outcomes[4] # Finished
                else:
                    return self._outcomes[0] # Correct
            else:
                self.wrong_guesses += 1
                self.global_store['remaining_objects'] = \
                    self.global_store['pointed_objects']
                self.already_asked = 0
                if self.wrong_guesses == 4:
                    speak_goal = TalkRequestGoal()
                    speak_goal.data.language = Voice.kEnglish
                    speak_goal.data.sentence = 'I give up.'
                    self.action_dict['Speak'].send_goal(speak_goal)
                    self.action_dict['Speak'].wait_for_result()  
                    self.global_store['remaining_objects'] = []
                    self.global_store['pointed_objects'] = []
                    return self._outcomes[3] # Give up
                else:
                    speak_goal = TalkRequestGoal()
                    speak_goal.data.language = Voice.kEnglish
                    speak_goal.data.sentence = "Oh no! I'll have to try again!"
                    self.action_dict['Speak'].send_goal(speak_goal)
                    self.action_dict['Speak'].wait_for_result()  
                    return self._outcomes[1] # Incorrect

        else: # We need to ask a question

            question, yes_obj, no_obj = get_question(remaining_objects, 
                                                     self.already_asked)
            self.already_asked += 1

            if question == None: 
                self.global_store['remaining_objects'] = \
                    random.choice(remaining_objects)
                return self._outcomes[2]

            answered = False

            while not answered:
                speak_goal = SpeakAndListenGoal()
                speak_goal.question = question
                speak_goal.candidates = ['yes', 'no']
                speak_goal.params = []
                speak_goal.timeout = 25

                self.action_dict['SpeakAndListen'].send_goal(speak_goal)
                self.action_dict['SpeakAndListen'].wait_for_result()

                result = self.action_dict['SpeakAndListen'].get_result()
                if result.succeeded:
                    self.global_store['last_response'] = result.answer
                    answered = True

            if self.global_store['last_response'] == 'yes':
                self.global_store['remaining_objects'] = yes_obj
            else:
                self.global_store['remaining_objects'] = no_obj
            
            return self._outcomes[2]


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