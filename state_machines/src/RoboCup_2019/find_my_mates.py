#!/usr/bin/env python3
""" Code for the find my mate task.

This file contains code for the find my mate task, including the
state machine itself.

Author: Charlie Street
Owner: Charlie Street

"""

import rospy
import smach
import actionlib

from reusable_states import * # pylint: disable=unused-wildcard-import
from set_up_clients import create_stage_1_clients
from orion_actions.msg import SOMObservation, Relation, SearchPersonNotMetGoal

import time


class LookForPeopleState(ActionServiceState):
    """ State for searching for friends of the operator. """

    def __init__(self, action_dict, global_store):
        outcomes = ['PERSON_FOUND', 'NOBODY_FOUND']
        super(LookForPeopleState, self).__init__(action_dict=action_dict,
                                                 global_store=global_store,
                                                 outcomes=outcomes)
        
    def execute(self, userdata):
        goal = SearchPersonNotMetGoal()
        goal.met_before = list(map(lambda x: str(x), 
                               self.global_store['people_found']))
                               
        pose = rospy.wait_for_message('/global_pose', PoseStamped)
        pose = pose.pose

        goal.room_name = self.action_dict['SOMGetRoom'](pose).room_name

        self.action_dict['SearchPersonNotMet'].send_goal(goal)
        self.action_dict['SearchPersonNotMet'].wait_for_result()

        result = self.action_dict['SearchPersonNotMet'].get_result()

        if result.success:
            self.global_store['last_person'] = result.obj_id
        else:
            return self._outcomes[1]



class GiveOperatorInfoState(ActionServiceState):
    """ State for giving operator info about mates. """

    def __init__(self, action_dict, global_store):
        outcomes = ['SUCCESS']
        super(GiveOperatorInfoState, self).__init__(action_dict=action_dict,
                                                    global_store=global_store,
                                                    outcomes=outcomes)
    
    def execute(self, userdata):
        obj1 = SOMObservation()
        obj1.type = 'person'

        matches = self.action_dict['SOMQuery'](obj1, Relation(), 
                                               SOMObservation())

        operator_name = ''
        people_information = []

        # Extract information from matches
        for match in matches:
            person = match.obj1
            if person.task_role == 'operator':
                operator_name = person.name
            else:
                person_info = {}
                person_info['name'] = person.name
                person_info['age'] = person.age
                person_info['gender'] = person.gender
                person_info['shirt_colour'] = person.shirt_colour
                person_info['room'].pose_estimate.most_likely_room
                people_information.append(person_info)
        
        info_string = ("Hi " + operator_name + ", I have some people to tell " +
                      "you about.")
        
        for person in people_information:
            person_string = ""
            if person['room'] != '':
                person_string += " In the " + person['room'] + " I met "
            else:
                person_string += " I met "
            
            if person['name'] != '':
                person_string += person['name'] + ', '
            else:
                person_string += 'someone, '
            
            if person['age'] != 0:
                person_string += ('who I think is around the age of ' + 
                                  str(person['age']) + ', ')
            
            if person['gender'] != '':
                person_string += 'who is ' + person['gender'] + ', '
            
            if person['shirt_colour'] != '':
                person_string += 'and who is wearing ' + person['shirt_colour']
            
            if person_string[-2] == ',':
                person_string = person_string[0:-2] + '.'
            else:
                person_string += '.'
            info_string += person_string
        goal = SpeakGoal()
        goal.sentence = info_string
        self.action_dict['Speak'].send_goal(goal)
        self.action_dict['Speak'].wait_for_result()

        return self._outcomes[0]


class ShouldIContinueState(ActionServiceState):
    """ State determines whether we should continue or go back. """

    def __init__(self, action_dict, global_store):
        outcomes = ['YES', 'NO']
        super(ShouldIContinueState, self).__init__(action_dict=action_dict,
                                                   global_store=global_store,
                                                   outcomes=outcomes)
    
    def execute(self, userdata):
        time_elapsed = time.time() - self.global_store['start_time']
        if time_elapsed > 210: # 3 and a half minutes
            return self._outcomes[1]
        elif len(self.global_store['people_found']) == 3:
            return self._outcomes[1]
        
        return self._outcomes[0]


def go_to_instruction_point(action_dict):
    """ Returns the navigation location of the instruction point. """
    obj1 = SOMObservation()
    obj1.type = 'find_my_mates_point_of_interest'

    return get_location_of_object(action_dict, obj1, 
                                  Relation(), SOMObservation())


def get_operator_location(action_dict, global_store):
    """ Gets the location of our operator. """

    operator = action_dict['SOMLookup'](global_store['operator'])

    return operator.pose_estimate.most_recent_pose

## TODO - create a duplicate of this in a robocup2022 directory, then refactor

def create_state_machine(action_dict):
    """ This function creates and returns the state machine for the task. """

    # Initialise the global store
    global_store = {}
    global_store['start_time'] = time.time()
    global_store['last_person'] = None
    global_store['people_found'] = []

    # Create the state machine
    sm = smach.StateMachine(outcomes=['TASK_SUCCESS', 'TASK_FAILURE'])

    with sm:
        
        # Initially set nav goal to instruction point
        func = lambda: go_to_instruction_point(action_dict)
        smach.StateMachine.add('SetNavToInstructionPoint',
                               SetNavGoalState(action_dict, global_store, func),
                               transitions={'SUCCESS':'StartSpeak'})

        # Start by stating intentions
        phrase = "Hi, I'm Bam Bam and I'm here to find some mates!"
        smach.StateMachine.add('StartSpeak',
                               SpeakState(action_dict, global_store, phrase),
                               transitions={'SUCCESS':'CheckDoorOpen',
                                            'FAILURE':'CheckDoorOpen'})
        
        # Now wait for the door to be opened
        smach.StateMachine.add('CheckDoorOpen',
                               CheckDoorIsOpenState(action_dict, global_store),
                               transitions={'OPEN':'NavToInstructionPoint',
                                            'CLOSED':'CheckDoorOpen'})
        
        # Now navigate to the operator
        smach.StateMachine.add('NavToInstructionPoint',
                               NavigateState(action_dict, global_store),
                               transitions={'SUCCESS':'AskForOperator',
                                            'FAILURE':'NavToInstructionPoint',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Ask for an operator
        question = ("Hi, nice to meet you! Are you the operator who is looking "+
                   "for their friends! If so, please tell me your name.")
        smach.StateMachine.add('AskForOperator',
                               SpeakAndListenState(action_dict,
                                                   global_store,
                                                   question,
                                                   NAMES,
                                                   [],
                                                   30),
                               transitions={'SUCCESS':'MemoriseOperator',
                                            'FAILURE':'AskForOperator',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})

        # Memorise operator
        smach.StateMachine.add('MemoriseOperator',
                               OperatorDetectState(action_dict, global_store),
                               transitions={'SUCCESS':'OnMyWay',
                                            'FAILURE':'AskForOperator'})

        # Head on the way
        phrase = ("Right, I'm off to find your mates! Don't worry, I never " +
                 "forget a friendly face! In the words of Arnold Schwarzenegger"
                 + ", I'll be back!")
        smach.StateMachine.add('OnMyWay',
                               SpeakState(action_dict, global_store, phrase),
                               transitions={'SUCCESS':'LookForPeople',
                                            'FAILURE':'LookForPeople'})
        
        # Start looking for people
        smach.StateMachine.add('LookForPeople',
                               LookForPeopleState(action_dict, global_store),
                               transitions={'PERSON_FOUND':'TalkToPerson',
                                            'NOBODY_FOUND':'SetOpDestination'})
        
        # Get information from person
        question = ("Hi, I'm Bam Bam, nice to meet you! What is your name?")
        smach.StateMachine.add('TalkToPerson',
                               SpeakAndListenState(action_dict,
                                                   global_store,
                                                   question,
                                                   NAMES,
                                                   [],
                                                   30),
                               transitions={'SUCCESS':'MemorisePerson',
                                            'FAILURE':'TalkToPerson',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # memorise the person
        smach.StateMachine.add('MemorisePerson',
                               MemorisePersonState(action_dict, global_store),
                               transitions={'SUCCESS':'ThankYou',
                                            'FAILURE':'TalkToPerson'})
        
        # Thank the person
        phrase = ("Thank you, I think I got all that. I need to go now, my " +
                 "home planet needs me! Beep boop")
        smach.StateMachine.add('ThankYou',
                               SpeakState(action_dict, global_store, phrase),
                               transitions={'SUCCESS':'ShouldIContinue',
                                            'FAILURE':'ShouldIContinue'})
        
        # Check whether I should continue
        smach.StateMachine.add('ShouldIContinue',
                               ShouldIContinueState(action_dict, global_store),
                               transitions={'YES':'LookForPeople',
                                            'NO':'SetOpDestination'})
        
        # Set operator destination
        function = lambda : get_operator_location(action_dict, global_store)
        smach.StateMachine.add('SetOpDestination',
                               SetNavGoalState(action_dict, 
                                               global_store, 
                                               function),
                               transitions={'SUCCESS':'BackToOp'})
        
        # Navigate back to the operator
        smach.StateMachine.add('BackToOp',
                               NavigateState(action_dict, global_store),
                               transitions={'SUCCESS':'GiveOperatorInfo',
                                            'FAILURE':'BackToOp',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Give the operator information
        smach.StateMachine.add('GiveOperatorInfo',
                               GiveOperatorInfoState(action_dict, global_store),
                               transitions={'SUCCESS':'ThankOp'})
        
        # Thank the operator
        smach.StateMachine.add('ThankOp',
                               SpeakState(action_dict, global_store, phrase),
                               transitions={'SUCCESS':'TASK_SUCCESS',
                                            'FAILURE':'TASK_SUCCESS'})

    return sm


if __name__ == '__main__':
    action_dict = create_stage_1_clients(4)
    sm = create_state_machine(action_dict)
    sm.execute()