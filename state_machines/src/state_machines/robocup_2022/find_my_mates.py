#!/usr/bin/env python3
""" Code for the find my mate task.

This file contains code for the find my mate task, including the
state machine itself.

Author: Ricardo Cannizzaro
Owner: Ricardo Cannizzaro

"""

import os
import rospy
import smach_ros
import actionlib

from state_machines.reusable_states import * # pylint: disable=unused-wildcard-import
# from set_up_clients import create_stage_1_clients
from orion_actions.msg import SOMObservation, Relation, SearchPersonNotMetGoal
from geometry_msgs.msg import Pose

import time  # TODO - replace calls to rospy.time library


# class LookForPeopleState(ActionServiceState):
#     """ State for searching for friends of the operator. """

#     def __init__(self, action_dict, global_store):
#         outcomes = ['PERSON_FOUND', 'NOBODY_FOUND']
#         super(LookForPeopleState, self).__init__(action_dict=action_dict,
#                                                  global_store=global_store,
#                                                  outcomes=outcomes)
        
#     def execute(self, userdata):
#         goal = SearchPersonNotMetGoal()
#         goal.met_before = list(map(lambda x: str(x), 
#                                self.global_store['people_found']))
                               
#         pose = rospy.wait_for_message('/global_pose', PoseStamped)
#         pose = pose.pose

#         goal.room_name = self.action_dict['SOMGetRoom'](pose).room_name

#         self.action_dict['SearchPersonNotMet'].send_goal(goal)
#         self.action_dict['SearchPersonNotMet'].wait_for_result()

#         result = self.action_dict['SearchPersonNotMet'].get_result()

#         if result.success:
#             self.global_store['last_person'] = result.obj_id
#         else:
#             return self._outcomes[1]



# class GiveOperatorInfoState(ActionServiceState):
#     """ State for giving operator info about mates. """

#     def __init__(self, action_dict, global_store):
#         outcomes = ['SUCCESS']
#         super(GiveOperatorInfoState, self).__init__(action_dict=action_dict,
#                                                     global_store=global_store,
#                                                     outcomes=outcomes)
    
#     def execute(self, userdata):
#         obj1 = SOMObservation()
#         obj1.type = 'person'

#         matches = self.action_dict['SOMQuery'](obj1, Relation(), 
#                                                SOMObservation())

#         operator_name = ''
#         people_information = []

#         # Extract information from matches
#         for match in matches:
#             person = match.obj1
#             if person.task_role == 'operator':
#                 operator_name = person.name
#             else:
#                 person_info = {}
#                 person_info['name'] = person.name
#                 person_info['age'] = person.age
#                 person_info['gender'] = person.gender
#                 person_info['shirt_colour'] = person.shirt_colour
#                 person_info['room'].pose_estimate.most_likely_room
#                 people_information.append(person_info)
        
#         info_string = ("Hi " + operator_name + ", I have some people to tell " +
#                       "you about.")
        
#         for person in people_information:
#             person_string = ""
#             if person['room'] != '':
#                 person_string += " In the " + person['room'] + " I met "
#             else:
#                 person_string += " I met "
            
#             if person['name'] != '':
#                 person_string += person['name'] + ', '
#             else:
#                 person_string += 'someone, '
            
#             if person['age'] != 0:
#                 person_string += ('who I think is around the age of ' + 
#                                   str(person['age']) + ', ')
            
#             if person['gender'] != '':
#                 person_string += 'who is ' + person['gender'] + ', '
            
#             if person['shirt_colour'] != '':
#                 person_string += 'and who is wearing ' + person['shirt_colour']
            
#             if person_string[-2] == ',':
#                 person_string = person_string[0:-2] + '.'
#             else:
#                 person_string += '.'
#             info_string += person_string
#         goal = SpeakGoal()
#         goal.sentence = info_string
#         self.action_dict['Speak'].send_goal(goal)
#         self.action_dict['Speak'].wait_for_result()

#         return self._outcomes[0]


# class ShouldIContinueState(ActionServiceState):
#     """ State determines whether we should continue or go back. """

#     def __init__(self, action_dict, global_store):
#         outcomes = ['YES', 'NO']
#         super(ShouldIContinueState, self).__init__(action_dict=action_dict,
#                                                    global_store=global_store,
#                                                    outcomes=outcomes)
    
#     def execute(self, userdata):
#         time_elapsed = time.time() - self.global_store['start_time']
#         if time_elapsed > 210: # 3 and a half minutes
#             return self._outcomes[1]
#         elif len(self.global_store['people_found']) == 3:
#             return self._outcomes[1]
        
#         return self._outcomes[0]


# def go_to_instruction_point(action_dict):
#     """ Returns the navigation location of the instruction point. """
#     obj1 = SOMObservation()
#     obj1.type = 'find_my_mates_point_of_interest'

#     return get_location_of_object(action_dict, obj1, 
#                                   Relation(), SOMObservation())


# def get_operator_location(action_dict, global_store):
#     """ Gets the location of our operator. """

#     operator = action_dict['SOMLookup'](global_store['operator'])

#     return operator.pose_estimate.most_recent_pose

## TODO - create a duplicate of this in a robocup2022 directory, then refactor

def create_state_machine():
    """ This function creates and returns the state machine for the task. """

    # Create the state machine
    sm = smach.StateMachine(outcomes=['task_success', 'task_failure'])

    # Create state machine userdata dictionary elements

    sm.userdata.person_names = NAMES
    # Load up huge database of additional names (if necessary)
    # import rospkg
    # rospack = rospkg.RosPack()
    # name_file = \
    #     os.path.join(rospack.get_path('state_machines'),'grammars/names.txt')
    # with open(name_file, 'r') as in_file:
    #     sm.userdata.person_names += in_file.read().splitlines()

    sm.userdata.speak_and_listen_params_empty = []
    sm.userdata.speak_and_listen_timeout = 5
    sm.userdata.speak_and_listen_failures = 0
    sm.userdata.speak_and_listen_failure_threshold = 3

    sm.userdata.simple_navigation_failures = 0
    sm.userdata.simple_navigation_failure_threshold = 3

    sm.userdata.wait_for_start_signal_phrase = "I am waiting for the start signal."
    sm.userdata.task_intentions_phrase = "Hi, I'm Bam Bam and I'm here to find some mates! Let's go!"

    sm.userdata.nav_repeat_failure_phrase = "Navigation failed too many times, terminating task."
    sm.userdata.speak_and_listen_repeat_failure_phrase = "Speech recognition failed too many times, terminating task."

    sm.userdata.introduction_to_operator_phrase = "Hi, nice to meet you! I am here to help look for your friends!"
    sm.userdata.ask_operator_name_phrase = "What is your name?"
    sm.userdata.speech_recognition_failure_phrase = "I'm sorry but I did understand. Let's try that again."

    # set the robot's pose to speak to the operator - TODO
    sm.userdata.operator_pose = Pose()
    sm.userdata.operator_pose.position.x = 0.0
    sm.userdata.operator_pose.position.y = 0.0
    sm.userdata.operator_pose.position.z = 0.0
    sm.userdata.operator_pose.orientation.x = 0.0
    sm.userdata.operator_pose.orientation.y = 0.0
    sm.userdata.operator_pose.orientation.z = 0.0
    sm.userdata.operator_pose.orientation.w = 1.0

    # speaking to guests
    sm.userdata.introduction_to_guest_phrase = "Hi, I'm Bam Bam, welcome to the party! I'm going to learn some information about you so I can tell the host about you!"
    sm.userdata.no_one_there_phrase = "Hmmm. I don't think anyone is there."

    # set arena exit pose - TODO
    sm.userdata.exit_pose = Pose()
    sm.userdata.exit_pose.position.x = 0.0
    sm.userdata.exit_pose.position.y = 0.0
    sm.userdata.exit_pose.position.z = 0.0
    sm.userdata.exit_pose.orientation.x = 0.0
    sm.userdata.exit_pose.orientation.y = 0.0
    sm.userdata.exit_pose.orientation.z = 0.0
    sm.userdata.exit_pose.orientation.w = 1.0

    sm.userdata.announce_finish_phrase = "I have exited the arena. I am now stopping."

    sm.userdata.operator_name = "Isaac Asimov"   # a default name for testing, this will be overridden by ASK_OPERATOR_NAME state

    # guest tracking
    sm.userdata.guest_som_human_ids = []
    sm.userdata.guest_som_obj_ids = []

    # task params
    sm.userdata.expected_num_guests = 3
    sm.userdata.max_search_duration = 30 # seconds

    with sm:

        # save initial robot state
        # smach.StateMachine.add('STORE_INITIAL_LOCATION',
        #                         GetRobotLocationState(),
        #                         transitions={'stored':'INTRO'},
        #                         remapping={'robot_location':'start_pose'})

        # announce waiting for start signal
        # smach.StateMachine.add('ANNOUNCE_WAIT_FOR_START_SIGNAL',
        #                         SpeakState(),
        #                         transitions={'success':'WAIT_FOR_START_SIGNAL'},
        #                         remapping={'phrase':'wait_for_start_signal_phrase'})
        
        # # wait for the start signal - TODO - test
        # smach.StateMachine.add('WAIT_FOR_START_SIGNAL',
        #                         CheckDoorIsOpenState(),
        #                         transitions={   'open':'ANNOUNCE_TASK_INTENTIONS', 
        #                                         'closed':'WAIT_FOR_START_SIGNAL'})
        
        # # announce task intentions
        # smach.StateMachine.add('ANNOUNCE_TASK_INTENTIONS',
        #                         SpeakState(),
        #                         transitions={'success':'SAVE_START_TIME'},
        #                         remapping={'phrase':'task_intentions_phrase'})

        # save the start time
        smach.StateMachine.add('SAVE_START_TIME',
                                GetTime(),
                                transitions={'success':'LEARN_GUEST_SUB'},
                                remapping={'current_time':'task_start_time'})
        
        # # navigate to operator - TODO - consider changing to top nav
        # smach.StateMachine.add('NAV_TO_OPERATOR',
        #                        SimpleNavigateState(),
        #                        transitions={'success':'INTRODUCTION_TO_OPERATOR',
        #                                     'failure':'NAV_TO_OPERATOR',
        #                                     'repeat_failure':'ANNOUNCE_REPEAT_NAV_FAILURE'},
        #                         remapping={'pose':'operator_pose',
        #                                    'number_of_failures': 'simple_navigation_failures',
        #                                    'failure_threshold':'simple_navigation_failure_threshold'})

        # # announce nav repeat failure
        # smach.StateMachine.add('ANNOUNCE_REPEAT_NAV_FAILURE',
        #                         SpeakState(),
        #                         transitions={'success':'task_failure'},
        #                         remapping={'phrase':'nav_repeat_failure_phrase'})

        # # introduce to operator
        # smach.StateMachine.add('INTRODUCTION_TO_OPERATOR',
        #                         SpeakState(),
        #                         transitions={'success':'ASK_OPERATOR_NAME'},
        #                         remapping={'phrase':'introduction_to_operator_phrase'})

        # # ask for operator's name
        # smach.StateMachine.add('ASK_OPERATOR_NAME',
        #                        SpeakAndListenState(),
        #                         # transitions={'success': 'ANNOUNCE_SEARCH_START',
        #                         transitions={'success': 'SAVE_OPERATOR_INFO_TO_SOM',
        #                                     'failure':'ANNOUNCE_MISSED_NAME',
        #                                     'repeat_failure':'ANNOUNCE_REPEAT_SPEECH_RECOGNITION_FAILURE'},
        #                         remapping={'question':'ask_operator_name_phrase',
        #                                     'operator_response': 'operator_name',
        #                                     'candidates':'person_names',
        #                                     'params':'speak_and_listen_params_empty',
        #                                     'timeout':'speak_and_listen_timeout',
        #                                     'number_of_failures': 'speak_and_listen_failures',
        #                                     'failure_threshold': 'speak_and_listen_failure_threshold'})

        # # announce that we missed the name, and that we will try again
        # smach.StateMachine.add('ANNOUNCE_MISSED_NAME',
        #                         SpeakState(),
        #                         transitions={'success':'ASK_OPERATOR_NAME'},
        #                         remapping={'phrase':'speech_recognition_failure_phrase'})

        # # announce speech recognition repeat failure
        # smach.StateMachine.add('ANNOUNCE_REPEAT_SPEECH_RECOGNITION_FAILURE',
        #                         SpeakState(),
        #                         transitions={'success':'task_failure'},
        #                         remapping={'phrase':'speak_and_listen_repeat_failure_phrase'})

        # save the operator info to the SOM
        # smach.StateMachine.add('SAVE_OPERATOR_INFO_TO_SOM',
        #                        SaveOperatorToSOM(),
        #                        transitions={'success':'CREATE_PHRASE_START_SEARCH',
        #                                     'failure':'task_failure'},
        #                         remapping={'operator_name':'operator_name', 
        #                                     'operator_som_id':'operator_som_id'})

        # # create the search start phrase
        # smach.StateMachine.add('CREATE_PHRASE_START_SEARCH',
        #                         CreatePhraseStartSearchForPeopleState(),
        #                         transitions={'success':'ANNOUNCE_SEARCH_START'},
        #                         remapping={'operator_name':'operator_name',
        #                                     'phrase':'announce_search_start_phrase'})
        
        # # announce search start
        # smach.StateMachine.add('ANNOUNCE_SEARCH_START',
        #                         SpeakState(),
        #                         transitions={'success':'ANNOUNCE_GUEST_INTRO'},
        #                         remapping={'phrase':'announce_search_start_phrase'})

        # TODO - implement state or sub state machine to search for person
        # OLD CODE
        # # Start looking for people
        # smach.StateMachine.add('LookForPeople',
        #                        LookForPeopleState(action_dict, global_store),
        #                        transitions={'PERSON_FOUND':'TalkToPerson',
        #                                     'NOBODY_FOUND':'SetOpDestination'})

        smach.StateMachine.add('LEARN_GUEST_SUB', 
                                create_learn_guest_sub_state_machine(),
                                transitions={'success':'SHOULD_I_CONTINUE_GUEST_SEARCH',
                                            'failure':'SHOULD_I_CONTINUE_GUEST_SEARCH'},
                                remapping={'guest_som_human_ids':'guest_som_human_ids',
                                            'guest_som_obj_ids':'guest_som_obj_ids',
                                            'person_names':'person_names'})

        smach.StateMachine.add('SHOULD_I_CONTINUE_GUEST_SEARCH', 
                                ShouldIContinueGuestSearchState(),
                                transitions={'yes':'LEARN_GUEST_SUB',
                                            'no':'ANNOUNCE_FINISH'},
                                remapping={'guest_som_human_ids':'guest_som_human_ids',
                                            'max_search_duration':'max_search_duration',
                                            'expected_num_guests':'expected_num_guests',
                                            'start_time':'task_start_time'})

        

        # OLD CODE
        # # Give the operator information
        # smach.StateMachine.add('GiveOperatorInfo',
        #                        GiveOperatorInfoState(action_dict, global_store),
        #                        transitions={'SUCCESS':'ThankOp'})
        #
        # OLD CODE
        # # Thank the operator
        # smach.StateMachine.add('ThankOp',
        #                        SpeakState(action_dict, global_store, phrase),
        #                        transitions={'SUCCESS':'TASK_SUCCESS',
        #                                     'FAILURE':'TASK_SUCCESS'})

        # TODO - Return to operator
        # TODO - Report to operator details of known guests
        # TODO - Thank operator and announce that we're done
        # TODO - Reset FaceDB? Or do this manually between runs?

        # leave the arena
        # TODO - consider changing to topological navigation state
        smach.StateMachine.add('NAV_TO_EXIT',
                                SimpleNavigateState(),
                                transitions={'success':'ANNOUNCE_FINISH',
                                            'failure':'NAV_TO_EXIT',
                                            'repeat_failure':'task_failure'},
                                remapping={'pose':'exit_pose',
                                           'number_of_failures': 'simple_navigation_failures',
                                           'failure_threshold':'simple_navigation_failure_threshold'})

        smach.StateMachine.add('ANNOUNCE_FINISH',
                                SpeakState(),
                                transitions={'success':'task_success'},
                                remapping={'phrase':'announce_finish_phrase'})

    return sm


if __name__ == '__main__':
    rospy.init_node('find_my_mates_state_machine')

    # Create the state machine
    sm = create_state_machine()

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    sm.execute()

    # Run until ctl+c command is received
    rospy.spin()
    sis.stop()