#!/usr/bin/env python3
""" Smach state machine to test Speech states

Author: Ricardo Cannizzaro
Owner: Ricardo Cannizzaro
"""

import os
import rospy
import smach
import smach_ros
import actionlib

from state_machines.reusable_states import * # pylint: disable=unused-wildcard-import


def create_state_machine(userdata=None):
    """ This function builds the state machine for the test.

    Args:
        userdata: A dictionary of optional input arguments to override default

    Returns:
        sm: The complete state machine
    """

    # Create the state machine
    sm = smach.StateMachine(outcomes=['task_success', 'task_failure'])
    
    # Define state parameters
    sm.userdata.ask_name_phrase = "Hi, what's your name?"
    sm.userdata.speak_listen_failures = 0
    sm.userdata.speak_listen_failure_threshold = 3
    sm.userdata.operator_names = NAMES
    # Load up huge database of additional names (if necessary)
    # import rospkg
    # rospack = rospkg.RosPack()
    # name_file = \
    #     os.path.join(rospack.get_path('state_machines'),'grammars/names.txt')
    # with open(name_file, 'r') as in_file:
    #     sm.userdata.operator_names += in_file.read().splitlines()

    sm.userdata.speak_and_listen_params_empty = []
    sm.userdata.speak_and_listen_timeout = 5  # seconds

    sm.userdata.one_word_question_phrase = "What do you want me to pick up?"   # change this as you need
    sm.userdata.one_word_question_candidates = OBJECTS   # change this as you need

    sm.userdata.task_question_phrase = "What do you want me to do?"   # change this as you need
    sm.userdata.task_question_candidates = []   # change this as you need

    # override default userdata with input arguments
    if userdata is not None:
        sm.userdata.update(userdata)

    with sm:

        smach.StateMachine.add('ASK_OPERATOR_NAME',
                               SpeakAndListenState(),
                                transitions={'success': 'ASK_ONE_WORD_QUESTION',
                                            'failure':'ASK_OPERATOR_NAME',
                                            'repeat_failure':'task_failure'},
                                remapping={'question':'ask_name_phrase',
                                            'operator_response': 'operator_name',
                                            'candidates':'operator_names',
                                            'params':'speak_and_listen_params_empty',
                                            'timeout':'speak_and_listen_timeout',
                                            'number_of_failures': 'speak_listen_failures',
                                            'failure_threshold': 'speak_listen_failure_threshold'})
        
        smach.StateMachine.add('ASK_ONE_WORD_QUESTION',
                               SpeakAndListenState(),
                                transitions={'success': 'ASK_TASK_QUESTION',
                                            'failure':'ASK_TASK_QUESTION',
                                            'repeat_failure':'task_failure'},
                                remapping={'question':'one_word_question_phrase',
                                            'operator_response': 'one_word_question_response',
                                            'candidates':'one_word_question_candidates',
                                            'params':'speak_and_listen_params_empty',
                                            'timeout':'speak_and_listen_timeout',
                                            'number_of_failures': 'speak_listen_failures',
                                            'failure_threshold': 'speak_listen_failure_threshold'})
        
        smach.StateMachine.add('ASK_TASK_QUESTION',
                               SpeakAndListenState(),
                                transitions={'success': 'task_success',
                                            'failure':'ASK_TASK_QUESTION',
                                            'repeat_failure':'task_failure'},
                                remapping={'question':'task_question_phrase',
                                            'operator_response': 'task_question_response',
                                            'candidates':'task_question_candidates',
                                            'params':'speak_and_listen_params_empty',
                                            'timeout':'speak_and_listen_timeout',
                                            'number_of_failures': 'speak_listen_failures',
                                            'failure_threshold': 'speak_listen_failure_threshold'})


    return sm


if __name__ == '__main__':
    rospy.init_node('speech_test')

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