#!/usr/bin/env python3
""" Smach state machine for ORI open day demonstration!

Author: Charlie Street
Owner: Ricardo Cannizzaro
"""

import os
import rospy
import smach
import smach_ros
import actionlib

from reusable_states import * # pylint: disable=unused-wildcard-import
from set_up_clients import create_open_day_clients
from geometry_msgs.msg import Pose

# TODO - remove
# class SpeakToOperatorState(ActionServiceState):
#     """ Smach state for the robot to say stuff.

#     This class has the robot say something and return success if it has been
#     said. This says the operator name and the item that's been picked up

#     Attributes:
#         phrase: What we want the robot to say
#     """
#     def __init__(self, action_dict, global_store):
#         outcomes = ['success']
#         super(SpeakToOperatorState, self).__init__(action_dict=action_dict,
#                                                    global_store=global_store,
#                                                    outcomes=outcomes)
    
#     def execute(self, userdata):

#         #obj1 = SOMObservation()
#         #obj1.obj_id = self.global_store['people_found'][0]
#         #rel = Relation()
#         #obj2 = SOMObservation()

#         #matches = self.action_dict['SOMQuery'](obj1, rel, obj2, Pose()).matches
        
#         #name = matches[0].obj1.name
#         name = self.global_store['operator_name']
#         picked_up = self.global_store['pick_up']

#         phrase = "Hi, " + name + ", I've brought you the " + picked_up 

#         action_goal = TalkRequestGoal()
#         action_goal.data.language = Voice.kEnglish
#         action_goal.data.sentence = phrase
#         self.action_dict['Speak'].send_goal(action_goal)
#         self.action_dict['Speak'].wait_for_result()

#         # Can only succeed
#         return self._outcomes[0]

# class SpeakAndListenPickupState(ActionServiceState):
#     """ Smach state for speaking and then listening for a response.

#     This state will get the robot to say something and then wait for a 
#     response.
#     """

#     def __init__(self, 
#                  action_dict, 
#                  global_store,
#                  candidates, 
#                  params, 
#                  timeout):
#         """ Constructor initialises fields and calls super constructor.

#         Args:
#             action_dict: As in super class
#             global_store: As in super class
#             question: The question to ask
#             candidates: Candidate sentences
#             params: Optional parameters for candidate sentences
#             timeout: The timeout for listening
#         """

#         outcomes = ['success', 'failure', 'repeat_failure']
#         self.candidates = candidates
#         self.params = params
#         self.timeout = timeout
#         super(SpeakAndListenPickupState, self).__init__(action_dict=action_dict,
#                                                         global_store=global_store,
#                                                         outcomes=outcomes)
        
#         if 'speak_listen_failure' not in self.global_store:
#             self.global_store['speak_listen_failure'] = 0
    
#     def execute(self, userdata):
#         speak_listen_goal = SpeakAndListenGoal()
#         speak_listen_goal.question = ("Can someone help me pick up the " +
#                                       self.global_store['pick_up'] + 
#                                       " and say ready " + 
#                                       "when they are ready?")
#         speak_listen_goal.candidates = self.candidates
#         speak_listen_goal.params = self.params
#         speak_listen_goal.timeout = self.timeout

#         self.action_dict['SpeakAndListen'].send_goal(speak_listen_goal)
#         self.action_dict['SpeakAndListen'].wait_for_result()

#         result = self.action_dict['SpeakAndListen'].get_result()
#         if result.succeeded:
#             self.global_store['last_response'] = result.answer
#             self.global_store['speak_listen_failure'] = 0
#             return self._outcomes[0]
#         else:
#             self.global_store['speak_listen_failure'] += 1
#             if self.global_store['speak_listen_failure'] >= FAILURE_THRESHOLD:
#                 return self._outcomes[2]
#             return self._outcomes[1]


def create_state_machine(userdata=None):
    """ This function builds the state machine for the ORI open day demo.

    Args:
        userdata: A dictionary of optional input arguments to override default

    Returns:
        sm: The complete state machine
    """

    # Create the state machine
    sm = smach.StateMachine(outcomes=['task_success', 'task_failure'])
    sm.userdata.nav_failures = 0
    sm.userdata.nav_failure_threshold = 3
    sm.userdata.intro_phrase = ("Hi, my name is Bam Bam, and " + 
                 "today we're going to show you what I can do!")
    # sm.userdata.intro_phrase = "Hi, my name is Bam Bam."
    sm.userdata.operator_question = "Hi, what's your name?"
    sm.userdata.speak_listen_failures = 0
    sm.userdata.speak_listen_failure_threshold = 3
    sm.userdata.operator_names = ['Gemma', 'Acacia', 'Ollie', 'Nick', 'Hollie', 
          'Charlie', 'Matt', 'Daniele', 'Chris', 'Paul', 'Lars', 'Jon',
          'Michael', 'Matthew', 'Ricardo']
    # Load up huge database of additional names (if necessary)
    # import rospkg
    # rospack = rospkg.RosPack()
    # name_file = \
    #     os.path.join(rospack.get_path('state_machines'),'grammars/names.txt')
    # with open(name_file, 'r') as in_file:
    #     sm.userdata.operator_names += in_file.read().splitlines()

    sm.userdata.speak_and_listen_params_empty = []
    sm.userdata.speak_and_listen_timeout = 5

    sm.userdata.simple_navigation_failures = 0
    sm.userdata.simple_navigation_failure_threshold = 3

    sm.userdata.operator_pose = Pose()
    sm.userdata.operator_pose.position.x = 1.57894771198
    sm.userdata.operator_pose.position.y = 0.59994803628
    sm.userdata.operator_pose.position.z = 0.0
    sm.userdata.operator_pose.orientation.x = 0.0
    sm.userdata.operator_pose.orientation.y = 0.0
    sm.userdata.operator_pose.orientation.z = -0.362624641735
    sm.userdata.operator_pose.orientation.w = 0.931935281662

    sm.userdata.pickup_object_names = ['potted plant', 'cup', 'bottle']
    sm.userdata.object_pickup_question = "Great! What would you like me to pick up?"

    sm.userdata.pickup_pose = Pose()
    sm.userdata.pickup_pose.position.x = 0.23832461091673765
    sm.userdata.pickup_pose.position.y = 0.0
    sm.userdata.pickup_pose.position.z = 0.0
    sm.userdata.pickup_pose.orientation.x = 0.0
    sm.userdata.pickup_pose.orientation.y = 0.0
    sm.userdata.pickup_pose.orientation.z = -0.7027350031010926
    sm.userdata.pickup_pose.orientation.w =  0.711451695771756


    sm.userdata.object_ar_markers = AR_MARKERS
    sm.userdata.pickup_failures = 0
    sm.userdata.pickup_failure_threshold = 3

    sm.userdata.ask_for_help_speak_and_listen_timeout = 10

    sm.userdata.ask_when_ready_question = ("Please say ready when you're ready "
                    "for me to hand this back to you?")
    sm.userdata.ask_when_ready_responses = READY

    sm.userdata.operator_farewell_phrase = "It looks like my job here is done. Have a nice day!"

    sm.userdata.final_announcement_phrase = "I'm back where I started, woohoo!"

    # override default userdata with input arguments
    if userdata is not None:
        sm.userdata.update(userdata)

    with sm:
        
        smach.StateMachine.add('STORE_INITIAL_LOCATION',
                                GetRobotLocationState(),
                                transitions={'stored':'INTRO'},
                                remapping={'robot_location':'start_pose'})

        smach.StateMachine.add('INTRO',
                                SpeakState(),
                                transitions={'success':'NAV_TO_OPERATOR'},
                                remapping={'phrase':'intro_phrase'})

        smach.StateMachine.add('NAV_TO_OPERATOR',
                               SimpleNavigateState(),
                               transitions={'success':'ASK_OPERATOR_NAME',
                                            'failure':'NAV_TO_OPERATOR',
                                            'repeat_failure':'task_failure'},
                                remapping={'pose':'operator_pose',
                                           'number_of_failures': 'simple_navigation_failures',
                                           'failure_threshold':'simple_navigation_failure_threshold'})
    
        smach.StateMachine.add('ASK_OPERATOR_NAME',
                               SpeakAndListenState(),
                                transitions={'success': 'ASK_PICK_UP_OBJ',
                                            'failure':'ASK_OPERATOR_NAME',
                                            'repeat_failure':'task_failure'},
                                remapping={'question':'operator_question',
                                            'operator_response': 'operator_name',
                                            'candidates':'operator_names',
                                            'params':'speak_and_listen_params_empty',
                                            'timeout':'speak_and_listen_timeout',
                                            'number_of_failures': 'speak_listen_failures',
                                            'failure_threshold': 'speak_listen_failure_threshold'})

        smach.StateMachine.add('ASK_PICK_UP_OBJ',
                               SpeakAndListenState(),
                                transitions={'success': 'NAV_TO_PICKUP',
                                            'failure':'ASK_PICK_UP_OBJ',
                                            'repeat_failure':'task_failure'},
                                remapping={'question':'object_pickup_question',
                                            'operator_response': 'pickup_object_name',
                                            'candidates':'pickup_object_names',
                                            'params':'speak_and_listen_params_empty',
                                            'timeout':'speak_and_listen_timeout',
                                            'number_of_failures': 'speak_listen_failures',
                                            'failure_threshold': 'speak_listen_failure_threshold'})

        smach.StateMachine.add('NAV_TO_PICKUP',
                               SimpleNavigateState(),
                               transitions={'success':'PICKUP_OBJECT',
                                            'failure':'NAV_TO_PICKUP',
                                            'repeat_failure':'task_failure'},
                                remapping={'pose':'pickup_pose',
                                           'number_of_failures': 'simple_navigation_failures',
                                           'failure_threshold': 'simple_navigation_failure_threshold'})
                
        smach.StateMachine.add('PICKUP_OBJECT',
                               PickUpObjectState(),
                               transitions={'success':'NAV_BACK_TO_OPERATOR',
                                            'failure':'PICKUP_OBJECT',
                                            'repeat_failure':'CREATE_ASK_FOR_HELP_PHRASE'},
                                remapping={ 'object_name':'pickup_object_name', 
                                            'number_of_failures':'pickup_failures', 
                                            'failure_threshold':'pickup_failure_threshold', 
                                            'ar_marker_ids':'object_ar_markers'})

        smach.StateMachine.add('CREATE_ASK_FOR_HELP_PHRASE',
                                CreatePhraseAskForHelpPickupObjectState(),
                                transitions={'success':'ASK_FOR_HELP'},
                                remapping={'object_name': 'pickup_object_name',
                                           'phrase':'operator_return_phrase'})

        smach.StateMachine.add('ASK_FOR_HELP',
                               SpeakAndListenState(),
                                transitions={'success': 'RECEIVE_ITEM_FROM_OPERATOR',
                                            'failure':'ASK_FOR_HELP',
                                            'repeat_failure':'task_failure'},
                                remapping={'question':'operator_return_phrase',
                                            'operator_response': 'ask_when_ready_response',
                                            'candidates':'ask_when_ready_responses',
                                            'params':'speak_and_listen_params_empty',
                                            'timeout':'ask_for_help_speak_and_listen_timeout',
                                            'number_of_failures': 'speak_listen_failures',
                                            'failure_threshold': 'speak_listen_failure_threshold'})

        smach.StateMachine.add('RECEIVE_ITEM_FROM_OPERATOR',
                               ReceiveObjectFromOperatorState(),
                               transitions={'success':'NAV_BACK_TO_OPERATOR',
                                            'failure':'task_failure'})

        smach.StateMachine.add('NAV_BACK_TO_OPERATOR',
                                SimpleNavigateState(),
                                transitions={'success':'CREATE_OPERATOR_RETURN_PHRASE',
                                            'failure':'NAV_BACK_TO_OPERATOR',
                                            'repeat_failure':'task_failure'},
                                remapping={'pose':'operator_pose',
                                           'number_of_failures': 'simple_navigation_failures',
                                           'failure_threshold':'simple_navigation_failure_threshold'})


        smach.StateMachine.add('CREATE_OPERATOR_RETURN_PHRASE',
                                CreatePhraseAnnounceRetrievedItemToNamedOperatorState(),
                                transitions={'success':'SPEAK_TO_OPERATOR'},
                                remapping={'operator_name':'operator_name',
                                           'object_name': 'pickup_object_name',
                                           'phrase':'operator_return_phrase'})

        smach.StateMachine.add('SPEAK_TO_OPERATOR',
                                SpeakState(),
                                transitions={'success':'ARRIVAL_QUESTION'},
                                remapping={'phrase':'operator_return_phrase'})

        # This does not seem to be used in the open day demo - TODO: remove
        # """phrase = ("Now I'm ready to follow you. Please go slow and say " +
        #           "cancel when you want me to stop.")
        # smach.StateMachine.add('ReadyToFollow',
        #                        SpeakState(action_dict, global_store, phrase),
        #                        transitions={'success': 'Follow',
        #                                     'failure': 'Follow'})
        #
        #
        # smach.StateMachine.add('Follow',
        #                         make_follow_hotword_state(action_dict, 
        #                                                   global_store),
        #                         transitions={'success': 'ArrivalQuestion',
        #                                      'failure': 'Follow',
        #                                      'repeat_failure': 'task_failure'})"""


        smach.StateMachine.add('ARRIVAL_QUESTION',
                               SpeakAndListenState(),
                                transitions={'success': 'GIVE_ITEM',
                                            'failure':'ARRIVAL_QUESTION',
                                            'repeat_failure':'task_failure'},
                                remapping={'question':'ask_when_ready_question',
                                            'operator_response': 'ask_when_ready_response',
                                            'candidates':'ask_when_ready_responses',
                                            'params':'speak_and_listen_params_empty',
                                            'timeout':'speak_and_listen_timeout',
                                            'number_of_failures': 'speak_listen_failures',
                                            'failure_threshold': 'speak_listen_failure_threshold'})

        smach.StateMachine.add('GIVE_ITEM',
                               HandoverObjectToOperatorState(),
                               transitions={'success': 'THANK_OPERATOR',
                                            'failure': 'task_failure'})
    
        smach.StateMachine.add('THANK_OPERATOR',
                                SpeakState(),
                                transitions={'success':'NAV_TO_START'},
                                remapping={'phrase':'operator_farewell_phrase'})

        smach.StateMachine.add('NAV_TO_START',
                                SimpleNavigateState(),
                                transitions={'success':'FINAL_ANNOUNCEMENT',
                                            'failure':'NAV_TO_START',
                                            'repeat_failure':'task_failure'},
                                remapping={'pose':'start_pose',
                                           'number_of_failures': 'simple_navigation_failures',
                                           'failure_threshold':'simple_navigation_failure_threshold'})
        
        smach.StateMachine.add('FINAL_ANNOUNCEMENT',
                                SpeakState(),
                                transitions={'success':'task_success'},
                                remapping={'phrase':'final_announcement_phrase'})

    
    return sm


if __name__ == '__main__':
    rospy.init_node('open_day_demo_state_machine')

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