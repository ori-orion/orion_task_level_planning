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

from Reusable_States.include_all import *;

# from state_machines.reusable_states import * # pylint: disable=unused-wildcard-import
# from set_up_clients import create_open_day_clients
from geometry_msgs.msg import Pose


def create_state_machine(userdata=None):
    """ This function builds the state machine for the ORI open day demo.

    Args:
        userdata: A dictionary of optional input arguments to override default

    Returns:
        sm: The complete state machine
    """

    # Create the state machine
    sm = smach.StateMachine(outcomes=[TASK_SUCCESS, TASK_FAILURE])
    sm.userdata.intro_phrase = ("Hi, my name is Bam Bam, and " + 
                 "today we're going to show you what I can do!")
    # sm.userdata.intro_phrase = "Hi, my name is Bam Bam." # alternate, shorter intro
    sm.userdata.operator_question = "Hi, what's your name?"
    sm.userdata.speak_listen_failures = 0
    sm.userdata.speak_listen_failure_threshold = 3
    sm.userdata.operator_names = NAMES
    # Load up huge database of additional names (if necessary)
    import rospkg
    rospack = rospkg.RosPack()
    name_file = \
        os.path.join(rospack.get_path('state_machines'),'grammars/names.txt')
    with open(name_file, 'r') as in_file:
        sm.userdata.operator_names += in_file.read().splitlines()

    sm.userdata.speak_and_listen_params_empty = []
    sm.userdata.speak_and_listen_timeout = 5

    sm.userdata.simple_navigation_failures = 0
    sm.userdata.simple_navigation_failure_threshold = 3

    # Use for real world (ground floor meeting room, etc)
    sm.userdata.operator_pose = Pose()
    sm.userdata.operator_pose.position.x = 1.123
    sm.userdata.operator_pose.position.y = -0.112
    sm.userdata.operator_pose.position.z = 0.0
    sm.userdata.operator_pose.orientation.x = 0.0
    sm.userdata.operator_pose.orientation.y = 0.0
    sm.userdata.operator_pose.orientation.z = -0.7141349154217675
    sm.userdata.operator_pose.orientation.w = 0.7000080875072408

    # Use for Gazebo sim
    # sm.userdata.operator_pose = Pose()
    # sm.userdata.operator_pose.position.x = 1.57894771198
    # sm.userdata.operator_pose.position.y = 0.59994803628
    # sm.userdata.operator_pose.position.z = 0.0
    # sm.userdata.operator_pose.orientation.x = 0.0
    # sm.userdata.operator_pose.orientation.y = 0.0
    # sm.userdata.operator_pose.orientation.z = -0.362624641735
    # sm.userdata.operator_pose.orientation.w = 0.931935281662

    # Use for Gazebo sim - go directly to bottle position
    # sm.userdata.operator_pose = Pose()
    # sm.userdata.operator_pose.position.x = 0.23832461091673765
    # sm.userdata.operator_pose.position.y = 0.0
    # sm.userdata.operator_pose.position.z = 0.0
    # sm.userdata.operator_pose.orientation.x = 0.0
    # sm.userdata.operator_pose.orientation.y = 0.0
    # sm.userdata.operator_pose.orientation.z = -0.7027350031010926
    # sm.userdata.operator_pose.orientation.w =  0.711451695771756

    sm.userdata.pickup_object_names = OBJECTS
    sm.userdata.object_pickup_question = "Great! What would you like me to pick up?"

    # Use for real world (ground floor meeting room, etc)
    sm.userdata.pickup_pose = Pose()
    sm.userdata.pickup_pose.position.x = 1.123
    sm.userdata.pickup_pose.position.y = -0.112
    sm.userdata.pickup_pose.position.z = 0.0
    sm.userdata.pickup_pose.orientation.x = 0.0
    sm.userdata.pickup_pose.orientation.y = 0.0
    sm.userdata.pickup_pose.orientation.z = -0.7141349154217675
    sm.userdata.pickup_pose.orientation.w = 0.7000080875072408

    # Use for Gazebo sim - to pickup water bottle
    # sm.userdata.pickup_pose = Pose()
    # sm.userdata.pickup_pose.position.x = 0.23832461091673765
    # sm.userdata.pickup_pose.position.y = 0.0
    # sm.userdata.pickup_pose.position.z = 0.0
    # sm.userdata.pickup_pose.orientation.x = 0.0
    # sm.userdata.pickup_pose.orientation.y = 0.0
    # sm.userdata.pickup_pose.orientation.z = -0.7027350031010926
    # sm.userdata.pickup_pose.orientation.w =  0.711451695771756

    execute_nav_commands = False;

    sm.userdata.object_ar_markers = AR_MARKERS
    sm.userdata.pickup_failures = 0
    sm.userdata.pickup_failure_threshold = 3

    sm.userdata.ask_for_help_speak_and_listen_timeout = 10

    sm.userdata.ask_when_ready_question = ("Please say ready when you're ready "
                    "for me to hand this back to you?")
    sm.userdata.ask_when_ready_responses = READY

    # override default userdata with input arguments
    if userdata is not None:
        sm.userdata.update(userdata)

    with sm:
        
        smach.StateMachine.add('STORE_INITIAL_LOCATION',
                                GetRobotLocationState(),
                                transitions={'stored':'INTRO'},
                                remapping={'robot_location':'start_pose'})

        smach.StateMachine.add('INTRO',
                                SpeakState("Hi, my name is Bam Bam, and `today we're going to show you what I can do!"),
                                transitions={SUCCESS:'NAV_TO_OPERATOR'})#,
                                #remapping={'phrase':'intro_phrase'})

        smach.StateMachine.add('NAV_TO_OPERATOR',
                               SimpleNavigateState(execute_nav_commands),
                               transitions={SUCCESS:'ASK_OPERATOR_NAME',
                                            FAILURE:'NAV_TO_OPERATOR',
                                            REPEAT_FAILURE:TASK_FAILURE},
                                remapping={'pose':'operator_pose',
                                           'number_of_failures': 'simple_navigation_failures',
                                           'failure_threshold':'simple_navigation_failure_threshold'})
    
        smach.StateMachine.add('ASK_OPERATOR_NAME',
                               SpeakAndListenState(),
                                transitions={SUCCESS: 'ASK_PICK_UP_OBJ',
                                            FAILURE:'ASK_OPERATOR_NAME',
                                            REPEAT_FAILURE:TASK_FAILURE},
                                remapping={'question':'operator_question',
                                            'operator_response': 'operator_name',
                                            'candidates':'operator_names',
                                            'params':'speak_and_listen_params_empty',
                                            'timeout':'speak_and_listen_timeout',
                                            'number_of_failures': 'speak_listen_failures',
                                            'failure_threshold': 'speak_listen_failure_threshold'})

        smach.StateMachine.add('ASK_PICK_UP_OBJ',
                               SpeakAndListenState(),
                                transitions={SUCCESS: 'NAV_TO_PICKUP',
                                            FAILURE:'ASK_PICK_UP_OBJ',
                                            REPEAT_FAILURE:TASK_FAILURE},
                                remapping={'question':'object_pickup_question',
                                            'operator_response': 'pickup_object_name',
                                            'candidates':'pickup_object_names',
                                            'params':'speak_and_listen_params_empty',
                                            'timeout':'speak_and_listen_timeout',
                                            'number_of_failures': 'speak_listen_failures',
                                            'failure_threshold': 'speak_listen_failure_threshold'})


        smach.StateMachine.add('NAV_TO_PICKUP',
                               SimpleNavigateState(execute_nav_commands),
                               transitions={SUCCESS:'CHECK_IN_SOM',
                                            FAILURE:'NAV_TO_PICKUP',
                                            REPEAT_FAILURE:TASK_FAILURE},
                                remapping={'pose':'pickup_pose',
                                           'number_of_failures': 'simple_navigation_failures',
                                           'failure_threshold': 'simple_navigation_failure_threshold'})

        smach.StateMachine.add(
            'CHECK_IN_SOM',
            has_seen_object(),
            transitions={
                'object_seen':'PICKUP_OBJECT',
                'object_not_seen':'LookRound',
                FAILURE:TASK_FAILURE},
            remapping={'object_class':'pickup_object_name'});

        smach.StateMachine.add(
            'LookRound',
            SpinState(),
            transitions={
                SUCCESS:'CHECK_IN_SOM'},
            remapping={});
                
        smach.StateMachine.add('PICKUP_OBJECT',
                               PickUpObjectState(),
                               transitions={SUCCESS:'NAV_BACK_TO_OPERATOR',
                                            FAILURE:'PICKUP_OBJECT',
                                            REPEAT_FAILURE:'CREATE_ASK_FOR_HELP_PHRASE'},
                                remapping={ 'object_name':'pickup_object_name', 
                                            'number_of_failures':'pickup_failures', 
                                            'failure_threshold':'pickup_failure_threshold', 
                                            'ar_marker_ids':'object_ar_markers'})

        smach.StateMachine.add('CREATE_ASK_FOR_HELP_PHRASE',
                                CreatePhraseAskForHelpPickupObjectState(),
                                transitions={SUCCESS:'ASK_FOR_HELP'},
                                remapping={'object_name': 'pickup_object_name',
                                           'phrase':'operator_return_phrase'})

        smach.StateMachine.add('ASK_FOR_HELP',
                               SpeakAndListenState(),
                                transitions={SUCCESS: 'RECEIVE_ITEM_FROM_OPERATOR',
                                            FAILURE:'ASK_FOR_HELP',
                                            REPEAT_FAILURE:TASK_FAILURE},
                                remapping={'question':'operator_return_phrase',
                                            'operator_response': 'ask_when_ready_response',
                                            'candidates':'ask_when_ready_responses',
                                            'params':'speak_and_listen_params_empty',
                                            'timeout':'ask_for_help_speak_and_listen_timeout',
                                            'number_of_failures': 'speak_listen_failures',
                                            'failure_threshold': 'speak_listen_failure_threshold'})

        smach.StateMachine.add('RECEIVE_ITEM_FROM_OPERATOR',
                               ReceiveObjectFromOperatorState(),
                               transitions={SUCCESS:'NAV_BACK_TO_OPERATOR',
                                            FAILURE:TASK_FAILURE})

        smach.StateMachine.add('NAV_BACK_TO_OPERATOR',
                                SimpleNavigateState(execute_nav_commands),
                                transitions={SUCCESS:'CREATE_OPERATOR_RETURN_PHRASE',
                                            FAILURE:'NAV_BACK_TO_OPERATOR',
                                            REPEAT_FAILURE:TASK_FAILURE},
                                remapping={'pose':'operator_pose',
                                           'number_of_failures': 'simple_navigation_failures',
                                           'failure_threshold':'simple_navigation_failure_threshold'})


        smach.StateMachine.add('CREATE_OPERATOR_RETURN_PHRASE',
                                CreatePhraseAnnounceRetrievedItemToNamedOperatorState(),
                                transitions={SUCCESS:'SPEAK_TO_OPERATOR'},
                                remapping={'operator_name':'operator_name',
                                           'object_name': 'pickup_object_name',
                                           'phrase':'operator_return_phrase'})

        smach.StateMachine.add('SPEAK_TO_OPERATOR',
                                SpeakState(),
                                transitions={SUCCESS:'ARRIVAL_QUESTION'},
                                remapping={'phrase':'operator_return_phrase'})

        # This does not seem to be used in the open day demo - TODO: remove
        # """phrase = ("Now I'm ready to follow you. Please go slow and say " +
        #           "cancel when you want me to stop.")
        # smach.StateMachine.add('ReadyToFollow',
        #                        SpeakState(action_dict, global_store, phrase),
        #                        transitions={SUCCESS: 'Follow',
        #                                     FAILURE: 'Follow'})
        #
        #
        # smach.StateMachine.add('Follow',
        #                         make_follow_hotword_state(action_dict, 
        #                                                   global_store),
        #                         transitions={SUCCESS: 'ArrivalQuestion',
        #                                      FAILURE: 'Follow',
        #                                      REPEAT_FAILURE: TASK_FAILURE})"""


        smach.StateMachine.add('ARRIVAL_QUESTION',
                               SpeakAndListenState(question="Please say ready when you're ready for me to hand this back to you?"),
                                transitions={SUCCESS: 'GIVE_ITEM',
                                            FAILURE:'ARRIVAL_QUESTION',
                                            REPEAT_FAILURE:TASK_FAILURE},
                                remapping={ 'operator_response': 'ask_when_ready_response',
                                            'candidates':'ask_when_ready_responses',
                                            'params':'speak_and_listen_params_empty',
                                            'timeout':'speak_and_listen_timeout',
                                            'number_of_failures': 'speak_listen_failures',
                                            'failure_threshold': 'speak_listen_failure_threshold'})

        smach.StateMachine.add('GIVE_ITEM',
                               HandoverObjectToOperatorState(),
                               transitions={SUCCESS: 'THANK_OPERATOR',
                                            FAILURE: TASK_FAILURE})
    
        smach.StateMachine.add('THANK_OPERATOR',
                                SpeakState("It looks like my job here is done. Have a nice day!"),
                                transitions={SUCCESS:'NAV_TO_START'})

        smach.StateMachine.add('NAV_TO_START',
                                SimpleNavigateState(execute_nav_commands),
                                transitions={SUCCESS:'FINAL_ANNOUNCEMENT',
                                            FAILURE:'NAV_TO_START',
                                            REPEAT_FAILURE:TASK_FAILURE},
                                remapping={'pose':'start_pose',
                                           'number_of_failures': 'simple_navigation_failures',
                                           'failure_threshold':'simple_navigation_failure_threshold'})
        
        smach.StateMachine.add('FINAL_ANNOUNCEMENT',
                                SpeakState("I'm back where I started, woohoo!"),
                                transitions={SUCCESS:TASK_SUCCESS})

    return sm

# TODO - remove once the commented-out SOM interaction is understood
# class SpeakToOperatorState(ActionServiceState):
#     """ Smach state for the robot to say stuff.
#
#     This class has the robot say something and return success if it has been
#     said. This says the operator name and the item that's been picked up
#
#     Attributes:
#         phrase: What we want the robot to say
#     """
#     def __init__(self, action_dict, global_store):
#         outcomes = [SUCCESS]
#         super(SpeakToOperatorState, self).__init__(action_dict=action_dict,
#                                                    global_store=global_store,
#                                                    outcomes=outcomes)
#   
#     def execute(self, userdata):
#
#         #obj1 = SOMObservation()
#         #obj1.obj_id = self.global_store['people_found'][0]
#         #rel = Relation()
#         #obj2 = SOMObservation()
#
#         #matches = self.action_dict['SOMQuery'](obj1, rel, obj2, Pose()).matches
#        
#         #name = matches[0].obj1.name
#         name = self.global_store['operator_name']
#         picked_up = self.global_store['pick_up']
#
#         phrase = "Hi, " + name + ", I've brought you the " + picked_up 
#
#         action_goal = TalkRequestGoal()
#         action_goal.data.language = Voice.kEnglish
#         action_goal.data.sentence = phrase
#         self.action_dict['Speak'].send_goal(action_goal)
#         self.action_dict['Speak'].wait_for_result()
#
#         # Can only succeed
#         return self._outcomes[0]


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