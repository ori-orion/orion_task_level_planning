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


def create_state_machine():
    """ This function creates and returns the state machine for the task. """

    # Create the state machine
    sm = smach.StateMachine(outcomes=['task_success', 'task_failure'])

    # Create state machine userdata dictionary elements
    
    # Task params
    sm.userdata.expected_num_guests = 3 # TODO - change to 3
    sm.userdata.max_search_duration = 200 # seconds

    sm.userdata.person_names = NAMES
    # Load up huge database of additional names (if necessary)
    # import rospkg
    # rospack = rospkg.RosPack()
    # name_file = \
    #     os.path.join(rospack.get_path('state_machines'),'grammars/names.txt')
    # with open(name_file, 'r') as in_file:
    #     sm.userdata.person_names += in_file.read().splitlines()

    sm.userdata.hotword_timeout = 15 # seconds

    sm.userdata.speak_and_listen_params_empty = []
    sm.userdata.speak_and_listen_timeout = 5
    sm.userdata.speak_and_listen_failures = 0
    sm.userdata.speak_and_listen_failure_threshold = 2

    sm.userdata.simple_navigation_failures = 0
    sm.userdata.simple_navigation_failure_threshold = 3
    sm.userdata.simple_navigation_distance_to_humans = 0.5 # metres

    sm.userdata.topological_navigation_failure_threshold = 3

    sm.userdata.task_intentions_phrase = "Hi, I'm Bam Bam and I'm here to find some mates!"

    sm.userdata.nav_repeat_failure_phrase = "Navigation failed too many times, terminating task."
    sm.userdata.speak_and_listen_repeat_failure_phrase = "Speech recognition failed too many times, terminating task."

    sm.userdata.introduction_to_operator_phrase = "Hi, nice to meet you! I am here to help look for your friends!"
    sm.userdata.ask_operator_name_phrase = "What is your name?"
    sm.userdata.speech_recognition_failure_phrase = "I'm sorry but I didn't understand. Let's try that again."

    # set the robot's pose to speak to the operator - TODO
    operator_pose = Pose();
    operator_pose.position.x = 3.7600877304982285;
    operator_pose.position.y = 1.057383767080644;
    operator_pose.position.z = 0.0;
    operator_pose.orientation.x = 0.0;
    operator_pose.orientation.y = 0.0;
    operator_pose.orientation.z = -0.61682206674639;
    operator_pose.orientation.w = 0.7871026222639028;
    sm.userdata.operator_pose = operator_pose;

    # speaking to guests
    sm.userdata.introduction_to_guest_phrase = "Hi, I'm Bam Bam, welcome to the party! I'm going to learn some information about you so I can tell the host about you!"
    sm.userdata.no_one_there_phrase = "Hmmm. I don't think anyone is there."
    
    # search updates
    sm.userdata.continue_search_phrase = "I am continuing to search for guests"
    sm.userdata.finish_search_phrase = "I am finished searching for guests and am returning to the host"

    # farewell operator
    sm.userdata.farewell_operator_phrase = "My job here is done. Have a nice day!"


    mid_room_pose = Pose();
    mid_room_pose.position.x = 2.1717187101331037;
    mid_room_pose.position.y = -0.2144994235965718;
    mid_room_pose.position.z = 0.0;
    mid_room_pose.orientation.x = 0.0;
    mid_room_pose.orientation.y = 0.0;
    mid_room_pose.orientation.z = -0.6771155204369553;
    mid_room_pose.orientation.w = 0.7358767369494643;
    sm.userdata.mid_room_pose = mid_room_pose;

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
    sm.userdata.operator_som_id = "1234"       # SAVE_OPERATOR_INFO_TO_SOM state will set this

    # guest tracking
    sm.userdata.guest_som_human_ids = []
    sm.userdata.guest_som_obj_ids = []

    # top nav
    sm.userdata.node_list = ['Node1', 'Node2', 'Node3']
    sm.userdata.nodes_not_searched = list(sm.userdata.node_list)  # used in the guest search sub state machine

    # Where is the operator starting out?
    sm.userdata.operator_room_node_id = "Living1";
    # Which room are the guests in?
    sm.userdata.guest_room_node_id = "Living1";

    sm.userdata.exit_room_node_id = "Exit";

    sm.userdata.number_of_failures = 0;
    
    # In some cases, we don't want to navigate to the topological node 
    # if the last one we went to was that node. (I.e., if we're then#
    # simply going to search for something in that room.)
    # Note that if this is the same as the first node we nav to, we won't nav to that node at all. 
    # (Nice small hack)
    sm.userdata.prev_node_nav_to = "Living1";

    sm.userdata.nearest_to = None;

    with sm:
        # TODO - remove after testing
        # short-ciruit straight to SEARCH_FOR_GUEST_SUB to test logic 
        # smach.StateMachine.add('SEARCH_FOR_GUEST_SUB', 
        #                         create_search_for_guest_sub_state_machine(),
        #                         transitions={'success':'CREATE_POSE_TO_APPROACH_GUEST',
        #                                     'failure':'ANNOUNCE_FINISH_SEARCH'},
        #                         remapping={'nodes_not_searched':'nodes_not_searched',
        #                                     'operator_uid':'operator_som_id',
        #                                     'failure_threshold':'topological_navigation_failure_threshold'})
        
        # # wait for the start signal - this has been replaced by the WAIT_FOR_HOTWORD state
        #   TODO - fix and test the check door state for future competitions
        smach.StateMachine.add('WAIT_FOR_START_SIGNAL',
                                CheckDoorIsOpenState(),
                                transitions={   'open':'SAVE_START_TIME', 
                                                'closed':'WAIT_FOR_START_SIGNAL'})

        # save the start time
        smach.StateMachine.add('SAVE_START_TIME',
                                GetTime(),
                                transitions={'success':'NavThroughDoor'}, # correct transition
                                # transitions={'success':'SEARCH_FOR_GUEST_SUB'}, # TODO - switch for testing
                                # transitions={'success':'LEARN_GUEST_SUB'}, # TODO - switch for testing
                                remapping={'current_time':'task_start_time'})

        smach.StateMachine.add(
            'NavThroughDoor',
            TopologicalNavigateState(stop_repeat_navigation=True),
            transitions={
                'success':'NAV_TO_OPERATOR',
                'failure':'NAV_TO_OPERATOR',
                'repeat_failure':'NAV_TO_OPERATOR'},
            remapping={'node_id':'operator_room_node_id'});

        # wait for hotword to start the task
        # smach.StateMachine.add('WAIT_FOR_HOTWORD',
        #                         WaitForHotwordState(),
        #                         transitions={'success': 'ANNOUNCE_TASK_INTENTIONS',
        #                                      'failure': 'WAIT_FOR_HOTWORD'},
        #                         remapping={'timeout':'hotword_timeout'})
        
        # announce task intentions
        smach.StateMachine.add('ANNOUNCE_TASK_INTENTIONS',
                                SpeakState(),
                                transitions={'success':'SAVE_START_TIME'},
                                remapping={'phrase':'task_intentions_phrase'})
        
        """
        navigate to operator
        Outputs: 
            closest_human:Human     - Does none of the talking to the human.
            human_object_uid:str    - What is the object uid of the human in question. (Makes it slightly more general for later logic)
            operator_pose:Pose      - Returns the pose of the operator.
        """
        # smach.StateMachine.add(
        #     'NAV_TO_OPERATOR',
        #     create_search_for_human(),
        #     transitions={
        #         'success':'INTRODUCTION_TO_OPERATOR',
        #         'failure':'ANNOUNCE_REPEAT_NAV_FAILURE'},
        #     remapping={
        #         'room_node_uid':'operator_room_node_id',
        #         'failure_threshold':'simple_navigation_failure_threshold',
        #         'human_pose':'operator_pose'})

        smach.StateMachine.add(
            'NAV_TO_OPERATOR',
            SimpleNavigateState(),
            transitions={
                'success':'LOOK_AT_OPERATOR',
                'failure':'NAV_TO_OPERATOR',
                'repeat_failure':'LOOK_AT_OPERATOR'},
            remapping={'pose':'operator_pose'});

        smach.StateMachine.add(
            'LOOK_AT_OPERATOR',
            LookUpState(),
            transitions={'success':'INTRODUCTION_TO_OPERATOR'});

        # announce nav repeat failure
        smach.StateMachine.add('ANNOUNCE_REPEAT_NAV_FAILURE',
                                SpeakState(),
                                transitions={'success':'task_failure'},
                                remapping={'phrase':'nav_repeat_failure_phrase'})

        # introduce to operator
        smach.StateMachine.add('INTRODUCTION_TO_OPERATOR',
                                SpeakState(),
                                transitions={'success':'ASK_OPERATOR_NAME'},
                                remapping={'phrase':'introduction_to_operator_phrase'})

        # ask for operator's name - Replaced by AskPersonNameState state below - remove after testing
        # smach.StateMachine.add('ASK_OPERATOR_NAME',
        #                        SpeakAndListenState(),
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

        # ask for operator's name - New ask guest name action server - TODO - test        
        smach.StateMachine.add('ASK_OPERATOR_NAME',
                                AskPersonNameState(),
                                transitions={'success': 'SearchForOperator',
                                            'failure':'ANNOUNCE_MISSED_NAME',
                                            'repeat_failure':'SearchForOperator'},
                                remapping={'question':'ask_operator_name_phrase',
                                            'recognised_name': 'operator_name',
                                            'timeout':'speak_and_listen_timeout',
                                            'number_of_failures': 'speak_and_listen_failures',
                                            'failure_threshold': 'speak_and_listen_failure_threshold'})

        # announce that we missed the name, and that we will try again
        smach.StateMachine.add('ANNOUNCE_MISSED_NAME',
                                SpeakState(),
                                transitions={'success':'ASK_OPERATOR_NAME'},
                                remapping={'phrase':'speech_recognition_failure_phrase'})

        # announce speech recognition repeat failure
        smach.StateMachine.add('ANNOUNCE_REPEAT_SPEECH_RECOGNITION_FAILURE',
                                SpeakState(),
                                transitions={'success':'task_failure'},
                                remapping={'phrase':'speak_and_listen_repeat_failure_phrase'})

        smach.StateMachine.add(
            'SearchForOperator',
            GetNearestHuman(),
            transitions={
                'new_human_found':'SAVE_OPERATOR_INFO_TO_SOM',
                'human_not_found':'SAVE_OPERATOR_INFO_TO_SOM_HARDCODED_BACKUP',
                'existing_human_found':'SAVE_OPERATOR_INFO_TO_SOM'},
            remapping={'nearest_to':'operator_pose'});

        # save the operator info to the SOM
        smach.StateMachine.add('SAVE_OPERATOR_INFO_TO_SOM',
                               SaveOperatorToSOM(),
                               transitions={'success':'CREATE_PHRASE_START_SEARCH',
                                            'failure':'task_failure'},
                                remapping={'operator_name':'operator_name', 
                                            'operator_som_id':'operator_som_id'})

        smach.StateMachine.add('SAVE_OPERATOR_INFO_TO_SOM_HARDCODED_BACKUP',
                       SaveOperatorToSOM(operator_pose=operator_pose),
                       transitions={'success':'CREATE_PHRASE_START_SEARCH',
                                    'failure':'task_failure'},
                        remapping={'operator_name':'operator_name', 
                                    'operator_som_id':'operator_som_id'})

        # create the search start phrase
        smach.StateMachine.add('CREATE_PHRASE_START_SEARCH',
                                CreatePhraseStartSearchForPeopleState(),
                                transitions={'success':'ANNOUNCE_SEARCH_START'},
                                remapping={'operator_name':'operator_name',
                                            'phrase':'announce_search_start_phrase'})
        
        # announce search start
        smach.StateMachine.add('ANNOUNCE_SEARCH_START',
                                SpeakState(),
                                transitions={'success':'NAV_TO_ROOM_CENTRE'},
                                remapping={'phrase':'announce_search_start_phrase'})

        smach.StateMachine.add(
            "NAV_TO_ROOM_CENTRE",
            SimpleNavigateState(),
            transitions={
                'success':'SEARCH_FOR_GUEST_SUB',
                'failure':'NAV_TO_ROOM_CENTRE',
                'repeat_failure':'SEARCH_FOR_GUEST_SUB'},
            remapping={'pose':'mid_room_pose'});

        # start the search for an un-spoken-to guest
        # create_search_for_guest_sub_state_machine()
        smach.StateMachine.add('SEARCH_FOR_GUEST_SUB', 
                                create_search_for_human(),
                                transitions={'success':'SHOULD_I_CONTINUE_GUEST_SEARCH_INTERMEDIATE',
                                            'failure':'ANNOUNCE_FINISH_SEARCH'},
                                remapping={'room_node_uid':'guest_room_node_id',
                                            'failure_threshold':'topological_navigation_failure_threshold'})

        # # Create pose to approach the guest
        # smach.StateMachine.add('CREATE_POSE_TO_APPROACH_GUEST',
        #                         CreatePoseToApproachHuman(),
        #                         transitions={'success':'NAV_TO_GUEST',
        #                                      'failure':'SEARCH_FOR_GUEST_SUB'},
        #                         remapping={'human_id':'guest_uid',
        #                                     'distance_to_human': 'simple_navigation_distance_to_humans',
        #                                     'approach_pose':'approach_guest_pose'}) 
        
        # # navigate to guest
        # smach.StateMachine.add('NAV_TO_GUEST',
        #                        SimpleNavigateState(),
        #                        transitions={'success':'LEARN_GUEST_SUB',
        #                                     'failure':'NAV_TO_GUEST',
        #                                     'repeat_failure':'ANNOUNCE_REPEAT_NAV_FAILURE'},
        #                         remapping={'pose':'approach_guest_pose',
        #                                    'number_of_failures': 'simple_navigation_failures',
        #                                    'failure_threshold':'simple_navigation_failure_threshold'})

        smach.StateMachine.add('SHOULD_I_CONTINUE_GUEST_SEARCH_INTERMEDIATE', 
                                ShouldIContinueGuestSearchState(),
                                transitions={'yes':'LEARN_GUEST_SUB',
                                            'no':'ANNOUNCE_FINISH_SEARCH'},
                                remapping={'guest_som_human_ids':'guest_som_human_ids',
                                            'max_search_duration':'max_search_duration',
                                            'expected_num_guests':'expected_num_guests',
                                            'start_time':'task_start_time'})

        # run the LEARN_GUEST_SUB sub-state machine  
        smach.StateMachine.add('LEARN_GUEST_SUB', 
                                create_learn_guest_sub_state_machine(),
                                transitions={'success':'SHOULD_I_CONTINUE_GUEST_SEARCH_POST_GUEST',
                                            'failure':'SHOULD_I_CONTINUE_GUEST_SEARCH_POST_GUEST'},
                                remapping={'guest_som_human_ids':'guest_som_human_ids',
                                            'guest_som_obj_ids':'guest_som_obj_ids',
                                            'person_names':'person_names'})

        smach.StateMachine.add('SHOULD_I_CONTINUE_GUEST_SEARCH_POST_GUEST', 
                                ShouldIContinueGuestSearchState(),
                                transitions={'yes':'ANNOUNCE_CONTINUE_SEARCH',
                                            'no':'ANNOUNCE_FINISH_SEARCH'},
                                remapping={'guest_som_human_ids':'guest_som_human_ids',
                                            'max_search_duration':'max_search_duration',
                                            'expected_num_guests':'expected_num_guests',
                                            'start_time':'task_start_time'})
        
        
        smach.StateMachine.add('ANNOUNCE_CONTINUE_SEARCH',
                                SpeakState(),
                                transitions={'success':'SEARCH_FOR_GUEST_SUB'},
                                remapping={'phrase':'continue_search_phrase'}) 

        smach.StateMachine.add('ANNOUNCE_FINISH_SEARCH',
                                SpeakState(),
                                transitions={'success':'NAV_RETURN_TO_OPERATOR'},   # correct transition
                                # transitions={'success':'ANNOUNCE_GUEST_DETAILS_TO_OPERATOR'},   # switch for testing withpiout simple nav
                                remapping={'phrase':'finish_search_phrase'}) 
        
        # smach.StateMachine.add("GetOperatorPose")

        # navigate back to operator - TODO - consider changing to top nav
        smach.StateMachine.add('NAV_RETURN_TO_OPERATOR',
                               SimpleNavigateState(),
                               transitions={'success':'GET_GUEST_RELATIONS',
                                            'failure':'NAV_RETURN_TO_OPERATOR',
                                            'repeat_failure':'GET_GUEST_RELATIONS'},
                                remapping={'pose':'operator_pose',
                                           'number_of_failures': 'simple_navigation_failures',
                                           'failure_threshold':'simple_navigation_failure_threshold'});

        smach.StateMachine.add(
            'GET_GUEST_RELATIONS',
            GetHumanRelativeLoc(),
            transitions={
                'success':'ANNOUNCE_GUEST_DETAILS_TO_OPERATOR',
                'no_relevant_matches_found':'ANNOUNCE_GUEST_DETAILS_TO_OPERATOR'});

        smach.StateMachine.add('ANNOUNCE_GUEST_DETAILS_TO_OPERATOR', 
                                AnnounceGuestDetailsToOperator(),
                                transitions={'success':'FAREWELL_OPERATOR'},
                                remapping={'guest_som_human_ids':'guest_som_human_ids',
                                            'guest_som_obj_ids':'guest_som_obj_ids'})   

        # Farewell operator
        smach.StateMachine.add('FAREWELL_OPERATOR',
                                SpeakState(),
                                # transitions={'success':'NAV_TO_EXIT'},  # TODO - put back in
                                transitions={'success':'SAVE_END_TIME'},
                                remapping={'phrase':'farewell_operator_phrase'}) 

        # leave the arena
        # TODO - consider changing to topological navigation state
        smach.StateMachine.add(
            'NAV_TO_EXIT',
            TopologicalNavigateState(),
            transitions={'success':'SAVE_END_TIME',
                        'failure':'NAV_TO_EXIT',
                        'repeat_failure':'task_failure'},
            remapping={'node_id':'exit_room_node_id'})

        # save the end time
        smach.StateMachine.add('SAVE_END_TIME',
                                GetTime(),
                                transitions={'success':'ANNOUNCE_FINISH'},
                                remapping={'current_time':'task_end_time'})

        smach.StateMachine.add('ANNOUNCE_FINISH',
                                SpeakState(),
                                transitions={'success':'task_success'},
                                remapping={'phrase':'announce_finish_phrase'})
        
        # TODO - Reset e
    return sm;

rospy.init_node('find_my_mates_state_machine');

# Create the state machine
sm = create_state_machine()

# Create and start the introspection server
# sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
# sis.start()

# Execute the state machine
sm.execute()

# Run until ctl+c command is received
rospy.spin()
# sis.stop()
