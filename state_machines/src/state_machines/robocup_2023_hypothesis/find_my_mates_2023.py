#!/usr/bin/env python3
""" Code for the find my mate task.

This file contains code for the find my mate task, including the
state machine itself.

Author: Ricardo Cannizzaro
Owner: Matthew Munks

"""

from ast import operator
import os
from time import sleep
import rospy
import smach_ros
import actionlib

from state_machines.SubStateMachines.include_all import *;
# from state_machines.SubStateMachines.create_sub_state_machines import *;

# from state_machines.reusable_states import * # pylint: disable=unused-wildcard-import
# from set_up_clients import create_stage_1_clients
from orion_actions.msg import SOMObservation, Relation, SearchPersonNotMetGoal
from geometry_msgs.msg import Pose

# import time  # TODO - replace calls to rospy.time library



def create_state_machine():
    """ This function creates and returns the state machine for the task. """

    # Create the state machine
    sm = smach.StateMachine(outcomes=[TASK_SUCCESS, TASK_FAILURE]);

    # Wait for the prameters to be loaded.
    while (not rospy.has_param('params_loaded')):
        rospy.sleep(0.5);

    # Create state machine userdata dictionary elements
    
    # Task params
    sm.userdata.expected_num_guests = 4 # TODO - change to 3
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

    sm.userdata.failure_threshold = 3;

    sm.userdata.ask_operator_name_phrase = "What is your name?"

    # set the robot's pose to speak to the operator - TODO
    # Defaults to...
    operator_pose = Pose();
    operator_pose = utils.dict_to_obj(rospy.get_param('/operator_pose_approx'), operator_pose);
    sm.userdata.operator_pose = operator_pose;

    centre_of_room_pose = Pose();
    centre_of_room_pose = utils.dict_to_obj(rospy.get_param('centre_of_room_pose'), centre_of_room_pose);
    sm.userdata.centre_of_room_pose = centre_of_room_pose;

    execute_nav_commands = True;
    if rospy.has_param('execute_navigation_commands'):
        if rospy.get_param('execute_navigation_commands') == False:
            execute_nav_commands = False;

    # speaking to guests
    # sm.userdata.introduction_to_guest_phrase = "Hi, I'm Bam Bam, welcome to the party! I'm going to learn some information about you so I can tell the host about you!"
    # sm.userdata.no_one_there_phrase = "Hmmm. I don't think anyone is there."


    mid_room_pose = Pose();
    mid_room_pose.position.x = 2.1717187101331037;
    mid_room_pose.position.y = -0.2144994235965718;
    # mid_room_pose.position.x = -18;
    # mid_room_pose.position.y = -9;
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
        smach.StateMachine.add(
            'Startup',
            create_wait_for_startup(),
            transitions={
                SUCCESS:'NavThroughDoor'});

        smach.StateMachine.add(
            'NavThroughDoor',
            TopologicalNavigateState(
                execute_nav_commands=execute_nav_commands, 
                stop_repeat_navigation=True),
            transitions={
                SUCCESS:'NAV_TO_OPERATOR',
                FAILURE:'NAV_TO_OPERATOR',
                REPEAT_FAILURE:'NAV_TO_OPERATOR'},
            remapping={'node_id':'operator_room_node_id'});
        
        # announce task intentions
        smach.StateMachine.add('ANNOUNCE_TASK_INTENTIONS',
                                SpeakState(phrase="Hi, I'm Bam Bam and I'm here to find some mates!"),
                                transitions={SUCCESS:'SAVE_START_TIME'},
                                remapping={})
        
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
        #         SUCCESS:'INTRODUCTION_TO_OPERATOR',
        #         FAILURE:'ANNOUNCE_REPEAT_NAV_FAILURE'},
        #     remapping={
        #         'room_node_uid':'operator_room_node_id',
        #         'failure_threshold':'simple_navigation_failure_threshold',
        #         'human_pose':'operator_pose'})

        smach.StateMachine.add(
            'NAV_TO_OPERATOR',
            SimpleNavigateState(execute_nav_commands=execute_nav_commands),
            transitions={
                SUCCESS:'LOOK_AT_OPERATOR',
                FAILURE:'NAV_TO_OPERATOR',
                REPEAT_FAILURE:'LOOK_AT_OPERATOR'},
            remapping={'pose':'operator_pose'});

        smach.StateMachine.add(
            'LOOK_AT_OPERATOR',
            LookUpState(),
            transitions={SUCCESS:'INTRODUCTION_TO_OPERATOR'});

        # introduce to operator
        smach.StateMachine.add(
            'INTRODUCTION_TO_OPERATOR',
            SpeakState(phrase="Hi, nice to meet you! I am here to help look for your friends!"),
            transitions={SUCCESS:'NAV_TO_ROOM_CENTRE'},
            remapping={})


        smach.StateMachine.add(
            "NAV_TO_ROOM_CENTRE",
            SimpleNavigateState(execute_nav_commands=execute_nav_commands),
            transitions={
                SUCCESS:'SEARCH_FOR_GUEST_SUB',
                FAILURE:'NAV_TO_ROOM_CENTRE',
                REPEAT_FAILURE:'SEARCH_FOR_GUEST_SUB'},
            remapping={'pose':'centre_of_room_pose'});

        # start the search for an un-spoken-to guest
        # create_search_for_guest_sub_state_machine()
        smach.StateMachine.add(
            'SEARCH_FOR_GUEST_SUB', 
            create_search_for_human(execute_nav_commands=execute_nav_commands),
            transitions={SUCCESS:'PointAtAllGuests',
                        FAILURE:'ANNOUNCE_FINISH_SEARCH',
                        'one_person_found':'ANNOUNCE_FINISH_SEARCH'},
            remapping={
                'room_node_uid':'guest_room_node_id',
                'failure_threshold':'topological_navigation_failure_threshold',
                'approximate_operator_pose':'operator_pose'})

        smach.StateMachine.add(
            'PointAtAllGuests',
            create_point_to_all_guests(),
            transitions={
                SUCCESS:'ANNOUNCE_FINISH_SEARCH',
                FAILURE:TASK_FAILURE},
            remapping={'guests':'guests'});

        # At this point we have an array of guests:Human[], of each guest that we have seen.
        # Going from here, we would need to ask names etc, but the real question is whether that's
        # worth it. 

        #region Old infrastructure for asking about guests.
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
                                transitions={SUCCESS:'SHOULD_I_CONTINUE_GUEST_SEARCH_POST_GUEST',
                                            FAILURE:'SHOULD_I_CONTINUE_GUEST_SEARCH_POST_GUEST'},
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
                                SpeakState(phrase="I am continuing to search for guests"),
                                transitions={SUCCESS:'SEARCH_FOR_GUEST_SUB'},
                                remapping={}) 
        #endregion

        smach.StateMachine.add('ANNOUNCE_FINISH_SEARCH',
                                SpeakState(phrase="I have finished searching for guests."),
                                transitions={SUCCESS:'LookBackAtOperator'},   # correct transition
                                # transitions={SUCCESS:'ANNOUNCE_GUEST_DETAILS_TO_OPERATOR'},   # switch for testing withpiout simple nav
                                remapping={});
        
        # smach.StateMachine.add("GetOperatorPose")

        # navigate back to operator - TODO - consider changing to top nav
        smach.StateMachine.add(
            'NAV_RETURN_TO_OPERATOR',
            SimpleNavigateState(execute_nav_commands=execute_nav_commands),
            transitions={SUCCESS:'GET_GUEST_RELATIONS',
                         FAILURE:'NAV_RETURN_TO_OPERATOR',
                         REPEAT_FAILURE:'GET_GUEST_RELATIONS'},
            remapping={'pose':'operator_pose',
                       'number_of_failures': 'simple_navigation_failures',
                       'failure_threshold':'simple_navigation_failure_threshold'});
        
        smach.StateMachine.add(
            "LookBackAtOperator",
            LookAtPoint(),
            transitions={SUCCESS:'ANNOUNCE_FINISH'},
            remapping={'pose':'operator_pose'});

        smach.StateMachine.add(
            'GET_GUEST_RELATIONS',
            GetHumanRelativeLoc(),
            transitions={
                SUCCESS:'ANNOUNCE_GUEST_DETAILS_TO_OPERATOR',
                'no_relevant_matches_found':'ANNOUNCE_GUEST_DETAILS_TO_OPERATOR'});

        smach.StateMachine.add('ANNOUNCE_GUEST_DETAILS_TO_OPERATOR', 
                                AnnounceGuestDetailsToOperator(),
                                transitions={SUCCESS:'FAREWELL_OPERATOR'},
                                remapping={'guest_som_human_ids':'guest_som_human_ids',
                                            'guest_som_obj_ids':'guest_som_obj_ids'})   

        # Farewell operator
        smach.StateMachine.add('FAREWELL_OPERATOR',
                                SpeakState(phrase="My job here is done. Have a nice day!"),
                                # transitions={SUCCESS:'NAV_TO_EXIT'},  # TODO - put back in
                                transitions={SUCCESS:'SAVE_END_TIME'},
                                remapping={}) 

        # leave the arena
        # TODO - consider changing to topological navigation state
        smach.StateMachine.add(
            'NAV_TO_EXIT',
            TopologicalNavigateState(execute_nav_commands=execute_nav_commands),
            transitions={SUCCESS:'SAVE_END_TIME',
                        FAILURE:'NAV_TO_EXIT',
                        REPEAT_FAILURE:TASK_FAILURE},
            remapping={'node_id':'exit_room_node_id'})

        # save the end time
        smach.StateMachine.add('SAVE_END_TIME',
                                GetTime(),
                                transitions={SUCCESS:'ANNOUNCE_FINISH'},
                                remapping={'current_time':'task_end_time'})

        smach.StateMachine.add('ANNOUNCE_FINISH',
                                SpeakState(phrase="I have exited the arena. I am now stopping."),
                                transitions={SUCCESS:TASK_SUCCESS},
                                remapping={})

        sm = setupErrorStates(sm);
        
        # TODO - Reset e
    return sm;        smach.StateMachine.add(
            "LookBackAtOperator",
            LookAtPoint(),
            transitions={SUCCESS:'ANNOUNCE_FINISH'},
            remapping={'pose':'operator_pose'});

rospy.init_node('find_my_mates_state_machine');

# Create the state machine
sm = create_state_machine()

# Create and start the introspection server
sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
sis.start()

# Execute the state machine
sm.execute()

# Run until ctl+c command is received
rospy.spin()
# sis.stop()
