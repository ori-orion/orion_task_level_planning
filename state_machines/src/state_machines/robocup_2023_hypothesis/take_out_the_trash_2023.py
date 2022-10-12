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

from state_machines.Reusable_States.include_all import *;
from state_machines.Reusable_States.create_sub_state_machines import *;

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


    # set the robot's pose to speak to the operator - TODO
    # Defaults to...
    operator_pose = Pose();
    operator_pose = utils.dict_to_obj(rospy.get_param('/operator_pose_approx'), operator_pose);
    sm.userdata.operator_pose = operator_pose;

    centre_of_room_pose = Pose();
    centre_of_room_pose = utils.dict_to_obj(rospy.get_param('centre_of_room_pose'), centre_of_room_pose);
    sm.userdata.centre_of_room_pose = centre_of_room_pose;

    # speaking to guests
    # sm.userdata.introduction_to_guest_phrase = "Hi, I'm Bam Bam, welcome to the party! I'm going to learn some information about you so I can tell the host about you!"
    # sm.userdata.no_one_there_phrase = "Hmmm. I don't think anyone is there."


 
    sm.userdata.bin_1_pose = Pose();
    sm.userdata.bin_2_pose = Pose();

    sm.userdata.drop_point = Pose();
    
    # In some cases, we don't want to navigate to the topological node 
    # if the last one we went to was that node. (I.e., if we're then#
    # simply going to search for something in that room.)
    # Note that if this is the same as the first node we nav to, we won't nav to that node at all. 
    # (Nice small hack)
    sm.userdata.prev_node_nav_to = "Living1";

    sm.userdata.nearest_to = None;


    with sm:       
        # # wait for the start signal - this has been replaced by the WAIT_FOR_HOTWORD state
        #   TODO - fix and test the check door state for future competitions
        smach.StateMachine.add('WAIT_FOR_START_SIGNAL',
                                CheckDoorIsOpenState(),
                                transitions={'open':'SAVE_START_TIME', 
                                             'closed':'WAIT_FOR_START_SIGNAL'})

        # save the start time
        smach.StateMachine.add('SAVE_START_TIME',
                                GetTime(),
                                transitions={SUCCESS:'NavThroughDoor'}, 
                                remapping={'current_time':'task_start_time'})

        smach.StateMachine.add(
            'NavThroughDoor',
            SimpleNavigateState(),
            transitions={
                SUCCESS:'NAV_TO_OPERATOR',
                FAILURE:'NAV_TO_OPERATOR',
                REPEAT_FAILURE:'NAV_TO_OPERATOR'},
            remapping={'pose':'bin_1_pose'});

        smach.StateMachine.add(
            'PickUpBinBag',
            PickUpObjectState(object_name="bin_bag"),
            transitions={
                SUCCESS:'NavToDropOff1',
                FAILURE:'PickUpBinBag',
                REPEAT_FAILURE:TASK_FAILURE});
 
        smach.StateMachine.add(
            'NavToDropOff1',
            create_drop_off_bin_bag(),
            transitions={
                SUCCESS:'awefkbaw',
                FAILURE:'PickUpBinBag',
                REPEAT_FAILURE:TASK_FAILURE});

        smach.StateMachine.add(
            'NavToBin2',
            SimpleNavigateState(),
            transitions={
                SUCCESS:'PickUpBinBag',
                FAILURE:'NavToBin2',
                REPEAT_FAILURE:'TASK_FAILURE'},
            remapping={'pose':'bin_2_pose'});

        smach.StateMachine.add(
            'PickUpBinBag',
            PickUpObjectState(object_name="bin_bag"),
            transitions={
                SUCCESS:'NavToDropOff2',
                FAILURE:'PickUpBinBag',
                REPEAT_FAILURE:TASK_FAILURE});

        smach.StateMachine.add(
            'NavToDropOff2',
            create_drop_off_bin_bag(),
            transitions={
                SUCCESS:'TASK_SUCCESS',
                FAILURE:'NavToDropOff2',
                REPEAT_FAILURE:TASK_FAILURE});
        

        # wait for hotword to start the task
        # smach.StateMachine.add('WAIT_FOR_HOTWORD',
        #                         WaitForHotwordState(),
        #                         transitions={SUCCESS: 'ANNOUNCE_TASK_INTENTIONS',
        #                                      FAILURE: 'WAIT_FOR_HOTWORD'},
        #                         remapping={'timeout':'hotword_timeout'})
        
        # announce task intentions
        smach.StateMachine.add('ANNOUNCE_TASK_INTENTIONS',
                                SpeakState(phrase="Hi, I'm Bam Bam and I'm here to find some mates!"),
                                transitions={SUCCESS:'SAVE_START_TIME'},
                                remapping={})
        

        
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
