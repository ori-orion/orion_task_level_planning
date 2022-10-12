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


    bin_1_pose = Pose();
    bin_1_pose = utils.dict_to_obj(rospy.get_param('bin_1_pose'), bin_1_pose);
    sm.userdata.bin_1_pose = bin_1_pose;

    bin_2_pose = Pose();
    bin_2_pose = utils.dict_to_obj(rospy.get_param('bin_2_pose'), bin_2_pose);
    sm.userdata.bin_2_pose = bin_2_pose;

    drop_point = Pose();
    drop_point = utils.dict_to_obj(rospy.get_param('drop_off_location'), drop_point);
    sm.userdata.drop_point = drop_point;
    
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
        smach.StateMachine.add(
            'WAIT_FOR_START_SIGNAL',
            CheckDoorIsOpenState(),
            transitions={'open':'SAVE_START_TIME', 
                         'closed':'WAIT_FOR_START_SIGNAL'})

        # save the start time
        smach.StateMachine.add(
            'SAVE_START_TIME',
            GetTime(),
            transitions={SUCCESS:'DropOff1'}, 
            remapping={'current_time':'task_start_time'})
 
        smach.StateMachine.add(
            'DropOff1',
            create_drop_off_bin_bag(),
            transitions={
                SUCCESS:'DropOff2',
                FAILURE:'DropOff2'},
            remapping={
                'pick_up_location':'bin_1_pose',
                'drop_off_location':'drop_point'});

        smach.StateMachine.add(
            'DropOff2',
            create_drop_off_bin_bag(),
            transitions={
                SUCCESS:TASK_SUCCESS,
                FAILURE:TASK_FAILURE},
            remapping={
                'pick_up_location':'bin_2_pose',
                'drop_off_location':'drop_point'});
        
    return sm;

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
