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


    with sm:
        smach.StateMachine.add(
            'LookAtOperator',
            LookUpState(height=1.7),
            transitions={SUCCESS: 'SpeakToOperator'});
        
        smach.StateMachine.add(
            'SpeakToOperator',
            AskFromSelection(
                questions=[
                    ("name", "Hi, I'm Bam Bam. I hope you're enjoying the day. What's your name?", NAMES)
                ],
                append_result_to_array=False),
            transitions={
                SUCCESS:'GiveInformation',
                "no_response":'GiveInformation'});

        smach.StateMachine.add(
            'GiveInformation',
            SpeakState(phrase="I am a Toyota Human Support Robot, designed to help people with everyday tasks around the house. \
                I can do lots of things: recognise human speech, detect objects with my camera, fetch things with my gripper,\
                take out the trash, and even act as a host for your next party! I am used for research by PhD students, as well\
                as for a care home projcet. I am also used by team Orion, to compete at the annual RoboCup at Home competition."),
            transitions={SUCCESS:TASK_SUCCESS});
    
        sm = setupErrorStates(sm);
        
        # TODO - Reset e
    return sm;

rospy.init_node('generic_hsr_welcome');

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
