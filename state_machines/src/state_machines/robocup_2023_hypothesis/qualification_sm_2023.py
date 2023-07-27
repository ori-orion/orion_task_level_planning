#!/usr/bin/env python3
""" Code for the find my mate task.

This file contains code for the find my mate task, including the
state machine itself.

Author: Matthew Munks
Owner: Matthew Munks

"""

import os
import rospy
import smach_ros
import actionlib

from state_machines.Reusable_States.include_all import *;
from state_machines.SubStateMachines.include_all import *;

# from state_machines.reusable_states import * # pylint: disable=unused-wildcard-import
# from set_up_clients import create_stage_1_clients
# from orion_actions.msg import SOMObservation, Relation, SearchPersonNotMetGoal
from geometry_msgs.msg import Pose

import time  # TODO - replace calls to rospy.time library

def construct_qualification_sm():
    sm = smach.StateMachine(outcomes=['task_success', 'task_failure'])

    if rospy.has_param('centre_of_room_pose') and rospy.has_param('exit_pose'):
        sm.userdata.centre_of_room_pose = utils.dict_to_obj( 
            rospy.get_param('centre_of_room_pose'),
            Pose());
        sm.userdata.exit_pose = utils.dict_to_obj( 
            rospy.get_param('exit_pose'),
            Pose());

    execute_nav_commands = rospy.get_param('execute_navigation_commands');

    sm.userdata.hotword_timeout = 3;

    sm.userdata.number_of_failures = 0;
    sm.userdata.failure_threshold = 3;

    end_loc = Pose();
    end_loc.orientation.w = 1;
    sm.userdata.pose = end_loc;

    with sm:
        smach.StateMachine.add(
            'Startup',
            create_wait_for_startup(),
            transitions={SUCCESS:'NAV_TO_CENTRE_OF_ROOM'});

        smach.StateMachine.add(
            'NAV_TO_CENTRE_OF_ROOM',
            SimpleNavigateState_v2(execute_nav_commands=execute_nav_commands),
            transitions={
                SUCCESS:'WAIT_FOR_WRIST_WRENCH',
                NAVIGATIONAL_FAILURE:'WAIT_FOR_WRIST_WRENCH'},
            remapping={'pose':'centre_of_room_pose'});

        smach.StateMachine.add(
            'WAIT_FOR_WRIST_WRENCH',
            WaitForWristWrench(),
            transitions={
                'success':'Speak'});        

        smach.StateMachine.add(
            'WAIT_FOR_HOTWORD',
            WaitForHotwordState(),
            transitions={
                SUCCESS:'Speak', 
                'failure':'WAIT_FOR_HOTWORD'},
            remapping={'timeout':'hotword_timeout'});
        
        smach.StateMachine.add(
            'Speak',
            SpeakState('Thank you, moving to the exit.'),
            transitions={
                SUCCESS:'NAV_TO_EXIT'});

        smach.StateMachine.add(
            'NAV_TO_EXIT',
            SimpleNavigateState_v2(execute_nav_commands=execute_nav_commands),
            transitions={
                SUCCESS:TASK_SUCCESS, 
                NAVIGATIONAL_FAILURE:TASK_FAILURE},
            remapping={'pose':'exit_pose'});

    return sm;

if __name__ == '__main__':
    rospy.init_node('qualification_state_machine');

    sm = construct_qualification_sm();

    sm.execute();

    rospy.spin();
    







