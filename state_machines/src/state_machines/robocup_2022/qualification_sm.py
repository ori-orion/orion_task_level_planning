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

from state_machines.reusable_states import * # pylint: disable=unused-wildcard-import
# from set_up_clients import create_stage_1_clients
from orion_actions.msg import SOMObservation, Relation, SearchPersonNotMetGoal
from geometry_msgs.msg import Pose

import time  # TODO - replace calls to rospy.time library

def construct_qualification_sm():
    sm = smach.StateMachine(outcomes=['task_success', 'task_failure'])

    sm.userdata.navigate_to_node = "Node1";
    sm.userdata.exit_node = "Node3";

    sm.userdata.hotword_timeout = 3;

    sm.userdata.number_of_failures = 0;
    sm.userdata.failure_threshold = 3;

    end_loc = Pose();
    end_loc.orientation.w = 1;
    sm.userdata.pose = end_loc;

    with sm:


        smach.StateMachine.add(
            'WAIT_FOR_START_SIGNAL',
            CheckDoorIsOpenState(),
            transitions={'open':'GET_INITIAL_LOCATION', 
                         'closed':'WAIT_FOR_START_SIGNAL'})

        smach.StateMachine.add(
            'GET_INITIAL_LOCATION',
            GetRobotLocationState(),
            transitions={'stored':'NAV_TO_NEAREST_NODE'},
            remapping={'robot_location':'start_pose'});

        smach.StateMachine.add(
            'NAV_TO_NEAREST_NODE',
            TopologicalNavigateState(),
            transitions={
                'success':'WAIT_FOR_HOTWORD',
                'failure':'NAV_TO_NEAREST_NODE',
                'repeat_failure':'task_failure'},
            remapping={'node_id':'navigate_to_node'});

        smach.StateMachine.add(
            'WAIT_FOR_HOTWORD',
            WaitForHotwordState(),
            transitions={
                'success':'NAV_TO_EXIT', 
                'failure':'WAIT_FOR_HOTWORD'},
            remapping={'timeout':'hotword_timeout'});

        smach.StateMachine.add(
            'NAV_TO_EXIT',
            TopologicalNavigateState(),
            transitions={
                'success':'task_success', 
                'failure':'NAV_TO_EXIT', 
                'repeat_failure':'task_failure'},
            remapping={'node_id':'exit_node'});

        # smach.StateMachine.add(
        #     'NAV_TO_START',
        #     SimpleNavigateState(),
        #     transitions={
        #         'success':'task_success', 
        #         'failure':'NAV_TO_START', 
        #         'repeat_failure':'task_failure'},
        #     remapping={'pose':'start_pose'});

    return sm;

if __name__ == '__main__':
    rospy.init_node('qualification_state_machine');

    sm = construct_qualification_sm();

    sm.execute();

    rospy.spin();
    







