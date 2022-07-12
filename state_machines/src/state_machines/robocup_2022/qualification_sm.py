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
    sm.userdata.exit_node = "Node2";

    sm.userdata.hotword_timeout = 3;

    sm.userdata.number_of_failures = 0;
    sm.userdata.failure_threshold = 3;

    end_loc = Pose();
    end_loc.orientation.w = 1;
    sm.userdata.pose = end_loc;
    
    """
    Pose 1
    header: 
      seq: 12696
      stamp: 
        secs: 1657606062
        nsecs: 462087244
      frame_id: "map"
    pose: 
      position: 
        x: 2.810693510548858
        y: 0.07427706091467959
        z: 0.0
      orientation: 
        x: 0.0
        y: 0.0
        z: 0.0264994773989117
        w: 0.9996488271876202
    """

    look_at_pose = Pose();
    look_at_pose.position.x = 2.810693510548858;
    look_at_pose.position.y = 0.07427706091467959;
    look_at_pose.position.z = 0.0;
    look_at_pose.orientation.x = 0.0;
    look_at_pose.orientation.y = 0.0;
    look_at_pose.orientation.z = 0.0264994773989117;
    look_at_pose.orientation.w = 0.9996488271876202;

    """
    Pose 2:
    header: 
    seq: 17671
    stamp: 
        secs: 1657606112
        nsecs: 212031691
    frame_id: "map"
    pose: 
    position: 
        x: 2.7241360652618876
        y: 2.4989383095654563
        z: 0.0
    orientation: 
        x: 0.0
        y: 0.0
        z: 0.7055165951344321
        w: 0.7086933991437467
    """

    intermediate_pose = Pose();
    intermediate_pose.position.x = 2.7241360652618876
    intermediate_pose.position.y = 2.4989383095654563
    intermediate_pose.position.z = 0.0
    intermediate_pose.orientation.x = 0.0;
    intermediate_pose.orientation.y = 0.0;
    intermediate_pose.orientation.z = 0.7055165951344321;
    intermediate_pose.orientation.w = 0.7086933991437467;

    """
    Pose 3:
    header: 
    seq: 6730
    stamp: 
        secs: 1657606527
        nsecs:  54727887
    frame_id: "map"
    pose: 
    position: 
        x: -0.00303225730938404
        y: -0.002021541005732918
        z: 0.0
    orientation: 
        x: 0.0
        y: 0.0
        z: 0.0041878305375389665
        w: 0.9999912309992467
    ---
    """
    final_pose = Pose();
    final_pose.position.x = -0.00303225730938404;
    final_pose.position.y = -0.002021541005732918;
    final_pose.position.z =  0.0;
    final_pose.orientation.x = 0.0
    final_pose.orientation.y = 0.0
    final_pose.orientation.z = 0.0041878305375389665
    final_pose.orientation.w = 0.9999912309992467
    sm.userdata.final_pose = final_pose;

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

        # smach.StateMachine.add(
        #     'NavToOperator',
        #     SimpleNavigateState(),
        #     transitions={
        #         'success':'WAIT_FOR_HOTWORD',
        #         'failure':'NavToOperator',
        #         }
        # )

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

        smach.StateMachine.add(
            'NAV_TO_START',
            SimpleNavigateState(),
            transitions={
                'success':'task_success', 
                'failure':'NAV_TO_START', 
                'repeat_failure':'task_failure'},
            remapping={'pose':'start_pose'});

    return sm;

if __name__ == '__main__':
    rospy.init_node('qualification_state_machine');

    sm = construct_qualification_sm();

    sm.execute();

    rospy.spin();
    







