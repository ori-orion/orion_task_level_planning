#!/usr/bin/env python3
""" Smach state machine to test Manipulation states

Author: Ricardo Cannizzaro
Owner: Ricardo Cannizzaro
"""

import os
import rospy
import smach
import smach_ros
import actionlib

from state_machines.reusable_states import * # pylint: disable=unused-wildcard-import


def create_state_machine(userdata=None):
    """ This function builds the state machine for the test.

    Args:
        userdata: A dictionary of optional input arguments to override default

    Returns:
        sm: The complete state machine
    """

    # Create the state machine
    sm = smach.StateMachine(outcomes=['task_success', 'task_failure'])
    
    # Define state parameters

    sm.userdata.simple_navigation_failures = 0
    sm.userdata.simple_navigation_failure_threshold = 3
    
    # Pickup Pose - change as needed
    sm.userdata.pickup_pose = Pose()
    sm.userdata.pickup_pose.position.x = 0.23832461091673765
    sm.userdata.pickup_pose.position.y = 0.0
    sm.userdata.pickup_pose.position.z = 0.0
    sm.userdata.pickup_pose.orientation.x = 0.0
    sm.userdata.pickup_pose.orientation.y = 0.0
    sm.userdata.pickup_pose.orientation.z = -0.7027350031010926
    sm.userdata.pickup_pose.orientation.w =  0.711451695771756

    # Manipulation parameters
    sm.userdata.pickup_object_name = "bottle"
    sm.userdata.object_ar_markers = AR_MARKERS
    sm.userdata.pickup_failures = 0
    sm.userdata.pickup_failure_threshold = 3

    # Putdown Pose - change as needed
    sm.userdata.putdown_pose = Pose()
    sm.userdata.putdown_pose.position.x = 2.7320108204126865
    sm.userdata.putdown_pose.position.y = 0.0
    sm.userdata.putdown_pose.position.z = 0.0
    sm.userdata.putdown_pose.orientation.x = 0.0
    sm.userdata.putdown_pose.orientation.y = 0.0
    sm.userdata.putdown_pose.orientation.z = 0.0
    sm.userdata.putdown_pose.orientation.w = 1.0

    # override default userdata with input arguments
    if userdata is not None:
        sm.userdata.update(userdata)

    with sm:

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
                               transitions={'success':'NAV_TO_PUTDOWN',
                                            'failure':'PICKUP_OBJECT',
                                            'repeat_failure':'task_failure'},
                                remapping={ 'object_name':'pickup_object_name', 
                                            'number_of_failures':'pickup_failures', 
                                            'failure_threshold':'pickup_failure_threshold', 
                                            'ar_marker_ids':'object_ar_markers'})

        smach.StateMachine.add('NAV_TO_PUTDOWN',
                               SimpleNavigateState(),
                               transitions={'success':'GIVE_ITEM',
                                            'failure':'NAV_TO_PUTDOWN',
                                            'repeat_failure':'task_failure'},
                                remapping={'pose':'putdown_pose',
                                           'number_of_failures': 'simple_navigation_failures',
                                           'failure_threshold': 'simple_navigation_failure_threshold'})

        smach.StateMachine.add('GIVE_ITEM',
                               HandoverObjectToOperatorState(),
                               transitions={'success': 'task_success',
                                            'failure': 'task_failure'})


    return sm


if __name__ == '__main__':
    rospy.init_node('speech_test')

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