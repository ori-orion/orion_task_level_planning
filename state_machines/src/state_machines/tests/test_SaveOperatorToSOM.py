#!/usr/bin/env python3
""" Smach state machine to test SaveOperatorToSOM state

Author: Ricardo Cannizzaro
Owner: Ricardo Cannizzaro
"""

import os
import rospy
import smach
import smach_ros
import actionlib

# from state_machines.reusable_states import * # pylint: disable=unused-wildcard-import
from state_machines.Reusable_States.include_all import *;

def create_state_machine(userdata=None):
    """ This function builds the state machine for the test.

    Args:
        userdata: A dictionary of optional input arguments to override default

    Returns:
        sm: The complete state machine
    """

    # Create the state machine
    sm = smach.StateMachine(outcomes=['task_success', 'task_failure'])
    sm.userdata.operator_name = "Ricardo"


    # override default userdata with input arguments
    if userdata is not None:
        sm.userdata.update(userdata)

    with sm:
        
        smach.StateMachine.add('SAVE_OPERATOR_TO_SOM',
                                SaveOperatorToSOM(),
                                transitions={'success':'task_success',
                                            'failure':'task_failure'},
                                remapping={'operator_name':'operator_name'})


    return sm


if __name__ == '__main__':
    rospy.init_node('SaveOperatorToSOM_test')

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