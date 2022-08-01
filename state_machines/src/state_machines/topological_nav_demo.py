#!/usr/bin/env python3
""" Smach state machine to demo ori topological navigation states

Author: Ricardo Cannizzaro
Owner: Ricardo Cannizzaro
"""

import os
import rospy
import smach
import smach_ros
import actionlib

# from reusable_states import * # pylint: disable=unused-wildcard-import
from state_machines.Reusable_States.include_all import *;


def create_state_machine(userdata=None):
    """ This function builds the state machine for the demo.

    Args:
        userdata: A dictionary of optional input arguments to override default

    Returns:
        sm: The complete state machine
    """

    # Create the state machine
    sm = smach.StateMachine(outcomes=['task_success', 'task_failure'])
    sm.userdata.nav_failures = 0
    sm.userdata.nav_failure_threshold = 3
    
    sm.userdata.node_id_1 = "Node1"
    sm.userdata.node_id_2 = "Node2"

    # override default userdata with input arguments
    if userdata is not None:
        sm.userdata.update(userdata)

    with sm:
        
        smach.StateMachine.add('TOP_NAV_1',
                                TopologicalNavigateState(),
                                transitions={'success':'TOP_NAV_2',
                                                'failure':'TOP_NAV_1',
                                                'repeat_failure':'task_failure'},
                                remapping={'node_id':'node_id_1',
                                            'number_of_failures':'nav_failures',
                                            'failure_threshold':'nav_failure_threshold'})

        smach.StateMachine.add('TOP_NAV_2',
                                TopologicalNavigateState(),
                                transitions={'success':'task_success',
                                                'failure':'TOP_NAV_2',
                                                'repeat_failure':'task_failure'},
                                remapping={'node_id':'node_id_2',
                                            'number_of_failures':'nav_failures',
                                            'failure_threshold':'nav_failure_threshold'})

    return sm


if __name__ == '__main__':
    rospy.init_node('topological_navigation_demo')

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