#!/usr/bin/env python3
""" Smach state machine to test SaveGuestToSOM state

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
    sm.userdata.guest_attributes = {}
    
    sm.userdata.name = "Ricardo"
    sm.userdata.gender = "Male"
    sm.userdata.pronouns = "He/Him"
    sm.userdata.face_id = "Ricardo"
    sm.userdata.face_attributes = ["Has_Face", "Wearing_Glasses"]
    # sm.userdata.face_attributes = []

    sm.userdata.guest_attributes_2 = {}
    sm.userdata.guest_attributes_2["name"] = "Matthew"
    sm.userdata.guest_som_human_ids = []
    sm.userdata.guest_som_obj_ids = []

    # override default userdata with input arguments
    if userdata is not None:
        sm.userdata.update(userdata)

    with sm:   
        smach.StateMachine.add('CREATE_GUEST_ATTRIBUTES_DICT',
                                CreateGuestAttributesDict(),
                                transitions={'success':'SAVE_GUEST_TO_SOM'},
                                remapping={'name':'name','gender':'gender','pronouns':'pronouns','face_id':'face_id','face_attributes':'face_attributes',
                                        'guest_attributes':'guest_attributes'})

        smach.StateMachine.add('SAVE_GUEST_TO_SOM',
                                SaveGuestToSOM(),
                                transitions={'success':'task_success',
                                            'failure':'task_failure'},
                                remapping={'guest_attributes':'guest_attributes'})
                
        # smach.StateMachine.add('SAVE_GUEST_TO_SOM_2',
        #                         SaveGuestToSOM(),
        #                         transitions={'success':'task_success',
        #                                     'failure':'task_failure'},
        #                         remapping={'guest_attributes':'guest_attributes_2'})


    return sm


if __name__ == '__main__':
    rospy.init_node('SaveGuestToSOM_test')

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