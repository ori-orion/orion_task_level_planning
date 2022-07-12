#!/usr/bin/env python3
""" Smach state machine to test AnnounceGuestDetailsToOperator state

Author: Mia Mijovic
Owner: Mia Mijovic
"""

import os
import rospy
import smach
import smach_ros
import actionlib

from state_machines.reusable_states import * # pylint: disable=unused-wildcard-import

from orion_actions.srv import SOMAddObservationRequest, SOMAddObservation; 


def create_state_machine(userdata=None):
    """ This function builds the state machine for the test.

    Args:
        userdata: A dictionary of optional input arguments to override default

    Returns:
        sm: The complete state machine
    """

    sm = smach.StateMachine(outcomes=['task_success', 'task_failure'])
    sm.userdata.guest_attributes = {'name':'Mia', 'face_attributes': ['Blond_Hair', 'No_Beard', 'Striaght_Hair', 'Pale_Skin']}
    sm.userdata.guest_som_human_ids = ['1']
    sm.userdata.guest_som_obj_ids = ['1']
    
    # override default userdata with input arguments
    if userdata is not None:
        sm.userdata.update(userdata)

    with sm:   
        smach.StateMachine.add('SAVE_GUEST_TO_SOM',
                                SaveGuestToSOM(),
                                transitions={'success':'ANNOUNCE_GUEST_DETAILS',
                                            'failure':'task_failure'},
                                remapping={'guest_attributes':'guest_attributes'})
                
        smach.StateMachine.add('ANNOUNCE_GUEST_DETAILS',
                                AnnounceGuestDetailsToOperator(),
                                transitions={'success':'task_success'},
                                remapping={'guest_attributes':'guest_attributes'})


    return sm


if __name__ == '__main__':
    rospy.init_node('AnnounceGuest_test')

    object_observation_srv = rospy.ServiceProxy('/som/observations/input', SOMAddObservation);
    adding = SOMAddObservationRequest();
    adding.adding.class_ = 'person';
    adding.adding.observed_at = rospy.Time.now();
    adding.adding.observation_batch_num = 1;
    object_observation_srv(adding);

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