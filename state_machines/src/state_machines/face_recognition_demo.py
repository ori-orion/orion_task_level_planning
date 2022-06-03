#!/usr/bin/env python3
""" Smach state machine to demo face recognition

Author: Ricardo Cannizzaro
Owner: Ricardo Cannizzaro
"""

import os
import rospy
import smach
import smach_ros
import actionlib

from reusable_states import * # pylint: disable=unused-wildcard-import


def create_state_machine(userdata=None):
    """ This function builds the state machine for the demo.

    Args:
        userdata: A dictionary of optional input arguments to override default

    Returns:
        sm: The complete state machine
    """

    # Create the state machine
    sm = smach.StateMachine(outcomes=['task_success', 'task_failure'])
    sm.userdata.operator_name = "Ricardo"
    sm.userdata.guest_name = "Guest"

    # override default userdata with input arguments
    if userdata is not None:
        sm.userdata.update(userdata)

    with sm:
        
        smach.StateMachine.add('REGISTER_FACE',
                                RegisterFace(),
                                transitions={'success':'RECOGNISE_OPEATOR_FACE'},
                                remapping={'face_id':'operator_name'})

        smach.StateMachine.add('RECOGNISE_OPEATOR_FACE',
                                RecogniseFace(),
                                transitions={'success':'DETECT_OPERATOR_FACE_ATTRIBUTES_BY_TOPIC'},
                                remapping={'face_id':'operator_detected_face_id',
                                            'face_match_score':'operator_face_match_score'})
        
        smach.StateMachine.add('DETECT_OPERATOR_FACE_ATTRIBUTES_BY_TOPIC',
                                DetectFaceAttributes(),
                                transitions={'success':'DETECT_OPERATOR_FACE_ATTRIBUTES_BY_DB'},
                                remapping={'face_attributes':'operator_face_attributes',
                                            'num_attributes':'operator_num_attributes'  })

        smach.StateMachine.add('DETECT_OPERATOR_FACE_ATTRIBUTES_BY_DB',
                                DetectFaceAttributes(),
                                transitions={'success':'CLEAR_FACE_DB'},
                                remapping={'face_attributes':'operator_face_attributes',
                                            'num_attributes':'operator_num_attributes'  })
        
        smach.StateMachine.add('CLEAR_FACE_DB',
                                ClearFaceDB(),
                                transitions={'success':'task_success'})

    return sm


if __name__ == '__main__':
    rospy.init_node('face_recognition_demo')

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