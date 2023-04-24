import rospy



from ast import operator
import os
from time import sleep
import rospy
import smach_ros
import actionlib

from state_machines.SubStateMachines.include_all import *;
from state_machines.Reusable_States.training import *;

# from state_machines.SubStateMachines.create_sub_state_machines import *;

# from state_machines.reusable_states import * # pylint: disable=unused-wildcard-import
# from set_up_clients import create_stage_1_clients
from orion_actions.msg import SOMObservation, Relation, SearchPersonNotMetGoal
from geometry_msgs.msg import Pose

# import time  # TODO - replace calls to rospy.time library



def create_state_machine():
    """ This function creates and returns the state machine. """

    # Create the state machine
    sm = smach.StateMachine(outcomes=[TASK_SUCCESS, TASK_FAILURE]);

    # Wait for the prameters to be loaded.
    while (not rospy.has_param('params_loaded')):
        rospy.sleep(0.5);

    # Create state machine userdata dictionary elements

    # Task params
    sm.userdata.obj_class = 'bottle'
    sm.userdata.failure_prahse = 'Failed finding placement'

    dims = (0, 1, 0.1, 0.1)
    height = 0.2
    radius = 0.02


    with sm:

        smach.StateMachine.add(
            'CREATE_OBJECT_QUERY',
            CreateSOMQuery(
                CreateSOMQuery.OBJECT_QUERY, 
                save_time=False),
            transitions={
                SUCCESS: 'FIND_OBJECT'},
            remapping={
                'class_': 'obj_class'
            })
        
        smach.StateMachine.add(
            'FIND_OBJECT',
            PerformSOMQuery(),
            transitions={
                SUCCESS:'MANIPULATION',
                FAILURE:TASK_FAILURE},
            remapping={})

        smach.StateMachine.add(
            'FIND_PLACEMENT',
            PlaceNextTo(dims, height, radius),
            transitions={
                SUCCESS: TASK_SUCCESS,
                FAILURE: 'PRINT_FAILURE'
            },
            remapping={}
        )


        smach.StateMachine.add(
            'PRINT_SUCCESS',
            PrintToConsole(),
            transitions = {
                SUCCESS: TASK_FAILURE
            },
            remapping = {
                'var' : 'failure_phrase'
            }
        )
        
        
        sm = setupErrorStates(sm)
        
        # TODO - Reset e
    return sm
    


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
