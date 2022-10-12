#!/usr/bin/env python3

from state_machines.Reusable_States.training import *;
import smach_ros

def create_training_1():
    sm = smach.StateMachine(outcomes=[TASK_SUCCESS, TASK_FAILURE]);

    with sm:
        smach.StateMachine.add('WaitForStart',
            ReadInFromConsole(),
            transitions={SUCCESS:'HelloWorld',
                         FAILURE:'FailureState'})


        smach.StateMachine.add('HelloWorld',
            PrintToConsole(),
            transitions={SUCCESS:TASK_SUCCESS,
                         FAILURE:TASK_FAILURE})

        smach.StateMachine.add('FailureState',
            PrintToConsole("Failure..."),
            transitions={SUCCESS:TASK_SUCCESS,
                         FAILURE:TASK_FAILURE})

    return sm;

rospy.init_node('find_my_mates_state_machine');

# Create the state machine
sm = create_training_1()

# Create and start the introspection server
sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
sis.start()

# Execute the state machine
sm.execute()

# Run until ctl+c command is received
rospy.spin()
