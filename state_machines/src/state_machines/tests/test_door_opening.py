#!/usr/bin/env python3

from state_machines.Reusable_States.include_all import *;

# from state_machines.reusable_states import * # pylint: disable=unused-wildcard-import

if __name__ == '__main__':
    rospy.init_node('testing_door_opening');

    sm = smach.StateMachine(outcomes=['task_success', 'task_failure']);

    with sm:
        smach.StateMachine.add('WAIT_FOR_START_SIGNAL',
                        CheckDoorIsOpenState(),
                        transitions={   'open':'task_success', 
                                        'closed':'WAIT_FOR_START_SIGNAL'});

    sm.execute();

    print("Door is open!");