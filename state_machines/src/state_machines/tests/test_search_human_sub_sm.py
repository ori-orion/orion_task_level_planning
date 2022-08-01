#!/usr/bin/env python3

# from state_machines.reusable_states import * # pylint: disable=unused-wildcard-import
from state_machines.Reusable_States.create_sub_state_machines import *;

if __name__ == '__main__':
    rospy.init_node('testing_search_humans');

    sub_sm = create_search_for_human();

    sub_sm.userdata.room_node_uid = 'Node3';
    sub_sm.userdata.number_of_failures = 3;
    sub_sm.userdata.failure_threshold = 3;

    # sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    # sis.start()

    # Execute the state machine
    sub_sm.execute();

    print(sub_sm._current_outcome);

    rospy.spin();

