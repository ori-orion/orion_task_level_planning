#!/usr/bin/env python3

import smach;
import rospy;

from state_machines.reusable_states import * # pylint: disable=unused-wildcard-import

def create_state_machine():
    sm = smach.StateMachine(outcomes=['task_success', 'task_failure'])

    sm.userdata.speech_1 = "Hi, I'm Bam Bam. What's your name?";
    sm.userdata.speech_2 = "I've brought you the potted plant. Please say ready when you are ready to receive the potted plant.";
    sm.userdata.speech_3 = "Have a nice day.";
    sm.userdata.number_of_failures = 0;
    sm.userdata.failure_threshold = 3;
    sm.userdata.timeout = 5;
    sm.userdata.hotword_timeout = 5;

    nav_out_pose:Pose = Pose();
    nav_out_pose.position.x = -17.1;
    nav_out_pose.position.y = -9.16;
    sm.userdata.nav_out_pose = nav_out_pose;

    with sm:
        smach.StateMachine.add('WAIT_FOR_HOTWORD',
                                WaitForHotwordState(),
                                transitions={'success': 'LOOK_AT_OPERATOR',
                                             'failure': 'WAIT_FOR_HOTWORD'},
                                remapping={'timeout':'hotword_timeout'});

        smach.StateMachine.add(
            'LOOK_AT_OPERATOR',
            LookUpState(height=0.8),
            transitions={'success':'INTRODUCTION_TO_OPERATOR'});

        smach.StateMachine.add('INTRODUCTION_TO_OPERATOR',
                                AskPersonNameState(),
                                transitions={'success': 'BROUGHT_POTTED_PLANT',
                                            'failure':'task_failure',
                                            'repeat_failure':'task_failure'},
                                remapping={'question':'speech_1'});

        smach.StateMachine.add('BROUGHT_POTTED_PLANT',
                                SpeakState(),
                                transitions={'success':'WAIT_FOR_HOTWORD_2'},
                                remapping={'phrase':'speech_2'});

        smach.StateMachine.add('WAIT_FOR_HOTWORD_2',
                        WaitForHotwordState(),
                        transitions={'success': 'HAND_PLANT_TO_OPERATOR',
                                     'failure': 'WAIT_FOR_HOTWORD_2'},
                        remapping={'timeout':'hotword_timeout'});

        smach.StateMachine.add('HAND_PLANT_TO_OPERATOR',
            HandoverObjectToOperatorState(),
            transitions={'success':'FINISH', 'failure':'task_failure'});
        

        smach.StateMachine.add('FINISH',
                        SpeakState(),
                        transitions={'success':'NAV_OUT'},
                        remapping={'phrase':'speech_3'});

        smach.StateMachine.add(
            "NAV_OUT",
            SimpleNavigateState(),
            transitions={
                'success':'task_success',
                'failure':'NAV_OUT',
                'repeat_failure':'task_success'},
            remapping={'pose':'nav_out_pose'});
    return sm;

rospy.init_node('find_my_mates_state_machine');

# Create the state machine
sm = create_state_machine()

# Create and start the introspection server
# sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
# sis.start()

# Execute the state machine
sm.execute()

# Run until ctl+c command is received
rospy.spin()
# sis.stop()