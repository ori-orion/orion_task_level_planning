#!/usr/bin/env python3
""" Code for the ORI open day welcome demo

This file contains code for the find my mate task, including the
state machine itself.

Author: Ricardo Cannizzaro
Owner: Ricardo Cannizzaro

"""

from ast import operator
import os
from time import sleep
import rospy
import smach_ros
import actionlib
import typing

from state_machines.Reusable_States.include_all import *;
from state_machines.Reusable_States.create_sub_state_machines import *;

# from state_machines.reusable_states import * # pylint: disable=unused-wildcard-import
# from set_up_clients import create_stage_1_clients
from orion_actions.msg import SOMObservation, Relation, SearchPersonNotMetGoal
from geometry_msgs.msg import Pose

# import time  # TODO - replace calls to rospy.time library

class CreatePhraseFamiliar(smach.State):
    """ Smach state to create the phrase to announce the start of the search for people

    This class always returns success.

    input_keys:
        operator_name: the name of the operator
    output_keys:
        phrase: the returned phrase
    """
    def __init__(self):
        smach.State.__init__(self,
                                outcomes=['success'],
                                input_keys=['operator_name'],
                                output_keys=['phrase'])

    def execute(self, userdata):
        userdata.phrase = "Ok, " + userdata.operator_name + ", let me tell you a bit about myself!"

        # Can only succeed
        return 'success'

## TODO
def create_dynamic_speak_state(label, text_template, transitions=None, remapping=None, *args, **kwargs):
    dynamic_phrase = ""
    # dynamic_phrase = text_template.format() # TODO

    ## TODO - work out how to parameterise the construction of the phrase. Maybe with format arguments
    sm = smach.StateMachine(outcomes=transitions.values());

    with sm:
        smach.StateMachine.add(label,
                                SpeakState(phrase=dynamic_phrase),
                                transitions=transitions,
                                remapping=remapping) 
    return sm

def create_state_machine():
    """ This function creates and returns the state machine for the task. """

    # Create the state machine
    sm = smach.StateMachine(outcomes=[TASK_SUCCESS, TASK_FAILURE]);

    # Wait for the prameters to be loaded.
    # while (not rospy.has_param('params_loaded')):
    #     rospy.sleep(0.5);

    # Create state machine userdata dictionary elements
    
    # Task params
    sm.userdata.expected_num_guests = 4 # TODO - change to 3
    sm.userdata.max_search_duration = 200 # seconds

    sm.userdata.person_names = NAMES
    # Load up huge database of additional names (if necessary)
    import rospkg
    rospack = rospkg.RosPack()
    name_file = \
        os.path.join(rospack.get_path('state_machines'),'grammars/names.txt')
    with open(name_file, 'r') as in_file:
        sm.userdata.person_names += in_file.read().splitlines()

    sm.userdata.hotword_timeout = 15 # seconds

    sm.userdata.speak_and_listen_params_empty = []
    sm.userdata.speak_and_listen_timeout = 5
    sm.userdata.speak_and_listen_failures = 0
    sm.userdata.speak_and_listen_failure_threshold = 2

    sm.userdata.simple_navigation_failures = 0
    sm.userdata.simple_navigation_failure_threshold = 3
    sm.userdata.simple_navigation_distance_to_humans = 0.5 # metres

    sm.userdata.topological_navigation_failure_threshold = 3

    sm.userdata.failure_threshold = 3;

    sm.userdata.ask_operator_name_phrase = "What is your name?"

    # set the robot's pose to speak to the operator - TODO
    # Defaults to...
    operator_pose = Pose();
    # operator_pose = utils.dict_to_obj(rospy.get_param('/operator_pose'), operator_pose);
    sm.userdata.operator_pose = operator_pose;

    # speaking to guests
    # sm.userdata.introduction_to_guest_phrase = "Hi, I'm Bam Bam, welcome to the party! I'm going to learn some information about you so I can tell the host about you!"
    # sm.userdata.no_one_there_phrase = "Hmmm. I don't think anyone is there."


    mid_room_pose = Pose();
    mid_room_pose.position.x = 2.1717187101331037;
    mid_room_pose.position.y = -0.2144994235965718;
    # mid_room_pose.position.x = -18;
    # mid_room_pose.position.y = -9;
    mid_room_pose.position.z = 0.0;
    mid_room_pose.orientation.x = 0.0;
    mid_room_pose.orientation.y = 0.0;
    mid_room_pose.orientation.z = -0.6771155204369553;
    mid_room_pose.orientation.w = 0.7358767369494643;
    sm.userdata.mid_room_pose = mid_room_pose;

    # set arena exit pose - TODO
    sm.userdata.exit_pose = Pose()
    sm.userdata.exit_pose.position.x = 0.0
    sm.userdata.exit_pose.position.y = 0.0
    sm.userdata.exit_pose.position.z = 0.0
    sm.userdata.exit_pose.orientation.x = 0.0
    sm.userdata.exit_pose.orientation.y = 0.0
    sm.userdata.exit_pose.orientation.z = 0.0
    sm.userdata.exit_pose.orientation.w = 1.0

    sm.userdata.operator_name = "Isaac Asimov"   # a default name for testing, this will be overridden by ASK_OPERATOR_NAME state
    sm.userdata.operator_som_id = "1234"       # SAVE_OPERATOR_INFO_TO_SOM state will set this

    # guest tracking
    sm.userdata.guest_som_human_ids = []
    sm.userdata.guest_som_obj_ids = []

    # top nav
    sm.userdata.node_list = ['Node1', 'Node2', 'Node3']
    sm.userdata.nodes_not_searched = list(sm.userdata.node_list)  # used in the guest search sub state machine

    # Where is the operator starting out?
    sm.userdata.operator_room_node_id = "Living1";
    # Which room are the guests in?
    sm.userdata.guest_room_node_id = "Living1";

    sm.userdata.exit_room_node_id = "Exit";

    sm.userdata.number_of_failures = 0;
    
    # In some cases, we don't want to navigate to the topological node 
    # if the last one we went to was that node. (I.e., if we're then#
    # simply going to search for something in that room.)
    # Note that if this is the same as the first node we nav to, we won't nav to that node at all. 
    # (Nice small hack)
    sm.userdata.prev_node_nav_to = "Living1";

    sm.userdata.nearest_to = None;


    with sm:

        # look-up at human
        smach.StateMachine.add(
            'LOOK_AT_OPERATOR',
            LookUpState(height=1.7),
            transitions={SUCCESS:'INTRODUCTION_TO_OPERATOR'});


        # introduce to operator
        smach.StateMachine.add('INTRODUCTION_TO_OPERATOR',
                                SpeakState(phrase="Hi, I'm Bam Bam! Welcome to the ORI Open Day event!"),
                                transitions={SUCCESS:'ASK_OPERATOR_NAME'},
                                remapping={})
            

        # ask for operator's name - New ask guest name action server     
        smach.StateMachine.add('ASK_OPERATOR_NAME',
                                AskPersonNameState(),
                                transitions={SUCCESS: 'GIVE_INFORMATION',
                                            FAILURE:'ANNOUNCE_MISSED_NAME',
                                            'repeat_failure':'ANNOUNCE_MISSED_NAME'},
                                remapping={'question':'ask_operator_name_phrase',
                                            'recognised_name': 'operator_name',
                                            'timeout':'speak_and_listen_timeout',
                                            'number_of_failures': 'speak_and_listen_failures',
                                            'failure_threshold': 'speak_and_listen_failure_threshold'})

        # announce that we missed the name, and that we will try again
        smach.StateMachine.add('ANNOUNCE_MISSED_NAME',
                                SpeakState(phrase="I'm sorry but I didn't understand. Let's try that again."),
                                transitions={SUCCESS:'ASK_OPERATOR_NAME'},
                                remapping={})

        # create the familiar phrase
        # # Note that the output here is the input into 'ANNOUNCE_SEARCH_START'.
        # smach.StateMachine.add('CREATE_PHRASE_FAMILIAR',
        #                         CreatePhraseFamiliar(),
        #                         transitions={SUCCESS:'SPEAK_PHRASE_FAMILIAR'},
        #                         remapping={'operator_name':'operator_name',
        #                                     'phrase':'familiar_phrase'})
        
        # # announce search start
        # smach.StateMachine.add('SPEAK_PHRASE_FAMILIAR',
        #                         SpeakState(),
        #                         transitions={SUCCESS:'GIVE_INFORMATION'},
        #                         remapping={'phrase':'familiar_phrase'})

         # give information
        information_phrase = "Let me tell you a bit about myself! I am a Toyota Human Support Robot, designed to help people with everyday tasks around the house. \
                                I can do lots of things: recognise human speech, detect objects with my camera, fetch things with my gripper, \
                                    take out the trash, and even act as a host at your next party!\
                                        I am used for research by PhD students, as well as for a home care project. \
                                            I am also used by Team Orion, to compete at the annual RoboCup at Home competition."
        smach.StateMachine.add('GIVE_INFORMATION',
                                SpeakState(phrase=information_phrase),
                                transitions={SUCCESS:'FAREWELL_OPERATOR'},
                                remapping={})
      

        # Farewell operator
        smach.StateMachine.add('FAREWELL_OPERATOR',
                                SpeakState(phrase="It was nice to meet you! I hope to see you again soon. Goodbye!"),
                                transitions={SUCCESS:"LOOK_AT_NEUTRAL"},
                                remapping={}) 

        # look to neutral at human
        smach.StateMachine.add(
            'LOOK_AT_NEUTRAL',
            LookUpState(height=1.2),
            transitions={SUCCESS:TASK_SUCCESS});

        sm = setupErrorStates(sm);
        
    return sm;

rospy.init_node('find_my_mates_state_machine');

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
