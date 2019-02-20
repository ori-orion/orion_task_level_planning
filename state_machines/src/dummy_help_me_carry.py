#!/usr/bin/env python
""" File containing states for dummy help_me_carry.

This module contains states for the dummy version of the
help_me_carry task for RoboCup.

Author: Charlie Street

"""

import rospy
import smach
from dummy_behaviours.dummy_behaviours import *

class StateJumperColour(smach.State):
    """ SMACH state for speaking jumper colour."""
    # TODO: Make this have operator data passed in
    def __init__(self):
        """ Constructor initialises attributes and calls super constructor."""

        outcomes = ['Correct_Colour', 'Incorrect_Colour']
        super(StateJumperColour, self).__init__(outcomes=outcomes)

        # Super constructor seems to make it a set so set it afterwards
        self._outcomes = outcomes


    def execute(self, userdata):
        """ Executes the behaviour within the state."""
        messages = ['Correct Colour Stated!',
                    'Incorrect Colour Stated']
        probs = [0.7,0.3]
        return dummy_behaviour(self._outcomes, probs, messages)



def make_and_start_state_machine():
    """ Function for starting node/state machine."""
    rospy.init_node('dummy_help_me_carry')

    # Create the state machine
    sm = smach.StateMachine(outcomes=['TASK_SUCCESS', 'TASK_FAILURE'])

    # Add our states
    with sm:
        smach.StateMachine.add('StateJumperColour',
                                StateJumperColour(),
                                transitions={'Correct_Colour':'TASK_SUCCESS',
                                             'Incorrect_Colour':'TASK_FAILURE'})
    # Execute the State Machine
    _ = sm.execute()

if __name__ == '__main__':
    make_and_start_state_machine()