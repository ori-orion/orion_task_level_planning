#!/usr/bin/env python
""" File containing states for dummy help_me_carry.

This module contains states for the dummy version of the
help_me_carry task for RoboCup.

Author: Charlie Street

"""

import rospy
import smach
from smach import Concurrence
import time
import numpy as np
from dummy_behaviours.dummy_behaviours import *

class StateJumperColour(smach.State):
    """ SMACH state for speaking jumper colour."""
    def __init__(self):
        """ Constructor initialises attributes and calls super constructor."""

        outcomes = ['Correct_Colour']
        super(StateJumperColour, self).__init__(outcomes=outcomes)

        # Super constructor seems to make it a set so set it afterwards
        self._outcomes = outcomes


    def execute(self, userdata):
        """ Executes the behaviour within the state."""
        rospy.loginfo('Colour: ' + userdata.in_op_info)

        userdata.out_op_info = userdata.in_op_info
        userdata.out_start_location = userdata.in_start_location

        return self._outcomes[0]



class FindItem(smach.State):
    """ SMACH state for searching for an item. """
    def __init__(self):
        """ Constructor initialises attributes and calls super constructor."""

        outcomes = ['Item_Found', 'Item_Not_Found']
        super(FindItem, self).__init__(outcomes=outcomes)

        self._outcomes = outcomes
    
    def execute(self, userdata):
        """Executes find_item behaviour."""
        item = userdata.requested_item
        messages = ['Found: ' + item, 'Could not find: ' + item]
        probs = [0.9, 0.1]

        userdata.out_op_info = userdata.in_op_info
        userdata.out_start_location = userdata.in_start_location

        return dummy_behaviour(self._outcomes, probs, messages)


class MonitoredNav(smach.State):
    """ SMACH state for monitored navigation. """
    def __init__(self):
        """ Constructor initialises attributes and calls super constructor."""

        outcomes = ['Reached_Destination', 'Navigation_Failure']
        super(MonitoredNav, self).__init__(outcomes=outcomes)

        self._outcomes = outcomes
    
    def execute(self, userdata):
        """Executes monitored navigation."""
        destination = userdata.in_start_location
        messages = ['Arrived back at: ' + destination, 'Navigation Failure']
        probs = [0.99, 0.01]

        userdata.out_op_info = userdata.in_op_info
        userdata.out_start_location = userdata.in_start_location

        return dummy_behaviour(self._outcomes, probs, messages)

class StartBackgroundSystems(smach.State):
    """ SMACH state for travelling back to a known location. """
    def __init__(self):
        """ Constructor initialises attributes and calls super constructor."""

        outcomes = ['Started_Background_Systems', 'Failed_Start_Up']
        super(StartBackgroundSystems, self).__init__(outcomes=outcomes)

        self._outcomes = outcomes

    def execute(self, userdata):
        """Starts up base robot behaviours."""
        messages = ['Started people tracking, object/hot-word detection',
                    'Failed to start up base systems.']
        probs = [0.99, 0.01]

        userdata.out_start_location = 'WayPoint1'
        
        return dummy_behaviour(self._outcomes, probs, messages)

class WaitForOperator(smach.State):
    """SMACH state for detecting the operator in the first instance."""
    def __init__(self):
        """ Constructor initialises attributes and calls super constructor."""

        outcomes = ['Operator_Found', 'Time_Out']
        super(WaitForOperator, self).__init__(outcomes=outcomes)

        self._outcomes = outcomes
    
    def execute(self, userdata):
        """ Looks for operator."""
        messages = ['Operator Found', 'Time Out looking for operator']
        probs = [0.9, 0.1]

        outcome = dummy_behaviour(self._outcomes, probs, messages)
        if outcome == 'Operator_Found':
            userdata.out_op_info = 'Red'
        
        return outcome

# <---- Code for concurrent Follow Behaviour ---->

class DetectFollowSignal(smach.State):
    """SMACH state for detecting the end of follow signal from the operator."""
    def __init__(self):
        """ Constructor initialises attributes and calls super constructor."""

        outcomes = ['Signal_Detected', 'Time_Out']
        super(DetectFollowSignal, self).__init__(outcomes=outcomes)

        self._outcomes = outcomes
    
    def execute(self, userdata):
        """Waits for signal from operator."""
        messages = ['Signal from operator wearing: ' + userdata.in_op_info,
                    'Time Out waiting for signal']
        probs = [0.99, 0.01]
        time.sleep(0.1)

        userdata.out_op_info = userdata.in_op_info
        userdata.out_start_location = userdata.in_start_location

        return dummy_behaviour(self._outcomes, probs, messages)

class FollowWithCamera(smach.State):
    """State for keeping operator in view of camera."""

    def __init__(self):
        """ Constructor initialises attributes and calls super constructor."""
        outcomes = ['Lost_Operator']
        super(FollowWithCamera, self).__init__(outcomes=outcomes)

        self._outcomes = outcomes

    def execute(self, userdata):
        """Follows until lost."""
        rospy.loginfo('Following Operator wearing ' + userdata.op_info)
        while np.random.rand() < 0.99:
            time.sleep(0.01)
        
        rospy.loginfo('Lost Operator')

        userdata.out_op_info = userdata.in_op_info
        userdata.out_start_location = userdata.in_start_location

        return self._outcomes[0]

def follow_child_cb(outcome_map):

    if outcome_map['Sig_Detect'] == 'Signal_Detected':
        return True
    
    if outcome_map['Follow_Camera'] == 'Lost_Operator':
        return True
    
    if outcome_map['Follow_Nav'] == 'Navigation_Failure':
        return True
    
    return False

def make_follow_concurrent_state():
    """ Function creates concurrent container for the follow behaviour."""
    con = Concurrence(outcomes=['Follow_Success', 
                                'Follow_Nav_Failure',
                                'Follow_Cam_Failure'],
                      default_outcome='Follow_Failure',
                      input_keys=['in_op_info', 'in_start_location'],
                      output_keys=['out_op_info', 'out_start_location'],
                      child_termination_cb = follow_child_cb,
                      outcome_map={'Follow_Success': 
                                    {'Sig_Detect': 'Signal_Detected'},
                                   'Follow_Nav_Failure': 
                                    {'Follow_Nav': 'Navigation_Failure'},
                                   'Follow_Cam_Failure': 
                                    {'Follow_Camera': 'Lost_Operator'}})
    with con:
        Concurrence.add('Sig_Detect', DetectFollowSignal(),
                        remapping={'in_op_info':'in_op_info',
                                   'in_start_location':'in_start_location',
                                   'out_op_info':'out_op_info',
                                   'out_start_location':'out_start_location'})
        Concurrence.add('Follow_Nav', MonitoredNav(),
                        remapping={'in_op_info':'in_op_info',
                                   'in_start_location':'in_start_location',
                                   'out_op_info':'out_op_info',
                                   'out_start_location':'out_start_location'})
        Concurrence.add('Follow_Camera', FollowWithCamera(),
                        remapping={'in_op_info':'in_op_info',
                                   'in_start_location':'in_start_location',
                                   'out_op_info':'out_op_info',
                                   'out_start_location':'out_start_location'})
    
    return con

def make_and_start_state_machine():
    """ Function for starting node/state machine."""
    rospy.init_node('dummy_help_me_carry')

    # Create the state machine
    sm = smach.StateMachine(outcomes=['TASK_SUCCESS', 'TASK_FAILURE'])

    # Add our states
    with sm:

        # Add start-up state
        smach.StateMachine.add('Start_Up',
                                StartBackgroundSystems(),
                                transitions={'Started_Background_Systems':
                                             'Op_Detect',
                                             'Failed_Start_Up':
                                             'TASK_FAILURE'})
        # Add operator waiting state
        smach.StateMachine.add('Op_Detect',
                               transitions={}, # TODO: Fill in!
                               remapping={'in_start_location':
                                          'out_start_location'})
        
        # TODO: Add rest of states!

        smach.StateMachine.add('State_Jumper_Colour',
                                StateJumperColour(),
                                transitions={'Correct_Colour':'TASK_SUCCESS',
                                             'Incorrect_Colour':'TASK_FAILURE'})
    # Execute the State Machine
    _ = sm.execute()

if __name__ == '__main__':
    make_and_start_state_machine()