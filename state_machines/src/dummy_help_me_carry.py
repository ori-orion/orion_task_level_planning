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

class GlobalStoreState(smach.State):
    """ Overwriting smach state to have global data store."""
    def __init__(self, global_store, outcomes):
        """ Constructor initialises new fields and calls super constructor.

        Args:
            global_store: A dictionary of strings to data items
            outcomes: The state outcomes

        """
        self.global_store = global_store
        super(GlobalStoreState, self).__init__(outcomes=outcomes)
        # Need to set afterwards
        self._outcomes = outcomes

class StateJumperColour(GlobalStoreState):
    """ SMACH state for speaking jumper colour."""
    def __init__(self, global_store):
        """ Constructor initialises attributes and calls super constructor."""

        outcomes = ['Correct_Colour']
        super(StateJumperColour, self).__init__(global_store=global_store,
                                                outcomes=outcomes)

    def execute(self, userdata):
        """ Executes the behaviour within the state."""
        rospy.loginfo('Colour: ' + str(self.global_store['op_info']))

        return self._outcomes[0]



class FindItem(GlobalStoreState):
    """ SMACH state for searching for an item. """
    def __init__(self, global_store):
        """ Constructor initialises attributes and calls super constructor."""

        outcomes = ['Item_Found', 'Item_Not_Found']
        super(FindItem, self).__init__(global_store=global_store,
                                       outcomes=outcomes)
    
    def execute(self, userdata):
        """Executes find_item behaviour."""
        item = self.global_store['item']
        messages = ['Found: ' + item, 'Could not find: ' + item]
        probs = [0.9, 0.1]


        return dummy_behaviour(self._outcomes, probs, messages)


class MonitoredNav(GlobalStoreState):
    """ SMACH state for monitored navigation. """
    def __init__(self, global_store):
        """ Constructor initialises attributes and calls super constructor."""

        outcomes = ['Reached_Destination', 'Navigation_Failure']
        super(MonitoredNav, self).__init__(global_store=global_store,
                                           outcomes=outcomes)
    
    def execute(self, userdata):
        """Executes monitored navigation."""
        destination = self.global_store['start_location']
        messages = ['Arrived back at: ' + destination, 'Navigation Failure']
        probs = [0.99, 0.01]

        return dummy_behaviour(self._outcomes, probs, messages)

class StartBackgroundSystems(GlobalStoreState):
    """ SMACH state for travelling back to a known location. """
    def __init__(self, global_store):
        """ Constructor initialises attributes and calls super constructor."""

        outcomes = ['Started_Background_Systems', 'Failed_Start_Up']
        super(StartBackgroundSystems, self).__init__(global_store=global_store,
                                                     outcomes=outcomes)
    def execute(self, userdata):
        """Starts up base robot behaviours."""
        messages = ['Started people tracking, object/hot-word detection',
                    'Failed to start up base systems.']
        probs = [0.99, 0.01]

        self.global_store['start_location'] = 'WayPoint1'
        
        return dummy_behaviour(self._outcomes, probs, messages)

class Memorise(GlobalStoreState):
    """ SMACH state for the Memorise Behaviour which can lead to System Failure"""
    def __init__(self, global_store):
        outcomes = ['Failure', 'Memorised', 'RepeatedFailure']
        super(Memorise, self).__init__(global_store=global_store,
                                       outcomes=outcomes)
        self.counter = 0
        self.try_allow = 5

    # It requires the userdata which indicates the Memorised_flag
    def execute(self,userdata):
        probs = [0.1,0.9]
        msgs = ['Failed to memorise', 'Memorised']
        outcome = dummy_behaviour(self._outcomes[0:2], probs, msgs)
        if outcome == 'Memorised':
            self.global_store['op_info'] = 'Red'
        else:
            if self.counter > self.try_allow:
                return 'RepeatedFailure'
            else:
                self.counter += 1
                return 'Failure'

class AskForOperator(GlobalStoreState):
    """ SMACH state for the Ask For Operator"""
    def __init__(self, global_store):
        
        self.counter = 0
        self.try_allow = 5

        outcomes = ['Failure', 'Operator_Found', 'Repeat_Failure']
        smach.State.__init__(global_store=global_store,
                             outcomes=outcomes)


    def execute(self,userdata):
        probs = [0.1, 0.9]
        msgs = ['Failed to Find operator', 'Found Operator']
        
        outcome = dummy_behaviour(self._outcomes[0:2], probs, msgs)
        if outcome == 'Failure':
            self.counter += 1
        
        if self.counter > self.try_allow:
            return 'Repeat_Failure'
        else:
            return outcome



class WaitForRequest(GlobalStoreState):
    """ SMACH state for the wait for request"""
    def __init__(self, global_store):

        self.counter = 0
        self.try_allow = 5

        outcomes = ['Time_Out', 'Receive_Request', 'Repeat_Timeout']
        super(WaitForRequest, self).__init__(global_store=global_store,
                                             outcomes=outcomes)

    # It requires the userdata which indicates the request_flag
    def execute(self,userdata):
        probs = [0.1, 0.9]
        msgs=['Time Out Waiting For Request', 'Request Received']
        outcome = dummy_behaviour(self._outcomes[0:2], probs, msgs)

        if outcome == 'Receive_Request':
            self.global_store['item'] = 'Coffee Mug'

        if outcome == 'Time_Out':
            self.counter += 1
        
        if self.counter > self.try_allow:
            return 'Repeat_Timeout'
        else:
            return outcome


class AskForRequest(GlobalStoreState):
    """ SMACH state for the ask for request"""
    def __init__(self, global_store):
        super(AskForRequest, self).__init__(global_store=global_store,
                                            outcomes=['Asked'])

    #It executes the dummy version of asking for request
    def execute(self,userdata):
        rospy.loginfo('Asking for request')
        return 'Asked'

class WaitForOperator(GlobalStoreState):
    """SMACH state for detecting the operator in the first instance."""
    def __init__(self, global_store):
        """ Constructor initialises attributes and calls super constructor."""

        outcomes = ['Operator_Found', 'Time_Out']
        super(WaitForOperator, self).__init__(global_store=global_store,
                                              outcomes=outcomes)
    
    def execute(self, userdata):
        """ Looks for operator."""
        messages = ['Operator Found', 'Time Out looking for operator']
        probs = [0.9, 0.1]

        outcome = dummy_behaviour(self._outcomes, probs, messages)
        
        return outcome

# <---- Code for concurrent Follow Behaviour ---->

class DetectFollowSignal(GlobalStoreState):
    """SMACH state for detecting the end of follow signal from the operator."""
    def __init__(self, global_store):
        """ Constructor initialises attributes and calls super constructor."""

        outcomes = ['Signal_Detected', 'Time_Out']
        super(DetectFollowSignal, self).__init__(global_store=global_store,
                                                 outcomes=outcomes)
    def execute(self, userdata):
        """Waits for signal from operator."""
        messages = ['Signal from operator wearing: ' + userdata.in_op_info,
                    'Time Out waiting for signal']
        probs = [0.99, 0.01]
        time.sleep(0.1)

        return dummy_behaviour(self._outcomes, probs, messages)

class FollowWithCamera(GlobalStoreState):
    """State for keeping operator in view of camera."""

    def __init__(self, global_store):
        """ Constructor initialises attributes and calls super constructor."""
        outcomes = ['Lost_Operator']
        super(FollowWithCamera, self).__init__(global_store=global_store,
                                               outcomes=outcomes)

    def execute(self, userdata):
        """Follows until lost."""
        colour = str(self.global_store['op_info'])
        rospy.loginfo('Following Operator wearing ' + colour)
        while np.random.rand() < 0.99:
            time.sleep(0.01)
        
        rospy.loginfo('Lost Operator')

        return self._outcomes[0]

def follow_child_cb(outcome_map):

    if outcome_map['Sig_Detect'] == 'Signal_Detected':
        return True
    
    if outcome_map['Follow_Camera'] == 'Lost_Operator':
        return True
    
    if outcome_map['Follow_Nav'] == 'Navigation_Failure':
        return True
    
    return False

def make_follow_concurrent_state(global_store):
    """ Function creates concurrent container for the follow behaviour."""
    con = Concurrence(outcomes=['Follow_Success', 
                                'Follow_Nav_Failure',
                                'Follow_Cam_Failure'],
                      default_outcome='Follow_Failure',
                      child_termination_cb = follow_child_cb,
                      outcome_map={'Follow_Success': 
                                    {'Sig_Detect': 'Signal_Detected'},
                                   'Follow_Nav_Failure': 
                                    {'Follow_Nav': 'Navigation_Failure'},
                                   'Follow_Cam_Failure': 
                                    {'Follow_Camera': 'Lost_Operator'}})
    with con:
        Concurrence.add('Sig_Detect', DetectFollowSignal(global_store))
        Concurrence.add('Follow_Nav', MonitoredNav(global_store))
        Concurrence.add('Follow_Camera', FollowWithCamera(global_store))
    
    return con

def make_and_start_state_machine():
    """ Function for starting node/state machine."""
    rospy.init_node('dummy_help_me_carry')

    # Create the state machine
    sm = smach.StateMachine(outcomes=['TASK_SUCCESS', 'TASK_FAILURE'])

    global_store = {}
    
    # Add our states
    with sm:
        
        # Add start-up state
        smach.StateMachine.add('Start_Up',
                                StartBackgroundSystems(global_store),
                                transitions={'Started_Background_Systems':
                                             'Op_Detect',
                                             'Failed_Start_Up':
                                             'TASK_FAILURE'})
        # Add operator waiting state
        smach.StateMachine.add('Op_Detect',
                                WaitForOperator(global_store),
                                transitions={'Operator_Found': 'Memorise',
                                             'Time_Out': 'TASK_FAILURE'})

        # Add memorisation state
        smach.StateMachine.add('Memorise', Memorise(global_store),
                               transitions={'Failure':'Memorise',
                                            'Memorised':'Follow', 
                                            'RepeatedFailure':'TASK_FAILURE'})

        # Concurrent follow state
        concurrent = make_follow_concurrent_state(global_store)
        smach.StateMachine.add('Follow', concurrent,
                                transitions={'Follow_Success':
                                             'WaitForRequest',
                                             'Follow_Nav_Failure':
                                             'TASK_FAILURE',
                                             'Follow_Cam_Failure':
                                             'AskForOperator'})

        # State for asking operator if lost
        smach.StateMachine.add('AskForOperator', AskForOperator(global_store),
                               transitions = {'Failure': 'AskForOperator',
                                              'Operator_Found':'Follow',
                                              'Repeat_Failure':'TASK_FAILURE'})

        # State for waiting for item request
        smach.StateMachine.add('WaitForRequest', WaitForRequest(global_store),
                               transitions = {'Time_Out': 'AskForRequest',
                                              'Receive_Request':'Find_Item',
                                              'Repeat_Timeout':'TASK_FAILURE'})

        # Asking operator for help if no request given
        smach.StateMachine.add('AskForRequest', AskForRequest(global_store),
                               transitions = {'Asked':'WaitForRequest'})

        # Try and find the item
        smach.StateMachine.add('FindItem', FindItem(global_store),
                               transitions={'Item_Found':'GetItem',
                                            'Item_Not_Found':'ReturnToOp'})
        
        # Return to operator if item can't be found
        smach.StateMachine.add('ReturnToOp', ReturnToOperator(global_store),
                               transitions={'Found_Operator':
                                            'AskAssistance',
                                            'Cant_Find_Operator':
                                            'TASK_FAILURE'})
        
        # Ask operator for assistance
        smach.StateMachine.add('AskAssistance', AskForAssistance(global_store),
                               transitions={'Assistance_Given':'FindItem',
                                            'No_Assistance':'TASK_FAILURE'})

        # Pick up the item
        smach.StateMachine.add('GetItem', GetItem(global_store),
                               transitions={'Grasped':
                                            'TravelBack',
                                            'Not_Grasped':
                                            'GetItem',
                                            'Repeat_Not_Grasped':
                                            'TASK_FAILURE'})
        
        # Travel back to start spot
        smach.StateMachine.add('TravelBack', MonitoredNav(global_store),
                               transitions={'Reached_Destination':
                                            'PutOnFloor',
                                            'Navigation_Failure':
                                            'TASK_FAILURE'})

        # Put item on floor
        smach.StateMachine.add('PutOnFloor', PutOnFloor(global_store),
                               transitions={'On_Floor':'StateJumperColour',
                                            'Failure':'TASK_FAILURE'})

        # States jumper colour of operator
        smach.StateMachine.add('StateJumperColour',
                                StateJumperColour(global_store),
                                transitions={'Correct_Colour':'TASK_SUCCESS',
                                             'Incorrect_Colour':'TASK_FAILURE'})

    # Execute the State Machine
    _ = sm.execute()

if __name__ == '__main__':
    make_and_start_state_machine()