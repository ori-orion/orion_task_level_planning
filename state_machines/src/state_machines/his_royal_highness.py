#!/usr/bin/env python3
""" Smach state machine for Prince William H B Allen Centre Demo!

Author: Charlie Street
Owner: Charlie Street
"""

import rospy
import smach
import actionlib

from orion_task_level_planning.state_machines.src.state_machines.reusable_states_deprecated import * # pylint: disable=unused-wildcard-import
from set_up_clients import create_open_day_clients
from geometry_msgs.msg import Pose


TASK_OBJECTS = ['potted plant', 'cup', 'bottle']
PICK_UP = 'potted plant'

class SpeakToOperatorState(ActionServiceState):
    """ Smach state for the robot to say stuff.

    This class has the robot say something and return success if it has been
    said. This says the operator name and the item that's been picked up

    Attributes:
        phrase: What we want the robot to say
    """
    def __init__(self, action_dict, global_store):
        outcomes = ['SUCCESS']
        super(SpeakToOperatorState, self).__init__(action_dict=action_dict,
                                                   global_store=global_store,
                                                   outcomes=outcomes)
    
    def execute(self, userdata):

        picked_up = self.global_store['pick_up']

        phrase = "Hi, I've brought you the " + picked_up 

        action_goal = TalkRequestGoal()
        action_goal.data.language = Voice.kEnglish
        action_goal.data.sentence = phrase
        self.action_dict['Speak'].send_goal(action_goal)
        self.action_dict['Speak'].wait_for_result()

        # Can only succeed
        return self._outcomes[0]

class SpeakAndListenPickupState(ActionServiceState):
    """ Smach state for speaking and then listening for a response.

    This state will get the robot to say something and then wait for a 
    response.
    """

    def __init__(self, 
                 action_dict, 
                 global_store,
                 candidates, 
                 params, 
                 timeout):
        """ Constructor initialises fields and calls super constructor.

        Args:
            action_dict: As in super class
            global_store: As in super class
            question: The question to ask
            candidates: Candidate sentences
            params: Optional parameters for candidate sentences
            timeout: The timeout for listening
        """

        outcomes = ['SUCCESS', 'FAILURE', 'REPEAT_FAILURE']
        self.candidates = candidates
        self.params = params
        self.timeout = timeout
        super(SpeakAndListenPickupState, self).__init__(action_dict=action_dict,
                                                        global_store=global_store,
                                                        outcomes=outcomes)
        
        if 'speak_listen_failure' not in self.global_store:
            self.global_store['speak_listen_failure'] = 0
    
    def execute(self, userdata):
        speak_listen_goal = SpeakAndListenGoal()
        speak_listen_goal.question = ("Can someone help me pick up the " +
                                      self.global_store['pick_up'] + 
                                      " and say ready " + 
                                      "when they are ready?")
        speak_listen_goal.candidates = self.candidates
        speak_listen_goal.params = self.params
        speak_listen_goal.timeout = self.timeout

        self.action_dict['SpeakAndListen'].send_goal(speak_listen_goal)
        self.action_dict['SpeakAndListen'].wait_for_result()

        result = self.action_dict['SpeakAndListen'].get_result()
        if result.succeeded:
            self.global_store['last_response'] = result.answer
            self.global_store['speak_listen_failure'] = 0
            return self._outcomes[0]
        else:
            self.global_store['speak_listen_failure'] += 1
            if self.global_store['speak_listen_failure'] >= FAILURE_THRESHOLD:
                return self._outcomes[2]
            return self._outcomes[1]


def create_state_machine(action_dict):
    """ This function builds the state machine for the ORI open day demo.

    Args:
        action_dict: The dictionary from action server client names
                     to action server clients

    Returns:
        sm: The complete state machine
    """

    # Initialise global store
    global_store = {}
    global_store['people_found'] = []
    global_store['last_person'] = None

    # Create the state machine
    sm = smach.StateMachine(outcomes=['TASK_SUCCESS', 'TASK_FAILURE'])

    with sm:
        
        smach.StateMachine.add('StoreInitialLocation',
                               GetRobotLocationState(action_dict, global_store),
                               transitions={'STORED':'Intro'})

        phrase = ("Hi, my name is Bam Bam and I'm going to pick up the " + PICK_UP)
        smach.StateMachine.add('Intro',
                               SpeakState(action_dict, global_store, phrase),
                               transitions={'SUCCESS':'SetPickUp',
                                            'FAILURE':'SetPickUp'})
        

        func = lambda: PICK_UP
        smach.StateMachine.add('SetPickUp',
                               SetPickupFuncState(action_dict, 
                                                  global_store, 
                                                  func),
                               transitions={'SUCCESS':'SetNavToPickUp'})
        
        pickup_pose = Pose()
        pickup_pose.position.x = -3.08
        pickup_pose.position.y = 0.522
        pickup_pose.position.z = 0.0
        pickup_pose.orientation.x = 0.0
        pickup_pose.orientation.y = 0.0
        pickup_pose.orientation.z = 0.981269127772
        pickup_pose.orientation.w = 0.192641892852
        func = lambda: pickup_pose # TODO: Change

        smach.StateMachine.add('SetNavToPickUp',
                               SetNavGoalState(action_dict, global_store, func),
                               transitions={'SUCCESS':'NavToPickUp'})

        smach.StateMachine.add('NavToPickUp',
                               NavigateState(action_dict, global_store),
                               transitions={'SUCCESS':'PickUpItem',
                                            'FAILURE':'NavToPickUp',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        smach.StateMachine.add('PickUpItem',
                               PickUpObjectState(action_dict, global_store),
                               transitions={'SUCCESS':'SetNavBackToOperator',
                                            'FAILURE':'AskForHelp'})

        question = "Can someone please pass me the " + PICK_UP + "?"
        smach.StateMachine.add('AskForHelp',
                               SpeakState(action_dict,global_store,question),
                                transitions={'SUCCESS':'ReceiveItem',
                                             'FAILURE':'ReceiveItem'})

        smach.StateMachine.add('ReceiveItem',
                               ReceiveObjectFromOperatorState(action_dict,
                                                              global_store),
                               transitions={'SUCCESS':'SetNavBackToOperator',
                                            'FAILURE':'TASK_FAILURE'})


        operator_pose = Pose()
        operator_pose.position.x = -5.01
        operator_pose.position.y = 0.167
        operator_pose.position.z = 0.0
        operator_pose.orientation.x = 0.0
        operator_pose.orientation.y = 0.0
        operator_pose.orientation.z = -0.992289634729
        operator_pose.orientation.w = 0.123940634214
        func = lambda: operator_pose # TODO: Change!
        smach.StateMachine.add('SetNavBackToOperator',
                               SetNavGoalState(action_dict, global_store, func),
                               transitions={'SUCCESS':'NavBackToOperator'})

        smach.StateMachine.add('NavBackToOperator',
                               NavigateState(action_dict, global_store),
                               transitions={'SUCCESS':'SpeakToOperator',
                                            'FAILURE':'NavBackToOperator',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})

        smach.StateMachine.add('SpeakToOperator',
                               SpeakToOperatorState(action_dict, global_store),
                               transitions={'SUCCESS':'GiveItemBack'})

        smach.StateMachine.add('GiveItemBack',
                               HandoverObjectToOperatorState(action_dict,
                                                             global_store),
                               transitions={'SUCCESS': 'ThankOperator',
                                            'FAILURE': 'TASK_FAILURE'})


        phrase = ("It looks like my job here is done. Have a nice day!")
        smach.StateMachine.add('ThankOperator',
                               SpeakState(action_dict, global_store, phrase),
                               transitions={'SUCCESS':'SetNavToStart',
                                            'FAILURE':'SetNavToStart'})

        func = lambda : global_store['stored_location']  
        smach.StateMachine.add('SetNavToStart',
                               SetNavGoalState(action_dict, global_store, func),
                               transitions={'SUCCESS':'NavToStart'}) 

        smach.StateMachine.add('NavToStart',
                               NavigateState(action_dict, global_store),
                               transitions={'SUCCESS': 'Finish',
                                            'FAILURE': 'NavToStart',
                                            'REPEAT_FAILURE': 'TASK_FAILURE'})

        phrase = "I'm now back where I started!"
        smach.StateMachine.add('Finish',
                               SpeakState(action_dict, global_store, phrase),
                               transitions={'SUCCESS':'TASK_SUCCESS',
                                            'FAILURE':'TASK_FAILURE'})  
    
    return sm


if __name__ == '__main__':
    rospy.init_node('his_royal_highness_state_machine')
    action_dict = create_open_day_clients()
    sm = create_state_machine(action_dict)
    sm.execute()