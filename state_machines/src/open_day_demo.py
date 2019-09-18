#!/usr/bin/env python
""" Smach state machine for ORI open day demonstration!

Author: Charlie Street
Owner: Charlie Street
"""

import rospy
import smach
import actionlib

from reusable_states import * # pylint: disable=unused-wildcard-import
from set_up_clients import create_open_day_clients
from geometry_msgs.msg import Pose


TASK_OBJECTS = ['potted plant', 'cup', 'bottle']

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

        obj1 = SOMObservation()
        obj1.obj_id = self.global_store['people_found'][0]
        rel = Relation()
        obj2 = SOMObservation()

        matches = self.action_dict['SOMQuery'](obj1, rel, obj2, Pose()).matches
        
        name = matches[0].obj1.name

        picked_up = self.global_store['pick_up']

        phrase = "Hi, " + name + ", I've brought you the " + picked_up 

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

        phrase = ("Hi, my name is Bam Bam, and " + 
                 "today we're going to show you what I can do!")
        smach.StateMachine.add('Intro',
                               SpeakState(action_dict, global_store, phrase),
                               transitions={'SUCCESS':'SetNavToOperator',
                                            'FAILURE':'SetNavToOperator'})
        

        operator_pose = Pose()
        operator_pose.position.x = -1.01430902499
        operator_pose.position.y = -2.56958996387
        operator_pose.position.z = 0.0
        operator_pose.orientation.x = 0.0
        operator_pose.orientation.y = 0.0
        operator_pose.orientation.z = 0.144234027818
        operator_pose.orientation.w = 0.989543604506
        func = lambda: operator_pose # TODO: Set to location of operator
        smach.StateMachine.add('SetNavToOperator',
                               SetNavGoalState(action_dict, global_store, func),
                               transitions={'SUCCESS':'NavToOperator'})

        smach.StateMachine.add('NavToOperator',
                               NavigateState(action_dict, global_store),
                               transitions={'SUCCESS':'AskOperatorName',
                                            'FAILURE':'NavToOperator',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
    
        question = ("Hi, what's your name?")
        smach.StateMachine.add('AskOperatorName',
                               SpeakAndListenState(action_dict,
                                                   global_store,
                                                   question,
                                                   NAMES,
                                                   [],
                                                   20),
                               transitions={'SUCCESS': 'DetectOperator',
                                            'FAILURE':'AskOperatorName',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Detect and memorise the operator
        smach.StateMachine.add('DetectOperator',
                               OperatorDetectState(action_dict, global_store),
                               transitions={'SUCCESS': 'AskPickUp',
                                            'FAILURE': 'AskOperatorName'})
        
        question = ("Great! What would you like me to pick up?")
        smach.StateMachine.add('AskPickUp',
                               SpeakAndListenState(action_dict,
                                                   global_store,
                                                   question,
                                                   TASK_OBJECTS,
                                                   [],
                                                   20),
                               transitions={'SUCCESS': 'SetPickUp',
                                            'FAILURE':'AskPickUp',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})

        func = lambda: global_store['last_response']
        smach.StateMachine.add('SetPickUp',
                               SetPickupFuncState(action_dict, 
                                                  global_store, 
                                                  func),
                               transitions={'SUCCESS':'SetNavToPickUp'})
        
        pickup_pose = Pose()
        pickup_pose.position.x = 1.31152069282
        pickup_pose.position.y = -2.64591352064
        pickup_pose.position.z = 0.0
        pickup_pose.orientation.x = 0.0
        pickup_pose.orientation.y = 0.0
        pickup_pose.orientation.z = 0.214779706997
        pickup_pose.orientation.w = 0.976662519739
        func = lambda: pickup_pose # TODO: Set to Pick up location
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

        smach.StateMachine.add('AskForHelp',
                               SpeakAndListenPickupState(action_dict, 
                                                         global_store, 
                                                         READY,
                                                         [],
                                                         7),
                                transitions={'SUCCESS':'ReceiveItem',
                                             'FAILURE':'AskForHelp',
                                             'REPEAT_FAILURE':'TASK_FAILURE'})

        smach.StateMachine.add('ReceiveItem',
                               ReceiveObjectFromOperatorState(action_dict,
                                                              global_store),
                               transitions={'SUCCESS':'SetNavBackToOperator',
                                            'FAILURE':'TASK_FAILURE'})
        
        func = lambda: operator_pose # Set back to operator location 
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
                               transitions={'SUCCESS':'ArrivalQuestion'})

        """phrase = ("Now I'm ready to follow you. Please go slow and say " +
                  "cancel when you want me to stop.")
        smach.StateMachine.add('ReadyToFollow',
                               SpeakState(action_dict, global_store, phrase),
                               transitions={'SUCCESS': 'Follow',
                                            'FAILURE': 'Follow'})
        

        smach.StateMachine.add('Follow',
                                make_follow_hotword_state(action_dict, 
                                                          global_store),
                                transitions={'SUCCESS': 'ArrivalQuestion',
                                             'FAILURE': 'Follow',
                                             'REPEAT_FAILURE': 'TASK_FAILURE'})"""


        question = ("Please say ready when you're ready "
                    "for me to hand this back to you?")
        smach.StateMachine.add('ArrivalQuestion',
                               SpeakAndListenState(action_dict, 
                                                   global_store, 
                                                   question, 
                                                   READY,
                                                   [],
                                                   7),
                                transitions={'SUCCESS':'GiveItemBack',
                                             'FAILURE':'ArrivalQuestion',
                                             'REPEAT_FAILURE':'TASK_FAILURE'})

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

        phrase = "I'm back where I started, woohoo!"
        smach.StateMachine.add('Finish',
                               SpeakState(action_dict, global_store, phrase),
                               transitions={'SUCCESS':'TASK_SUCCESS',
                                            'FAILURE':'TASK_FAILURE'})  
    
    return sm


if __name__ == '__main__':
    rospy.init_node('open_day_demo_state_machine')
    action_dict = create_open_day_clients()
    sm = create_state_machine(action_dict)
    sm.execute()