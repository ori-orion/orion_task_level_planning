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


TASK_OBJECTS = ['potted plant', 'tomato', 'flashlight']

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

        phrase = ("Hi, my name is Bam Bam, welcome to the Oxford Robotics " + 
                 "Institute! Today we're going to show you what I can do!")
        smach.StateMachine.add('Intro',
                               SpeakState(action_dict, global_store, phrase),
                               transitions={'SUCCESS':'SetNavToOperator',
                                            'FAILURE':'SetNavToOperator'})
        

        operator_pose = Pose()
        operator_pose.position.x = -1.90554933177
        operator_pose.position.y = 6.14218817664
        operator_pose.position.z = 0.0
        operator_pose.orientation.x = 0.0
        operator_pose.orientation.y = 0.0
        operator_pose.orientation.z = -0.420347484759
        operator_pose.orientation.w = 0.907363208455
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
        
        question = ("And what would you like me to pick up?")
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
        pickup_pose.position.x = -2.72972419023
        pickup_pose.position.y = 5.54487004211
        pickup_pose.position.z = 0.0
        pickup_pose.orientation.x = 0.0
        pickup_pose.orientation.y = 0.0
        pickup_pose.orientation.z = -0.832897757204
        pickup_pose.orientation.w = 0.553426893135
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

        question = ("Can someone help me pick up the object and say ready " + 
                    "when they are ready?")
        smach.StateMachine.add('AskForHelp',
                               SpeakAndHotwordState(action_dict, 
                                                   global_store, 
                                                   question, 
                                                   ['ready'],
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
                               transitions={'SUCCESS':'ReadyToFollow'})

        phrase = ("Now I'm ready to follow you. Please go slow and say my " +
                  "name when you want me to stop.")
        smach.StateMachine.add('ReadyToFollow',
                               SpeakState(action_dict, global_store, phrase),
                               transitions={'SUCCESS': 'Follow',
                                            'FAILURE': 'Follow'})
        

        smach.StateMachine.add('Follow',
                                make_follow_hotword_state(action_dict, 
                                                          global_store),
                                transitions={'SUCCESS': 'ArrivalQuestion',
                                             'FAILURE': 'Follow',
                                             'REPEAT_FAILURE': 'TASK_FAILURE'})


        question = ("Yay, we've arrived! Please say ready when you're ready "
                    "for me to hand this back to you?")
        smach.StateMachine.add('ArrivalQuestion',
                               SpeakAndHotwordState(action_dict, 
                                                   global_store, 
                                                   question, 
                                                   ['ready'],
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