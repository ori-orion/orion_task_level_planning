#!/usr/bin/env python3
""" Smach state machine for ORI open day demonstration!

Author: Charlie Street
Owner: Charlie Street
"""

import os
import rospy
import smach
import smach_ros
import actionlib

from reusable_states import * # pylint: disable=unused-wildcard-import
from set_up_clients import create_open_day_clients
from geometry_msgs.msg import Pose


TASK_OBJECTS = ['potted plant', 'cup', 'bottle']

TASK_NAMES = ['Gemma', 'Acacia', 'Ollie', 'Nick', 'Hollie', 
          'Charlie', 'Matt', 'Daniele', 'Chris', 'Paul', 'Lars', 'Jon',
          'Michael', 'Matthew', 'Ricardo']

# Load up all of our names
import rospkg
rospack = rospkg.RosPack()
name_file = \
    os.path.join(rospack.get_path('state_machines'),'grammars/names.txt')
# with open(name_file, 'r') as in_file:
#     TASK_NAMES += in_file.read().splitlines()


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

        #obj1 = SOMObservation()
        #obj1.obj_id = self.global_store['people_found'][0]
        #rel = Relation()
        #obj2 = SOMObservation()

        #matches = self.action_dict['SOMQuery'](obj1, rel, obj2, Pose()).matches
        
        #name = matches[0].obj1.name
        name = self.global_store['operator_name']
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
    sm = smach.StateMachine(outcomes=['task_success', 'task_failure'])
    sm.userdata.nav_failures = 0
    sm.userdata.nav_failure_threshold = 3
    sm.userdata.intro_phrase = ("Hi, my name is Bam Bam, and " + 
                 "today we're going to show you what I can do!")
    sm.userdata.operator_question = "Hi, what's your name?"
    sm.userdata.speak_listen_failures = 0
    sm.userdata.speak_listen_failure_threshold = 3
    sm.userdata.task_names = TASK_NAMES
    sm.userdata.speak_and_listen_params_empty = []
    sm.userdata.speak_and_listen_timeout = 5

    with sm:
        
        smach.StateMachine.add('STORE_INITIAL_LOCATION',
                                GetRobotLocationState(),
                                transitions={'stored':'INTRO'},
                                remapping={'robot_location':'robot_location'})

        # userdata.phrase = ("Hi, my name is Bam Bam, and " + 
        #          "today we're going to show you what I can do!")

        # smach.StateMachine.add('INTRO',
        #                         SpeakState(),
        #                         transitions={'success':'SetNavToOperator',
        #                                      'failure':'SetNavToOperator'},
        #                         remapping={'phrase':'intro_phrase'})

        smach.StateMachine.add('INTRO',
                                SpeakState(),
                                transitions={'success':'ASK_OPERATOR_NAME',
                                             'failure':'task_failure'},
                                remapping={'phrase':'intro_phrase'})
        # orig
        # def __init__(self, 
        #          action_dict, 
        #          global_store, 
        #          question, 
        #          candidates, 
        #          params, 
        #          timeout):

        # operator_pose = Pose()
        # operator_pose.position.x = 1.57894771198
        # operator_pose.position.y = 0.59994803628
        # operator_pose.position.z = 0.0
        # operator_pose.orientation.x = 0.0
        # operator_pose.orientation.y = 0.0
        # operator_pose.orientation.z = -0.362624641735
        # operator_pose.orientation.w = 0.931935281662
        # func = lambda: operator_pose
        # # smach.StateMachine.add('SetNavToOperator',
        # #                        SetNavGoalState(action_dict, global_store, func),
        # #                        transitions={'success':'NavToOperator'})

        # TODO - fix userdata.nav_failures input and output (need to reset and re-use in later states)
        # smach.StateMachine.add('NAV_TO_OPERATOR',
        #                        NavigateState(),
        #                        transitions={'success':'AskOperatorName',
        #                                     'failure':'NavToOperator',
        #                                     'repeat_failure':'TASK_FAILURE'})
    
        # question = ("Hi, what's your name?")
        smach.StateMachine.add('ASK_OPERATOR_NAME',
                               SpeakAndListenState(),
                                transitions={'success': 'task_success',
                                            'failure':'ASK_OPERATOR_NAME',
                                            'repeat_failure':'task_failure'},
                                remapping={'question':'operator_question',
                                            'operator_response': 'operator_name',
                                            'candidates':'task_names',
                                            'params':'speak_and_listen_params_empty',
                                            'timeout':'speak_and_listen_timeout'})
        
        # # Detect and memorise the operator
        # smach.StateMachine.add('DetectOperator',
        #                        OperatorDetectState(action_dict, global_store),
        #                        transitions={'SUCCESS': 'AskPickUp',
        #                                     'FAILURE': 'AskOperatorName'})
        
        # question = ("Great! What would you like me to pick up?")
        # smach.StateMachine.add('AskPickUp',
        #                        SpeakAndListenState(action_dict,
        #                                            global_store,
        #                                            question,
        #                                            TASK_OBJECTS,
        #                                            [],
        #                                            20),
        #                        transitions={'SUCCESS': 'SetPickUp',
        #                                     'FAILURE':'AskPickUp',
        #                                     'REPEAT_FAILURE':'TASK_FAILURE'})

        # func = lambda: global_store['last_response']
        # smach.StateMachine.add('SetPickUp',
        #                        SetPickupFuncState(action_dict, 
        #                                           global_store, 
        #                                           func),
        #                        transitions={'SUCCESS':'SetNavToPickUp'})
        
        # pickup_pose = Pose()
        # pickup_pose.position.x = 0.829577886698
        # pickup_pose.position.y = -0.00474571408639
        # pickup_pose.position.z = 0.0
        # pickup_pose.orientation.x = 0.0
        # pickup_pose.orientation.y = 0.0
        # pickup_pose.orientation.z = -0.653883288824
        # pickup_pose.orientation.w = 0.756595429934
        # func = lambda: pickup_pose
        # smach.StateMachine.add('SetNavToPickUp',
        #                        SetNavGoalState(action_dict, global_store, func),
        #                        transitions={'SUCCESS':'NavToPickUp'})

        # smach.StateMachine.add('NavToPickUp',
        #                        NavigateState(action_dict, global_store),
        #                        transitions={'SUCCESS':'PickUpItem',
        #                                     'FAILURE':'NavToPickUp',
        #                                     'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # smach.StateMachine.add('PickUpItem',
        #                        PickUpObjectState(action_dict, global_store),
        #                        transitions={'SUCCESS':'SetNavBackToOperator',
        #                                     'FAILURE':'AskForHelp'})

        # smach.StateMachine.add('AskForHelp',
        #                        SpeakAndListenPickupState(action_dict, 
        #                                                  global_store, 
        #                                                  READY,
        #                                                  [],
        #                                                  7),
        #                         transitions={'SUCCESS':'ReceiveItem',
        #                                      'FAILURE':'AskForHelp',
        #                                      'REPEAT_FAILURE':'TASK_FAILURE'})

        # smach.StateMachine.add('ReceiveItem',
        #                        ReceiveObjectFromOperatorState(action_dict,
        #                                                       global_store),
        #                        transitions={'SUCCESS':'SetNavBackToOperator',
        #                                     'FAILURE':'TASK_FAILURE'})
        
        # func = lambda: operator_pose # Set back to operator location 
        # smach.StateMachine.add('SetNavBackToOperator',
        #                        SetNavGoalState(action_dict, global_store, func),
        #                        transitions={'SUCCESS':'NavBackToOperator'})

        # smach.StateMachine.add('NavBackToOperator',
        #                        NavigateState(action_dict, global_store),
        #                        transitions={'SUCCESS':'SpeakToOperator',
        #                                     'FAILURE':'NavBackToOperator',
        #                                     'REPEAT_FAILURE':'TASK_FAILURE'})

        # smach.StateMachine.add('SpeakToOperator',
        #                        SpeakToOperatorState(action_dict, global_store),
        #                        transitions={'SUCCESS':'ArrivalQuestion'})

        # """phrase = ("Now I'm ready to follow you. Please go slow and say " +
        #           "cancel when you want me to stop.")
        # smach.StateMachine.add('ReadyToFollow',
        #                        SpeakState(action_dict, global_store, phrase),
        #                        transitions={'SUCCESS': 'Follow',
        #                                     'FAILURE': 'Follow'})
        

        # smach.StateMachine.add('Follow',
        #                         make_follow_hotword_state(action_dict, 
        #                                                   global_store),
        #                         transitions={'SUCCESS': 'ArrivalQuestion',
        #                                      'FAILURE': 'Follow',
        #                                      'REPEAT_FAILURE': 'TASK_FAILURE'})"""


        # question = ("Please say ready when you're ready "
        #             "for me to hand this back to you?")
        # smach.StateMachine.add('ArrivalQuestion',
        #                        SpeakAndListenState(action_dict, 
        #                                            global_store, 
        #                                            question, 
        #                                            READY,
        #                                            [],
        #                                            7),
        #                         transitions={'SUCCESS':'GiveItemBack',
        #                                      'FAILURE':'ArrivalQuestion',
        #                                      'REPEAT_FAILURE':'TASK_FAILURE'})

        # smach.StateMachine.add('GiveItemBack',
        #                        HandoverObjectToOperatorState(action_dict,
        #                                                      global_store),
        #                        transitions={'SUCCESS': 'ThankOperator',
        #                                     'FAILURE': 'TASK_FAILURE'})


        # phrase = ("It looks like my job here is done. Have a nice day!")
        # smach.StateMachine.add('ThankOperator',
        #                        SpeakState(action_dict, global_store, phrase),
        #                        transitions={'SUCCESS':'SetNavToStart',
        #                                     'FAILURE':'SetNavToStart'})

        # func = lambda : global_store['stored_location']  
        # smach.StateMachine.add('SetNavToStart',
        #                        SetNavGoalState(action_dict, global_store, func),
        #                        transitions={'SUCCESS':'NavToStart'}) 

        # smach.StateMachine.add('NavToStart',
        #                        NavigateState(action_dict, global_store),
        #                        transitions={'SUCCESS': 'Finish',
        #                                     'FAILURE': 'NavToStart',
        #                                     'REPEAT_FAILURE': 'TASK_FAILURE'})

        # phrase = "I'm back where I started, woohoo!"
        # smach.StateMachine.add('Finish',
        #                        SpeakState(action_dict, global_store, phrase),
        #                        transitions={'SUCCESS':'TASK_SUCCESS',
        #                                     'FAILURE':'TASK_FAILURE'})  
    
    return sm


if __name__ == '__main__':
    rospy.init_node('open_day_demo_state_machine')
    action_dict = {}
    # action_dict = create_open_day_clients()

    

    # Create the state machine
    sm = create_state_machine(action_dict)

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    sm.execute()

    # Run until ctl+c command is received
    rospy.spin()
    sis.stop()