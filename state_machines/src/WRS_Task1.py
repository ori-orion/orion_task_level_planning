#!/usr/bin/env python
""" File for carry my luggage task for stage 1 of RoboCup@home 2019.

This file contains the state machine for task 1
of WRS 2020.

Author: Mia Mijovic
Owner: Mia Mijovic

"""

import rospy
import smach
import actionlib

from reusable_states import * # pylint: disable=unused-wildcard-import
from set_up_clients import create_stage_1_clients
from orion_actions.msg import SOMObservation, Relation

import time


class SearchForItemState(ActionServiceState):
    """ This state carries out a search for a misplaced item. """

    def __init__(self, action_dict, global_store):
        outcomes = ['OBJECT_FOUND', 'NO_OBJECTS']

        super(SearchForItemState, self).__init__(action_dict=action_dict,
                                                 global_store=global_store,
                                                 outcomes=outcomes)
    
    def execute(self, userdata):
        # TODO: Fill in!
        # Should update tf frame for pick up and also store information
        # About object in grasp so we can search its default location
        pass


class GetItemLocationState(ActionServiceState):
    """ This state gets the default location of the item being grasped. """
    #This one is copied from clean_up task so should be modified if used

    def __init__(self, action_dict, global_store):
        outcomes = ['SUCCESS', 'FAILURE']

        super(GetItemLocationState, self).__init__(action_dict=action_dict,
                                                   global_store=global_store,
                                                   outcomes=outcomes)
    
    def execute(self, userdata):
        item = self.global_store['pick_up']

        obj1 = SOMObservation()
        obj1.type = item
        
        rel = Relation()
        obj2 = SOMObservation()

        try:
            self.global_store['nav_location'] = \
                get_location_of_object(self.action_dict, obj1, rel, obj2)
            return self._outcomes[0]
        except:
            return self._outcomes[1]


class GetItemLocationAndCheckState(ActionServiceState):
    "This state gets the default location of displaced item and checks the position"
    "of the drawers"

    def __init__(self, action_dict, global_store):
        outcomes = ['NO_ACTION_REQ', 'ACTION_REQUIRED', 'FAILURE']

        super(GetItemLocationAndCheck, self).__init__(action_dict=action_dict,
                                                      global_store = global_store,
                                                      outcomes=outcomes)

    def execute(self, userdata):
        #TODO fill this in
        pass
        
#TODO Finish this state

class ShouldIContinueState(ActionServiceState):
    """ State determines whether we should continue or go back. """
    #Put after success when placing an object down

    def __init__(self, action_dict, global_store):
        outcomes = ['YES', 'NO']
        super(ShouldIContinueState, self).__init__(action_dict=action_dict,
                                                   global_store=global_store,
                                                   outcomes=outcomes)
    
    def execute(self, userdata):
        time_elapsed = time.time() - self.global_store['start_time']
        if time_elapsed > 540: # 9 minutes
            return self._outcomes[1]
        elif len(self.global_store['objects_displaced']) == []:
            return self._outcomes[1]
        
        return self._outcomes[0]


class UpdateItemLocationState(ActionServiceState):
    """ This state updates item location based on speech. """

    def __init__(self, action_dict, global_store):
        outcomes = ['SUCCESS', 'FAILURE']

        super(UpdateItemLocationState, self).__init__(action_dict=action_dict,
                                                      global_store=global_store,
                                                      outcomes=outcomes)
    
    def execute(self, userdata):
        # TODO: Fill in!
        # Should set navigation goal in global store and parse speech with soma
        pass

def create_state_machine(action_dict):
    """ This function builds the state machine for the Tidy Up task.

    The complete state machine for the tidy up task is returned
    from this function.

    Args:
        action_dict: The dictionary from action server client names
                     to action server clients

    Returns:
        sm: The complete state machine
    """

    # Initialise global store
    global_store = {}
    global_store['requested_object'] = None
    global_store['displaced_objects'] = []
    global_store['drawer_layout'] = [0 0 0]

    # Create the state machine
    sm = smach.StateMachine(outcomes=['TASK_SUCCESS', 'TASK_FAILURE'])

    with sm:

        # Detect and memorise the operator
        """smach.StateMachine.add('DetectOperator',
                               OperatorDetectState(action_dict, global_store),
                               transitions={'SUCCESS': 'ListenForObject',
                                            'FAILURE': 'StartTalking'})

        smach.StateMachine.add('StartTalking')

        smach.StateMachine.add('ListenForObject',
                                )"""

        #Exploring - Search for the next item
        smach.StateMachine.add('SearchForItem',
                                SearchForItemState(action_dict, global_store),
                                transitions = {'OBJECTFOUND': 'GetItemLocationAndCheck',
                                                'NO_OBJECTS': 'SearchForItem',
                                                'TIMEOUT': 'TASK_SUCCESS'})
                                #TODO SearchForItemState
                                # Should I continue State?

        # Get the items default location
        smach.StateMachine.add('GetItemLocationAndCheck',
                               GetItemLocationAndCheckState(action_dict, global_store),
                               transitions={'NO_ACTION_REQ':'GraspItem',
                                            'ACTION_REQUIRED': 'NavToDrawer'
                                            'FAILURE':'SearchForItem'})
                                            #TODO GetItemLocationAndCheckState
        
        # Ask for help if can't find item location
        question = ("Can someone please tell me where to place the object I am"+
                   " holding? Thank you.")
        smach.StateMachine.add('AskForHelpLocation',
                               SpeakAndListenState(action_dict,
                                                   global_store,
                                                   question,
                                                   map(RELATIONS, 
                                                   (lambda x: x + ' <param>')),
                                                   OBJECTS,
                                                   30),
                               transitions={'SUCCESS':'UpdateItemLocation',
                                            'FAILURE':'AskForHelpLocation',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})

        # Check if the drawers are open 
        #Do global knowledge about drawersand their orientation
        #Store/Get robot location
        #Smach viewer

        # Pick up the object
        smach.StateMachine.add('GraspItem',   
                               PickUpObjectState(action_dict, global_store),
                               transitions={'SUCCESS':'NavToObjectLocation',
                                            'Failure':'AskForHelpGrasping'})
    
        
        # Ask for help if can't grasp object
        question = ("I can't pick up this object. Can someone help me please " +
                    "and let me know once they ready to hand it to me?")
        smach.StateMachine.add('AskForHelpGrasping',
                               SpeakAndListenState(action_dict,
                                                   global_store,
                                                   question,
                                                   READY,
                                                   [],
                                                   20),
                               transitions={'SUCCESS':'ReceiveObject',
                                            'FAILURE':'AskForHelpGrasping',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})

        # Receive object from operator
        smach.StateMachine.add('ReceiveObject',
                               ReceiveObjectFromOperatorState(action_dict,
                                                              global_store),
                               transitions={'SUCCESS':'GetItemLocation',
                                            'FAILURE':'AskForHelpGrasping'})

    """   #TODO CheckItemLocationState
        #Checks if the designated location for object is drawer
        smach.StateMachine.add('CheckLocation',
                                CheckItemLocationState(action_dict, 
                                                        global_store),
                                transitions={'IS_DRAWER':'CheckDrawer',
                                             'NOT_DRAWER':'GraspItem'})

        #Checks if the drawer is open/accessible
        #TODO NavToDrawer - can it be NavToObejctLocation where obj is drawer?
        #If so, how not to lose current obj location
        #TODO IsDrawerOpenState - turn towards drawers and see the layout
        smach.StateMachine.add('CheckDrawer',
                                IsDrawerOpenState(action_dict, global_store),
                                transitions={'OPEN':'NavToObjectLocation',
                                             'NOT_OPEN':'OpenDrawer'})
"""
    

        #Navigates to drawers
        smach.StateMachine.add('NavToDrawer',
                                NavigateState(action_dict, global_store),
                                transitions={'SUCCESS': 'OpenDawer',
                                             'FAILURE': 'NavToDrawer'
                                             'REPEAT_FAILURE':'TASK_FAILURE'})

        #Opens/closes the drawers
        smach.StateMachine.add('OpenDrawer',
                                OpenDrawerState(action_dict, global_store),
                                transitions={'SUCCESS':'NavBackToObjectLocation',
                                             'FAILURE':'AskForHelpOpening'})
                                             #TODO OpenDrawerState

        #Ask for help if can't open the drawers
        question = ("I can't open this drawer. Can someone help me please " +
                    "and let me know once they're done?")
        smach.StateMachine.add('AskForHelpOpening',
                                SpeakAndListenState(action_dict,
                                                   global_store,
                                                   question,
                                                   READY,
                                                   [],
                                                   20),
                                transitions={'SUCCESS':'NavToObjectLocation',
                                             'FAILURE':'TASK_FAILURE'})

        #Navigate back to the object
        smach.StateMachine.add('NavBackToObjectLocation',
                                NavToObjectLocationState(action_dict, global_store),
                                transitions={'SUCCESS': 'GraspItem',
                                             'FAILURE' : 'TASK_FAILURE'})
        
        
        # Navigate to the objects default location
        smach.StateMachine.add('NavToObjectLocation',
                               NavigateState(action_dict, global_store),
                               transitions={'SUCCESS':'PlaceObject',
                                            'FAILURE':'NavToObjectLocation',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Place the object down TODO Place Object anywhere?
        smach.StateMachine.add('PlaceObject',
                               PutObjectOnSurfaceState(action_dict, 
                                                       global_store),
                               transitions={'SUCCESS':'SearchForItem',
                                            'FAILURE':'TASK_FAILURE'})
        
    
                        
    return sm
            

if __name__ == '__main__':
    action_dict = create_stage_1_clients(2)
    sm = create_state_machine(action_dict)
    sm.execute()
