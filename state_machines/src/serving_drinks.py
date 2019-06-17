#!/usr/bin/env python
""" File contains code for Serving Drinks task.

This file contains the state machine for the serving drinks task.

Author: Charlie Street
"""


import rospy
import smach
import actionlib

from reusable_states import * # pylint: disable=unused-wildcard-import
from set_up_clients import create_stage_1_clients
from orion_actions.msg import SOMObservation, Relation

class FindPersonState(ActionServiceState):
    """ State to find someone without a drink. """
    def __init__(self, action_dict, global_store):
        outcomes = ['PERSON_FOUND', 'NO_PEOPLE']
        super(FindPersonState, self).__init__(action_dict=action_dict,
                                              global_store=global_store,
                                              outcomes=outcomes)
    
    def execute(self, userdata):
        # TODO: Fill in !
        pass


class CheckDrinkState(ActionServiceState):
    """ State to check if a drink is available and set its info if so. """
    def __init__(self, action_dict, global_store):
        outcomes = ['DRINK_FINE', 'DRINK_NOT_AVAILABLE']
        super(CheckDrinkState, self).__init__(action_dict=action_dict,
                                              global_store=global_store,
                                              outcomes=outcomes)
    
    def execute(self, userdata):

        drink = ""
        for avail_drink in DRINKS:
            if avail_drink in self.global_store['last_response']:
                drink = avail_drink
                break

        if drink in self.global_store['drinks']:
            # Update drink in person SOM object
            obj = SOMObservation()
            obj.obj_id = self.global_store['people_found'][-1]
            obj.drink = drink
            self.action_dict['SOMObserve'](obj)
            return self._outcomes[0]
        else:
            return self._outcomes[1]


def go_to_bar(action_dict):
    """ Gets location of bar. """
    obj1 = SOMObservation()
    obj1.type = 'serving_drinks_point_of_interest'

    return get_location_of_object(action_dict, obj1, 
                                  Relation(), SOMObservation())


def go_to_last_person(action_dict, global_store):
    """ Function gets location of last person met. """

    person_object = action_dict['SOMLookup'](global_store['people_found'][-1])

    pose = person_object.pose_estimate.most_recent_pose
    return pose


def create_state_machine(action_dict):
    """ Function creates and returns the state machine for this task. """

    global_store = {}
    global_store['people_found'] = []


    sm = smach.StateMachine(outcomes=['TASK_SUCCESS', 'TASK_FAILURE'])

    with sm:

        # Set nav goal to bar
        func = lambda: go_to_bar(action_dict)
        smach.StateMachine.add('SetNavToBar',
                               SetNavGoalState(action_dict, global_store, func),
                               transitions={'SUCCESS':'StartTalking'})


        # Start by speaking
        phrase = "Hi, I'm Bam Bam, I'm gonna go see what's at the bar!"
        smach.StateMachine.add('StartTalking',
                               SpeakState(action_dict, global_store, phrase),
                               transitions={'SUCCESS':'NavToBar',
                                            'FAILURE':'NavToBar'})
        
        # Navigate To The Bar
        smach.StateMachine.add('NavToBar',
                               NavigateState(action_dict, global_store),
                               transitions={'SUCCESS':'CheckBarDrinks',
                                            'FAILURE':'NavToBar',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Check for bar drinks
        smach.StateMachine.add('CheckBarDrinks',
                               CheckForBarDrinksState(action_dict, 
                                                      global_store),
                               transitions={'IDENTIFIED':'SetNavToLivingRoom'})
        
        # Set nav goal to living room
        func = lambda : None # TODO: Needs to be sorted out
        smach.StateMachine.add('SetNavToLivingRoom',
                               SetNavGoalState(action_dict, global_store, func),
                               transitions={'SUCCESS':'NavToLivingRoom'})
        
        # Navigate to living room
        smach.StateMachine.add('NavToLivingRoom',
                               NavigateState(action_dict, global_store),
                               transitions={'SUCCESS':'FindPerson',
                                            'FAILURE':'NavToLivingRoom',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Find someone to take an order from
        smach.StateMachine.add('FindPerson',
                               FindPersonState(action_dict, global_store),
                               transitions={'PERSON_FOUND':'TakeName',
                                            'NO_PEOPLE':'TASK_SUCCESS'})
        
        # Get someones name
        question = "Hi, I'm Bam Bam, can I ask your name?"
        smach.StateMachine.add('TakeName',
                               SpeakAndListenState(action_dict,
                                                   global_store,
                                                   question,
                                                   NAMES,
                                                   [],
                                                   20),
                               transitions={'SUCCESS':'MemorisePerson',
                                            'FAILURE':'TakeName',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Memorise person
        smach.StateMachine.add('MemorisePerson',
                               MemorisePersonState(action_dict, global_store),
                               transitions={'SUCCESS':'TakeDrink',
                                           'FAILURE':'FindPerson'})
        
        # Take drinks order
        question = "What would you like to drink?"
        smach.StateMachine.add('TakeDrink',
                               SpeakAndListenState(action_dict,
                                                   global_store,
                                                   question,
                                                   DRINKS,
                                                   [],
                                                   20),
                                transitions={'SUCCESS':'CheckDrink',
                                             'FAILURE':'TakeDrink',
                                             'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Check the drink order
        smach.StateMachine.add('CheckDrink',
                               CheckDrinkState(action_dict, global_store),
                               transitions={'DRINK_FINE':'OnMyWay',
                                            'DRINK_NOT_AVAILABLE':'BadDrink'})
        
        # Say that robot is going to get the drink
        phrase = "OK! I'll be right back with your drink!"
        smach.StateMachine.add('OnMyWay',
                               SpeakState(action_dict, global_store, phrase),
                               transitions={'SUCCESS':'SetBarForOrder',
                                            'FAILURE':'SetBarForOrder'})
        
        # Say that the drink is unavailable
        phrase = "Sorry, that drink isn't available!"
        smach.StateMachine.add('BadDrink',
                               SpeakState(action_dict, global_store, phrase),
                               transitions={'SUCCESS':'TakeDrink',
                                            'FAILURE':'TakeDrink'})
        
        # Set nav goal to bar/drink
        func = lambda: go_to_bar(action_dict)
        smach.StateMachine.add('SetBarForOrder',
                               SetNavGoalState(action_dict, global_store, func),
                               transitions={'SUCCESS':'NavToBarForOrder'})
        
        # Navigate to bar/drink
        smach.StateMachine.add('NavToBarForOrder',
                               NavigateState(action_dict, global_store),
                               transitions={'SUCCESS':'GrabDrink',
                                            'FAILURE':'NavToBarForOrder',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Try and grab the drink
        smach.StateMachine.add('GrabDrink',
                               PickUpObjectState(action_dict, global_store),
                               transitions={'SUCCESS':'SetNavToPerson',
                                            'FAILURE':'AskForHelp'})

        # Ask For Help
        question = ("I can't pick up this drink! Can you help me bartender? " +
                   "If you can, tell me when you are ready to hand me the " +
                   "drink.")
        smach.StateMachine.add('AskForHelp',
                               SpeakAndListenState(action_dict,
                                                   global_store,
                                                   question,
                                                   READY,
                                                   [],
                                                   30),
                               transitions={'SUCCESS':'HandoverDrink',
                                            'FAILURE':'AskForHelp',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Handover Drink
        smach.StateMachine.add('HandoverDrink',
                               ReceiveObjectFromOperatorState(action_dict,
                                                              global_store),
                               transitions={'SUCCESS':'SetNavToPerson',
                                            'FAILURE':'AskForHelp'})
        
        # Set navigation goal back to person
        func = lambda : go_to_last_person(action_dict, global_store)
        smach.StateMachine.add('SetNavToPerson',
                               SetNavGoalState(action_dict, global_store, func),
                               transitions={'SUCCESS':'NavToPerson'})
        
        # Navigate to human
        smach.StateMachine.add('NavToPerson',
                               NavigateState(action_dict, global_store),
                               transitions={'SUCCESS':'DrinkArrival',
                                            'FAILURE':'NavToPerson',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})

        # Tell person of arrival
        question = ("Hi! I'm back! And I have your drink. Please tell me " +
                    "when you are ready for me to hand it to you.")
        smach.StateMachine.add('DrinkArrival',
                               SpeakAndListenState(action_dict,
                                                   global_store,
                                                   question,
                                                   READY,
                                                   [],
                                                   20),
                               transitions={'SUCCESS':'HandoverDrinkToGuest',
                                            'FAILURE':'DrinkArrival',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Give drink to guest
        smach.StateMachine.add('HandoverDrinkToGuest',
                               HandoverObjectToOperatorState(action_dict,
                                                             global_store),
                               transitions={'SUCCESS':'ThankGuest',
                                            'FAILURE':'DrinkArrival'})
        
        # Thank guest
        phrase = ("I hope you enjoy your drink! " +
                 "Rememeber to always drink responsibly.")
        smach.StateMachine.add('ThankGuest',
                               SpeakState(action_dict, global_store, phrase),
                               transitions={'SUCCESS':'FindPerson',
                                            'FAILURE':'FindPerson'})
                                        
    
    return sm

if __name__ == '__main__':
    action_dict = create_stage_1_clients(7)
    sm = create_state_machine(action_dict)
    sm.execute()