#!/usr/bin/env python3
""" Code for the Receptionist task.

This file contains the state machine and other code specific to the
Receptionist task.

Author: Charlie Street
Owner: Charlie Street
"""

import rospy
import smach
import actionlib

from orion_task_level_planning.state_machines.src.state_machines.reusable_states_deprecated import * # pylint: disable=unused-wildcard-import
from set_up_clients import create_stage_1_clients
from orion_actions.msg import SOMObservation, Relation


class DetectDoorKnockState(ActionServiceState):
    """ A state to detect a door knock in the vicinity of the robot. """

    def __init__(self, action_dict, global_store, timeout):
        outcomes = ['DETECTED', 'TIMEOUT']
        self.timeout = timeout
        super(DetectDoorKnockState, self).__init__(action_dict=action_dict,
                                                   global_store=global_store,
                                                   outcomes=outcomes)

    def execute(self, userdata):
        # TODO: Fill in!
        # Listen for a door knock, and return if heard. Otherwise timeout.
        pass


class LookForOldestGuestState(ActionServiceState):
    """ A state to look for the oldest guest in the robots vicinity. """

    def __init__(self, action_dict, global_store):
        outcomes = ['SUCCESS', 'FAILURE']
        super(LookForOldestGuestState, self).__init__(action_dict=action_dict,
                                                      global_store=global_store,
                                                      outcomes=outcomes)

    def execute(self, userdata):
        # TODO: Fill in!
        # Look in robots vicinity for oldest guest and set nav to them
        # Note there may only be one guest
        pass


class UpdateWithDrinkState(ActionServiceState):
    """ Adds the drink information to the last person met. """
    def __init__(self, action_dict, global_store):
        outcomes = ['SUCCESS']
        super(UpdateWithDrinkState, self).__init__(action_dict=action_dict,
                                                   global_store=global_store,
                                                   outcomes=outcomes)

    def execute(self, userdata):

        drink = ""
        for avail_drink in DRINKS:
            if avail_drink in self.global_store['last_response']:
                drink = avail_drink
                break

        # Get last person memorised
        last_person = self.global_store['people_found'][-1]

        obj1 = SOMObservation()
        obj1.obj_id = last_person
        obj1.drink = drink

        self.action_dict['SOMObserve'](obj1)
        return self._outcomes[0]



class IntroduceGuestState(ActionServiceState):
    """ Introduces a guest to other guests. """
    def __init__(self, action_dict, global_store):
        outcomes = ['SUCCESS']
        super(IntroduceGuestState, self).__init__(action_dict=action_dict,
                                                  global_store=global_store,
                                                  outcomes=outcomes)

    def execute(self, userdata):
        index = self.global_store['next_to_introduce']
        self.global_store['next_to_introduce'] += 1

        person_id = self.global_store['people_found'][index]
        person = self.action_dict['SOMLookup'](person_id)
        name = person.name
        drink = person.drink

        sentence = ''
        # If new guest
        if index == len(self.global_store['people_found'] - 1): 
            sentence = ('Everyone, this is ' + name + ' and they like ' + 
                       'to drink ' + drink + '.')
        else:
            last_person_id = self.global_store['people_found'][-1]
            last_person = self.action_dict['SOMLookup'](last_person_id)  
            last_name = last_person.name
            sentence = (last_name + ', this is ' + name + ' and they like ' +
                       'to drink ' + drink)
        
        speak_goal = SpeakGoal()
        speak_goal.sentence = sentence
        self.action_dict['Speak'].send_goal(speak_goal)
        self.action_dict['Speak'].wait_for_result()

        return self._outcomes[0]


class DecideSeatingPlanState(ActionServiceState):
    """ Updates seating plan and speaks if people should swap around """
    def __init__(self, action_dict, global_store):
        outcomes = ['SUCCESS']
        super(DecideSeatingPlanState, self).__init__(action_dict=action_dict,
                                                     global_store=global_store,
                                                     outcomes=outcomes)

    def execute(self, userdata):
        youngest_on_sofa = None
        youngest_on_sofa_name = None
        youngest_age = 150

        for person_id in self.global_store['on_sofa'][0:-1]:
            person = self.action_dict['SOMLookup'](person_id)
            if person.age != 0 and person.age < youngest_age:
                youngest_age = person.age
                youngest_on_sofa = person_id
                youngest_on_sofa_name = person.name

        sentence = ''
        if youngest_on_sofa == None:
            sentence = 'Please make yourself feel at home.'
        else: # Compare to new person's age
            new_person_id = self.global_store['people_found'][-1]
            new_person = self.action_dict['SOMLookup'](new_person_id)
            if new_person.age < youngest_age:
                sentence = 'Please make yourself feel at home.'
            else:
                sentence = (youngest_on_sofa_name + ', would you kindly let ' +
                           new_person.name + 'sit on the sofa as they are ' +
                           'the elder person here?')
                self.global_store['on_sofa'].append(new_person_id)
                index = self.global_store['on_sofa'].index(youngest_on_sofa)
                del self.global_store['on_sofa'][index]
                self.global_store['not_on_sofa'].append(youngest_on_sofa)
            
        speak_goal = SpeakGoal()
        speak_goal.sentence = sentence
        self.action_dict['Speak'].send_goal(speak_goal)
        self.action_dict['Speak'].wait_for_result()

        return self._outcomes[0]


class FindPersonState(ActionServiceState):
    """ A state to determine who to introduce the next. """

    def __init__(self, action_dict, global_store):
        outcomes = ['PERSON_FOUND', 'NO_PEOPLE_LEFT']
        super(FindPersonState, self).__init__(action_dict=action_dict,
                                              global_store=global_store,
                                              outcomes=outcomes)
        
    def execute(self, userdata):
        if (self.global_store['next_to_introduce'] >= 
            len(self.global_store['people_found'])):
            self.global_store['next_to_introduce'] = 0
            return self._outcomes[1]
        else:
            index = self.global_store['next_to_introduce']
            person_id = self.global_store['people_found'][index]
            person = self.action_dict['SOMLookup'](person_id)
            name = person.name
            self.global_store['point_at'] = name


def go_to_door(action_dict):
    """ Door for welcoming guests is POI for this task. """
    obj1 = SOMObservation()
    obj1.type = 'receptionist_point_of_interest'

    return get_location_of_object(action_dict, obj1, 
                                  Relation(), SOMObservation())


def go_to_sofa(action_dict):
    """ Get location of sofa in living room. """
    obj1 = SOMObservation()
    obj1.type = 'sofa'
    obj1.room_name = 'living_room'

    return get_location_of_object(action_dict, obj1, 
                                  Relation(), SOMObservation())


def create_state_machine(action_dict):
    """ File creates and returns state machine for receptionist task. """

    # Get john's information
    obj = SOMObservation()
    obj.name = 'john'
    matches = action_dict['SOMQuery'](obj, Relation(), SOMObservation())
    john_id = matches[0].obj_id

    # Initialise global store
    global_store = {}
    global_store['on_sofa'] = [john_id]
    global_store['people_found'] = [john_id]
    global_store['not_on_sofa'] = []
    global_store['next_to_introduce'] = 0

    # Create the state machine
    sm = smach.StateMachine(outcomes=['TASK_SUCCESS', 'TASK_FAILURE'])

    with sm:
        
        # Start talking
        phrase = "Hi, I'm Bam Bam, and I'm here to be a receptionist!"
        smach.StateMachine.add('StartTalking',
                               SpeakState(action_dict, global_store, phrase),
                               transitions={'SUCCESS':'DetectDoorKnock',
                                            'FAILURE':'DetectDoorKnock'})
        
        # Detect a door knock
        smach.StateMachine.add('DetectDoorKnock',
                               DetectDoorKnockState(action_dict, 
                                                    global_store,
                                                    30),
                               transitions={'DETECTED':'SetNavToDoor',
                                            'TIMEOUT':'SpeakNoDoor'})
                                        
        # Announce no knock heard
        phrase = ("I haven't heard a door knock, but I have a suspicion there "+
                  "might be some people waiting for me.")
        smach.StateMachine.add('SpeakNoDoor',
                               SpeakState(action_dict, global_store, phrase),
                               transitions={'SUCCESS':'SetNavToDoor',
                                            'FAILURE':'SetNavToDoor'})
        
        # Set nav goal to door
        func = lambda: go_to_door(action_dict)
        smach.StateMachine.add('SetNavToDoor',
                               SetNavGoalState(action_dict, global_store, func),
                               transitions={'SUCCESS':'NavToDoor'})
        
        # Navigate to the door
        smach.StateMachine.add('NavToDoor',
                               NavigateState(action_dict, global_store),
                               transitions={'SUCCESS':'CheckAndOpenDoor',
                                            'FAILURE':'NavToDoor',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Check and open the door
        smach.StateMachine.add('CheckAndOpenDoor',
                               CheckAndOpenDoorState(action_dict, global_store),
                               transitions={'SUCCESS':'LookForOldestGuest',
                                            'FAILURE':'AskForDoorHelp'})
        
        # Ask for help opening the door
        question = ("I'm having trouble opening this door. Could someone " + 
                    "please help me open it and let me know when they have?")
        smach.StateMachine.add('AskForDoorHelp',
                               SpeakAndListenState(action_dict,
                                                   global_store,
                                                   question,
                                                   ['door is open', 'I have'],
                                                   [],
                                                   20),
                               transitions={'SUCCESS':'CheckDoorOpen',
                                            'FAILURE':'AskForDoorHelp',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Check door is now open
        smach.StateMachine.add('CheckDoorOpen',
                               CheckDoorIsOpenState(action_dict, global_store),
                               transitions={'SUCCESS':'LookForOldestGuest',
                                            'FAILURE':'AskForDoorHelp'})
        
        # Look for oldest guest (both could arrive at once)
        smach.StateMachine.add('LookForOldestGuest',
                               LookForOldestGuestState(action_dict, 
                                                       global_store),
                               transitions={'SUCCESS':'NavToGuest',
                                            'FAILURE':'TASK_FAILURE'})
        
        # Navigate to the guest
        smach.StateMachine.add('NavToGuest',
                               NavigateState(action_dict, global_store),
                               transitions={'SUCCESS':'AskName',
                                            'FAILURE':'NavToGuest',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        

        # Ask the guests name
        question = ("Hi, I'm Bam Bam, nice to meet you. What's your name?")
        smach.StateMachine.add('AskName',
                               SpeakAndListenState(action_dict,
                                                   global_store,
                                                   question,
                                                   NAMES,
                                                   [],
                                                   30),
                               transitions={'SUCCESS':'MemorisePerson',
                                            'FAILURE':'AskName',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Memorise the person
        smach.StateMachine.add('MemorisePerson',
                                MemorisePersonState(action_dict, global_store),
                                transitions={'SUCCESS':'AskDrink',
                                             'FAILURE':'TASK_FAILURE'})

        # Ask their preferred drink
        question = ("Cool! And what is your preferred drink?")
        smach.StateMachine.add('AskDrink',
                               SpeakAndListenState(action_dict,
                                                   global_store,
                                                   question,
                                                   DRINKS,
                                                   [],
                                                   20),
                               transitions={'SUCCESS':'UpdateWithDrink',
                                            'FAILURE':'AskDrink',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Update with drink
        smach.StateMachine.add('UpdateWithDrink',
                               UpdateWithDrinkState(action_dict, global_store),
                               transitions={'SUCCESS':'SpeakFollowMe'})
        
        # Start person following robot
        phrase = ("Sweet! Please follow me and I'll introduce you to the " + 
                  "other guests.")
        smach.StateMachine.add('SpeakFollowMe',
                               SpeakState(action_dict, global_store, func),
                               transitions={'SUCCESS':'SetNavToSofa',
                                            'FAILURE':'SpeakFollowMe'})
        
        # Set nav goal to sofa in living room
        func = lambda : go_to_sofa(action_dict)
        smach.StateMachine.add('SetNavToSofa',
                               SetNavGoalState(action_dict, global_store, func),
                               transitions={'SUCCESS':'NavToSofa'})
        
        # Navigate to the sofa
        smach.StateMachine.add('NavToSofa',
                               NavigateState(action_dict, global_store),
                               transitions={'SUCCESS':'FindPersonToIntroduce',
                                            'FAILURE':'NavToSofa',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        

        # Find the next person to introduce
        smach.StateMachine.add('FindPersonToIntroduce',
                               FindPersonState(action_dict, global_store),
                               transitions={'PERSON_FOUND':'PointToGuest',
                                            'NO_PEOPLE_LEFT':
                                            'DecideSeatingPlan'})
                    
        # Point to guest
        smach.StateMachine.add('PointToGuest',
                               PointToObjectState(action_dict, global_store),
                               transitions={'SUCCESS':'IntroduceGuest',
                                            'FAILURE':'FindPersonToIntroduce'})
        
        # Introduce guest
        smach.StateMachine.add('IntroduceGuest',
                               IntroduceGuestState(action_dict, global_store),
                               transitions={'SUCCESS':'FindPersonToIntroduce'})
        
        # Update the seating plan
        smach.StateMachine.add('DecideSeatingPlan',
                               DecideSeatingPlanState(action_dict, 
                                                      global_store),
                               transitions={'SUCCESS':'SetNavToDoor'})
        
        
        
    
    return sm


if __name__ == '__main__':
    action_dict = create_stage_1_clients(6)
    sm = create_state_machine(action_dict)
    sm.execute()

