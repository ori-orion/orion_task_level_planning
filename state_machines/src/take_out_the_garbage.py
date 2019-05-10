#!/usr/bin/env python
""" File for Take Out The Garbage Task.

This file contains the state machine and states specific to the 
Take Out The Garbage

Author: Charlie Street
Owner: Charlie Street
"""

import rospy
import smach
import actionlib

from reusable_states import * # pylint: disable=unused-wildcard-import
from set_up_clients import create_stage_1_clients
from orion_actions.msg import SOMObservation, Relation


class FindNearestBinState(ActionServiceState):
    """ State updates the global store information with the new bin."""

    def __init__(self, action_dict, global_store):
        outcomes=['BIN_FOUND', 'NO_BINS_LEFT']
        super(FindNearestBinState, self).__init__(action_dict=action_dict,
                                                  global_store=global_store,
                                                  outcomes=outcomes)
    
    def execute(self, userdata):
        obj1 = SOMObservation()
        obj1.type = 'bin'
        rel = Relation()
        obj2 = SOMObservation()

        matches = self.action_dict['SOMQuery'](obj1, rel, obj2)

        if len(matches) == 0:
            return self._outcomes[1]
        
        for match in matches:
            if match.obj1.obj_id not in self.global_store['bins_taken_out']:
                pose = match.obj1.pose_estimate.most_likely_pose

                x = pose.position.x
                y = pose.position.y

                quat = [pose.orientation.x, pose.orientation.y, 
                        pose.orientation.z, pose.orientation.w]
                
                (_, _, yaw) = euler_from_quaternion(quat)

                theta = yaw

                self.global_store['current_bin'] = match.obj1.obj_id
                self.global_store['nav_location'] = (x, y, theta)
                return self._outcomes[0]

        if self.global_store['current_bin'] == None:
            return self._outcomes[1]


class OpenBinState(ActionServiceState):
    """ State for opening a bin lid."""

    def __init__(self, action_dict, global_store):
        outcomes = ['SUCCESS', 'FAILURE']
        super(OpenBinState, self).__init__(action_dict=action_dict, 
                                           global_store=global_store, 
                                           outcomes=outcomes)

    def execute(self, userdata):
        bin_lid_goal = OpenBinLidGoal()
        self.action_dict['OpenBinLid'].send_goal(bin_lid_goal)
        self.action_dict['OpenBinLid'].wait_for_result()

        result = self.action_dict['OpenBinLid'].get_result().result
        self.global_store['pick_up'] = 'garbage bag'
        if result:
            return self._outcomes[0]
        else:
            return self._outcomes[1]


class SetCollectZoneState(ActionServiceState):
    """ State that sets the collection zone as the navigation destination. """

    def __init__(self, action_dict, global_store):
        outcomes=['SUCCESS']
        super(SetCollectZoneState, self).__init__(action_dict=action_dict,
                                                  global_store=global_store,
                                                  outcomes=outcomes)
    
    def execute(self, userdata):
        obj1 = SOMObservation()
        rel = Relation()
        obj2 = SOMObservation()

        obj1.type = 'take_out_the_garbage_point_of_interest'

        loc = get_location_of_object(self.action_dict, obj1, rel, obj2)

        self.global_store['nav_location'] = loc


class UpdateBinInfoState(ActionServiceState):
    """ State that updates our knowledge about the bin collection """

    def __init__(self, action_dict, global_store):
        outcomes=['SUCCESS']
        super(UpdateBinInfoState, self).__init__(action_dict=action_dict,
                                                 global_store=global_store,
                                                 outcomes=outcomes)
    
    def execute(self, userdata):
        current_bin = self.global_store['current_bin']
        self.global_store['bins_taken_out'].append(current_bin)
        self.global_store['current_bin'] = None
        return self._outcomes[0]


def create_state_machine(action_dict):
    """ Creates the state machine to be used for the task.

    Args:
        action_dict: The dictionary from names to action clients for the task
    
    Returns:
        sm: The state machine to be executed
    """

    # Initialise global store
    global_store = {}
    global_store['bins_taken_out'] = []
    global_store['current_bin'] = None


    # Create the state machine
    sm = smach.StateMachine(outcomes=['TASK_SUCCESS', 'TASK_FAILURE'])

    with sm:

        # Check if door is open
        smach.StateMachine.add('IsDoorOpen', 
                               CheckDoorIsOpenState(action_dict, global_store),
                               transitions={'OPEN':'FindNearestBin',
                                            'CLOSED':'IsDoorOpen'})

        # Find the nearest Bin
        smach.StateMachine.add('FindNearestBin',
                               FindNearestBinState(action_dict, global_store),
                               transitions={'BIN_FOUND':'NavToBin',
                                            'NO_BINS_LEFT':'TASK_SUCCESS'})
        
        # Navigate to the nearest bin
        smach.StateMachine.add('NavToBin',
                               NavigateState(action_dict, global_store),
                               transitions={'SUCCESS':'GrabLid',
                                            'FAILURE': 'NavToBin',
                                            'REPEAT_FAILURE': 'TASK_FAILURE'})
        
        # Grab the lid of the bin
        smach.StateMachine.add('GrabLid',
                               OpenBinState(action_dict, global_store),
                               transitions={'SUCCESS':'PlaceLidOnFloor',
                                            'FAILURE':'AskForHelpWithLid'})


        # Ask for help with lid
        question = ("Please could someone remove the bin lid for me and " + 
                   "let me know?")
        smach.StateMachine.add('AskForHelpWithLid',
                               SpeakAndListenState(action_dict,
                                                   global_store,
                                                   question,
                                                   ['I will help'],
                                                   [],
                                                   20),
                                transitions={'SUCCESS':'GrabGarbage',
                                             'FAILURE':'AskForHelpWithLid',
                                             'REPEAT_FAILURE':'TASK_FAILURE'})
        

        # Place the bin lid on the floor
        smach.StateMachine.add('PlaceLidOnFloor',
                               PutObjectOnFloorState(action_dict, global_store),
                               transitions={'SUCCESS':'GrabGarbage',
                                            'FAILURE':'AskForHandover'})

        # Ask for help putting bin lid on floor
        question = ("Please could someone take the bin lid from me and " +
                   "let me know?")
        smach.StateMachine.add('AskForHandover',
                               SpeakAndListenState(action_dict,
                                                   global_store,
                                                   question,
                                                   ['I will'],
                                                   [],
                                                   20),
                                transitions={'SUCCESS':'HandoverLid',
                                             'FAILURE':'AskForHandover',
                                             'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Handover lid
        smach.StateMachine.add('HandoverLid',
                               HandoverObjectToOperatorState(action_dict, 
                                                             global_store),
                               transitions={'SUCCESS':'GrabGarbage',
                                            'FAILURE':'AskForHandover'})
        
        # Grab the Garbage
        smach.StateMachine.add('GrabGrabage',
                               PickUpObjectState(action_dict, global_store),
                               transitions={'SUCCESS':'SetCollectZone',
                                            'FAILURE':'AskForHelpWithGarbage'})
        
        # Ask for help with garbage
        question = ("Could someone pass me the garbage bag please and " +
                   "let me know once you have?")
        smach.StateMachine.add('AskForHelpWithGarbage',
                               SpeakAndListenState(action_dict,
                                                   global_store,
                                                   question,
                                                   ['I will'],
                                                   [],
                                                   timeout=30),
                               transitions={'SUCCESS':'HandoverGarbage',
                                            'FAILURE':'AskForHelpWithGarbage',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Recive garbage
        smach.StateMachine.add('HandoverGarbage',
                               ReceiveObjectFromOperatorState(action_dict,
                                                              global_store),
                               transitions={'SUCCESS':'SetCollectZone',
                                            'FAILURE':'AskForHelpWithGarbage'})
        
        # Set the collection zone as the nav point
        smach.StateMachine.add('SetCollectZone',
                               SetCollectZoneState(action_dict, global_store),
                               transitions={'SUCCESS':'NavToCollectZone'})
        
        # Navigate to the collection zone
        smach.StateMachine.add('NavToCollectZone',
                               NavigateState(action_dict, global_store),
                               transitions={'SUCCESS':'DropGarbage',
                                            'FAILURE':'NavToCollectZone',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Drop the garbage in the collection zone
        smach.StateMachine.add('DropGarbage',
                               PutObjectOnFloorState(action_dict, global_store),
                               transitions={'SUCCESS':'UpdateBinInfo',
                                            'FAILURE':'AskForHelpDropping'})
        
        # Ask for help dropping
        question = ("Will someone help me put this bag on the floor and let me "+
                   "know once they have?")
        smach.StateMachine.add('AskForHelpDropping',
                               SpeakAndListenState(action_dict,
                                                   global_store,
                                                   question,
                                                   ['I will'],
                                                   [],
                                                   20),
                               transitions={'SUCCESS':'GiveGarbage',
                                            'FAILURE':'AskForHelpDropping',
                                            'REPEAT_FAILURE':'TASK_FAILURE'})
        
        # Give garbage bag to person
        smach.StateMachine.add('GiveGarbage',
                               HandoverObjectToOperatorState(action_dict, 
                                                             global_store),
                               transitions={'SUCCESS':'UpdateBinInfo',
                                            'FAILURE':'AskForHelpDropping'})

        # Update bin info
        smach.StateMachine.add('UpdateBinInfo',
                               UpdateBinInfoState(action_dict, global_store),
                               transitions={'SUCCESS':'FindNearestBin'})

            
    return sm

if __name__ == '__main__':
    action_dict = create_stage_1_clients(10)
    sm = create_state_machine(action_dict)
    sm.execute()
