#!/usr/bin/env python
""" File for setting up action clients for state machines.

This file contains functions for setting up the action clients for
each of the RoboCup tasks, in the form of a dictionary from action names
to clients.

Author: Charlie Street
Owner: Charlie Street

"""

import rospy
import actionlib
from orion_actions.srv import * # pylint: disable=unused-wildcard-import
from orion_actions.msg import * # pylint: disable=unused-wildcard-import
from move_base_msgs.msg import MoveBaseAction
from tmc_msgs.msg import TalkRequestAction

def create_stage_1_clients(task_number):
    """ Function returns the dictionary of all clients needed for task.

    This function returns a dictionary of names to clients.

    Args:
        task_number: The number of the task in stage 1.
    
    Returns:
        action_dict: The dictionary from service names to clients for them.
    """

    action_dict = {}
    rospy.loginfo('Setting up SOM service proxies...')
    # Start with semantic mapping stuff
    rospy.wait_for_service('som/clear_database')
    action_dict['SOMClearDatabase'] = rospy.ServiceProxy('som/clear_database',
                                                         SOMClearDatabase)
    rospy.wait_for_service('som/delete')
    action_dict['SOMDelete'] = rospy.ServiceProxy('som/delete', SOMDelete)
    rospy.wait_for_service('som/get_all_objects')
    action_dict['SOMGetAllObjects'] = rospy.ServiceProxy('som/get_all_objects',
                                                         SOMGetAllObjects)
    rospy.wait_for_service('som/lookup')
    action_dict['SOMLookup'] = rospy.ServiceProxy('som/lookup', SOMLookup)
    rospy.wait_for_service('som/observe')
    action_dict['SOMObserve'] = rospy.ServiceProxy('som/observe', SOMObserve)
    rospy.wait_for_service('som/query')
    action_dict['SOMQuery'] = rospy.ServiceProxy('som/query', SOMQuery)
    rospy.loginfo('SOM service proxies set up...')
    # Now add common action clients
    rospy.loginfo('Setting up Move Base client...')
    action_dict['Navigate'] = actionlib.SimpleActionClient('/move_base/move', 
                                                           MoveBaseAction)
    action_dict['Navigate'].wait_for_server() # TODO: Change if necessary
    rospy.loginfo('Move Base ready...')
    rospy.loginfo('Can I speak?...')
    action_dict['Speak'] = actionlib.SimpleActionClient('talk_request_action', 
                                                        TalkRequestAction)
    action_dict['Speak'].wait_for_server()
    rospy.loginfo('I can speak!...')
    rospy.loginfo('Can I listen?...')
    action_dict['SpeakAndListen'] = \
        actionlib.SimpleActionClient('speak_and_listen', SpeakAndListenAction)
    action_dict['SpeakAndListen'].wait_for_server()
    rospy.loginfo('I can listen!...')

    # Now do task specific stuff
    if task_number == 1: # Carry My Luggage
        rospy.loginfo('Can I pick up objects?...')
        action_dict['PickUpObject'] = \
            actionlib.SimpleActionClient('pick_up_object', PickUpObjectAction)
        action_dict['PickUpObject'].wait_for_server()
        rospy.loginfo('I can pick up objects!...')
        rospy.loginfo('Can I get pointed objects?...')
        """action_dict['GetPointedObject'] = \
            actionlib.SimpleActionClient('get_pointed_object', 
                                         GetPointedObjectAction)
        action_dict['GetPointedObject'].wait_for_server()""" # TODO: Bring back later
        rospy.loginfo('I can get pointed objects!')
        rospy.loginfo('Can I receive objects?...')
        action_dict['ReceiveObjectFromOperator'] = \
            actionlib.SimpleActionClient('receive_object_from_operator',
                                         ReceiveObjectFromOperatorAction)
        action_dict['ReceiveObjectFromOperator'].wait_for_server()
        rospy.loginfo('I can receive objects!...')
        rospy.loginfo('Can I follow you to the end of the earth?...')
        action_dict['Follow'] = actionlib.SimpleActionClient('follow', 
                                                             FollowAction)
        action_dict['Follow'].wait_for_server()
        rospy.loginfo('I can!')
        rospy.loginfo('Can I listen for a hothothotword?...')
        action_dict['HotwordListen'] = \
            actionlib.SimpleActionClient('hotword_listen', HotwordListenAction)
        action_dict['HotwordListen'].wait_for_server()
        rospy.loginfo('I can!...')
        rospy.loginfo('Can I give you objects?...')
        action_dict['GiveObjectToOperator'] = \
            actionlib.SimpleActionClient('give_object_to_operator', 
                                         GiveObjectToOperatorAction)
        action_dict['GiveObjectToOperator'].wait_for_server()
        rospy.loginfo('I can!...')

    elif task_number == 2: # Clean Up
        action_dict['PickUpObject'] = \
            actionlib.SimpleActionClient('pick_up_object', PickUpObjectAction)
        action_dict['PickUpObject'].wait_for_server()
        action_dict['ReceiveObjectFromOperator'] = \
            actionlib.SimpleActionClient('receive_object_from_operator',
                                         ReceiveObjectFromOperatorAction)
        action_dict['ReceiveObjectFromOperator'].wait_for_server()
        action_dict['GiveObjectToOperator'] = \
            actionlib.SimpleActionClient('give_object_to_operator', 
                                         GiveObjectToOperatorAction)
        action_dict['GiveObjectToOperator'].wait_for_server()
        action_dict['IsDoorOpen'] = actionlib.SimpleActionClient('is_door_open',
                                                               IsDoorOpenAction)
        action_dict['IsDoorOpen'].wait_for_server()
        action_dict['OpenDoor'] = actionlib.SimpleActionClient('open_door',
                                                               OpenDoorAction)
        action_dict['OpenDoor'].wait_for_server()
        action_dict['PutObjectOnSurface'] = \
            actionlib.SimpleActionClient('put_object_on_surface',
                                         PutObjectOnSurfaceAction)
        action_dict['PutObjectOnSurface'].wait_for_server()

    elif task_number == 3: # Farewell
        action_dict['IsDoorOpen'] = actionlib.SimpleActionClient('is_door_open',
                                                               IsDoorOpenAction)
        action_dict['IsDoorOpen'].wait_for_server()
        action_dict['PickUpObject'] = \
            actionlib.SimpleActionClient('pick_up_object', PickUpObjectAction)
        action_dict['PickUpObject'].wait_for_server()
        action_dict['ReceiveObjectFromOperator'] = \
            actionlib.SimpleActionClient('receive_object_from_operator',
                                         ReceiveObjectFromOperatorAction)
        action_dict['ReceiveObjectFromOperator'].wait_for_server()
        action_dict['GiveObjectToOperator'] = \
            actionlib.SimpleActionClient('give_object_to_operator', 
                                         GiveObjectToOperatorAction)
        action_dict['GiveObjectToOperator'].wait_for_server()

    elif task_number == 4: # Find my mates
        action_dict['IsDoorOpen'] = actionlib.SimpleActionClient('is_door_open',
                                                               IsDoorOpenAction)
        action_dict['IsDoorOpen'].wait_for_server()

    elif task_number == 5: # GPSR
        action_dict['IsDoorOpen'] = actionlib.SimpleActionClient('is_door_open',
                                                               IsDoorOpenAction)
        action_dict['IsDoorOpen'].wait_for_server()

    elif task_number == 6: # Receptionist
        action_dict['IsDoorOpen'] = actionlib.SimpleActionClient('is_door_open',
                                                               IsDoorOpenAction)
        action_dict['IsDoorOpen'].wait_for_server()
        action_dict['OpenDoor'] = actionlib.SimpleActionClient('open_door',
                                                               OpenDoorAction)
        action_dict['OpenDoor'].wait_for_server()
        action_dict['PointToObject'] = \
            actionlib.SimpleActionClient('point_to_object',PointToObjectAction)
        action_dict['PointToObject'].wait_for_server()

    elif task_number == 7: # Serving Drinks
        action_dict['PickUpObject'] = \
            actionlib.SimpleActionClient('pick_up_object', PickUpObjectAction)
        action_dict['PickUpObject'].wait_for_server()
        action_dict['ReceiveObjectFromOperator'] = \
            actionlib.SimpleActionClient('receive_object_from_operator',
                                         ReceiveObjectFromOperatorAction)
        action_dict['ReceiveObjectFromOperator'].wait_for_server()
        action_dict['GiveObjectToOperator'] = \
            actionlib.SimpleActionClient('give_object_to_operator', 
                                         GiveObjectToOperatorAction)
        action_dict['GiveObjectToOperator'].wait_for_server()
        action_dict['CheckForBarDrinks'] = \
            actionlib.SimpleActionClient('check_for_bar_drinks',
                                         CheckForBarDrinksAction)
        action_dict['CheckForBarDrinks'].wait_for_server()

    elif task_number == 8: # Serve The Breakfast
        action_dict['IsDoorOpen'] = actionlib.SimpleActionClient('is_door_open',
                                                               IsDoorOpenAction)
        action_dict['IsDoorOpen'].wait_for_server()
        action_dict['ReceiveObjectFromOperator'] = \
            actionlib.SimpleActionClient('receive_object_from_operator',
                                         ReceiveObjectFromOperatorAction)
        action_dict['ReceiveObjectFromOperator'].wait_for_server()
        action_dict['GiveObjectToOperator'] = \
            actionlib.SimpleActionClient('give_object_to_operator', 
                                         GiveObjectToOperatorAction)
        action_dict['GiveObjectToOperator'].wait_for_server()
        action_dict['PickUpObject'] = \
            actionlib.SimpleActionClient('pick_up_object', PickUpObjectAction)
        action_dict['PickUpObject'].wait_for_server()
        action_dict['PourInto'] = actionlib.SimpleActionClient('pour_into',
                                                               PourIntoAction)
        action_dict['PourInto'].wait_for_server()
        action_dict['PutObjectOnSurface'] = \
            actionlib.SimpleActionClient('put_object_on_surface',
                                         PutObjectOnSurfaceAction)
        action_dict['PutObjectOnSurface'].wait_for_server()
        action_dict['PlaceObjectRelative'] = \
            actionlib.SimpleActionClient('place_object_relative',
                                         PlaceObjectRelativeAction)
        action_dict['PlaceObjectRelative'].wait_for_server()

    elif task_number == 9: # Storing Groceries
        action_dict['ReceiveObjectFromOperator'] = \
            actionlib.SimpleActionClient('receive_object_from_operator',
                                         ReceiveObjectFromOperatorAction)
        action_dict['ReceiveObjectFromOperator'].wait_for_server()
        action_dict['GiveObjectToOperator'] = \
            actionlib.SimpleActionClient('give_object_to_operator', 
                                         GiveObjectToOperatorAction)
        action_dict['GiveObjectToOperator'].wait_for_server()
        action_dict['PlaceObjectRelative'] = \
            actionlib.SimpleActionClient('place_object_relative',
                                         PlaceObjectRelativeAction)
        action_dict['PlaceObjectRelative'].wait_for_server()
        action_dict['PickUpObject'] = \
            actionlib.SimpleActionClient('pick_up_object', PickUpObjectAction)
        action_dict['PickUpObject'].wait_for_server()
        action_dict['IsDoorOpen'] = actionlib.SimpleActionClient('is_door_open',
                                                               IsDoorOpenAction)
        action_dict['IsDoorOpen'].wait_for_server()
        action_dict['OpenDrawer'] = actionlib.SimpleActionClient('open_drawer',
                                                               OpenDrawerAction)
        action_dict['OpenDrawer'].wait_for_server()

    elif task_number == 10: # Take Out The Garbage
        action_dict['ReceiveObjectFromOperator'] = \
            actionlib.SimpleActionClient('receive_object_from_operator',
                                         ReceiveObjectFromOperatorAction)
        action_dict['ReceiveObjectFromOperator'].wait_for_server()
        action_dict['GiveObjectToOperator'] = \
            actionlib.SimpleActionClient('give_object_to_operator', 
                                         GiveObjectToOperatorAction)
        action_dict['GiveObjectToOperator'].wait_for_server()
        action_dict['PickUpObject'] = \
            actionlib.SimpleActionClient('pick_up_object', PickUpObjectAction)
        action_dict['PickUpObject'].wait_for_server()
        action_dict['IsDoorOpen'] = actionlib.SimpleActionClient('is_door_open',
                                                               IsDoorOpenAction)
        action_dict['IsDoorOpen'].wait_for_server()
        action_dict['PutObjectOnFloor'] = \
            actionlib.SimpleActionClient('put_object_on_floor', 
                                         PutObjectOnFloorAction)
        action_dict['PutObjectOnFloor'].wait_for_server()
        action_dict['OpenBinLid'] = actionlib.SimpleActionClient('open_bin_lid',
                                                               OpenBinLidAction)
        action_dict['OpenBinLid'].wait_for_server()

    else:
        raise Exception("Invalid Task Number Passed In!")

    return action_dict


def create_stage_2_clients(task_number):
    """ Same as create_stage_1_clients but for stage 2. """
    if task_number == 1: # Clean the table
        # TODO: Sort out
        return {}
    elif task_number == 4:
        # TODO: Sort out
        return {}
    elif task_number == 5:
        # TODO: Sort out
        return {}
    elif task_number == 7:
        # TODO: Sort out
        return {}
    else:
        raise Exception("Invalid Task Number Passed In!")



def create_final_clients(task_number):
    """ Same as create_stage_1_clients but for the final. """
    return {}