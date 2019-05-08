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
from orion_actions.srv import SOMClearDatabase

def create_stage_1_clients(task_number):
    """ Function returns the dictionary of all clients needed for task.

    This function returns a dictionary of names to clients.

    Args:
        task_number: The number of the task in stage 1.
    
    Returns:
        action_dict: The dictionary from service names to clients for them.
    """

    action_dict = {}

    # Start with semantic mapping stuff
    rospy.wait_for_service('SOM_clear_database')
    action_dict['SOMClearDatabase'] = rospy.ServiceProxy('SOM_clear_database',
                                                         SOMClearDatabase)

    # Now do task specific stuff
    if task_number == 1:
        pass
    elif task_number == 2:
        pass
    elif task_number == 3:
        pass
    elif task_number == 4:
        pass
    elif task_number == 5:
        pass 
    elif task_number == 6:
        pass
    elif task_number == 7:
        pass
    elif task_number == 8:
        pass
    elif task_number == 9:
        pass
    elif task_number == 10:
        pass
    else:
        raise Exception("Invalid Task Number Passed In!")

    return action_dict


def create_stage_2_clients(task_number):
    """ Same as create_stage_1_clients but for stage 2. """
    return {}


def create_final_clients(task_number):
    """ Same as create_stage_1_clients but for the final. """
    return {}