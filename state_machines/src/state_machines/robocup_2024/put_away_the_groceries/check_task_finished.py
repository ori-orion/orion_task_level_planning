#!/usr/bin/env python3

import rospy

from state_machines.Reusable_States.include_all import SmachBaseClass


TASK_FINISHED = "task_finished"
CONTINUE_TASK = "continue"

class CheckTaskFinished(SmachBaseClass):
    def __init__(self, num_objects_to_put_away: int):
        """
        Increase the number of objects stored, and check if we have stored enough
        objects to end the task.

        Possible outcomes:
        - `TASK_FINISHED`: the task has been completed
        - `CONTINUE_TASK`: need to put away more objects

        Input keys:
        - `num_objects_placed`: the number of objects already placed

        Output keys:
        - `num_objects_placed`: the number of objects placed

        Parameters:
        - `num_objects_to_put_away`: the total number of objects to be put away
        """
        SmachBaseClass.__init__(self, 
                                outcomes=[TASK_FINISHED, CONTINUE_TASK], 
                                input_keys=["num_objects_placed"],
                                output_keys=["num_objects_placed"])

        self.num_objects_to_put_away = num_objects_to_put_away
        
    def execute(self, userdata):
        userdata.num_objects_placed += 1
        if userdata.num_objects_placed < self.num_objects_to_put_away:
            return CONTINUE_TASK
        else:
            return TASK_FINISHED