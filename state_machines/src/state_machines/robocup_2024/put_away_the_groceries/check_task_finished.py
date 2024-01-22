#!/usr/bin/env python3

import rospy
from actionlib import SimpleActionClient
import math
from typing import List, Union

import hsrb_interface
import hsrb_interface.geometry as geometry
from geometry_msgs.msg import Point, Pose
hsrb_interface.robot.enable_interactive()

from state_machines.Reusable_States.utils import SmachBaseClass, SUCCESS, NAVIGATIONAL_FAILURE, MANIPULATION_FAILURE, distance_between_poses, \
                get_current_pose, NavigationalListener, MoveBaseGoal, MoveBaseAction, GoalStatus
from orion_actions.msg import SOMObject, PoseStamped, PickUpObjectGoal, PickUpObjectResult
from orion_actions.srv import NavigationalQuery, NavigationalQueryRequest, NavigationalQueryResponse, PickUpObjectAction


class CheckTaskFinished(SmachBaseClass):
    def __init__(self, num_objects_to_put_away: int):
        SmachBaseClass.__init__(self, 
                                outcomes=["task_finished", "continue"], 
                                input_keys=["num_objects_placed"],
                                output_keys=["num_objects_placed"])

        self.num_objects_to_put_away = num_objects_to_put_away
        
    def execute(self, userdata):
        userdata.num_objects_placed += 1
        if userdata.num_objects_placed < self.num_objects_to_put_away:
            return "continue"
        else:
            return "task_finished"