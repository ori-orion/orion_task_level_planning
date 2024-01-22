#!/usr/bin/env python3

import rospy
from actionlib import SimpleActionClient
import math
from typing import List, Union

import hsrb_interface
import hsrb_interface.geometry as geometry
from geometry_msgs.msg import Point, Pose
from state_machines.robocup_2024.put_away_the_groceries.common import find_navigation_goal, look_at_object, navigate_to_pose
hsrb_interface.robot.enable_interactive()

from state_machines.Reusable_States.utils import SmachBaseClass, SUCCESS, NAVIGATIONAL_FAILURE, MANIPULATION_FAILURE, distance_between_poses, \
                get_current_pose, NavigationalListener, MoveBaseGoal, MoveBaseAction, GoalStatus
from orion_actions.msg import SOMObject, PoseStamped, PickUpObjectGoal, PickUpObjectResult
from orion_actions.srv import NavigationalQuery, NavigationalQueryRequest, NavigationalQueryResponse, PickUpObjectAction


class PickUpObject(SmachBaseClass):

    DISTANCE_SAME_PLACE_THRESHOLD = 0.1;
    RETRY = 1;
    RETRY_STAYED_IN_SAME_PLACE = 2;
    SUCCESS = 3;

     # If the gripper is more closed than this, we will say it has not actually picked anything up.
    GRIPPER_DISTANCCE_THRESHOLD = 0.001;

    def __init__(self, execute_nav_commands:bool, max_num_failure_repetitions=4,
                 num_iterations_upon_failure=3,
                 wait_upon_completion=rospy.Duration(5)):
        SmachBaseClass.__init__(self, 
                                outcomes=[SUCCESS, MANIPULATION_FAILURE, NAVIGATIONAL_FAILURE], 
                                input_keys=["obj_to_pick_up"])
        self.execute_nav_commands = execute_nav_commands
        self.max_num_failure_repetitions = max_num_failure_repetitions
        self.num_iterations_upon_failure = num_iterations_upon_failure
        self.wait_upon_completion = wait_upon_completion
    
    
    def run_manipulation_comp(self, pick_up_goal: PickUpObjectGoal):
        self.pick_up_object_action_client.send_goal(pick_up_goal)
        self.pick_up_object_action_client.wait_for_result()

        result: PickUpObjectResult = self.pick_up_object_action_client.get_result()
        return result.result, result.failure_mode
    
    def pick_up_object(self, obj_to_pick_up: SOMObject):
        self.pick_up_object_action_client = SimpleActionClient('pick_up_object', PickUpObjectAction)
        self.pick_up_object_action_client.wait_for_server()

        pick_up_goal = PickUpObjectGoal();
        
        pick_up_goal.goal_tf = obj_to_pick_up.tf_name;
        pick_up_goal.publish_own_tf = False;

        for i in range(self.num_iterations_upon_failure):
            result, failure_mode = self.run_manipulation_comp(pick_up_goal=pick_up_goal);

            status = self.pick_up_object_action_client.get_state();
            print("status", status);
            print("Failure mode=", failure_mode)
            if result:
                gripper_distance = self.getGripperDistance();
                print("Gripper distance", gripper_distance);

                if gripper_distance > self.GRIPPER_DISTANCCE_THRESHOLD:
                    rospy.sleep(self.wait_upon_completion);
                    return True
                
            elif failure_mode == PickUpObjectResult.TF_NOT_FOUND or failure_mode == PickUpObjectResult.TF_TIMEOUT:
                rospy.loginfo("Tf error");
                pick_up_goal.publish_own_tf = True;
            elif failure_mode == PickUpObjectResult.GRASPING_FAILED or status == GoalStatus.ABORTED:
                rospy.loginfo("Grasping failed.");
                # return MANIPULATION_FAILURE;
            rospy.loginfo("Manipulation failed.");

        rospy.sleep(self.wait_upon_completion);
        return False


    def execute(self, userdata):
        obj_to_pick_up: SOMObject = userdata.obj_to_pick_up

        look_at_object(obj_to_pick_up, self.lookAtPoint)
        navigation_goal = find_navigation_goal(obj_to_pick_up)
        if navigation_goal is not None:
            is_object_reached = navigate_to_pose(navigation_goal, 
                                                 self.max_num_failure_repetitions,
                                                 self.execute_nav_commands)
            if not is_object_reached:
                return NAVIGATIONAL_FAILURE
            look_at_object(obj_to_pick_up, self.lookAtPoint)
        
        if not self.pick_up_object(obj_to_pick_up):
            return MANIPULATION_FAILURE
        
        return SUCCESS


            



