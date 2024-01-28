#!/usr/bin/env python3

import rospy
from actionlib import SimpleActionClient
import hsrb_interface
from actionlib_msgs.msg import GoalStatus

from orion_actions.msg import SOMObject, PickUpObjectGoal, PickUpObjectResult, PickUpObjectAction

from state_machines.robocup_2024.put_away_the_groceries.common import find_navigation_goal, look_at_object, navigate_to_pose
from state_machines.Reusable_States.include_all import NAVIGATIONAL_FAILURE, MANIPULATION_FAILURE, SmachBaseClass, SUCCESS

hsrb_interface.robot.enable_interactive()

class PickUpObject(SmachBaseClass):

    DISTANCE_SAME_PLACE_THRESHOLD = 0.1

     # If the gripper is more closed than this, we will say it has not actually picked anything up.
    GRIPPER_DISTANCE_THRESHOLD = 0.001

    def __init__(self, execute_nav_commands:bool, max_num_failure_repetitions=4,
                 num_iterations_upon_failure=3,
                 wait_upon_completion=rospy.Duration(5)):
        """
        Pick up the selected object from the table.
        Possible outcomes:
        - SUCCESS: the object has been picked up
        - MANIPULATION_FAILURE: the manipulation failed
        - NAVIGATIONAL_FAILURE: failed to move close to the object

        Input keys: 
        - `target_obj`: the object we want to pick up

        Parameters:
        - `execute_nav_commands`: whether to execute the navigation or skip it
        - `max_num_failure_repetitions`: maximum number of failures for the 
            navigation
        - `num_iterations_upon_failure`: num tries for manipulation
        - `wait_upon_completion`: how long to wait after picking up the object

        """
        SmachBaseClass.__init__(self, 
                                outcomes=[SUCCESS, MANIPULATION_FAILURE, NAVIGATIONAL_FAILURE], 
                                input_keys=["target_obj"])
        self.execute_nav_commands = execute_nav_commands
        self.max_num_failure_repetitions = max_num_failure_repetitions
        self.num_iterations_upon_failure = num_iterations_upon_failure
        self.wait_upon_completion = wait_upon_completion

    
    def pick_up_object(self, obj_to_pick_up: SOMObject):
        """
        Pick up the object from the table.

        - `obj_to_pick_up`: the object to pick up

        Returns `True` if the object was picked up, `False` otherwise.
        """
        pick_up_object_action_client = SimpleActionClient('pick_up_object', PickUpObjectAction)
        pick_up_object_action_client.wait_for_server()

        pick_up_goal = PickUpObjectGoal()
        
        pick_up_goal.goal_tf = obj_to_pick_up.tf_name
        pick_up_goal.publish_own_tf = False

        for _ in range(self.num_iterations_upon_failure):
            pick_up_object_action_client.send_goal(pick_up_goal)
            pick_up_object_action_client.wait_for_result()

            result: PickUpObjectResult = pick_up_object_action_client.get_result() #type: ignore
            is_successful = result.result
            failure_mode = result.failure_mode

            status = pick_up_object_action_client.get_state()
            print(f"status: {status}")
            print(f"Failure mode = {failure_mode}")
            if is_successful:
                gripper_distance = self.getGripperDistance()
                print(f"Gripper distance: {gripper_distance}")

                if gripper_distance > self.GRIPPER_DISTANCE_THRESHOLD:
                    rospy.sleep(self.wait_upon_completion)
                    return True
                
            elif failure_mode == PickUpObjectResult.TF_NOT_FOUND or failure_mode == PickUpObjectResult.TF_TIMEOUT:
                rospy.loginfo("Tf error")
                pick_up_goal.publish_own_tf = True
            elif failure_mode == PickUpObjectResult.GRASPING_FAILED or status == GoalStatus.ABORTED:
                rospy.loginfo("Grasping failed.")
            rospy.loginfo("Manipulation failed.")

        rospy.sleep(self.wait_upon_completion)
        return False


    def execute(self, userdata):
        obj_to_pick_up: SOMObject = userdata.target_obj

        look_at_object(obj_to_pick_up, self.lookAtPoint)
        
        # Navigate close to the object to pick up, if needed
        navigation_goal = find_navigation_goal(obj_to_pick_up)
        if navigation_goal is not None:
            is_object_reached = navigate_to_pose(navigation_goal, 
                                                 self.max_num_failure_repetitions,
                                                 self.execute_nav_commands)
            if not is_object_reached:
                return NAVIGATIONAL_FAILURE
            look_at_object(obj_to_pick_up, self.lookAtPoint)
        
        # Pick up object
        if not self.pick_up_object(obj_to_pick_up):
            return MANIPULATION_FAILURE
        
        return SUCCESS


            



