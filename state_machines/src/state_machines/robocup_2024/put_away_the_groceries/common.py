#!/usr/bin/env python3

import rospy
from enum import Enum
import math
from typing import Callable, List, Union

import hsrb_interface
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from geometry_msgs.msg import Pose, PoseStamped

from orion_actions.msg import SOMObject
from orion_actions.srv import SOMQueryObjectsRequest, SOMQueryObjects, SOMQueryObjectsResponse, \
                            NavigationalQuery, NavigationalQueryRequest, NavigationalQueryResponse
from orion_spin.msg import SpinAction, SpinGoal
from state_machines.Reusable_States.utils import distance_between_poses, get_current_pose
from state_machines.Reusable_States.navigational_states import NavigationalListener

hsrb_interface.robot.enable_interactive()

def look_around(spin_offset: float=0):
        """Move the head to look around
        
        - `spin_offset`: the spin offset to be used
        """
        client = SimpleActionClient('spin', SpinAction)
        client.wait_for_server()
        goal = SpinGoal()
        goal.only_look_forwards = True
        goal.height_to_look_at = 0.7
        goal.spin_offset = spin_offset

        client.send_goal(goal)
        client.wait_for_result()

def query_objects_from_som(query: SOMQueryObjectsRequest, distance_filter: float = 0):
        """Send query to the SOM, filter out objects too distant or with not 
        enough observations, and sort the remaining objects by category and 
        by num of observations (descending).
        
        - `query`: the query to be sent
        - `distance filter`: filter out objects more distant than this value 
            (if 0, all objects are kept)
        """

        rospy.wait_for_service('/som/objects/basic_query')
        object_query_srv = rospy.ServiceProxy('/som/objects/basic_query', 
                                              SOMQueryObjects)
        result: SOMQueryObjectsResponse = object_query_srv(query)
        objects_returned: List[SOMObject] = result.returns #type: ignore 

        # Filter out objects that are too distant
        if distance_filter != 0:
            current_pose = get_current_pose()
            objects_returned = [obj for obj in objects_returned if 
                                distance_between_poses(current_pose, obj.obj_position) < distance_filter]

        rospy.loginfo(f"\t\t {len(objects_returned)} entities found matching the query.")
        for element in objects_returned:
            print(f"\t({element.class_}, {element.category}, {element.num_observations})")

        return objects_returned

def look_at_object(target_obj: SOMObject, lookAtPoint: Callable):
    """Look towards the given object. Wait 0.5 seconds before returning.
    
    - `target_obj`: the object towards which we want to look
    - `lookAtPoint`: function self.lookAtPoint from SmachBaseClass (passed here for convenience) 
    """
    pose = target_obj.obj_position

    point_look_at = hsrb_interface.geometry.Vector3(pose.position.x, pose.position.y, pose.position.z)
    
    # NOTE: A very 'elegant' solution (that really needs to be changed at some point)!
    try:
        lookAtPoint(point_look_at, "map")
    except:
        point_look_at = hsrb_interface.geometry.Vector3(pose.position.x, pose.position.y, 1.2)
        try:
            lookAtPoint(point_look_at, reference_frame="map")
        except:
            lookAtPoint(hsrb_interface.geometry.Vector3(1, 0, 0.8), reference_frame="base_link")
        rospy.logwarn("Error with gaze_point directly at the human.")

    rospy.sleep(rospy.Duration(secs=0, nsecs=500_000_000))


def find_navigation_goal(target_obj: SOMObject, distance_from_pose: float = 0.9) -> Union[Pose, None]:
    """
    Get naviagation goal to get close to the object.
    
    - `target_obj`: the object toward which we want to move
    - `distance_from_pose`: we want to reach a position that is this far away from 
        the target object.

    Returns the target Pose, or `None` if we are already close enough.
    """
    rospy.wait_for_service("tlp/get_nav_goal")
    nav_goal_getter = rospy.ServiceProxy('tlp/get_nav_goal', NavigationalQuery)

    robot_pose = rospy.wait_for_message('/global_pose', PoseStamped).pose #type:ignore
    robot_pos = robot_pose.position
    target_pos = target_obj.obj_position.position
    distance_from_object = math.sqrt( (robot_pos.x-target_pos.x)**2 + (robot_pos.y-target_pos.y)**2 )
    if distance_from_object < 1.3:
        return None

    print("Navigation target pose:")
    print(target_obj.obj_position)
    nav_goal_getter_req = NavigationalQueryRequest()
    nav_goal_getter_req.navigating_within_reach_of = target_obj.obj_position.position
    nav_goal_getter_req.distance_from_obj = distance_from_pose
    nav_goal_getter_req.current_pose = get_current_pose().position
    print()
    print(nav_goal_getter_req)
    print()

    nav_goal_getter_resp: NavigationalQueryResponse = nav_goal_getter(nav_goal_getter_req)
    nav_goal_getter_resp.navigate_to.position.z = 0
    nav_goal_getter_resp.navigate_to.orientation = robot_pose.orientation

    print("Navigation goal created: ", nav_goal_getter_resp.navigate_to)
    return nav_goal_getter_resp.navigate_to

DISTANCE_SAME_PLACE_THRESHOLD = 0.1

class NavigationResult(Enum): 
    RETRY = 1
    RETRY_STAYED_IN_SAME_PLACE = 2
    SUCCESS = 3

def checkSamePlaceLoop(target_pose: Pose, navigate_action_client: SimpleActionClient) -> NavigationResult:
    """
    Run while loop and check every second until the robot has either reached the 
    target, or has stopped moving.

    - `target_pose`: the pose we are trying to reach
    - `navigate_action_client`: the ActionClient used to send the navigation goal

    Returns `SUCCESS` if the target is reached, or `RETRY_STAYED_IN_SAME_PLACE` 
    if the robot has stopped moving.
    """
    prev_current_pose = get_current_pose()
    while True:
        navigate_action_client.wait_for_result(rospy.Duration(1))
        print("------------------------------")
        print("Checking distance from goal...")
        if distance_between_poses(prev_current_pose, target_pose) < DISTANCE_SAME_PLACE_THRESHOLD:
            print("Goal reached")
            return NavigationResult.SUCCESS
        current_pose = get_current_pose()
        print("Checking if moved...")
        if distance_between_poses(current_pose, prev_current_pose) < DISTANCE_SAME_PLACE_THRESHOLD:
            print("Not moved")
            return NavigationResult.RETRY_STAYED_IN_SAME_PLACE
        print("Moved")
        prev_current_pose = current_pose

def execute_navigation(goal: MoveBaseGoal, 
                       target_pose:Pose, 
                       initial_pose:Pose, 
                       execute_nav_commands: bool,
                       navigate_action_client: SimpleActionClient) -> NavigationResult:
        """
        Executes the navigation action. 

        - `goal`: the goal to be sent to the action client
        - `target_pose`: the pose we are trying to reach
        - `initial_pose`: the initial pose of the robot
        - `execute_nav_commands`: whether to navigate or not
        - `navigation_action_client`: the client to be used for sending actions

        Returns the result of the navigation.
        """
        if not execute_nav_commands:
            return NavigationResult.SUCCESS
        
        navigate_action_client.send_goal(goal)

        # We want to be able to check to see if the robot has moved or not
        # (to check to see if path planning has failed.)
        
        rospy.loginfo("\t\tChecking to see if we've stayed in the same place for too long.")

        loop_result = checkSamePlaceLoop(target_pose, navigate_action_client)

        current_pose = get_current_pose()
        rospy.loginfo(f"\t\tdistance to target = {distance_between_poses(current_pose, target_pose)}")
        rospy.loginfo(f"\t\tdistance from initial pose = {distance_between_poses(current_pose, initial_pose)}")

        if loop_result == NavigationResult.RETRY_STAYED_IN_SAME_PLACE:
            navigate_action_client.cancel_all_goals()
            return NavigationResult.RETRY_STAYED_IN_SAME_PLACE
        
        navigate_action_client.wait_for_result()
        status = navigate_action_client.get_state()
        navigate_action_client.cancel_all_goals()
        if status == GoalStatus.SUCCEEDED:
            return NavigationResult.SUCCESS
        else:
            return NavigationResult.RETRY

def navigate_to_pose(target_pose: Pose, 
                     max_num_failure_repetitions: int,
                     execute_nav_commands: bool) -> bool:
        """Navigate to the given pose, trying multiple times if failing.
        
        - `target_pose`: the position we are trying to reach
        - `max_num_failure_repetitions`: how many times to try the navigation
        - `execute_nav_commands`: whether to navigate or not

        Returns True if the navigation was successful, False otherwise.
        """
        initial_pose = get_current_pose()

        if DISTANCE_SAME_PLACE_THRESHOLD < distance_between_poses(initial_pose, target_pose) and not execute_nav_commands:
            return True
        
        nav_listener = NavigationalListener()

        # Navigating without top nav
        rospy.loginfo('Navigating without top nav')
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = target_pose

        navigate_action_client = SimpleActionClient('move_base/move',  MoveBaseAction)
        rospy.loginfo('\t\tWaiting for move_base/move.')
        navigate_action_client.wait_for_server()

        for i in range(max_num_failure_repetitions):
            if i > 0:
                nav_listener.getOccupancyMap()
                new_goal, old_goal_fine = nav_listener.findClosestUnoccupiedPoint(target_pose.position)
                if old_goal_fine:
                    print("Old goal was fine")
                else:
                    print("Goal blocked. Recalculating goal.")
                target_pose.position = new_goal
            
            rospy.loginfo('\t\tSending nav goal.')
            status = execute_navigation(goal, 
                                        target_pose, 
                                        initial_pose, 
                                        execute_nav_commands,
                                        navigate_action_client)
            rospy.loginfo(f'status = {status}')
            if status == NavigationResult.SUCCESS:
                return True
        return False


MAST_JOINT_MAX = 0.69
MAST_JOINT_MIN = 0
def compute_safe_mast_height(target_height: float):
    """Make sure that the given height for the mast is valid (i.e. between the minimum
    and maximum heights).
    
    - `target_height`: the height of the mast

    Returns either the input height, or the closest valid height.
    """
    mast_height = min(target_height, MAST_JOINT_MAX)
    mast_height = max(target_height, MAST_JOINT_MIN)
    
    return mast_height