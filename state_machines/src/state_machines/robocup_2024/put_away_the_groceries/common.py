#!/usr/bin/env python3

import rospy
from actionlib import SimpleActionClient
import math
from typing import Callable, List, Union

import hsrb_interface

hsrb_interface.robot.enable_interactive()

from state_machines.Reusable_States.utils import SmachBaseClass, SUCCESS, distance_between_poses, get_current_pose
from orion_actions.msg import SOMObject
from orion_actions.srv import SOMQueryObjectsRequest, SOMQueryObjects, SOMQueryObjectsResponse
from orion_spin.msg import SpinAction, SpinGoal
from tmc_msgs.msg import TalkRequestAction, TalkRequestGoal, Voice
from orion_actions.msg import SOMObject, PoseStamped, PickUpObjectGoal, PickUpObjectResult
from orion_actions.srv import NavigationalQuery, NavigationalQueryRequest, NavigationalQueryResponse, PickUpObjectAction
from geometry_msgs.msg import Point, Pose
from state_machines.Reusable_States.utils import SmachBaseClass, SUCCESS, NAVIGATIONAL_FAILURE, MANIPULATION_FAILURE, distance_between_poses, \
                get_current_pose, NavigationalListener, MoveBaseGoal, MoveBaseAction, GoalStatus


def look_around(spin_offset: float=0):
        """Look around the table"""
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
        by num of observations (descending)."""

        rospy.wait_for_service('/som/objects/basic_query')
        object_query_srv = rospy.ServiceProxy('/som/objects/basic_query', 
                                              SOMQueryObjects)
        result: SOMQueryObjectsResponse = object_query_srv(query)
        output: List[SOMObject] = result.returns

        # Filter out objects that are too distant
        if distance_filter != 0:
            current_pose = get_current_pose()
            output_carry = []
            for element in output:
                if distance_between_poses(current_pose, element.obj_position) < distance_filter:
                    output_carry.append(element)
            output = output_carry

        rospy.loginfo(f"\t\t {len(output)} entities found matching the query.")
        for element in output:
            print(f"\t({element.class_}, {element.category}, {element.num_observations})")

        return output

def look_at_object(target_obj: SOMObject, lookAtPoint: Callable[[hsrb_interface.geometry.Vector3, str], None]):
    """Look towards object. """
    pose = target_obj.obj_position

    point_look_at = hsrb_interface.geometry.Vector3(pose.position.x, pose.position.y, pose.position.z)
    
    # NOTE: A very 'elegant' solution (that really needs to be changed at some point)!
    try:
        lookAtPoint(point_look_at, reference_frame="map")
    except:
        point_look_at = hsrb_interface.geometry.Vector3(pose.position.x, pose.position.y, 1.2)
        try:
            lookAtPoint(point_look_at, reference_frame="map")
        except:
            lookAtPoint(hsrb_interface.geometry.Vector3(1, 0, 0.8), reference_frame="base_link")
        rospy.logwarn("Error with gaze_point directly at the human.")

    rospy.sleep(rospy.Duration(0.5))


def find_navigation_goal(target_obj: SOMObject, distance_from_pose: float = 0.9) -> Union[Pose, None]:
    """Get naviagation goal to get close to the object."""
    rospy.wait_for_service("tlp/get_nav_goal")
    nav_goal_getter = rospy.ServiceProxy('tlp/get_nav_goal', NavigationalQuery)

    robot_pose: PoseStamped = rospy.wait_for_message('/global_pose', PoseStamped);
    robot_pos = robot_pose.position
    target_pos = target_obj.obj_position.position
    distance_from_object = math.sqrt( (robot_pos.x-target_pos.x)**2 + (robot_pos.y-target_pos.y)**2 )
    if distance_from_object < 1.3:
        return None

    print("Userdata.pose")
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

    print("Positing a nav goal of", nav_goal_getter_resp.navigate_to)
    return nav_goal_getter_resp.navigate_to

DISTANCE_SAME_PLACE_THRESHOLD = 0.1
RETRY = 1
RETRY_STAYED_IN_SAME_PLACE = 2
SUCCESS = 3

def checkSamePlaceLoop(target_pose, navigate_action_client: SimpleActionClient) -> int:
        prev_current_pose = get_current_pose();
        while True:
            navigate_action_client.wait_for_result(rospy.Duration(1));
            print("Checking goal dist", end="\t");
            if distance_between_poses(prev_current_pose, target_pose) < DISTANCE_SAME_PLACE_THRESHOLD:
                return SUCCESS;
            current_pose = get_current_pose();
            print("Checking if moved", end="\t");
            if distance_between_poses(current_pose, prev_current_pose) < DISTANCE_SAME_PLACE_THRESHOLD:
                print("Not moved");
                return RETRY_STAYED_IN_SAME_PLACE;
            print("Moved");
            prev_current_pose = current_pose;

def execute_navigation(goal: MoveBaseGoal, 
                       target_pose:Pose, 
                       initial_pose:Pose, 
                       execute_nav_commands: bool,
                       navigate_action_client: SimpleActionClient) -> int:
        """
        Executes the navigation action. 
        """
        if not execute_nav_commands:
            return SUCCESS
        
        navigate_action_client.send_goal(goal);

        # We want to be able to check to see if the robot has moved or not
        # (to check to see if path planning has failed.)
        
        rospy.loginfo("\t\tChecking to see if we've stayed in the same place for too long.");

        current_pose = get_current_pose();
        rospy.loginfo("\t\tdistance_between_poses(current_pose, target_pose)=" + str(distance_between_poses(current_pose, target_pose)));
        rospy.loginfo("\t\tdistance_between_poses(current_pose, initial_pose)=" + str(distance_between_poses(current_pose, initial_pose)));
        loop_result = checkSamePlaceLoop(target_pose);
        if loop_result == RETRY_STAYED_IN_SAME_PLACE:
            navigate_action_client.cancel_all_goals();
            return RETRY_STAYED_IN_SAME_PLACE;
        
        navigate_action_client.wait_for_result();
        status = navigate_action_client.get_state();
        navigate_action_client.cancel_all_goals()
        if status == GoalStatus.SUCCEEDED:
            return SUCCESS
        else:
            return RETRY

def navigate_to_pose(target_pose: Pose, 
                     max_num_failure_repetitions: int,
                     execute_nav_commands: bool) -> bool:
        """Navigate to the object"""
        initial_pose = get_current_pose();

        if DISTANCE_SAME_PLACE_THRESHOLD < distance_between_poses(initial_pose, target_pose) and execute_nav_commands == False:
            return True
        
        nav_listener = NavigationalListener();

        # Navigating without top nav
        rospy.loginfo('Navigating without top nav')
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = target_pose
        # rospy.loginfo(goal.target_pose.pose)

        navigate_action_client = SimpleActionClient('move_base/move',  MoveBaseAction)
        rospy.loginfo('\t\tWaiting for move_base/move.')
        navigate_action_client.wait_for_server()

        i = 0
        while i < max_num_failure_repetitions:
            if i > 0:
                nav_listener.getOccupancyMap();
                new_goal, old_goal_fine = nav_listener.findClosestUnoccupiedPoint(target_pose.position);
                if old_goal_fine:
                    print("Old goal was fine");
                else:
                    print("Goal blocked. Recalculating goal.");
                target_pose.position = new_goal;
            
            rospy.loginfo('\t\tSending nav goal.');
            status = execute_navigation(goal, 
                                        target_pose, 
                                        initial_pose, 
                                        execute_nav_commands,
                                        navigate_action_client)
            rospy.loginfo('status = ' + str(status))
            if status == SUCCESS:
                return True;
            else:
                i += 1;
            
        return False;