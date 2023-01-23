#!/usr/bin/env python3

from state_machines.Reusable_States.utils import *;

import smach;

from orion_actions.msg import *;
from orion_actions.srv import *;

import rospy;

from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
import actionlib
from actionlib_msgs.msg import GoalStatus

from geometry_msgs.msg import Pose, PoseStamped, Point

from ori_topological_navigation_msgs.msg import TraverseToNodeAction, TraverseToNodeGoal, PoseOverlay, TraverseToNodeResult

import math;

import nav_msgs.msg;

class NavigationalListener:

    def __init__(self, listen_to:str = "/base_path_planner/inflated_static_obstacle_map"):
        rospy.Subscriber(
            listen_to,
            nav_msgs.msg.GridCells,
            self.occupancyMapCallback);

        self.most_recent_map = None;

    def occupancyMapCallback(self, data:nav_msgs.msg.GridCells):
        print("Nav callback:")
        print("\theader      = ", data.header);
        print("\tcell_width  = ", data.cell_width);
        print("\tcell_height = ", data.cell_height);
        print("\tlen(cells)  = ", len(data.cells));

        self.most_recent_map = data;

    def isPointOccluded(self, point:Point) -> bool:
        if self.most_recent_map == None:
            return True;

        w = self.most_recent_map.cell_width;
        h = self.most_recent_map.cell_height;
        
        for point_cell in self.most_recent_map.cells:
            point_cell:Point;

            # Assuming a 2D map.
            if (point_cell.x - w < point.x < point_cell.x + w) and (point_cell.y - h < point.y < point_cell.y + h):
                return True;
        
        return False;




class GetRobotLocationState(smach.State):
    """ Smach state for getting the robot's current location.

    This state will get the robot's current location and return it in the userdata dict.
    """

    def __init__(self):
        smach.State.__init__(self,
                                outcomes = ['stored'],
                                output_keys=['robot_location'])

    def execute(self, userdata):
        # Wait for one message on topic and then set as the location
        pose = get_current_pose();
        userdata.robot_location = pose;
        rospy.loginfo(pose)
        return 'stored'


#region navigation states
class SimpleNavigateState(smach.State):
    """ State for navigating directly to a location on the map.

    This state is given a pose and navigates there.
    If the robot stays in the same place for more than a second, then assume failed nav.

    input_keys:
        pose: pose for the robot to navigate to
        number_of_failures: an external counter keeping track of the cumulative failure count (incremented in this state upon failure & reset upon success and repreat failure)
        failure_threshold: the number of cumulative failures required to return the repeat_failure outcome
    output_keys:
        number_of_failures: the updated failure counter upon state exit
    """

    DISTANCE_SAME_PLACE_THRESHOLD = 0.1;

    def __init__(self, execute_nav_commands:bool):
        smach.State.__init__(
            self,
            outcomes=[SUCCESS, FAILURE, REPEAT_FAILURE],
            input_keys=['pose', 'number_of_failures', 'failure_threshold'],
            output_keys=['number_of_failures']);

        self.execute_nav_commands = execute_nav_commands;

    def repeat_failure_infrastructure(self, userdata) -> str:
        userdata.number_of_failures += 1
        if userdata.number_of_failures >= userdata.failure_threshold:
             # reset number of failures because we've already triggered the repeat failure outcome
            userdata.number_of_failures = 0
            return REPEAT_FAILURE
        return FAILURE

    def execute(self, userdata):
        if self.execute_nav_commands == False:
            return SUCCESS;

        target_pose:Pose = userdata.pose;
        initial_pose = get_current_pose();

        # Navigating without top nav
        rospy.loginfo('Navigating without top nav')
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = target_pose;
        # rospy.loginfo(goal.target_pose.pose)

        navigate_action_client = actionlib.SimpleActionClient('move_base/move',  MoveBaseAction)
        rospy.loginfo('\t\tWaiting for move_base/move.');
        navigate_action_client.wait_for_server();
        rospy.loginfo('\t\tSending nav goal.');
        navigate_action_client.send_goal(goal);

        # We want to be able to check to see if the robot has moved or not
        # (to check to see if path planning has failed.)
        navigate_action_client.wait_for_result(rospy.Duration(2));
        rospy.loginfo("\t\tChecking to see if we've stayed in the same place for too long.");

        current_pose = get_current_pose();
        rospy.loginfo("\t\tdistance_between_poses(current_pose, target_pose)=" + str(distance_between_poses(current_pose, target_pose)));
        rospy.loginfo("\t\tdistance_between_poses(current_pose, initial_pose)=" + str(distance_between_poses(current_pose, initial_pose)));
        if (distance_between_poses(current_pose, target_pose) > self.DISTANCE_SAME_PLACE_THRESHOLD and 
            distance_between_poses(current_pose, initial_pose) < self.DISTANCE_SAME_PLACE_THRESHOLD):

            rospy.logerr('\t\tStayed in the same place for too long => FAILURE.')
            rospy.loginfo('status = ' + str(navigate_action_client.get_state()))
            return self.repeat_failure_infrastructure(userdata);

        navigate_action_client.wait_for_result();
        status = navigate_action_client.get_state();
        navigate_action_client.cancel_all_goals()
        rospy.loginfo('status = ' + str(status))
        if status == GoalStatus.SUCCEEDED:
            userdata.number_of_failures = 0;
            return SUCCESS
        else:
            return self.repeat_failure_infrastructure(userdata);

#TODO - make a topological localisation node
#       Subscribe to /topological_location - topic type from ori_topological_navigation_msgs : TopologicalLocation to get closest_node_id and current_node_id strings (empty string if not at any)
#       watif for one message and then set variable.


class TopologicalNavigateState(smach.State):
    """ State for navigating along the topological map.

    This state is given a topological map ID and navigates there.

    input_keys:
        node_id: (string) the topological node for the robot to navigate to
        number_of_failures: an external counter keeping track of the cumulative failure count (incremented in this state upon failure & reset upon success and repreat failure)
        failure_threshold: the number of cumulative failures required to return the repeat_failure outcome
    output_keys:
        number_of_failures: the updated failure counter upon state exit
        prev_node_nav_to    The node we just navigated to.
    """
    
    # If the robot is staying in the same location while the robot is trying to go to a node,
    # then we should preempt and retry. The preemption check is done at 1 second intervals. 
    # This is then the maximum distance it can have travelled in that time for us to preempt
    # the goal.  
    MAX_DISTANCE_TOPO_HALTED = 0.05;

    def __init__(self, execute_nav_commands, stop_repeat_navigation:bool = False):
        """
        stop_repeat_navigation:bool  - If we have just navigated to a node, we may get asked to go there in the near 
            future. In some cases, there is no point in this. (Say you are searching a room and have found something, 
            and then nav to the room node again). 
            If True then it will prevent us from navigating to the same node twice in a row.
            Otherwise it will navigate to the node as per normal.
        """
        self.stop_repeat_navigation = stop_repeat_navigation;
        self.execute_nav_commands = execute_nav_commands;

        smach.State.__init__(self,
                                outcomes=[SUCCESS, FAILURE, REPEAT_FAILURE],
                                input_keys=['node_id', 'number_of_failures', 'failure_threshold', 'prev_node_nav_to'],
                                output_keys=['number_of_failures', 'prev_node_nav_to']);

    def get_robot_pose(self) -> Pose:
        robot_pose:PoseStamped = rospy.wait_for_message('/global_pose', PoseStamped);
        return robot_pose.pose;

    def execute(self, userdata):
        if self.stop_repeat_navigation==True and userdata.node_id == userdata.prev_node_nav_to:
            rospy.loginfo("Repeat navigation to the same node prevented.")
            return SUCCESS

        if self.execute_nav_commands == False:
            return SUCCESS;

        # Navigating with top nav
        rospy.loginfo('Navigating with top nav to node "{}"'.format(userdata.node_id))

        # create action goal and call action server
        goal = TraverseToNodeGoal(node_id=userdata.node_id)

        topological_navigate_action_client = actionlib.SimpleActionClient('traverse_to_node',  TraverseToNodeAction)
        topological_navigate_action_client.wait_for_server()
        topological_navigate_action_client.send_goal(goal)

        # old_robot_pose = self.get_robot_pose();
        # while topological_navigate_action_client.get_state() == actionlib_msgs.msg.GoalStatus.ACTIVE:
        #     rospy.sleep(1);
        #     new_robot_pose = self.get_robot_pose();
        #     dist_between_poses = distance_between_poses(old_robot_pose, new_robot_pose);
        #     if dist_between_poses < TopologicalNavigateState.MAX_DISTANCE_TOPO_HALTED:
        #         rospy.loginfo("Preempting and rerunning topological nav goal.")                
        #         topological_navigate_action_client.cancel_all_goals();
        #         topological_navigate_action_client.send_goal(goal);
        
        topological_navigate_action_client.wait_for_result()
        result:TraverseToNodeResult = topological_navigate_action_client.get_result()

        # rospy.loginfo('result = ' + str(result.success))

        # Process action result
        #   Note: result.success returns True if node_id was reached
        if result.success:
            userdata.number_of_failures = 0;
            userdata.prev_node_nav_to = userdata.node_id;
            return SUCCESS
        else:
            userdata.number_of_failures += 1
            if userdata.number_of_failures >= userdata.failure_threshold:
                 # reset number of failures because we've already triggered the repeat failure outcome
                userdata.number_of_failures = 0
                return REPEAT_FAILURE
            return FAILURE

# class GetClosestNodeState(smach.State):
#     """
#     Inputs:
#         goal_pose:Pose:     The pose we want to navigate to.
#     Outputs:
#         closest_node:str:   The node we will go via.
#     """
#     def __init__(self):
#         smach.State.__init__(self, 
#                                 outcomes=[SUCCESS],
#                                 input_keys=['goal_pose'],
#                                 output_keys=['closest_node']);

#     def execute(self, userdata):
#         goal_pose:Pose = userdata.goal_pose;
#         userdata.closest_node = get_closest_node(goal_pose.position);
#         return SUCCESS;

class NavigateDistanceFromGoalSafely(smach.State):
    """
    We want to be able to navigate to a human and sit 1m away from them without colliding into anything.

    DISTANCE_FROM_POSE gives the distance we want to sit from the target pose 'pose'.

    Uses the node defined within state_machines/src/cpp/navigation.cpp
    """

    DISTANCE_FROM_POSE = 0.6;

    def __init__(self):
        smach.State.__init__(
            self, 
            outcomes=[SUCCESS],
            input_keys=['pose'],
            output_keys=['nav_target']);

        # self._mb_client = actionlib.SimpleActionClient('move_base/move', MoveBaseAction)

    def get_robot_pose(self) -> Pose:
        robot_pose:PoseStamped = rospy.wait_for_message('/global_pose', PoseStamped);
        return robot_pose.pose;

    def execute(self, userdata):
        rospy.wait_for_service("tlp/get_nav_goal");
        nav_goal_getter = rospy.ServiceProxy('tlp/get_nav_goal', NavigationalQuery);

        print(userdata.pose);
        nav_goal_getter_req = NavigationalQueryRequest();
        nav_goal_getter_req.navigating_within_reach_of = userdata.pose.position;
        nav_goal_getter_req.distance_from_obj = self.DISTANCE_FROM_POSE;
        nav_goal_getter_req.current_pose = get_current_pose().position;
        print(nav_goal_getter_req);

        nav_goal_getter_resp:NavigationalQueryResponse = nav_goal_getter(nav_goal_getter_req);
        nav_goal_getter_resp.navigate_to.position.z = 0;

        userdata.nav_target = nav_goal_getter_resp.navigate_to;

        print("Positing a nav goal of", nav_goal_getter_resp.navigate_to);

        return SUCCESS;

        self._mb_client.wait_for_server();

        rospy.loginfo('Navigating without top nav')
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = nav_goal_getter_resp.navigate_to;
        rospy.loginfo(goal.target_pose.pose);
        
        self._mb_client.send_goal(goal);
        
        # Waiting for the result as a backup.
        self._mb_client.wait_for_result();

        return SUCCESS;

class OrientRobot(smach.State):
    """
    Orients the robot towards a goal point.

    Inputs:
        userdata.orient_towards:Pose
    """
    def __init__(self):
        smach.State.__init__(
            self, 
            outcomes=[SUCCESS, FAILURE],
            input_keys=['orient_towards']);
    
    def get_robot_pose(self) -> Pose:
        robot_pose:PoseStamped = rospy.wait_for_message('/global_pose', PoseStamped);
        return robot_pose.pose;

    def execute(self, userdata):
        orient_towards:Pose = userdata.orient_towards;
        current_position:Point = self.get_robot_pose().position;

        position_delta:Point = Point();
        print(type(orient_towards));
        print(type(current_position));
        position_delta.x = orient_towards.position.x - current_position.x;
        position_delta.y = orient_towards.position.y - current_position.y;
        length = get_point_magnitude(position_delta);
        position_delta.x /= length;
        position_delta.y /= length;
        print("Position delta length: {0}".format(get_point_magnitude(position_delta)));
        nav_to:Pose = Pose();
        nav_to.position = current_position;
        nav_to.orientation.w = position_delta.x;
        nav_to.orientation.z = position_delta.y;

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = nav_to;
        # rospy.loginfo(goal.target_pose.pose)

        navigate_action_client = actionlib.SimpleActionClient('move_base/move',  MoveBaseAction)
        rospy.loginfo('\t\tWaiting for move_base/move.');
        navigate_action_client.wait_for_server();
        rospy.loginfo('\t\tSending nav goal.');
        navigate_action_client.send_goal(goal);
        
        navigate_action_client.wait_for_result();
        status = navigate_action_client.get_state();
        navigate_action_client.cancel_all_goals()
        rospy.loginfo('status = ' + str(status))
        if status == GoalStatus.SUCCEEDED:
            # userdata.number_of_failures = 0;
            return SUCCESS;
        else:
            return FAILURE;
#endregion

class GetNextNavLoc(smach.State):
    """
    This is set up specifically for the find my mates task.
    So the overall goal here is to work out where to go to next.
    Do we want to go to the next search node or do we want to go to the next
    nearest human?
    Note that this is designed to be executed after GetNearestOperator.
    Inputs:
        closest_human:Human
            The closest human entry.
        robot_location:Pose
            The current location of the robot.
    Outputs:
        pose_to_nav_to:Pose|None
            The pose we want the robot to navigate to if we're aiming for a pose.
    """
    # Gives the distance the robot will come to a stop from the human.
    DISTANCE_FROM_HUMAN = 0.3;  #m

    def __init__(self):
        smach.State.__init__(self, outcomes=['nav_to_node', 'nav_to_pose', FAILURE],
                                input_keys=['closest_human', 'robot_location'],
                                output_keys=['pose_to_nav_to']);

    def execute(self, userdata):

        nav_to_human:bool = True;
        human_loc:Pose = userdata.closest_human.obj_position;
        robot_location:Pose = userdata.robot_location;

        # Proxy for actual next node location. (Need to work out type of this!)
        next_node_pos = Point();

        # [Logic for choosing between node and humans]
        # If the human is behind the robot, go to the human.
        # If the human is infront of the next node, go to the human.
        # If the human is behind the next node, go to the next node.

        # Behind can be determined by whether the dot product between two vectors (the one
        # going robot->node and robot->human) is negative.
        # Nearness can be determined by the distance itself.

        # Is the human behind the robot?
        robot_to_node:Point = Point();
        robot_to_node.x = next_node_pos.x - robot_location.position.x;
        robot_to_node.y = next_node_pos.y - robot_location.position.y;
        robot_to_node.z = next_node_pos.z - robot_location.position.z;

        robot_to_human:Point = Point();
        robot_to_human.x = human_loc.position.x - robot_location.position.x;
        robot_to_human.y = human_loc.position.y - robot_location.position.y;
        robot_to_human.z = human_loc.position.z - robot_location.position.z;

        # RobotToNode dot RobotToHuman...
        RTN_dot_RTH = robot_to_node.x*robot_to_human.x + robot_to_node.y*robot_to_human.y + robot_to_node.z*robot_to_human.z;
        if RTN_dot_RTH < 0:
            nav_to_human = True;
        else:
            robot_to_node_len = get_point_magnitude(robot_to_node);
            robot_to_human_len = get_point_magnitude(robot_to_human);
            if robot_to_node_len < robot_to_human_len:
                nav_to_human = False;
            else:
                nav_to_human = True;

        # NOTE: This puts the robot a certain distance away from the we are looking for.
        # Potentially can be abstracted out into its own smach state.
        if nav_to_human:
            pose_to_nav_to:Pose = Pose();
            vec_to_human = Point();
            vec_to_human.x = human_loc.position.x - robot_location.position.x;
            vec_to_human.y = human_loc.position.y - robot_location.position.y;
            vec_to_human.z = human_loc.position.z - robot_location.position.z;

            vec_to_human_len:float = get_point_magnitude(vec_to_human);

            vec_to_human.x *= GetNextNavLoc.DISTANCE_FROM_HUMAN / vec_to_human_len;
            vec_to_human.y *= GetNextNavLoc.DISTANCE_FROM_HUMAN / vec_to_human_len;
            vec_to_human.z *= GetNextNavLoc.DISTANCE_FROM_HUMAN / vec_to_human_len;

            pose_to_nav_to.position.x = human_loc.position.x - vec_to_human.x;
            pose_to_nav_to.position.y = human_loc.position.y - vec_to_human.y;
            pose_to_nav_to.position.z = human_loc.position.z - vec_to_human.z;

            userdata.pose_to_nav_to = pose_to_nav_to;

            return 'nav_to_pose';
        else:
            return 'nav_to_node';

class SetSafePoseFromObject(smach.State):
    """
    Adjust a pose in to be a fixed distance from the point of interest.
    Inputs:
        pose:Pose
    Outputs:
        pose:Pose
    """

    # Gives the distance the robot will come to a stop from the human.
    DISTANCE_FROM_POSE = 1;  #m

    def __init__(self):
        smach.State.__init__(self, outcomes=[SUCCESS],
                                input_keys=['pose', 'robot_location'],
                                output_keys=['pose_out']);

    def execute(self, userdata):
        print(userdata.keys);

        robot_location:Pose = userdata.robot_location;
        pose:Pose = userdata.pose;

        # [Logic for choosing between node and humans]
        # If the human is behind the robot, go to the human.
        # If the human is infront of the next node, go to the human.
        # If the human is behind the next node, go to the next node.

        # Behind can be determined by whether the dot product between two vectors (the one 
        # going robot->node and robot->human) is negative.
        # Nearness can be determined by the distance itself.

        vec_to_pose = Point();
        vec_to_pose.x = pose.position.x - robot_location.position.x;
        vec_to_pose.y = pose.position.y - robot_location.position.y;
        vec_to_pose.z = pose.position.z - robot_location.position.z;

        angle = math.atan2(vec_to_pose.y, vec_to_pose.x);

        vec_to_pose_len:float = get_point_magnitude(vec_to_pose);

        vec_to_pose.x *= SetSafePoseFromObject.DISTANCE_FROM_POSE / vec_to_pose_len;
        vec_to_pose.y *= SetSafePoseFromObject.DISTANCE_FROM_POSE / vec_to_pose_len;
        vec_to_pose.z *= SetSafePoseFromObject.DISTANCE_FROM_POSE / vec_to_pose_len;

        new_pose:Pose = Pose();
        new_pose.position.x = pose.position.x - vec_to_pose.x;
        new_pose.position.y = pose.position.y - vec_to_pose.y;
        new_pose.position.z = pose.position.z - vec_to_pose.z;
        new_pose.orientation.w = math.cos(angle);
        new_pose.orientation.z = math.sin(angle);

        userdata.pose_out = new_pose;
            
        return SUCCESS;

class SearchForGuestNavToNextNode(smach.State):
    """ Smach state to navigate the robot through a sequence of topological nodes during the search for guests (non-operator people)

    Returns 'searched' if arrived at next node, 'exhausted_search' if no more nodes are available to visit, `failure` if navigation fails.

    input_keys:
        nodes_not_searched: list of topological node ids to visit during search, in search order
        failure_threshold: number of allowed failed attempts for each topological navigation action
    output_keys:
        nodes_not_searched: list of topological node ids that were not visited during search (does not include the final node because there may be another person there)
    """

    def __init__(self, execute_nav_commands):
        smach.State.__init__(self,
                                outcomes=['searched', 'exhausted_search', FAILURE, 'preempted'],
                                input_keys=['nodes_not_searched',
                                            'failure_threshold'],
                                output_keys=['nodes_not_searched'])

        self.execute_nav_commands = execute_nav_commands;

    def execute(self, userdata):
        if self.execute_nav_commands == False:
            return 'searched';

        # If topological nodes aren't working, then uncomment the lines below to skip attempts to use it:
        # Don't actually try to travel along the topological nodes because it's not working at the moment
        # while True:
        #     rospy.sleep(1.0)
        #     if self.preempt_requested():
        #         self.service_preempt()
        #         return 'preempted'
        # TODO - remove after testing

        # check if any nodes left to visit
        if not userdata.nodes_not_searched:
            rospy.loginfo('All nodes explored. Nothing to search next.')
            return 'exhausted_search'

        # create action server
        topological_navigate_action_client = actionlib.SimpleActionClient('traverse_to_node',  TraverseToNodeAction)
        topological_navigate_action_client.wait_for_server()

        # take the first node on the nodes not searched list
        node_id = userdata.nodes_not_searched[0]

        for attempt_num in range(userdata.failure_threshold):
            # Check for preempt
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'

            # create action goal and call action server
            goal = TraverseToNodeGoal(node_id=node_id)
            rospy.loginfo('Navigating with top nav to node "{}"'.format(node_id))

            topological_navigate_action_client.wait_for_server()
            topological_navigate_action_client.send_goal(goal)
            topological_navigate_action_client.wait_for_result()
            result = topological_navigate_action_client.get_result()

            rospy.loginfo('result = ' + str(result.success))

            # Process action result
            #   Note: result.success returns True if node_id was reached
            if result.success:
                break   # break out of attempts for-loop
            else:
                # attempt_num starts at 0, and we have just finished taking an attempt
                if (attempt_num + 1) >= userdata.failure_threshold:
                    rospy.logwarn('Navigating with top nav to node "{}" failed. Abandoning.'.format(node_id))
                    # now remove the node from the nodes_not_searched list, because the nav action failed
                    del userdata.nodes_not_searched[0]
                    return FAILURE
            # rospy.sleep(2)  # TODO - remove after testing
        # One final check for preempt. We only want to remove the node from the list if nothing was found.
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        # now remove the node from the nodes_not_searched list, because we have now searched it
        del userdata.nodes_not_searched[0]
        return 'searched'

if __name__ == '__main__':
    rospy.init_node('listening_to_nav');

    # NavigationalListener();

    sub_sm = smach.StateMachine(outcomes=[SUCCESS, FAILURE]);

    with sub_sm:
        smach.StateMachine.add(
            "OrientTowards",
            OrientRobot(),
            transitions={
                SUCCESS:SUCCESS,
                FAILURE:FAILURE});
        pass;
    
    sub_sm.userdata.orient_towards = Pose();
    sub_sm.userdata.orient_towards.position.x = -0.7;
    sub_sm.userdata.orient_towards.position.y = -1;
    sub_sm.execute();

    rospy.spin();

    rospy.spin();