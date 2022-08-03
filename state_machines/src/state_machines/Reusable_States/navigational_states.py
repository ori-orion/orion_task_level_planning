from state_machines.Reusable_States.utils import *;

import smach;

from orion_actions.msg import *;
from orion_actions.srv import *;

import rospy;

from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
import actionlib
from actionlib_msgs.msg import GoalStatus

from geometry_msgs.msg import Pose, PoseStamped

from ori_topological_navigation_msgs.msg import TraverseToNodeAction, TraverseToNodeGoal, PoseOverlay, TraverseToNodeResult

import math;

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
        pose = rospy.wait_for_message('/global_pose', PoseStamped)
        userdata.robot_location = pose.pose
        rospy.loginfo(pose)
        return 'stored'


#region navigation states
class SimpleNavigateState(smach.State):
    """ State for navigating directly to a location on the map.

    This state is given a pose and navigates there.

    input_keys:
        pose: pose for the robot to navigate to
        number_of_failures: an external counter keeping track of the cumulative failure count (incremented in this state upon failure & reset upon success and repreat failure)
        failure_threshold: the number of cumulative failures required to return the repeat_failure outcome
    output_keys:
        number_of_failures: the updated failure counter upon state exit
    """

    def __init__(self):
        smach.State.__init__(self,
                                outcomes=['success', 'failure', 'repeat_failure'],
                                input_keys=['pose', 'number_of_failures', 'failure_threshold'],
                                output_keys=['number_of_failures'])

    def execute(self, userdata):
        # Navigating without top nav
        rospy.loginfo('Navigating without top nav')
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = userdata.pose
        rospy.loginfo(goal.target_pose.pose)

        navigate_action_client = actionlib.SimpleActionClient('move_base/move',  MoveBaseAction)
        navigate_action_client.wait_for_server()
        navigate_action_client.send_goal(goal)
        navigate_action_client.wait_for_result()
        status = navigate_action_client.get_state()
        navigate_action_client.cancel_all_goals()
        rospy.loginfo('status = ' + str(status))
        if status == GoalStatus.SUCCEEDED:
            userdata.number_of_failures = 0
            return 'success'
        else:
            userdata.number_of_failures += 1
            if userdata.number_of_failures >= userdata.failure_threshold:
                 # reset number of failures because we've already triggered the repeat failure outcome
                userdata.number_of_failures = 0
                return 'repeat_failure'
            return 'failure'

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

    def __init__(self, stop_repeat_navigation:bool = False):
        """
        stop_repeat_navigation:bool  - If we have just navigated to a node, we may get asked to go there in the near 
            future. In some cases, there is no point in this. (Say you are searching a room and have found something, 
            and then nav to the room node again). 
            If True then it will prevent us from navigating to the same node twice in a row.
            Otherwise it will navigate to the node as per normal.
        """
        self.stop_repeat_navigation = stop_repeat_navigation;

        smach.State.__init__(self,
                                outcomes=['success', 'failure', 'repeat_failure'],
                                input_keys=['node_id', 'number_of_failures', 'failure_threshold', 'prev_node_nav_to'],
                                output_keys=['number_of_failures', 'prev_node_nav_to']);

    def get_robot_pose(self) -> Pose:
        robot_pose:PoseStamped = rospy.wait_for_message('/global_pose', PoseStamped);
        return robot_pose.pose;

    def execute(self, userdata):
        if self.stop_repeat_navigation==True and userdata.node_id == userdata.prev_node_nav_to:
            rospy.loginfo("Repeat navigation to the same node prevented.")
            return 'success'

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
            return 'success'
        else:
            userdata.number_of_failures += 1
            if userdata.number_of_failures >= userdata.failure_threshold:
                 # reset number of failures because we've already triggered the repeat failure outcome
                userdata.number_of_failures = 0
                return 'repeat_failure'
            return 'failure'

class GetClosestNodeState(smach.State):
    """
    Inputs:
        goal_pose:Pose:     The pose we want to navigate to.
    Outputs:
        closest_node:str:   The node we will go via.
    """
    def __init__(self):
        smach.State.__init__(self, 
                                outcomes=['success'],
                                input_keys=['goal_pose'],
                                output_keys=['closest_node']);

    def execute(self, userdata):
        goal_pose:Pose = userdata.goal_pose;
        userdata.closest_node = get_closest_node(goal_pose.position);
        return 'success';

class NavigateDistanceFromGoalSafely(smach.State):
    """
    We want to be able to navigate to a human and sit 1m away from them without colliding into anything.

    DISTANCE_FROM_POSE gives the distance we want to sit from the target pose 'pose'.
    """

    DISTANCE_FROM_POSE = 1;

    def __init__(self):
        smach.State.__init__(
            self, 
            outcomes=['success'],
            input_keys=['pose']);

        self._mb_client = actionlib.SimpleActionClient('move_base/move', MoveBaseAction)
        self._mb_client.wait_for_server()

    def get_robot_pose(self) -> Pose:
        robot_pose:PoseStamped = rospy.wait_for_message('/global_pose', PoseStamped);
        return robot_pose.pose;

    def execute(self, userdata):
        target_pose:Pose = userdata.pose;
        target_pose.position.z = 0;

        rospy.loginfo('Navigating without top nav')
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = target_pose;
        rospy.loginfo(goal.target_pose.pose)
        
        self._mb_client.send_goal(goal);
        print(self._mb_client.get_state());
        while self._mb_client.get_state() == actionlib_msgs.msg.GoalStatus.PENDING:
            rospy.sleep(0.1);

        while self._mb_client.get_state() == actionlib_msgs.msg.GoalStatus.ACTIVE:
            rospy.sleep(0.1);
            # rospy.loginfo("Checking location");
            robot_pose = self.get_robot_pose();
            robot_pose.position.z = 0;
            dist_between_poses = distance_between_poses(robot_pose, target_pose);
            if dist_between_poses < NavigateDistanceFromGoalSafely.DISTANCE_FROM_POSE:
                rospy.loginfo("Preempting move_base action because we are the distance we want to be from the goal.");
                self._mb_client.cancel_all_goals();
                break;
            pass;
        
        # Waiting for the result as a backup.
        self._mb_client.wait_for_result();

        return 'success';

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
        smach.State.__init__(self, outcomes=['nav_to_node', 'nav_to_pose', 'failure'],
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
        smach.State.__init__(self, outcomes=['success'],
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
            
        return 'success';

