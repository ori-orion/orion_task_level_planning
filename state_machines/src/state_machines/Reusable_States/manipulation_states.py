from state_machines.Reusable_States.utils import *;

import smach;

from orion_actions.msg import *;
from orion_actions.srv import *;

import rospy;

# from geometry_msgs.msg import Pose, PoseStamped;

import actionlib

import tf
import tf2_ros;
from manipulation.srv import FindPlacement
from typing import List;

GLOBAL_FRAME = "map";

class PickUpObjectState(smach.State):
    """ State for picking up an object

    This state picks up an object specified by name.

    input_keys:
        object_name: the object to be picked up
        number_of_failures: an external counter keeping track of the cumulative failure count (incremented in this state upon failure & reset upon success and repreat failure)
        failure_threshold: the number of cumulative failures required to return the repeat_failure outcome
        ar_marker_ids: a dictionary of AR marker IDs, keyed by object name
    output_keys:
        number_of_failures: the updated failure counter upon state exit
    """

    def __init__(self, object_name=None):
        smach.State.__init__(
            self,
            outcomes=[SUCCESS, FAILURE, REPEAT_FAILURE],
            input_keys=['object_name', 'number_of_failures', 'failure_threshold', 'ar_marker_ids'],
            output_keys=['number_of_failures']);

        self.object_name = object_name;

    def execute(self, userdata):
        if (self.object_name == None):
            object_name = userdata.object_name;
        else:
            object_name = self.object_name;

        pick_up_goal = PickUpObjectGoal()
        pick_up_goal.goal_tf = object_name.replace(" ", "_")          # need to replace spaces with underscores for ROS TF tree look-up

        # check if we can see the tf in the tf tree - if not, check if we need to fall back on an ar_marker, otherwise trigger the failure outcome
        tf_listener = tf.TransformListener()
        rospy.sleep(2)  # wait 2 seconds for the tf listener to gather tf data
        frames = tf_listener.getFrameStrings()

        matched_tf_from_tf_tree = "NOT_FOUND" # the matched tf name from the tree

        found_by_name = False
        for frame in frames:
            # perform a sub-string search in the frame string so we find the
            # frame we are looking for. Eg we find "potted_plant" in "potted_plant_1"
            if pick_up_goal.goal_tf in frame:
                found_by_name = True
                matched_tf_from_tf_tree = frame
                pick_up_goal.goal_tf = matched_tf_from_tf_tree # need to update the goal_tf to the actual frame name (TODO - check why pickup action server no longer resolves this)
                break

        if not found_by_name:
            if object_name in userdata.ar_marker_ids:
                ar_tf_string = 'ar_marker/' + str(userdata.ar_marker_ids[object_name])
                rospy.loginfo("Target TF '{}' not found in TF tree - using AR marker TF instead '{}'".format(pick_up_goal.goal_tf, ar_tf_string))

                found_by_ar_marker = False
                for frame in frames:
                    # perform a sub-string search in the frame string so we find the
                    # frame we are looking for. Eg we find "potted_plant" in "potted_plant_1"
                    if ar_tf_string in frame:
                        found_by_ar_marker = True
                        pick_up_goal.goal_tf = frame # need to update the goal_tf to the actual frame name (TODO - check why pickup action server no longer resolves this)
                        break

                if not found_by_ar_marker:
                    rospy.loginfo("AR marker TF was not found in TF tree '{}'. PickUpObjectState will now return failure state".format(ar_tf_string))
                    userdata.number_of_failures += 1
                    if userdata.number_of_failures >= userdata.failure_threshold:
                        userdata.number_of_failures = 0
                        return REPEAT_FAILURE
                    else:
                        return FAILURE
                else:
                    rospy.loginfo("PickUpObjectState found matching AR tf '{}' in tf-tree for task object {}".format(ar_tf_string, userdata.object_name))
            else:
                rospy.loginfo("Target TF '{}' not found in TF tree and no AR marker is known for object '{}'".format(pick_up_goal.goal_tf, userdata.object_name))
                rospy.loginfo("TF tree frames: '{}'".format(frames))
                rospy.loginfo("PickUpObjectState will now return failure state")
                userdata.number_of_failures += 1
                if userdata.number_of_failures >= userdata.failure_threshold:
                    userdata.number_of_failures = 0
                    return REPEAT_FAILURE
                else:
                    return FAILURE
        else:
            rospy.loginfo("PickUpObjectState found matching object tf '{}' in tf-tree for task object {}".format(matched_tf_from_tf_tree, userdata.object_name))


        # continue
        pick_up_object_action_client = actionlib.SimpleActionClient('pick_up_object', PickUpObjectAction)
        pick_up_object_action_client.wait_for_server()

        pick_up_object_action_client.send_goal(pick_up_goal)
        pick_up_object_action_client.wait_for_result()

        result = pick_up_object_action_client.get_result().result

        if result:
            userdata.number_of_failures = 0
            return SUCCESS
        else:
            userdata.number_of_failures += 1
            if userdata.number_of_failures >= userdata.failure_threshold:
                userdata.number_of_failures = 0
                return REPEAT_FAILURE
            else:
                return FAILURE


class HandoverObjectToOperatorState(smach.State):
    """ Smach state for handing a grasped object to an operator.

    This state hands over an object to the operator.
    """
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=[SUCCESS, FAILURE],
            input_keys=[],
            output_keys=[]);

    def execute(self, userdata):
        handover_goal = GiveObjectToOperatorGoal()

        give_object_to_operator_action_client = actionlib.SimpleActionClient('give_object_to_operator',
                                         GiveObjectToOperatorAction)
        give_object_to_operator_action_client.wait_for_server()
        give_object_to_operator_action_client.send_goal(handover_goal)
        give_object_to_operator_action_client.wait_for_result()

        success = give_object_to_operator_action_client.get_result().result
        if success:
            return SUCCESS
        else:
            return FAILURE


class ReceiveObjectFromOperatorState(smach.State):
    """ Smach state for receiving an object from an operator.

    This state grasps an object currently held by an operator.
    """

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=[SUCCESS, FAILURE],
            input_keys=[],
            output_keys=[]);

    def execute(self, userdata):
        receive_goal = ReceiveObjectFromOperatorGoal()

        receive_object_from_operator_action_client = actionlib.SimpleActionClient('receive_object_from_operator',
                                         ReceiveObjectFromOperatorAction)
        receive_object_from_operator_action_client.wait_for_server()
        receive_object_from_operator_action_client.send_goal(receive_goal)
        receive_object_from_operator_action_client.wait_for_result()

        result = receive_object_from_operator_action_client.get_result()
        success = result.result
        if success:
            return SUCCESS
        else:
            return FAILURE


class PutObjectOnSurfaceState(smach.State):
    """ Smach state for putting object on a surface in front of the robot.

    This state put an object held by the robot on a surface.
    """
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=[SUCCESS, FAILURE],
            input_keys=[],
            output_keys=[]);

    def execute(self, userdata):
        put_on_surface_goal = PutObjectOnSurfaceGoal()

        put_on_surface_action = actionlib.SimpleActionClient('put_object_on_surface', PutObjectOnSurfaceAction);
        put_on_surface_action.send_goal(put_on_surface_goal)
        put_on_surface_action.wait_for_result()

        success = put_on_surface_action.get_result().result
        if success:
            return SUCCESS
        else:
            return FAILURE


def getPlacementOptions(
        goal_tf:str, 
        dims:tuple, 
        max_height:float,
        radius:float,
        num_candidates:int) -> List[float]:
    """
    Uses the FindPlacement server within manipulation to get the best grasp pose.
        dims is a 3-tuple giving the rough dimensions of the object.
        radius is the distance away we want to look for.
        num_candidates is the number of candidates we want to find. 
        Returns a float64[3] giving xyz of the best pose.  
    """
    try:
        find_placement = rospy.ServiceProxy('find_placement_around', FindPlacement)
        resp = find_placement(goal_tf, dims, max_height, radius, num_candidates)
        return resp.position
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
    pass;

class PlaceNextTo(smach.State):
    def __init__(self, dims, max_height, radius, num_candidates=3):
        smach.State.__init__(
            self,
            outcomes=[SUCCESS, FAILURE],
            input_keys=['som_query_results'],
            output_keys=[]);

        self.dims = dims;
        self.max_height = max_height;
        self.radius = radius;
        self.num_candidates = num_candidates;

    def execute(self, userdata):
        som_query_results:List[dict] = userdata.som_query_results;

        first_response:dict = som_query_results[0];

        place_locations = getPlacementOptions(
            first_response["class_"] + "_0",
            self.dims,
            self.max_height,
            self.radius,
            self.num_candidates);
        pass;


point_at_uid_ref = 0;
class PointAtEntity(smach.State):

    def __init__(self, statement_having_pointed=None, statement_before_pointing=None):
        smach.State.__init__(
            self,
            outcomes=[SUCCESS, FAILURE],
            input_keys=['point_at_loc'],
            output_keys=[]);

        self.statement_having_pointed = "" if statement_having_pointed is None else statement_having_pointed;
        self.statement_before_pointing = "" if statement_before_pointing is None else statement_before_pointing;
        self.point_at_obj_server = actionlib.SimpleActionClient('point_to_object',PointToObjectAction);
        
        self.tfbroadcaster = tf2_ros.StaticTransformBroadcaster();
        self.tfBuffer = tf2_ros.Buffer();
        self.listener = tf2_ros.TransformListener(self.tfBuffer);

    def createPointAtTf_UID(self) -> str:
        global point_at_uid_ref;
        point_at_uid_ref += 1;
        return "POINT_AT_TF_UID_" + str(point_at_uid_ref);

    def execute(self, userdata):
        tf_uid = self.createPointAtTf_UID();

        point_at_loc:Pose = userdata.point_at_loc;

        self.point_at_obj_server.wait_for_server();

        transform = geometry_msgs.msg.TransformStamped()
        transform.transform.translation.x = point_at_loc.position.x;
        transform.transform.translation.y = point_at_loc.position.y;
        transform.transform.translation.z = point_at_loc.position.z;
        transform.transform.rotation.w = 1;
        transform.header.stamp = rospy.Time.now();
        transform.header.frame_id = GLOBAL_FRAME;
        transform.child_frame_id = tf_uid;
        self.tfbroadcaster.sendTransform([transform]);
        
        # trans = None;
        # while (trans is None):
        #     trans = self.tfBuffer.lookup_transform(
        #         tf_uid, 
        #         GLOBAL_FRAME, 
        #         rospy.Time(), timeout=rospy.Duration(2));
        
        goal = PointToObjectGoal();
        goal.goal_tf = tf_uid;
        goal.statement_before_pointing = self.statement_before_pointing;
        goal.statement_having_pointed = self.statement_having_pointed;

        self.point_at_obj_server.send_goal(goal);
        self.point_at_obj_server.wait_for_result();
        result:PointToObjectResult = self.point_at_obj_server.get_result();
        if result.result == True:
            return SUCCESS;
        else:
            return FAILURE;
    
class DropEntity(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=[SUCCESS, FAILURE],
            input_keys=[],
            output_keys=[]);

    def execute(self, userdata):
        return SUCCESS;