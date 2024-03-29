#!/usr/bin/env python3
from state_machines.Reusable_States.utils import *;

import smach;

from orion_actions.msg import *;
from orion_actions.srv import *;

import rospy;

# from geometry_msgs.msg import Pose, PoseStamped;

import actionlib
from actionlib_msgs.msg import GoalStatus

import tf
import tf2_ros;
from manipulation.srv import FindPlacement, FindPlacementRequest, FindPlacementResponse
from typing import List, Tuple;

import hsrb_interface;
hsrb_interface.robot.enable_interactive();

GLOBAL_FRAME = "map";

# To give greater resolution when looking at outcomes within the state machine. Not fully implemented.
MANIPULATION_FAILURE = 'manipulation_failure'

class PickUpObjectState(SmachBaseClass):
    """ 
    DEPRECATED in favour of PickUpObjectState_v2. Note that this searches the tf tree to find the thing to pick up.
    However, the function getting all the options is itself deprecated so it is better to use version 2.
    It also requires the failure infrastructure to be built into any state machine that is used.
    State for picking up an object

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
        SmachBaseClass.__init__(
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
        buffer = tf_listener._buffer;
        rospy.sleep(2)  # wait 2 seconds for the tf listener to gather tf data
        # Deprecated (I think - Matthew Munks 17/6/2023)
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
            

class PickUpObjectState_v2(SmachBaseClass):
    """
    Second iteration for picking up an object.

    Overall workflow:
        For num_repeats:
            Pick up object. 
            If success and gripper is somewhat open:
                return SUCCESS
            If tf error, publish own tf.
        [Failed num_repeats times]
        return MANIPULATION_FAILURE
        `gripper is somewhat open` is given by GRIPPER_DISTANCE_THRESHOLD. This has 
        a drawback in that it will not register if it's picked up a piece of paper 
        for instance. Instances of this are rare however, so this is a case that is
        not handled by this at present.


    INPUTS:
        num_iterations_upon_failure     Given that manipulatin fails, how many attempts are we going to make before failing?
        tf_name:str                     The tf name of the object we are trying to pick up.
        som_query_results:List[dict]    The dictionary from SOM for the object we are picking up. Assumes that the list is not empty.
    Outcomes:
        SUCCESS
        MANIPULATION_FAILURE

    Does none of the tf searching that the first iteration did.
    """
    # If the gripper is more closed than this, we will say it has not actually picked anything up.
    GRIPPER_DISTANCCE_THRESHOLD = 0.001;

    def __init__(self, num_iterations_upon_failure=3, read_from_som_query_results:bool=True, 
                wait_upon_completion=rospy.Duration(5)):
        input_keys = ['som_query_results'] if read_from_som_query_results else ['tf_name'];
        SmachBaseClass.__init__(
            self,
            outcomes=[SUCCESS, MANIPULATION_FAILURE],
            input_keys=input_keys,
            output_keys=[]);

        self.num_iterations_upon_failure = num_iterations_upon_failure;
        self.read_from_som_query_results = read_from_som_query_results;
        self.wait_upon_completion = wait_upon_completion;

    def run_manipulation_comp(self, pick_up_goal):
        self.pick_up_object_action_client.send_goal(pick_up_goal)
        self.pick_up_object_action_client.wait_for_result()


        result:PickUpObjectResult = self.pick_up_object_action_client.get_result()
        return result.result, result.failure_mode

    def execute(self, userdata):
        self.pick_up_object_action_client = actionlib.SimpleActionClient('pick_up_object', PickUpObjectAction)
        self.pick_up_object_action_client.wait_for_server()

        pick_up_goal = PickUpObjectGoal();
        if self.read_from_som_query_results:
            som_query_result:dict = userdata.som_query_results[0];
            pick_up_goal.goal_tf = som_query_result['tf_name']
        else:
            pick_up_goal.goal_tf = userdata.tf_name;
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
                    return SUCCESS;
                
            elif failure_mode==PickUpObjectResult.TF_NOT_FOUND or failure_mode==PickUpObjectResult.TF_TIMEOUT:
                rospy.loginfo("Tf error");
                pick_up_goal.publish_own_tf = True;
                pass;
            elif failure_mode==PickUpObjectResult.GRASPING_FAILED or status==GoalStatus.ABORTED:
                rospy.loginfo("Grasping failed.");
                # return MANIPULATION_FAILURE;
                pass;
            rospy.loginfo("Manipulation failed.");

        rospy.sleep(self.wait_upon_completion);
        return MANIPULATION_FAILURE;


class HandoverObjectToOperatorState(SmachBaseClass):
    """ Smach state for handing a grasped object to an operator.

    This state hands over an object to the operator.
    """
    def __init__(self):
        SmachBaseClass.__init__(
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


class ReceiveObjectFromOperatorState(SmachBaseClass):
    """ Smach state for receiving an object from an operator.

    This state grasps an object currently held by an operator.
    """

    def __init__(self):
        SmachBaseClass.__init__(
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


def putObjOnSurfaceAction(goal:PutObjectOnSurfaceGoal=None):
    if goal == None:
        goal = PutObjectOnSurfaceGoal();
    print('Calling put object on surface server')
    print(goal)
    put_on_surface_action = actionlib.SimpleActionClient('put_object_on_surface', PutObjectOnSurfaceAction);
    print("Waiting for the put down server.");
    put_on_surface_action.wait_for_server()
    print("Creating the action server");
    put_on_surface_action.send_goal(goal)
    print("Goal sent");
    put_on_surface_action.wait_for_result()
    print("Result received");

    return put_on_surface_action.get_result().result;


class PutObjectOnSurfaceState(SmachBaseClass):
    """ Smach state for putting object on a surface in front of the robot.

    This state put an object held by the robot on a surface.
    """
    def __init__(self):
        SmachBaseClass.__init__(
            self,
            outcomes=[SUCCESS, FAILURE],
            input_keys=[],
            output_keys=[]);

    def execute(self, userdata):
        success = putObjOnSurfaceAction();
        if success:
            return SUCCESS
        else:
            return FAILURE


def getPlacementOptions(
        goal_pos:List[float],
        dims:tuple, 
        max_height:float,
        radius:float,
        num_candidates:int,
        goal_tf:str="") -> Tuple[List[float], str]:
    """
    Uses the FindPlacement server within manipulation to get the best grasp pose.
        goal_tf/goal_pos are mutually distinct input options.
            If goal_tf=="", then goal_pos is what is fed in, this being the location of the object we want to avoid.
            Else, goal_tf is fed in, this being the tf of the object we want to avoid.
        dims is a 3-tuple giving the rough dimensions of the object.
        radius is the distance away we want to look for.
        num_candidates is the number of candidates we want to find. 
        Returns a tuple.
            out[0] - float64[3] giving xyz of the best pose.  
            out[1] - str giving the best tf for picking up the object.
    """
    try:
        find_placement = rospy.ServiceProxy('find_placement_around', FindPlacement);
        request:FindPlacementRequest = FindPlacementRequest();
        request.goal_tf = goal_tf;
        request.goal_pos = goal_pos;
        request.dims = dims;
        request.maxHeight = max_height;
        request.radius = radius;
        request.z_shift = 0.05;
        request.candidateNum = num_candidates;
        resp:FindPlacementResponse = find_placement(request);
        print("Loc found");
        return resp.position, resp.best_tf
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


class PlaceNextTo(SmachBaseClass):
    """
    Inputs:
        dims:tuple                          : The dimensions of the object to be put down.
        max_height:float                    : The height below the object to find the plane.
        radius:float                        : The radius around the goal position to search for placement candidates.
        num_candidates:int                  : The number of placement candidates to search through.
        input_put_down_obj_size:bool        : Give as input the size of the object to be put down.
        take_shelf_heights_as_input:bool    : Take as input a dictionary giving shelf heights and corresponding shelf tf names.
    Input keys:
        som_query_results:dict
        put_down_size:geometry_msgs.msg.Point   : Optional arg.
        shelf_height_dict:dict                  : Optional arg.
    Outcomes:
        SUCCESS                 : Entire motion correctly carried out.
        MANIPULATION_FAILURE    : Place object failed.
        FAILURE                 : Placement option was not found.
    The form of shelf_height_dict:
        shelf_height_dict = {
            heights:List[float],
            tf_names:List[str]}
        heights are then in increasing order.
    """
    def __init__(
        self, dims, max_height, 
        radius, num_candidates=16, 
        num_repeats=3, input_put_down_obj_size=False,
        take_shelf_heights_as_input=False):
        
        input_keys = ['som_query_results'];
        if input_put_down_obj_size:
            input_keys.append('put_down_size');
            
        if take_shelf_heights_as_input:
            input_keys.append('shelf_height_dict');

        SmachBaseClass.__init__(
            self,
            outcomes=[SUCCESS, MANIPULATION_FAILURE, FAILURE],
            input_keys=input_keys,
            output_keys=[]);

        self.dims = dims;
        self.max_height = max_height;
        self.radius = radius;
        self.num_candidates = num_candidates;
        self.num_repeats=num_repeats;
        self.input_put_down_obj_size = input_put_down_obj_size;
        self.take_shelf_heights_as_input = take_shelf_heights_as_input;
    
    # def speak(self, phrase_speaking):
    #     action_goal = TalkRequestGoal()
    #     action_goal.data.language = Voice.kEnglish  # enum for value: 1
    #     action_goal.data.sentence = phrase_speaking
    #     rospy.loginfo("HSR speaking phrase: '{}'".format(phrase_speaking))

    #     self.speak_action_client.wait_for_server()
    #     self.speak_action_client.send_goal(action_goal)
    # #     # self.speak_action_client.wait_for_result()
        

    def execute(self, userdata):
        som_query_results:List[dict] = userdata.som_query_results;

        first_response:SOMObject = som_query_results[0];

        self.speak_action_client = actionlib.SimpleActionClient('/talk_request_action', TalkRequestAction)
        
        if self.take_shelf_heights_as_input:
            shelf_height_dict:dict = userdata.shelf_height_dict;
            shelf_heights:List[float] = shelf_height_dict['heights'];
            shelf_tf_names:List[str] = shelf_height_dict['tf_names'];
            shelf_tf_name_using = shelf_tf_names[0];
            for height, tf_name in zip(shelf_heights, shelf_tf_names):
                if height > first_response.obj_position.position.z:
                    break;
                shelf_tf_name_using = tf_name;

        radius = self.radius;

        self.speak("Attempting to find a placement location.", wait_to_terminate=False);

        put_down_dims = self.dims;
        if self.input_put_down_obj_size:
            obj_size:Point = userdata.put_down_size;
            put_down_dims = ( obj_size.x, obj_size.y, self.dims[2] );
        print( "\tPut down dims:", put_down_dims );

        placement_option_found = False;
        for i in range(self.num_repeats):
            # best_tf is the name of the tf at which the (hypothetically) best tf for placing an object is at.
            place_locations, best_tf = getPlacementOptions(
                goal_pos=[
                    first_response.obj_position.position.x, 
                    first_response.obj_position.position.y, 
                    first_response.obj_position.position.z],
                dims=put_down_dims,
                max_height=self.max_height,
                radius=radius,
                num_candidates=self.num_candidates,
                goal_tf=first_response.tf_name);
            
            print("Getting placement options around {0}".format(first_response.tf_name));
            
            print("\t", place_locations);
            print("\t", best_tf);

            if len(best_tf) == 0:
                self.speak("No placement options were found. Retrying.", wait_to_terminate=False);
                radius *= 1.3;
            else:
                self.speak("A placement option was found. Executing now.", wait_to_terminate=False);
                placement_option_found = True;
                break;


        if placement_option_found:
            for i in range(self.num_repeats):
                goal = PutObjectOnSurfaceGoal();
                goal.goal_tf = best_tf;
                goal.drop_object_by_metres = 0.03;
                if self.input_put_down_obj_size:
                    goal.object_half_height = obj_size.z/2
                    print("\tSetting obj_half_height to {0}".format(goal.object_half_height));
                success = putObjOnSurfaceAction(goal);
                if success:
                    return SUCCESS
                else:
                    # self.speakPhrase("")
                    pass;
            return MANIPULATION_FAILURE 
        else:
            return FAILURE;


point_at_uid_ref = 0;
class PointAtEntity(SmachBaseClass):

    def __init__(self, statement_having_pointed=None, statement_before_pointing=None):
        SmachBaseClass.__init__(
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
    
class DropEntity(SmachBaseClass):
    def __init__(self):
        SmachBaseClass.__init__(
            self,
            outcomes=[SUCCESS, FAILURE],
            input_keys=[],
            output_keys=[]);

    def execute(self, userdata):
        self.getRobotInterface();
        self.moveToNeutral();
        self.gripper.command(1.2);
        return SUCCESS;