#!/usr/bin/env python3

import rospy
from actionlib import SimpleActionClient
import math
from typing import List, Union
import tf2_ros
import hsrb_interface
import manipulation.srv;
import hsrb_interface.geometry as geometry
from geometry_msgs.msg import Point, Pose
from state_machines.robocup_2024.put_away_the_groceries.common import find_navigation_goal, look_around, look_at_object, navigate_to_pose, query_objects_from_som
from state_machines.src.state_machines.Reusable_States.manipulation_states import GLOBAL_FRAME, getPlacementOptions, putObjOnSurfaceAction
hsrb_interface.robot.enable_interactive()

from state_machines.Reusable_States.utils import FAILURE, SmachBaseClass, SUCCESS, NAVIGATIONAL_FAILURE, MANIPULATION_FAILURE
from orion_actions.srv import SOMQueryObjectsRequest, SOMQueryObjects, SOMQueryObjectsResponse
from orion_actions.msg import SOMObject
#from orion_actions.srv import NavigationalQuery, NavigationalQueryRequest, NavigationalQueryResponse, PickUpObjectAction


class PutDownObject(SmachBaseClass):

    DISTANCE_SAME_PLACE_THRESHOLD = 0.1;
    RETRY = 1;
    RETRY_STAYED_IN_SAME_PLACE = 2;
    SUCCESS = 3;

     # If the gripper is more closed than this, we will say it has not actually picked anything up.
    GRIPPER_DISTANCCE_THRESHOLD = 0.001;

    def __init__(self, execute_nav_commands:bool, 
                 cabinet_mast_height: float):
        SmachBaseClass.__init__(self, 
                                outcomes=[SUCCESS], 
                                input_keys=["obj_to_put_down", "shelf_height_dict"])
        self.execute_nav_commands = execute_nav_commands
        self.cabinet_mast_height = cabinet_mast_height
        self.max_num_failure_repetitions = 3

        if rospy.has_param('find_placement_options'):
            find_placement_options:dict = rospy.get_param('find_placement_options')
            self.dims = find_placement_options['dims']
            self.height = find_placement_options['height']
            self.radius = find_placement_options['radius']
            print(find_placement_options)
        else:
            self.dims = (0.05, 0.05, 0.2)
            self.height = 0.3
            self.radius = 0.2


        self.buffer = tf2_ros.Buffer();


    def raise_mast(self):
        """Raise the mast to the cabinet height"""
        mast_height = min(self.cabinet_mast_height, self.MAST_JOINT_MAX)
        mast_height = max(mast_height, self.MAST_JOINT_MIN)
        
        if mast_height == 0:
            return
        
        self.moveToJointPositions({
            self.JOINT_ARM_LIFT   : mast_height,
            self.JOINT_ARM_FLEX   : -100 * math.pi / 180,
            self.JOINT_HEAD_PAN   : 0,
            self.JOINT_HEAD_TILT  : -math.pi / 6,
            self.JOINT_WRIST_FLEX : 0})
        
    def query(self, obj_to_put_down: SOMObject) -> List[SOMObject]:
        query = SOMQueryObjectsRequest()
        query.query.last_observed_at = rospy.Time.now()
        query.query.category = obj_to_put_down.category

        look_around()

        query_results = query_objects_from_som(query, distance_filter=2)

        if len(query_results) > 0:
            return query_results
        
        # If no  object is found, try an all-time query
        query = SOMQueryObjectsRequest()
        query.query.category = obj_to_put_down.category
        return query_objects_from_som(query, distance_filter=4)
    
    def navigate_close_to_object(self, obj_to_put_down: SOMObject):
        navigation_goal = find_navigation_goal(obj_to_put_down)

        if navigation_goal is None:
            return True
        look_at_object(obj_to_put_down, self.lookAtPoint)
        return navigate_to_pose(navigation_goal, self.max_num_failure_repetitions, self.execute_nav_commands)


    def place_down_object(self, obj_to_put_down: SOMObject) -> bool:

        speak_action_client = SimpleActionClient('/talk_request_action', TalkRequestAction)

        self.speak("Attempting to find a placement location.", wait_to_terminate=False);

        if self.input_put_down_obj_size:
            obj_size:Point = obj_to_put_down.size
            put_down_dims = ( obj_size.x, obj_size.y, self.dims[2] )
        print( "\tPut down dims:", put_down_dims );

        placement_option_found = False;
        for i in range(self.num_repeats):
            # best_tf is the name of the tf at which the (hypothetically) best tf for placing an object is at.
            place_locations, best_tf = getPlacementOptions(
                goal_pos=[
                    obj_to_put_down.obj_position.position.x, 
                    obj_to_put_down.obj_position.position.y, 
                    obj_to_put_down.obj_position.position.z],
                dims=put_down_dims,
                max_height=self.max_height,
                radius=radius,
                num_candidates=self.num_candidates,
                goal_tf=obj_to_put_down.tf_name);
            
            print("Getting placement options around {0}".format(obj_to_put_down.tf_name));
            
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
                    return True
            return self.place_down_object_backup(obj_to_put_down) 
        else:
            return False;

    def place_down_object_backup(self, obj_to_put_down: SOMObject) -> bool:
        print("Creating the occupancy map.")
        occupancy_map = SOMOccupancyMap(time_horizon=rospy.Duration(5));
        try:
            occupancy_map.createOccupancyMap(ignore_categories=["unknown"]);
        except:
            return FAILURE;
        print("Finding a placement location");
        loc, location_found = occupancy_map.findPlacementLocation(obj_to_put_down.size);
        
        if location_found==False:
            return FAILURE;

        tf_broadcaster = tf2_ros.StaticTransformBroadcaster();
        TF_NAME = "placement_location_backup";

        transform = geometry_msgs.msg.TransformStamped();
        transform.transform.translation = loc;
        transform.transform.rotation.w = 1;
        transform.header.stamp = rospy.Time.now();
        transform.header.frame_id = GLOBAL_FRAME;
        transform.child_frame_id = TF_NAME;
        tf_broadcaster.sendTransform([transform]);

        for i in range(3):
            put_down_goal = PutObjectOnSurfaceGoal();
            put_down_goal.goal_tf = TF_NAME;
            put_down_goal.drop_object_by_metres = 0.03;
            put_down_goal.object_half_height = obj_to_put_down.size.z;
            success = putObjOnSurfaceAction(put_down_goal);

            if success:
                return True
    
        return False

    def place_on_empty_shelf(self, obj_to_put_down: SOMObject, shelf_height_dict: dict):
        PLACEMENT_TF_NAME = "placement_tf_TLP"

        rospy.logwarn("Using the backup for finding placement locations. Something has failed along the way.");

        heights = shelf_height_dict["heights"];
        shelf_names:List[str] = shelf_height_dict["tf_names"];


        find_placement_on_empty_service = rospy.ServiceProxy(
            "/FindPlacementOnEmptySurface", manipulation.srv.FindPlacementOnEmptySurface);
        central_tf = shelf_names[ int(len(shelf_names)/2) ];

        trans_stamped = self.tf_buffer.lookup_transform("head_rgbd_sensor_rgb_frame", central_tf, rospy.Time(), timeout=rospy.Duration(1));
        rgbd_goal_transform = trans_stamped.transform;

        res:manipulation.srv.FindPlacementOnEmptySurfaceResponse = find_placement_on_empty_service(
            obj_to_put_down.size,     # dimension of object
            0.3,                        # max height from surface
            rgbd_goal_transform,
            10.0,                       # EPS plane search angle tolerance in degrees
            0.5,                        # Box crop size to search for plane in. Axis aligned w/ head frame.
            0.2                         # Minimum height of the surface to place on.
        )
        
        transform_name = "";
        using_regions = False;

        if res.success:
            t = geometry_msgs.msg.Transform()
            t.translation.x =   res.position[0];
            t.translation.y =   res.position[1];
            t.translation.z =   res.position[2];
            t.rotation.x =      0;
            t.rotation.y =      0;
            t.rotation.z =      0;
            t.rotation.w =      1;
            t_stamped = geometry_msgs.msg.TransformStamped();
            t_stamped.header.stamp = rospy.Time.now()
            t_stamped.header.frame_id = "map";
            t_stamped.child_frame_id = PLACEMENT_TF_NAME;
            t_stamped.transform = t;
            
            transform_name = PLACEMENT_TF_NAME;
            
            self.tf_broadcaster.sendTransform(t_stamped);
        
        else:
            region_query_srv = rospy.ServiceProxy( "/som/object_regions/region_query", SOMRegionQuery );

            num_items = [];
            for shelf_name in shelf_names:
                query = SOMRegionQueryRequest();
                query.region_name = shelf_name;
                query_returns:SOMRegionQueryResponse = region_query_srv( query );
                num_items.append( len(query_returns.returns) );
            
            min_index = np.argmin( np.asarray(num_items) );
            using_regions = True;
            transform_name = shelf_names[min_index];
            

        put_obj_on_surface_goal = PutObjectOnSurfaceGoal();
        put_obj_on_surface_goal.goal_tf = transform_name;
        put_obj_on_surface_goal.drop_object_by_metres = 0.03;
        put_obj_on_surface_goal.object_half_height = obj_to_put_down.size.z/2;
        for i in range(len(shelf_names)):
            success = putObjOnSurfaceAction(put_obj_on_surface_goal);
            if success:
                return SUCCESS;
            elif using_regions:
                min_index += 1;
                min_index %= len(shelf_names);
                put_obj_on_surface_goal.goal_tf = shelf_names[min_index];
        # If everything has failed, drop the item?

        return True

    def execute(self, userdata):
        self.raise_mast()
        query_results: List[SOMObject] = []
        while len(query_results) == 0:
            query_results = self.query(userdata.obj_to_put_down)

        obj_to_put_down = query_results[0]

        if self.navigate_close_to_object(obj_to_put_down):
            look_at_object(obj_to_put_down, self.lookAtPoint)
        
        self.moveToJointPositions({self.JOINT_HEAD_TILT:0})
        rospy.sleep(2)
        if self.place_down_object(obj_to_put_down):
            return SUCCESS
        
        self.place_on_empty_shelf(obj_to_put_down, userdata.shelf_height_dict)
        return SUCCESS
        


        
        

