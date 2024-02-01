#!/usr/bin/env python3

import rospy
import math
from typing import List
import tf2_ros
import hsrb_interface
from geometry_msgs.msg import Point, TransformStamped, Vector3, Transform
import numpy as np

from  manipulation.srv import FindPlacementOnEmptySurface
from orion_actions.srv import SOMQueryObjectsRequest, SOMRegionQuery, SOMRegionQueryRequest, \
    SOMRegionQueryResponse
from orion_actions.msg import SOMObject
from manipulation.srv import FindPlacementOnEmptySurfaceResponse
from state_machines.robocup_2024.put_away_the_groceries.common import compute_safe_mast_height, \
    find_navigation_goal, look_around, look_at_object, navigate_to_pose, query_objects_from_som
from state_machines.Reusable_States.include_all import GLOBAL_FRAME, getPlacementOptions,\
      putObjOnSurfaceAction, SmachBaseClass, SUCCESS, PutObjectOnSurfaceGoal, SOMOccupancyMap

hsrb_interface.robot.enable_interactive()

class PutDownObject(SmachBaseClass):

    DISTANCE_SAME_PLACE_THRESHOLD = 0.1
    MAST_JOINT_MAX = 0.69
    MAST_JOINT_MIN = 0

    def __init__(self, execute_nav_commands:bool, 
                 cabinet_mast_height: float,
                 num_repeats: int,
                 placement_options: dict,
                 num_candidates=16):
        """
        Put down the object on the correct shelf. It first try to use the
        "put_object_on_surface" action, and if this fails, it uses the OccupancyMap.
        As a last resource, it tries to put down the object on an an empty shelf.

        Possible outcomes:
        - `SUCCESS`: the object has been put down

        Input keys:
        - `target_obj`: the bject to put down
        - `shelf_height_dict`: a dictionary with keys "heights" and "tf_names",
            containing the heights and tf_names of the shelves

        Parameters:
        - `execute_nav_commands`: wether to execute the navigation commands
        - `cabinet_mast_height`: the height of the mast needed to look at the
            cabinet.
        - `num_repeats`: how many times the robot should try to find a placement
            before failing (at each stage)
        - `placements_options`: a dict containing  info used to find a placement.
            It should contain keys "dims" (the dimensions of each shelf), 
            "height" (the maximum height) and "radius" (how far away of the other 
            object we want to find a location).
        - `num_candidates`: how many candidates position should be checked.
        """
        SmachBaseClass.__init__(self, 
                                outcomes=[SUCCESS], 
                                input_keys=["target_obj", "shelf_height_dict"])
        self.execute_nav_commands = execute_nav_commands
        self.cabinet_mast_height = cabinet_mast_height
        self.max_num_failure_repetitions = 3

        self.num_repeats = num_repeats
        self.num_candidates = num_candidates

        self.dims = placement_options['dims']
        self.max_height = placement_options['height']
        self.radius = placement_options['radius']


        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster()


    def raise_mast(self):
        """Raise the mast to the cabinet height"""
        rospy.loginfo("Raising mast to cabinet height")
        mast_height = compute_safe_mast_height(self.cabinet_mast_height)
        
        if mast_height == 0:
            return
        self.moveToJointPositions({
            self.JOINT_ARM_LIFT   : mast_height,
            self.JOINT_ARM_FLEX   : -100 * math.pi / 180,
            self.JOINT_HEAD_PAN   : 0,
            self.JOINT_HEAD_TILT  : -math.pi / 6,
            self.JOINT_WRIST_FLEX : 0})
        
    def query_same_category(self, obj_to_put_down: SOMObject) -> List[SOMObject]:
        """
        Query the SOM for objects of the same category as the object to put down.

        Parameters:
        - `obj_to_put_down`: the object to put in the cabinet

        Returns the list of objects obtained from SOM.
        """
        rospy.loginfo(f"Querying SOM for objects of category {obj_to_put_down.category}")
        query = SOMQueryObjectsRequest()
        query.query.last_observed_at = rospy.Time.now()
        query.query.category = obj_to_put_down.category

        look_around()

        query_results = query_objects_from_som(query, distance_filter=2)
        if True or len(query_results) > 0:
            return query_results
        
        # If no  object is found, try an all-time query
        query = SOMQueryObjectsRequest()
        query.query.category = obj_to_put_down.category
        return query_objects_from_som(query, distance_filter=4)
    
    def navigate_close_to_object(self, obj_put_next_to: SOMObject):
        """
        Navigate close to the position next to which we want to put down the object

        Parameters:
        - `obj_put_next_to`: the object next to which we want to place
        """
        navigation_goal = find_navigation_goal(obj_put_next_to)

        if navigation_goal is None:
            return True
        look_at_object(obj_put_next_to, self.lookAtPoint)
        return navigate_to_pose(navigation_goal, self.max_num_failure_repetitions, self.execute_nav_commands)


    def place_down_object(self, obj_put_next_to: SOMObject, obj_to_put_down: SOMObject) -> bool:
        """
        Try to place down the object.

        Parameters:
        - `obj_put_next_to`: the object next to which we want to place down
        - `obj_to_put_down`: the object to put down

        Returns `True` if the object has been placed down successfully.
        """
        self.speak("Attempting to find a placement location.", wait_to_terminate=False)

        obj_size: Point = obj_to_put_down.size
        put_down_dims = ( obj_size.x, obj_size.y, self.dims[2] )
        rospy.loginfo(f"Put down dims: {put_down_dims}")

        placement_option_found = False
        radius = self.radius
        best_tf = ""
        for _ in range(self.num_repeats):
            # best_tf is the name of the tf at which the (hypothetically) best tf for placing an object is at.
            place_locations, best_tf = getPlacementOptions(
                goal_pos=[
                    obj_put_next_to.obj_position.position.x, 
                    obj_put_next_to.obj_position.position.y, 
                    obj_put_next_to.obj_position.position.z],
                dims=put_down_dims,
                max_height=self.max_height,
                radius=radius,
                num_candidates=self.num_candidates,
                goal_tf=obj_put_next_to.tf_name)
            
            rospy.loginfo(f"Getting placement options around {obj_put_next_to.tf_name}")
            
            rospy.loginfo(f"Locations: {place_locations}")
            rospy.loginfo(f"best tf: {best_tf}")

            if len(best_tf) == 0:
                self.speak("No placement options were found. Retrying.", wait_to_terminate=False)
                radius *= 1.3
            else:
                self.speak("A placement option was found. Executing now.", wait_to_terminate=False)
                placement_option_found = True
                break

        if placement_option_found:
            for _ in range(self.num_repeats):
                rospy.loginfo("Trying to put down object...")
                goal = PutObjectOnSurfaceGoal()
                goal.goal_tf = best_tf
                goal.drop_object_by_metres = 0.03
                goal.object_half_height = obj_size.z/2
                rospy.loginfo("Setting obj_half_height to {0}".format(goal.object_half_height))
                success = putObjOnSurfaceAction(goal)
                if success:
                    return True
        rospy.logwarn("Could not place object down, trying to use occupancy map...")
        return self.place_down_object_backup(obj_to_put_down) 
        

    def place_down_object_backup(self, obj_to_put_down: SOMObject) -> bool:
        """
        Try to put down the object using the occupancy map.

        Parameters:
        - `obj_to_put_down`: the object to put down

        Returns `True` if the object has been placed down successfully.
        """

        rospy.loginfo("Creating the occupancy map.")
        occupancy_map = SOMOccupancyMap(time_horizon=rospy.Duration(5))
        try:
            occupancy_map.createOccupancyMap(ignore_categories=["unknown"])
        except:
            return False
        rospy.loginfo("Finding a placement location")
        loc, location_found = occupancy_map.findPlacementLocation(obj_to_put_down.size)
        
        if not location_found:
            rospy.logwarn("Could not find placement location with occupancy map")
            return False

        tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
        TF_NAME = "placement_location_backup"

        transform = TransformStamped()
        transform.transform.translation = Vector3(loc.x, loc.y, loc.z)
        transform.transform.rotation.w = 1
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = GLOBAL_FRAME
        transform.child_frame_id = TF_NAME
        tf_broadcaster.sendTransform([transform])

        for _ in range(self.num_repeats):
            rospy.loginfo("Trying to put down object...")
            put_down_goal = PutObjectOnSurfaceGoal()
            put_down_goal.goal_tf = TF_NAME
            put_down_goal.drop_object_by_metres = 0.03
            put_down_goal.object_half_height = obj_to_put_down.size.z
            success = putObjOnSurfaceAction(put_down_goal)

            if success:
                return True
        rospy.logwarn("Could not put down object")
        return False

    def place_on_empty_shelf(self, obj_to_put_down: SOMObject, shelf_height_dict: dict):
        """
        Try to put down the object on an empty shelf.

        Parameters: 
        - `obj_to_put_down`: the object to put down
        - `shelf_height_dict`: a dict containing two keys, "heights" and "tf_names".
            Each is a list detailing the height and tf name of each shelf. 
        
        Returns `True` if the object was placed down successfully.
        """
        PLACEMENT_TF_NAME = "placement_tf_TLP"

        rospy.logwarn("Using the backup for finding placement locations. Something has failed along the way.")

        shelf_names:List[str] = shelf_height_dict["tf_names"]

        find_placement_on_empty_service = rospy.ServiceProxy(
            "/FindPlacementOnEmptySurface", FindPlacementOnEmptySurface)
        central_tf = shelf_names[ int(len(shelf_names)/2) ]

        trans_stamped: TransformStamped = self.tf_buffer.lookup_transform("head_rgbd_sensor_rgb_frame", 
                                                                          central_tf, rospy.Time(), 
                                                                          timeout=rospy.Duration(1))

        res: FindPlacementOnEmptySurfaceResponse = find_placement_on_empty_service(
            obj_to_put_down.size,     # dimension of object
            0.3,                        # max height from surface
            trans_stamped.transform,
            10.0,                       # EPS plane search angle tolerance in degrees
            0.5,                        # Box crop size to search for plane in. Axis aligned w/ head frame.
            0.2                         # Minimum height of the surface to place on.
        )
        
        transform_name = ""
        using_regions = False
        min_index = 0

        if res.success:
            t = Transform()
            t.translation.x =   res.position[0]
            t.translation.y =   res.position[1]
            t.translation.z =   res.position[2]
            t.rotation.x =      0
            t.rotation.y =      0
            t.rotation.z =      0
            t.rotation.w =      1
            t_stamped = TransformStamped()
            t_stamped.header.stamp = rospy.Time.now()
            t_stamped.header.frame_id = "map"
            t_stamped.child_frame_id = PLACEMENT_TF_NAME
            t_stamped.transform = t
            
            transform_name = PLACEMENT_TF_NAME
            
            self.tf_broadcaster.sendTransform(t_stamped)
        
        else:
            region_query_srv = rospy.ServiceProxy( "/som/object_regions/region_query", SOMRegionQuery )

            num_items = []
            for shelf_name in shelf_names:
                query = SOMRegionQueryRequest()
                query.region_name = shelf_name
                query_returns:SOMRegionQueryResponse = region_query_srv( query )
                returns:List[SOMObject] = query_returns.returns #type:ignore
                num_items.append( len(returns) )
            
            min_index = int(np.argmin( np.asarray(num_items) ))
            using_regions = True
            transform_name = shelf_names[min_index]
            

        put_obj_on_surface_goal = PutObjectOnSurfaceGoal()
        put_obj_on_surface_goal.goal_tf = transform_name
        put_obj_on_surface_goal.drop_object_by_metres = 0.03
        put_obj_on_surface_goal.object_half_height = obj_to_put_down.size.z/2
        for _ in range(len(shelf_names)):
            success = putObjOnSurfaceAction(put_obj_on_surface_goal)
            if success:
                return True
            elif using_regions:
                min_index += 1
                min_index %= len(shelf_names)
                put_obj_on_surface_goal.goal_tf = shelf_names[min_index]
        # If everything has failed, drop the item?
        return False

    def execute(self, userdata):
        self.raise_mast()
        obj_to_put_down: SOMObject = userdata.target_obj

        query_results: List[SOMObject] = []
        i = 0
        while len(query_results) == 0 and i < 3:
            query_results = self.query_same_category(obj_to_put_down)
            i += 1

        if len(query_results) > 0:
            # We choose the object next to which we want to place
            obj_put_next_to = query_results[0]
            rospy.loginfo(f"Will put down next  to {obj_put_next_to.class_} with tf: {obj_put_next_to.tf_name}")
            if self.navigate_close_to_object(obj_put_next_to):
                look_at_object(obj_put_next_to, self.lookAtPoint)
            
            self.moveToJointPositions({self.JOINT_HEAD_TILT:0})
            rospy.sleep(2)
            if self.place_down_object(obj_put_next_to, obj_to_put_down):
                return SUCCESS
        
        self.place_on_empty_shelf(obj_to_put_down, userdata.shelf_height_dict)
        return SUCCESS
        


        
        

