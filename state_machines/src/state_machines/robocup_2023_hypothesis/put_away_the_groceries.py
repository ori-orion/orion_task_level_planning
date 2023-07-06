#!/usr/bin/env python3
"""
Code for putting away the groceries.

Will observe items and transfer them from one area to another.

This makes EXTENSIVE use of the ros parameter server. Look within config/put_away_the_groceries.yaml to find
a list of all the parameters with explanations.

Notes:
    - We don't need to look around each time. 

Stuff todo:
    class name blacklist
    waypoints
    spin less
    fix bug
    reduce radius on placement.
    Ask the operator to place the object if no placement options found.

    Move further back.
    Tfs
    obj height.
"""

import rospy;
import smach_ros;
import actionlib;
import std_srvs.srv;
import manipulation.srv;

from state_machines.SubStateMachines.include_all import *;

from typing import List;


"""
In the case where we want to place something amidst other items.
For this we need to know where the other items are.
"""
class FindPlacementLocationBackup(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=[SUCCESS, FAILURE, MANIPULATION_FAILURE],
            input_keys=['put_down_size'],
            output_keys=[]);

    def execute(self, userdata):
        print("Creating the occupancy map.")
        occupancy_map = SOMOccupancyMap(time_horizon=rospy.Duration(5));
        try:
            occupancy_map.createOccupancyMap(ignore_categories=["unknown"]);
        except:
            return FAILURE;
        print("Finding a placement location");
        loc, location_found = occupancy_map.findPlacementLocation(userdata.put_down_size);
        
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
            put_down_goal.object_half_height = userdata.put_down_size.z;
            success = putObjOnSurfaceAction(put_down_goal);

            if success:
                return SUCCESS;
    
        return MANIPULATION_FAILURE;


"""
This is then the final backup. If all placement options have failed, we go here.
Cases:
    - If no object matching the catgegory has been found.
    - If no placement options are found at all.
"""
class FindShelfBackup(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=[SUCCESS],
            input_keys=['put_down_size', 'shelf_height_dict'],
            output_keys=[]);
    
        self.buffer = tf2_ros.Buffer();
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster();
        self.listener = tf2_ros.TransformListener(self.buffer);
    
    def execute(self, userdata):
        PLACEMENT_TF_NAME = "placement_tf_TLP"

        rospy.logwarn("Using the backup for finding placement locations. Something has failed along the way.");
        shelf_height_dict:dict = userdata.shelf_height_dict;

        heights = shelf_height_dict["heights"];
        shelf_names:List[str] = shelf_height_dict["tf_names"];


        find_placement_on_empty_service = rospy.ServiceProxy(
            "/FindPlacementOnEmptySurface", manipulation.srv.FindPlacementOnEmptySurface);
        central_tf = shelf_names[ int(len(shelf_names)/2) ];

        trans_stamped = self.tf_buffer.lookup_transform("head_rgbd_sensor_rgb_frame", central_tf, rospy.Time(), timeout=rospy.Duration(1));
        rgbd_goal_transform = trans_stamped.transform;

        res:manipulation.srv.FindPlacementOnEmptySurfaceResponse = find_placement_on_empty_service(
            userdata.put_down_size,     # dimension of object
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
        put_obj_on_surface_goal.object_half_height = userdata.put_down_size.z/2;
        for i in range(len(shelf_names)):
            success = putObjOnSurfaceAction(put_obj_on_surface_goal);
            if success:
                return SUCCESS;
            elif using_regions:
                min_index += 1;
                min_index %= len(shelf_names);
                put_obj_on_surface_goal.goal_tf = shelf_names[min_index];


        # If everything has failed, drop the item?

        return SUCCESS;



def sub_state_machine_pick_up_and_put_away():

    sm = smach.StateMachine(
        outcomes = [TASK_SUCCESS, TASK_FAILURE]
    );

    while (not rospy.has_param('params_loaded')):
        rospy.sleep(0.5);
    
    execute_nav_commands = True;
    if rospy.has_param('execute_navigation_commands'):
        if rospy.get_param('execute_navigation_commands') == False:
            execute_nav_commands = False;
    
    sm.userdata.pick_up_class = "apple";
    sm.userdata.place_next_to_class = "bottle";

    

    with sm:
        # smach.StateMachine.add(
        #    'Startup',
        #    create_wait_for_startup(),
        #    transitions={SUCCESS:'PickUpObj'});

        smach.StateMachine.add(
            'MoveToNeutral',#
            MoveToNeutralState(),
            transitions={SUCCESS:'PickUpObj'});
    
        smach.StateMachine.add(
            'PickUpObj',
            nav_and_pick_up_or_place_next_to(execute_nav_commands, pick_up=True),
            transitions={
                SUCCESS:'PlaceObjNextTo',
                FAILURE:TASK_FAILURE,
                'query_empty':TASK_FAILURE,
                MANIPULATION_FAILURE:TASK_FAILURE},
            remapping={'obj_type':'pick_up_class'});
    
        smach.StateMachine.add(
            'PlaceObjNextTo',
            nav_and_pick_up_or_place_next_to(execute_nav_commands, pick_up=False),
            transitions={
                SUCCESS:TASK_SUCCESS,
                FAILURE:TASK_FAILURE,
                'query_empty':TASK_FAILURE,
                MANIPULATION_FAILURE:TASK_FAILURE},
            remapping={'obj_type':'place_next_to_class'});

    return sm;



def create_state_machine():

    sm = smach.StateMachine(
        outcomes = [TASK_SUCCESS, TASK_FAILURE]
    );


    while (not rospy.has_param('params_loaded')):
        rospy.sleep(0.5);
    
    execute_nav_commands = True;
    if rospy.has_param('execute_navigation_commands'):
        if rospy.get_param('execute_navigation_commands') == False:
            execute_nav_commands = False;
    
    num_objects_to_put_away = 5

    sm.userdata.objects_placed = 0

    if rospy.has_param('cabinet_pose') and rospy.has_param('table_pose'):
        sm.userdata.cabinet_pose = utils.dict_to_obj(rospy.get_param('cabinet_pose'), Pose());
        sm.userdata.table_pose = utils.dict_to_obj(rospy.get_param('table_pose'), Pose());
    else:
        print("Cabinet pose and Table pose not found. Ros params not fully loaded.");
        raise Exception("Cabinet pose and Table pose not found. Ros params not fully loaded");
    
    if rospy.has_param("categories_to_pick_up"):
        categories_to_pick_up = rospy.get_param('categories_to_pick_up');
    else:
        print("Categories to pick up are not in the ros parameter list. Ros params not fully loaded.");
        raise Exception("Categories to pick up are not in the ros parameter list. Ros params not fully loaded.");

    min_num_observations = 0;
    if rospy.has_param('min_number_of_observations'):
        min_num_observations = rospy.get_param('min_number_of_observations');

    if rospy.has_param('table_mast_height') and rospy.has_param('cabinet_mast_height'):
        table_mast_height = rospy.get_param('table_mast_height');
        cabinet_mast_height = rospy.get_param('cabinet_mast_height');
    else:
        table_mast_height = 0;
        cabinet_mast_height = 0.5;

    transform_broadcaster = tf2_ros.StaticTransformBroadcaster();

    interactive_marker_server = InteractiveMarkerServer("zzz_task_level_planning/markers")
    visualisation_manager = RvizVisualisationManager(
        im_server=interactive_marker_server,
        colour_a=0.7, colour_r=0.0, colour_g=1.0, colour_b=0.2)
    if rospy.has_param('shelves_hardcoded'):
        shelves_hardcoded_dict:dict = rospy.get_param('shelves_hardcoded');
        shelves_z_vals = shelves_hardcoded_dict["z_vals"];
        shelves_width = shelves_hardcoded_dict["width"];
        shelves_depth = shelves_hardcoded_dict["depth"];
        shelves_pose = utils.dict_to_obj(shelves_hardcoded_dict["shelf_pose"], Pose());
        
        shelf_names = [];
        transforms_publishing = [];
        shelf_heights = [];
        
        for i, z_height in enumerate(shelves_z_vals):
            shelf_name = "shelf_{0}".format(i);
            shelf_names.append(shelf_name);
            individual_shelf_pose:Pose = copy.deepcopy(shelves_pose);
            individual_shelf_pose.position.z = z_height;
            shelf_heights.append(z_height);
            visualisation_manager.add_object(
                shelf_name,
                individual_shelf_pose,
                size=Point(shelves_width, shelves_depth, 0.01),
                obj_class=shelf_name,
                alpha_val=0.8,
                marker_type=Marker.CUBE);
            individual_tf = geometry_msgs.msg.TransformStamped();
            individual_tf.header.stamp = rospy.Time.now();
            individual_tf.header.frame_id = "map";
            individual_tf.child_frame_id = shelf_name;
            individual_tf.transform.translation = individual_shelf_pose.position;
            individual_tf.transform.rotation = individual_shelf_pose.orientation;
            transforms_publishing.append(individual_tf);
        
        transform_broadcaster.sendTransform(transforms_publishing);
        
        sm.userdata.shelf_height_dict = {
            "heights":shelf_heights,
            "tf_names":shelf_names}
        
        # input();
        
        som_region_delete_all = rospy.ServiceProxy( "/som/object_regions/delete_entries", std_srvs.srv.Empty );
        som_region_delete_all( std_srvs.srv.EmptyRequest() );
        som_region_add_basic = rospy.ServiceProxy( "/som/object_regions/input", SOMAddRegion );
        som_region_basic_query = rospy.ServiceProxy( "/som/object_regions/basic_query", SOMQueryRegions )

        for i in range(len(shelf_heights)):
            region_adding = SOMAddRegionRequest();
            region_adding.adding.dimension.x = shelves_width;
            region_adding.adding.dimension.y = shelves_depth;
            region_adding.adding.dimension.z = shelf_heights[i+1]-shelf_heights[i]-0.01 if i+1<len(shelf_heights) else 0.3;
            # region_adding.adding.corner_loc = copy.deepcopy( shelves_pose );
            region_adding.adding.corner_loc.translation = copy.deepcopy( shelves_pose.position );
            region_adding.adding.corner_loc.rotation = copy.deepcopy( shelves_pose.orientation );
            # We need a rotation in here somewhere. It should be - :(
            roll, pitch, yaw = euler_from_quaternion( [shelves_pose.orientation.x, shelves_pose.orientation.y, shelves_pose.orientation.z, shelves_pose.orientation.w ] );
            print(roll, pitch, yaw);
            translation_x = math.cos(yaw)*shelves_width - math.sin(yaw)*shelves_depth;
            translation_y = math.sin(yaw)*shelves_width + math.cos(yaw)*shelves_depth;
            region_adding.adding.corner_loc.translation.x += translation_x/2;
            region_adding.adding.corner_loc.translation.y -= translation_y/2;
            region_adding.adding.corner_loc.translation.z = shelf_heights[i];
            region_adding.adding.name = shelf_names[i];

            # print(region_adding);

            som_region_add_basic( region_adding );
        # print(som_region_basic_query( SOMQueryRegionsRequest() ));
        
        use_hardcoded_shelves:bool = rospy.get_param('use_hardcoded_shelves');
    else:
        rospy.logwarn("shelves_hardcoded not found.")

    # Keeps track of the tf strings that have been picked up/attempted to
    # be picked up so as to not get stuck in an infinite loop.
    sm.userdata.filter_tf_names_out = [];
    sm.userdata.class_name_filter = rospy.get_param('class_name_filter')

    
    def performSOMQueryAtTable():
        """
        This is to make the main body of the state machine cleaner and easier to work with.
        This itself will have feedback loops etc, some of which will feedback into themselves, 
        and some which will be external.
        Oututs:
            pick_up_object_class:str
            put_down_category:str
            som_query_results:List[SOMObject]
            tf_name:str
            put_down_size:geometry_msgs.msg.Point
        """
        sub_sm = smach.StateMachine(
            outcomes=["PickUpObj", FAILURE],
            input_keys=[
                'filter_tf_names_out',
                'class_name_filter'],
            output_keys=[
                'pick_up_object_class',
                'put_down_category',
                'som_query_results',
                'tf_name',
                'put_down_size',
                'obj_pose'
            ]);
        
        with sub_sm:
            # Setting up the query.
            smach.StateMachine.add(
                'CreateTableQuery',
                CreateSOMQuery(
                    CreateSOMQuery.OBJECT_QUERY, 
                    save_time=True),
                transitions={SUCCESS: 'AddMinObservationsTable'});
            smach.StateMachine.add(
                'AddMinObservationsTable',
                AddSOMEntry('num_observations', min_num_observations),
                transitions={
                    SUCCESS:'RaiseMastAtTable'});
            
            smach.StateMachine.add(
                'RaiseMastAtTable',
                RaiseMastState(table_mast_height),
                transitions={
                    SUCCESS:'SpinWhileAtTable'
                });
            smach.StateMachine.add(
                'SpinWhileAtTable',
                SpinState(spin_height=0.7, only_look_forwards=True, offset_instruction=SpinState.TAKE_OFFSET_FROM_USERDATA),
                transitions={
                    SUCCESS:'PerformQuery'},
                remapping={});

            smach.StateMachine.add(
                'PerformQuery',
                PerformSOMQuery(distance_filter=2),
                transitions={
                    SUCCESS: 'FilterOutObjNames'},
                remapping={})

            smach.StateMachine.add(
                'FilterOutObjNames',
                FilterSOMResultsAsPer('class_'),
                transitions={SUCCESS:'FilterOutTfs'},
                remapping={'filtering_by':'class_name_filter'})
            smach.StateMachine.add(
                'FilterOutTfs',
                FilterSOMResultsAsPer('tf_name'),
                transitions={SUCCESS:'SortListInput'},
                remapping={'filtering_by':'filter_tf_names_out'});
            smach.StateMachine.add(
                'SortListInput',
                SortSOMResultsAsPer(
                    'category', 
                    categories_to_pick_up, 
                    sort_by_num_observations_first=True, 
                    num_observations_filter_proportion=0.001, 
                    filter_for_duplicates_distance=0.02),
                transitions={
                    SUCCESS:'GetObjectToPickUp',
                    'list_empty':'ClearTfNameFilter'},
                remapping={'som_query_results_out':'som_query_results'});
            
            smach.StateMachine.add(
                'ClearTfNameFilter',
                SetToEmptyList(),
                transitions={SUCCESS:'ExplicitRemap'},
                remapping={'setting':'filter_tf_names_out'});
            smach.StateMachine.add(
                'ExplicitRemap',
                ExplicitRemap(),
                transitions={SUCCESS:'SortListInput_2'},
                remapping={
                    'in_key':'som_query_results',
                    'out_key':'som_query_results_1'})       # I think this is actually necessary given the use of som_query_results_out twice below.
            smach.StateMachine.add(
                'SortListInput_2',
                SortSOMResultsAsPer(
                    'category', 
                    categories_to_pick_up, 
                    sort_by_num_observations_first=True, 
                    num_observations_filter_proportion=0.001,
                    filter_for_duplicates_distance=0.02),
                transitions={
                    SUCCESS:'GetObjectToPickUp',
                    'list_empty':'CreateTableQuery'},
                remapping={
                    'som_query_results':'som_query_results_1',
                    'som_query_results_out':'som_query_results'});

            """
            Outputs from this state:
                pick_up_object_class:str
                put_down_category:str
                tf_name:str
                put_down_size:geometry_msgs.msg.Point
            """
            smach.StateMachine.add(
                'GetObjectToPickUp',
                GetPropertyAtIndex(properties_getting=['class_', 'category', 'tf_name', 'size', 'obj_position'], index=0),
                transitions={
                    SUCCESS:'TellOperatorClassCategory',
                    'index_out_of_range':'ClearTfNameFilter'},
                remapping={
                    'input_list':'som_query_results',
                    'class_':'pick_up_object_class',
                    'category':'put_down_category',
                    'size':'put_down_size',
                    'obj_position':'obj_pose'});
            
            smach.StateMachine.add(
                'TellOperatorClassCategory',
                SayArbitraryPhrase( 
                    "Trying to pick up the {0} of category {1}.",
                    ["pick_up_object_class", "put_down_category"]),
                    transitions={SUCCESS:"PickUpObj"});
        return sub_sm;


    with sm:
        smach.StateMachine.add(
            'Startup',
            create_wait_for_startup(),
            transitions={SUCCESS:'MoveToNeutral'});

        # NOTE: Startup state machine.
        # NOTE: Needs changing to nav-to-pose
        # smach.StateMachine.add(
        #     'NavToTable',
        #     navigate_within_distance_of_pose_input(execute_nav_commands),
        #     transitions={
        #         SUCCESS: 'CreateTableQuery',
        #         NAVIGATIONAL_FAILURE: TASK_FAILURE
        #     },
        #     remapping={'target_pose': 'table_pose'})
        smach.StateMachine.add(
            'MoveToNeutral',
            MoveToNeutralState(),
            transitions={SUCCESS:'NavToTable'});

        smach.StateMachine.add(
            'NavToTable',
            SimpleNavigateState_v2(execute_nav_commands),
            transitions={
                SUCCESS: 'PerformSOMAtTable',
                NAVIGATIONAL_FAILURE: TASK_FAILURE
            },
            remapping={'pose': 'table_pose'})
        
        smach.StateMachine.add(
            'PerformSOMAtTable',
            performSOMQueryAtTable(),
            transitions={
                'PickUpObj':'LookAtObj',
                FAILURE:TASK_FAILURE});
        
        smach.StateMachine.add(
            'LookAtObj',
            LookAtPoint(wait_duration_afterwards=rospy.Duration(0.5)),
            remapping={'pose':'obj_pose'},
            transitions={
                SUCCESS:'PickUpObj'});

        smach.StateMachine.add(
            'PickUpObj',
            nav_and_pick_up_or_place_next_to(execute_nav_commands, pick_up=True, som_query_already_performed=True),
            transitions={
                SUCCESS:             'NavToCabinet',
                FAILURE:             'AppendTfNameToTfNameFilter',
                'query_empty':       'AppendTfNameToTfNameFilter',
                MANIPULATION_FAILURE:'AppendTfNameToTfNameFilter'},
            remapping={'obj_type':'pick_up_object_class'});

        # Simple Nav state.
        # smach.StateMachine.add(
        #     'NavToCabinet',
        #     navigate_within_distance_of_pose_input(execute_nav_commands),
        #     transitions={
        #         SUCCESS: 'PutAwayObject',
        #         NAVIGATIONAL_FAILURE: TASK_FAILURE
        #     },
        #     remapping={
        #         'target_pose': 'cabinet_pose'
        #     })
        smach.StateMachine.add(
            'NavToCabinet',
            SimpleNavigateState_v2(execute_nav_commands),
            transitions={
                SUCCESS: 'PutAwayObject',       # 'RaiseMastAtCabinet',
                NAVIGATIONAL_FAILURE: TASK_FAILURE
            },
            remapping={'pose': 'cabinet_pose'})
        
        smach.StateMachine.add(
            'RaiseMastAtCabinet',
            RaiseMastState(cabinet_mast_height),
            transitions={
                SUCCESS:'PutAwayObject'
            });

        smach.StateMachine.add(
            'PutAwayObject',
            nav_and_pick_up_or_place_next_to(
                execute_nav_commands, pick_up=False, 
                find_same_category=True, 
                input_obj_size_for_place=True,
                input_hardcoded_shelf_for_placement=True),
            transitions={
                SUCCESS:             'IncreaseNumber',
                FAILURE:             'PlaceSpeechBackup',  #'FindEmptyShelfAndPlace',
                'query_empty':       'PlaceSpeechBackup',  #'FindEmptyShelfAndPlace',
                MANIPULATION_FAILURE:'PlaceSpeechBackup'},  #'PlacementBackup'},
            remapping={'obj_type':'put_down_category'})
        
        smach.StateMachine.add(
            "PlaceSpeechBackup",
            PlaceSpeechBackup(),
            transitions={SUCCESS:"AppendTfNameToTfNameFilter"})

        smach.StateMachine.add(
            'PlacementBackup',
            FindPlacementLocationBackup(),
            transitions={
                SUCCESS:             'IncreaseNumber',
                FAILURE:             'FindEmptyShelfAndPlace',
                MANIPULATION_FAILURE:'FindEmptyShelfAndPlace'})

        # Failsafe
        smach.StateMachine.add(
            'FindEmptyShelfAndPlace',
            FindShelfBackup(),
            transitions={SUCCESS:'AppendTfNameToTfNameFilter'});

        smach.StateMachine.add(
            'AppendTfNameToTfNameFilter',
            AppendToArrState(),
            transitions={SUCCESS:'IncreaseNumber'},
            remapping={
                'appending_to':  'filter_tf_names_out',
                'appending_with':'tf_name'});

        smach.StateMachine.add(
            'IncreaseNumber',
            IncrementValue(1),
            transitions={
                SUCCESS: 'CheckIfShouldContinue',
            },
            remapping={'val': 'objects_placed'})


        smach.StateMachine.add(
            'CheckIfShouldContinue',
            LessThanState(right=num_objects_to_put_away),
            transitions={
                TRUE_STR: 'NavToTable',
                FALSE_STR: TASK_SUCCESS
            },
            remapping={
                'left': 'objects_placed'
            });
    



    return sm


if __name__ == '__main__':
    rospy.init_node('pick_up_and_put_away_state_machine');
    
    # sm = sub_state_machine_pick_up_and_put_away();
    sm = create_state_machine();

    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    sm.execute();

