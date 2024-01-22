#!/usr/bin/env python3
import rospy;
import smach_ros;
import actionlib;
import std_srvs.srv;
import manipulation.srv;

from state_machines.SubStateMachines.include_all import *;

from typing import List;

from state_machines.robocup_2024.put_away_the_groceries.startup import StartupWaitForDoor
from state_machines.robocup_2024.put_away_the_groceries.choose_object_to_pick_up import ChooseObjectToPickUp
from state_machines.robocup_2024.put_away_the_groceries.pick_up_object import PickUpObject
from state_machines.robocup_2024.put_away_the_groceries.check_task_finished import CheckTaskFinished
from state_machines.robocup_2024.put_away_the_groceries.put_down_object import PutDownObject


def create_state_machine():

    num_objects_to_put_away = 5

    execute_nav_commands = True;
    if rospy.has_param('execute_navigation_commands'):
        if rospy.get_param('execute_navigation_commands') == False:
            execute_nav_commands = False;

    sm = smach.StateMachine(
        outcomes = [TASK_SUCCESS, TASK_FAILURE]
    );

    if rospy.has_param('cabinet_pose') and rospy.has_param('table_pose'):
        sm.userdata.cabinet_pose = utils.dict_to_obj(rospy.get_param('cabinet_pose'), Pose());
        sm.userdata.table_pose = utils.dict_to_obj(rospy.get_param('table_pose'), Pose());
    else:
        print("Cabinet pose and Table pose not found. Ros params not fully loaded.");
        raise Exception("Cabinet pose and Table pose not found. Ros params not fully loaded");

    min_num_observations = 0;
    if rospy.has_param('min_number_of_observations'):
        min_num_observations = rospy.get_param('min_number_of_observations');
    
    if rospy.has_param('table_mast_height') and rospy.has_param('cabinet_mast_height'):
        table_mast_height = rospy.get_param('table_mast_height');
        cabinet_mast_height = rospy.get_param('cabinet_mast_height');
    else:
        table_mast_height = 0;
        cabinet_mast_height = 0.5;
    
    if rospy.has_param("categories_to_pick_up"):
        categories_to_pick_up = rospy.get_param('categories_to_pick_up');
    else:
        print("Categories to pick up are not in the ros parameter list. Ros params not fully loaded.");
        raise Exception("Categories to pick up are not in the ros parameter list. Ros params not fully loaded.");

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
            "tf_names":shelf_names
        }
        
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
            roll, pitch, yaw = euler_from_quaternion( [
                shelves_pose.orientation.x, 
                shelves_pose.orientation.y, 
                shelves_pose.orientation.z, 
                shelves_pose.orientation.w ] );
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


    with sm:
        smach.StateMachine.add(
            'Startup',
            StartupWaitForDoor(),
            transitions={
                SUCCESS:'MoveToTable', 
                });

        smach.StateMachine.add(
            'MoveToTable',
            SimpleNavigateState_v2(execute_nav_commands),
            transitions={
                SUCCESS: 'ChooseObjectToPickUp',
                NAVIGATIONAL_FAILURE: TASK_FAILURE
            },
            remapping={'pose': 'table_pose'})

        smach.StateMachine.add(
            'ChooseObjectToPickUp',
            ChooseObjectToPickUp(min_num_observations, table_mast_height, categories_to_pick_up),
            transitions={
                SUCCESS: 'PickUpObject'
            }
        )

        smach.StateMachine.add(
            'PickUpObject',
            PickUpObject(execute_nav_commands),
            transitions={
                SUCCESS: 'NavToCabinet',
                NAVIGATIONAL_FAILURE: TASK_FAILURE,
                MANIPULATION_FAILURE: 'CheckIfFinished'
            }
        )

        smach.StateMachine.add(
            "CheckIfFinished",
            CheckTaskFinished(num_objects_to_put_away),
            transitions={
                "continue": "MoveToTable",
                "task_finished": TASK_SUCCESS
            }
        )

        smach.StateMachine.add(
            'NavToCabinet',
            SimpleNavigateState_v2(execute_nav_commands),
            transitions={
                SUCCESS: 'PutDownObject',       # 'RaiseMastAtCabinet',
                NAVIGATIONAL_FAILURE: TASK_FAILURE
            },
            remapping={'pose': 'cabinet_pose'})

        smach.StateMachine.add(
            'PutDownObject',
            PutDownObject(execute_nav_commands,
                          cabinet_mast_height),
            transitions={
                SUCCESS: 'CheckIfFinished'
            }
        )


    return sm


if __name__ == '__main__':
    rospy.init_node('put_away_the_groceries_state_machine');
    
    sm = create_state_machine();

    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    sm.execute();