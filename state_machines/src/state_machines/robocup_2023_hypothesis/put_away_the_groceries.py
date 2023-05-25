#!/usr/bin/env python3
"""
Code for putting away the groceries.

Will observe items and transfer them from one area to another.

Notes:
    - We don't need to look around each time. 
"""

import rospy;
import smach_ros;
import actionlib;

from state_machines.SubStateMachines.include_all import *;

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
    
    sm.userdata.pick_up_class = "bottle";
    sm.userdata.place_next_to_class = "cup";

    with sm:
        # smach.StateMachine.add(
        #    'Startup',
        #    create_wait_for_startup(),
        #    transitions={SUCCESS:'PickUpObj'});
    
        smach.StateMachine.add(
            'PickUpObj',
            nav_and_pick_up_or_place_next_to(execute_nav_commands, pick_up=True),
            transitions={
                SUCCESS:'PlaceObjNextTo',
                FAILURE:TASK_FAILURE,
                'query_empty':TASK_FAILURE},
            remapping={'obj_type':'pick_up_class'});
    
        smach.StateMachine.add(
            'PlaceObjNextTo',
            nav_and_pick_up_or_place_next_to(execute_nav_commands, pick_up=False),
            transitions={
                SUCCESS:TASK_SUCCESS,
                FAILURE:TASK_FAILURE,
                'query_empty':TASK_FAILURE},
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


    with sm:

        smach.StateMachine.add(
            'NavToTable',
            navigate_within_distance_of_pose_input(execute_nav_commands),
            transitions={
                SUCCESS: 'CreateTableQuery',
                FAILURE: TASK_FAILURE
            },
            remapping={
                'target_pose': 'table_pose'
            }
        )

        smach.StateMachine.add(
            'CreateTableQuery',
            CreateSOMQuery(
                CreateSOMQuery.OBJECT_QUERY, 
                save_time=True),
            transitions={
                SUCCESS: 'LookAtTable'},
            remapping={}
        )

        smach.StateMachine.add(
            'LookAtTable',
            SpinState(spin_height=0.7, only_look_forwards=True),
            transitions={
                SUCCESS:'PerformQuery'},
            remapping={});

        smach.StateMachine.add(
            'PerformQuery',
            PerformSOMQuery(distance_filter=4),
            transitions={
                SUCCESS: 'SortListInput',
                FAILURE: TASK_FAILURE},
            remapping={}
        )

        smach.StateMachine.add(
            'SortListInput',
            SortSOMResultsAsPer('category', categories_to_pick_up),
            transitions={
                SUCCESS:'GetObjectToPickUp',
                'list_empty':TASK_FAILURE},
            remapping={});

        smach.StateMachine.add(
            'GetObjectToPickUp',
            GetPropertyAtIndex(property_getting='obj_class', index=0),
            transitions={
                SUCCESS:'PickUpObj',
                'index_out_of_range':TASK_FAILURE},
            remapping={
                'input_list':'som_query_results',
                'output_param':'pick_up_object_class'}
        )

        smach.StateMachine.add(
            'PickUpObj',
            nav_and_pick_up_or_place_next_to(execute_nav_commands, pick_up=True),
            transitions={
                SUCCESS:'NavToCabinet',
                FAILURE:TASK_FAILURE,
                'query_empty':TASK_FAILURE},
            remapping={'obj_type':'pick_up_object_class'}
        )

        smach.StateMachine.add(
            'NavToCabinet',
            navigate_within_distance_of_pose_input(execute_nav_commands),
            transitions={
                SUCCESS: 'GetObjectCategory',
                FAILURE: TASK_FAILURE
            },
            remapping={
                'target_pose': 'cabinet_pose'
            }
        )
        # use category
        smach.StateMachine.add(
            'GetObjectCategory',
            GetPropertyAtIndex(property_getting='obj_class', index=0),
            transitions={
                SUCCESS:'PutAwayObject',
                'index_out_of_range':TASK_FAILURE},
            remapping={
                'input_list':'som_query_results',
                'output_param':'put_down_category'}
        )

        smach.StateMachine.add(
            'PutAwayObject',
            nav_and_pick_up_or_place_next_to(execute_nav_commands, pick_up=False, find_same_category=True),
            transitions={
                SUCCESS: 'IncreaseNumber',
                FAILURE:TASK_FAILURE,
                'query_empty':TASK_FAILURE},
            remapping={'obj_type':'put_down_category'}
        )
        
        smach.StateMachine.add(
            'IncreaseNumber',
            IncrementValue(1),
            transitions={
                SUCCESS: 'CheckIfShouldContinue',
            },
            remapping={'val': 'objects_placed'}
        )

        smach.StateMachine.add(
            'CheckIfShouldContinue',
            LessThanState(right=num_objects_to_put_away),
            transitions={
                TRUE_STR: 'NavToTable',
                FALSE_STR: TASK_SUCCESS
            },
            remapping={
                'left': 'objects_placed'
            }
        )

    return sm


if __name__ == '__main__':
    sm = sub_state_machine_pick_up_and_put_away();

    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    sm.execute();

