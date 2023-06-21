#!/usr/bin/env python3
"""
Code for putting away the groceries.

Will observe items and transfer them from one area to another.

This makes EXTENSIVE use of the ros parameter server. Look within config/put_away_the_groceries.yaml to find
a list of all the parameters with explanations.

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

    # Keeps track of the tf strings that have been picked up/attempted to
    # be picked up so as to not get stuck in an infinite loop.
    sm.userdata.filter_tf_names_out = []; 

    with sm:
        # NOTE: Startup state machine.
        # NOTE: Needs changing to nav-to-pose
        # smach.StateMachine.add(
        #     'NavToTable',
        #     navigate_within_distance_of_pose_input(execute_nav_commands),
        #     transitions={
        #         SUCCESS: 'CreateTableQuery',
        #         FAILURE: TASK_FAILURE
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
                SUCCESS: 'CreateTableQuery',
                FAILURE: TASK_FAILURE
            },
            remapping={'pose': 'table_pose'})
        

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
            SpinState(spin_height=0.7, only_look_forwards=True),
            transitions={
                SUCCESS:'PerformQuery'},
            remapping={});

        smach.StateMachine.add(
            'PerformQuery',
            PerformSOMQuery(distance_filter=2),
            transitions={
                SUCCESS: 'SortListInput'},
            remapping={})

        smach.StateMachine.add(
            'FilterOutTfs',
            FilterSOMResultsAsPer('tf_name'),
            transitions={SUCCESS:'SortListInput'},
            remapping={'filtering_by':'filter_tf_names_out'});
        smach.StateMachine.add(
            'SortListInput',
            SortSOMResultsAsPer('category', categories_to_pick_up, sort_by_num_observations_first=True, 
                                num_observations_filter_proportion=0.001),
            transitions={
                SUCCESS:'GetObjectToPickUp',
                'list_empty':'ClearTfNameFilter'},
            remapping={'som_query_results_out':'som_query_results'});
        
        smach.StateMachine.add(
            'ClearTfNameFilter',
            SetToEmptyList(),
            transitions={SUCCESS:'SortListInput_2'},
            remapping={'setting':'filter_tf_names_out'});
        smach.StateMachine.add(
            'SortListInput_2',
            SortSOMResultsAsPer('category', categories_to_pick_up, sort_by_num_observations_first=True, 
                                num_observations_filter_proportion=0.001),
            transitions={
                SUCCESS:'GetObjectToPickUp',
                'list_empty':'CreateTableQuery'},
            remapping={
                'som_query_results':'som_query_results_old',
                'som_query_results_out':'som_query_results'});

        smach.StateMachine.add(
            'GetObjectToPickUp',
            GetPropertyAtIndex(properties_getting=['class_', 'category', 'tf_name'], index=0),
            transitions={
                SUCCESS:'TellOperatorClassCategory',
                'index_out_of_range':'ClearTfNameFilter'},
            remapping={
                'input_list':'som_query_results',
                'class_':'pick_up_object_class',
                'category':'put_down_category'});
        
        smach.StateMachine.add(
            'TellOperatorClassCategory',
            SayArbitraryPhrase( 
                "Trying to pick up the {0} of category {1}.",
                ["pick_up_object_class", "put_down_category"]),
                transitions={SUCCESS:"PickUpObj"});

        smach.StateMachine.add(
            'PickUpObj',
            nav_and_pick_up_or_place_next_to(execute_nav_commands, pick_up=True, som_query_already_performed=True),
            transitions={
                SUCCESS:'NavToCabinet',
                FAILURE:TASK_FAILURE,
                'query_empty':TASK_FAILURE,
                MANIPULATION_FAILURE:'AppendTfNameToTfNameFilter'},
            remapping={'obj_type':'pick_up_object_class'})

        # Simple Nav state.
        # smach.StateMachine.add(
        #     'NavToCabinet',
        #     navigate_within_distance_of_pose_input(execute_nav_commands),
        #     transitions={
        #         SUCCESS: 'PutAwayObject',
        #         FAILURE: TASK_FAILURE
        #     },
        #     remapping={
        #         'target_pose': 'cabinet_pose'
        #     })
        smach.StateMachine.add(
            'NavToCabinet',
            SimpleNavigateState_v2(execute_nav_commands),
            transitions={
                SUCCESS: 'PutAwayObject',       # 'RaiseMastAtCabinet',
                FAILURE: TASK_FAILURE
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
            nav_and_pick_up_or_place_next_to(execute_nav_commands, pick_up=False, find_same_category=True),
            transitions={
                SUCCESS: 'IncreaseNumber',
                FAILURE:TASK_FAILURE,
                'query_empty':TASK_FAILURE,
                MANIPULATION_FAILURE:'AppendTfNameToTfNameFilter'},
            remapping={'obj_type':'put_down_category'})
        
        smach.StateMachine.add(
            'AppendTfNameToTfNameFilter',
            AppendToArrState(),
            transitions={SUCCESS:'IncreaseNumber'},
            remapping={
                'appending_to':'filter_tf_names_out',
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
    # sm = sub_state_machine_pick_up_and_put_away();
    sm = create_state_machine();

    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    sm.execute();

