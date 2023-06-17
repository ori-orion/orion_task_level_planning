#!/usr/bin/env python3
"""
Author: Matthew Munks
Maintainer: Matthew Munks

This is the set of sub-state machines surrounding picking things up and putting them down next to other things.

Sub state machines and their purposes:
 - navigate_within_distance_of_pose_input   - Navigate to a point within a given distance of a point so that you might
                                            hypothetically be able to pick up or put something down at that location.
 - navigate_within_distance_of_som_input    - Navigate to the location given by a SOM query, in way that you might
                                            hypothetically be able to pick up or put something down at that location.
 - search_for_entity                        - Searches for a given 
 - nav_within_reaching_distance_of          - 
 - nav_and_pick_up_or_place_next_to         - 
"""

import smach; 
from state_machines.Reusable_States.include_all import *;

def navigate_within_distance_of_pose_input(execute_nav_commands):
    """
    Basically the core of this set of sub-state machines.
    Looks at userdata.target_pose, works out the position of the nav goal and navigates there.
    Dependancies (28/4/2023):
        navigate_within_distance_of_som_input   - Not in use.
        nav_within_reaching_distance_of         - Used by anything using this functionality.
    """
    sub_sm = smach.StateMachine(
        outcomes=[SUCCESS, FAILURE],
        input_keys=['target_pose']);

    sub_sm.userdata.number_of_failures = 0;
    sub_sm.userdata.failure_threshold =3;

    with sub_sm:
        smach.StateMachine.add(
            "LookAtObject",
            LookAtPoint(z_looking_at=0.9),
            transitions={
                SUCCESS:'FindNavGoal'},
            remapping={
                'pose':'target_pose'});


        smach.StateMachine.add(
            'FindNavGoal',
            NavigateDistanceFromGoalSafely(),
            transitions={
                SUCCESS:'NavToGoal',
                "skip_navigation":SUCCESS},
            remapping={
                'pose':'target_pose',
                'nav_target':'nav_target'
            });

        smach.StateMachine.add(
            'NavToGoal',
            SimpleNavigateState(execute_nav_commands),
            transitions={
                SUCCESS:'LookAtObject_AGAIN',
                FAILURE:'NavToGoal',
                REPEAT_FAILURE: FAILURE},
            remapping={
                'pose':'nav_target'
            });

        smach.StateMachine.add(
            "LookAtObject_AGAIN",
            LookAtPoint(z_looking_at=0.9),
            transitions={
                SUCCESS:SUCCESS},
            remapping={
                'pose':'target_pose'});

    return sub_sm;

# Not currently in use.
# Deprecated
def navigate_within_distance_of_som_input(execute_nav_commands):
    """
    Navigates to a close distance from the first element that comes up from the query.

    outcomes:
        SUCCESS         : Self explanatory
        FAILURE         : Self explanatory
        'query_empty'   : If the query has nothing in it, then this is the response.
    inputs:
        som_query       : The query we will give the SOM system.
    Dependencies (28/4/2023):
        None - Deprecated. There are no current plans to use this. 
               nav_within_reaching_distance_of covers this functionality.
    """
    sub_sm = smach.StateMachine(
        outcomes=[SUCCESS, FAILURE, 'query_empty'],
        input_keys=['som_query']);

    with sub_sm:
        smach.StateMachine.add(
            'PerformQuery',
            PerformSOMQuery(),
            transitions={
                SUCCESS:'GetLocation'},
            remapping={});

        # Here we have som_query_results:List[...]. We want to get the first item in this query, and then
        # the position out of that as a node to navigate to. We can then navigate to that location. 

        smach.StateMachine.add(
            'GetLocation',
            GetPropertyAtIndex(properties_getting=['obj_position'], index=0),
            transitions={
                SUCCESS:'NavToLoc',
                'index_out_of_range':'query_empty'},
            remapping={
                'input_list':'som_query_results',
                'obj_position':'target_pose'});

        smach.StateMachine.add(
            'NavToLoc',
            navigate_within_distance_of_pose_input(execute_nav_commands),
            transitions={
                SUCCESS:SUCCESS,
                FAILURE:FAILURE});

    return sub_sm;


def search_for_entity(spin_first=True, find_same_category=False):
    """
    Spins on the spot in the persuit of seeing an object. 
    It will then query for said object. 
    If an object matching `userdata.obj_type` is seen, then a list of all items matching the query will be returned.
    Otherwise, an empty array will be returned.
    Dependencies (28/4/2023):
        nav_within_reaching_distance_of   
    """
    sub_sm = smach.StateMachine(
        outcomes=[SUCCESS, FAILURE, "item_not_seen"],
        input_keys=['obj_type'],
        output_keys=['som_query_results']);

    search_for_query_type = 'category' if find_same_category else 'class_'
    
    with sub_sm:
        def createSpinAndQuery(subsequent_state:SUCCESS):
            smach.StateMachine.add(
                'CreateObjQuery',
                CreateSOMQuery(
                    CreateSOMQuery.OBJECT_QUERY, 
                    save_time=True),
                transitions={
                    SUCCESS: 'AddEntryToSOMQuery'},
                remapping={});
            
            smach.StateMachine.add(
                'AddEntryToSOMQuery',
                AddSOMEntry(
                    field_adding_default=search_for_query_type),
                transitions={
                    SUCCESS: 'SpinOnSpot'},
                remapping={'value' :'obj_type'});

            smach.StateMachine.add(
                'SpinOnSpot',
                SpinState(spin_height=0.7, only_look_forwards=True),
                transitions={
                    SUCCESS:'PerformQuery'},
                remapping={});

            smach.StateMachine.add(
                'PerformQuery',
                PerformSOMQuery(distance_filter=4),
                transitions={
                    SUCCESS:'CheckSeenObject'},
                remapping={});
            
            smach.StateMachine.add(
                'CheckSeenObject',
                GetListEmpty(),
                transitions={
                    'list_not_empty': SUCCESS,
                    'list_empty': subsequent_state
                },
                remapping={
                    'input_list':'som_query_results'
                });
        
        def createAllTimeQuery(subsequent_state:SUCCESS):
            smach.StateMachine.add(
                'CreateAllTimeQuery',
                CreateSOMQuery(
                    CreateSOMQuery.OBJECT_QUERY, 
                    save_time=False),
                transitions={
                    SUCCESS: 'AddEntryToSOMQuery_AllTime'},
                remapping={});

            smach.StateMachine.add(
                'AddEntryToSOMQuery_AllTime',
                AddSOMEntry(
                    field_adding_default=search_for_query_type),
                transitions={
                    SUCCESS: 'PerformAllTimeQuery'},
                remapping={'value' :'obj_type'});
            
            smach.StateMachine.add(
                'PerformAllTimeQuery',
                PerformSOMQuery(distance_filter=4),
                transitions={
                    SUCCESS:'CheckSeenObjectAllTime'},
                remapping={});
        

            smach.StateMachine.add(
                'CheckSeenObjectAllTime',
                GetListEmpty(),
                transitions={
                    'list_not_empty': SUCCESS,
                    'list_empty': subsequent_state
                },
                remapping={
                    'input_list':'som_query_results'
                });

        if spin_first:
            createSpinAndQuery('CreateAllTimeQuery');
            createAllTimeQuery('item_not_seen');
        else:
            createAllTimeQuery('CreateObjQuery');
            createSpinAndQuery('item_not_seen');

    return sub_sm;


def nav_within_reaching_distance_of(execute_nav_commands, find_same_category=False, som_query_already_performed=False):
    """
    Input keys:
        obj_type            - The class of object we are looking to navigate to. Added if som_query_already_performed==False
        som_query_results   - If som_query_already_performed==True
    Output keys:
        som_query_results   - The output from the query performed in getting the object of interest.
    Inputs:
        find_same_category  - Whether we're performing the som query to find objects of a given type or merely of a given category. (Useful for placing an object next to another one).
    Dependencies (28/4/2023):
        nav_and_pick_up_or_place_next_to
    """
    if som_query_already_performed:
        input_keys = ['som_query_results']
    else:
        input_keys = ['obj_type'];

    sub_sm = smach.StateMachine(
        outcomes=[SUCCESS, FAILURE, 'query_empty'],
        input_keys=input_keys,
        output_keys=['som_query_results', 'tf_name']);

    with sub_sm:
        # Outputs som_query_results into userdata.
        if not som_query_already_performed:
            smach.StateMachine.add(
                'search_for_entity',
                search_for_entity(spin_first=True, find_same_category=find_same_category),
                transitions={
                    SUCCESS:'GetLocation',
                    FAILURE:FAILURE,
                    'item_not_seen':'query_empty'});

        smach.StateMachine.add(
            'GetLocation',
            GetPropertyAtIndex(properties_getting=['obj_position'], index=0),
            transitions={
                SUCCESS:'GetTfName',
                'index_out_of_range':'query_empty'},
            remapping={
                'input_list':'som_query_results',
                'obj_position':'target_pose'});
        
        smach.StateMachine.add(
            'GetTfName',
            GetPropertyAtIndex(properties_getting=['tf_name'], index=0),
            transitions={
                SUCCESS:'NavToLoc',
                'index_out_of_range':'query_empty'},
            remapping={
                'input_list':'som_query_results',
                'tf_name':'tf_name'});

        smach.StateMachine.add(
            'NavToLoc',
            # OrientRobot(),
            navigate_within_distance_of_pose_input(execute_nav_commands),
            transitions={
                SUCCESS:SUCCESS,
                FAILURE:FAILURE
            });
    return sub_sm;


def nav_and_pick_up_or_place_next_to(execute_nav_commands, pick_up:bool, find_same_category = False, som_query_already_performed=False):
    """
    Creates the state machine for either navigating and picking stuff up (pick_up==True)
        or navigating and putting stuff down (pick_up==False).
    Input arguments:
        execute_nav_commands    - Required argument for whether navigation will be allowed. Manipulation 
                                can navigate by itself, but apart from that, this stops the robot from 
                                navigating around while you're developing something that doesn't need it.
        pick_up                 - Argument for whether we're picking an object up, or putting one down.
                                This state machine does both, allowing for a reduction in the number of 
                                sub-state machines required for the system as a whole. 
        find_same_category      - Currently only applies to putting an object down. If we wish to put an 
                                object down next to another of the same type, set this flag to true rather
                                than false. TODO: Extend to picking up as well, though this is non-critical
                                for storing groceries, and may well be an extension to write when required.
    Input keys:
        obj_type                - The class of object we want to pick up/put the object we're holding next to. 
                                If we are putting an object down, this can also refer to the category of the 
                                object. Only if som_query_already_performed==False
        som_query_results       - The query results from the som query. Only if som_query_already_performed==True.
    Outcomes:
        SUCCESS                 - When the task happens.
        FAILURE                 - Returned for other failures. These tend to be 
        'query_empty'           - Returned when the object in question is not found by the system.
        MANIPULATION_FAILURE    - Returned when the manipulation component fails.                               
    Dependencies (28/4/2023):
        put_away_the_groceries.py
        open_day_demo_autonav.py
    """
    if som_query_already_performed:
        input_keys = ['som_query_results']
    else:
        input_keys = ['obj_type'];

    sub_sm = smach.StateMachine(
        outcomes=[SUCCESS, FAILURE, 'query_empty', MANIPULATION_FAILURE],
        input_keys=input_keys,
        output_keys=[]);
    
    PICK_UP_STATE = "PickUpObject";
    PUT_DOWN_STATE = "PutObjectDown";
    SECOND_STATE = PICK_UP_STATE if pick_up else PUT_DOWN_STATE;

    put_down_query_type = 'category' if find_same_category else 'class_'

    sub_sm.userdata.number_of_failures = 0;
    sub_sm.userdata.failure_threshold = 3;

    # Required fields for PlaceNextTo
    # Place in a YAML file?
    # if rospy.has_param('place_next_to_params'):
    if rospy.has_param('find_placement_options'):
        find_placement_options:dict = rospy.get_param('find_placement_options');
        dims = find_placement_options['dims'];
        height = find_placement_options['height'];
        radius = find_placement_options['radius'];
        print(find_placement_options);
    else:
        dims = (0.05, 0.05, 0.2);
        height = 0.3;
        radius = 0.2;

    with sub_sm:
        # Outputs som_query_results to userdata.
        # Output parameters:
        #   som_query_results
        #   tf_name
        smach.StateMachine.add(
            'nav_to_object',
            nav_within_reaching_distance_of(execute_nav_commands, find_same_category=find_same_category),
            transitions={
                SUCCESS:SECOND_STATE,
                FAILURE:SECOND_STATE,
                'query_empty':'query_empty'});

        if pick_up: 
            smach.StateMachine.add(
                PICK_UP_STATE,
                PickUpObjectState_v2(read_from_som_query_results=False),
                transitions={
                    SUCCESS:SUCCESS,
                    MANIPULATION_FAILURE:MANIPULATION_FAILURE},
                remapping={});
        else:
            # The motion of the head as it looks round makes for a slight offset in the position
            # between the actual location and the proposed one. This fixes that issue.
            # We can assume that it's in view however.
            smach.StateMachine.add(
                PUT_DOWN_STATE,
                has_seen_object(rospy.Duration(1),True),
                transitions={
                    'object_seen':'GetLocation',
                    'object_not_seen':FAILURE,
                    FAILURE:FAILURE},
                remapping={put_down_query_type:'obj_type'});

            # smach.StateMachine.add(
            #     PUT_DOWN_STATE,
            #     CreateSOMQuery(
            #         CreateSOMQuery.OBJECT_QUERY, 
            #         save_time=True),
            #     transitions={
            #         SUCCESS: 'WaitALittle'},
            #     remapping={put_down_query_type:'obj_type'});

            # smach.StateMachine.add(
            #     'WaitALittle',
            #     WaitForSecs(2),
            #     transitions={
            #         SUCCESS:'PerformQuery'},
            #     remapping={});

            # smach.StateMachine.add(
            #     'PerformQuery',
            #     PerformSOMQuery(distance_filter=4),
            #     transitions={
            #         SUCCESS:'GetLocation'},
            #     remapping={});
            
            smach.StateMachine.add(
                'GetLocation',
                GetPropertyAtIndex(properties_getting=['obj_position'], index=0),
                transitions={
                    SUCCESS:'LookAtObject',
                    'index_out_of_range':FAILURE},
                remapping={
                    'input_list':'som_query_results',
                    'obj_position':'target_pose'});
                
            smach.StateMachine.add(
                "LookAtObject",
                LookAtPoint(z_looking_at=0.9, set_head_to_neutral=True),
                transitions={
                    SUCCESS:'PlaceObj'},
                remapping={
                    'pose':'target_pose'});

            smach.StateMachine.add(
                "PlaceObj",
                PlaceNextTo(dims=dims, max_height=height, radius=radius),
                transitions={
                    SUCCESS:SUCCESS,
                    MANIPULATION_FAILURE:MANIPULATION_FAILURE});

    return sub_sm;


def test_search_for_entity():
    rospy.init_node('search_for_entity_test');

    sub_sm = search_for_entity();
    sub_sm.userdata.obj_type = 'potted_plant';

    sub_sm.execute();
    pass;
def test_pipeline():
    # This is set up for the simulation environment we commonly use.
    # roslaunch hsrb_gazebo_launch hsrb_megaweb2015_launch

    rospy.init_node('nav_test');

    # sub_sm = create_search_for_human(False);
    # sub_sm.userdata.approximate_operator_pose = Pose();
    # sub_sm.execute();

    # For simulation environment:
    # sub_sm = navigate_within_distance_of_pose_input(True);
    # sub_sm.userdata.target_pose = Pose();
    # sub_sm.userdata.target_pose.position.x = -0.3;
    # sub_sm.userdata.target_pose.position.y = -7.3;
    # sub_sm.userdata.target_pose.position.z = 1.107;

    sub_sm = nav_and_pick_up_or_place_next_to(True, pick_up=True);
    sub_sm.userdata.obj_type = 'potted_plant';
    
    # sub_sm = navigate_within_distance_of_pose_input(True);
    # sub_sm.userdata.target_pose = Pose();
    # sub_sm.userdata.target_pose.position.x = 4.9;#-0.3;
    # sub_sm.userdata.target_pose.position.y = 0.7;#-1.3;
    # sub_sm.userdata.target_pose.position.z = 1.107;
    
    sub_sm.execute();

if __name__ == '__main__':
    # test_search_for_entity();
    test_pipeline();
