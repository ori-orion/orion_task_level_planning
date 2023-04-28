#!/usr/bin/env python3

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
                SUCCESS:'NavToGoal'},
            remapping={
                'pose':'target_pose',
                'nav_target':'nav_target'
            });

        smach.StateMachine.add(
            'NavToGoal',
            SimpleNavigateState(execute_nav_commands),
            transitions={
                SUCCESS:SUCCESS,
                FAILURE:'LookAtObject',
                REPEAT_FAILURE: FAILURE},
            remapping={
                'pose':'nav_target'
            });

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
                SUCCESS:'GetLocation',
                FAILURE:FAILURE},
            remapping={});

        # Here we have som_query_results:List[...]. We want to get the first item in this query, and then
        # the position out of that as a node to navigate to. We can then navigate to that location. 

        smach.StateMachine.add(
            'GetLocation',
            GetPropertyAtIndex(property_getting='obj_position', index=0),
            transitions={
                SUCCESS:'NavToLoc',
                'index_out_of_range':'query_empty'},
            remapping={
                'input_list':'som_query_results',
                'output_param':'target_pose'});

        smach.StateMachine.add(
            'NavToLoc',
            navigate_within_distance_of_pose_input(execute_nav_commands),
            transitions={
                SUCCESS:SUCCESS,
                FAILURE:FAILURE});

    return sub_sm;


def search_for_entity(spin_first=True):
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
    
    with sub_sm:
        def createSpinAndQuery(subsequent_state:SUCCESS):
            smach.StateMachine.add(
                'CreateObjQuery',
                CreateSOMQuery(
                    CreateSOMQuery.OBJECT_QUERY, 
                    save_time=True),
                transitions={
                    SUCCESS: 'SpinOnSpot'},
                remapping={'class_':'obj_type'});

            smach.StateMachine.add(
                'SpinOnSpot',
                SpinState(spin_height=0.7),
                transitions={
                    SUCCESS:'PerformQuery'},
                remapping={});

            smach.StateMachine.add(
                'PerformQuery',
                PerformSOMQuery(distance_filter=4),
                transitions={
                    SUCCESS:'CheckSeenObject',
                    FAILURE:FAILURE},
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
                    SUCCESS: 'PerformAllTimeQuery'},
                remapping={'class_':'obj_type'});
            
            smach.StateMachine.add(
                'PerformAllTimeQuery',
                PerformSOMQuery(distance_filter=4),
                transitions={
                    SUCCESS:'CheckSeenObjectAllTime',
                    FAILURE:FAILURE},
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


def nav_within_reaching_distance_of(execute_nav_commands):
    """
    Input keys:
        obj_type            - The class of object we are looking to navigate to.
    Output keys:
        som_query_results   - The output from the query performed in getting the object of interest.
    Dependencies (28/4/2023):
        nav_and_pick_up_or_place_next_to
    """

    sub_sm = smach.StateMachine(
        outcomes=[SUCCESS, FAILURE, 'query_empty'],
        input_keys=['obj_type'],
        output_keys=['som_query_results']);

    with sub_sm:
        smach.StateMachine.add(
            'search_for_entity',
            search_for_entity(spin_first=True),
            transitions={
                SUCCESS:'GetLocation',
                FAILURE:FAILURE,
                'item_not_seen':'query_empty'});

        smach.StateMachine.add(
            'GetLocation',
            GetPropertyAtIndex(property_getting='obj_position', index=0),
            transitions={
                SUCCESS:'NavToLoc',
                'index_out_of_range':'query_empty'},
            remapping={
                'input_list':'som_query_results',
                'output_param':'target_pose'});

        smach.StateMachine.add(
            'NavToLoc',
            # OrientRobot(),
            navigate_within_distance_of_pose_input(execute_nav_commands),
            transitions={
                SUCCESS:SUCCESS,
                FAILURE:FAILURE
            });
    return sub_sm;


def nav_and_pick_up_or_place_next_to(execute_nav_commands, pick_up:bool):
    """
    Creates the state machine for either navigating and picking stuff up (pick_up==True)
        or navigating and putting stuff down (pick_up==False).
    Input keys:
        obj_type    - The class of object we want to pick up/put the object we're holding next to.
    Dependencies (28/4/2023):
        put_away_the_groceries.py
        open_day_demo_autonav.py
    """
    sub_sm = smach.StateMachine(
        outcomes=[SUCCESS, FAILURE, 'query_empty'],
        input_keys=['obj_type'],
        output_keys=[]);
    
    PICK_UP_STATE = "PickUpObject";
    PUT_DOWN_STATE = "PutObjectDown";
    SECOND_STATE = PICK_UP_STATE if pick_up else PUT_DOWN_STATE;

    # Required fields for PlaceNextTo
    # Place in a YAML file?
    # if rospy.has_param('place_next_to_params'):
    dims = (0.05, 0.05, 0.2) 
    height = 0.3
    radius = 0.2

    with sub_sm:
        # Outputs som_query_results to userdata.
        smach.StateMachine.add(
            'nav_to_object',
            nav_within_reaching_distance_of(execute_nav_commands),
            transitions={
                SUCCESS:SECOND_STATE,
                FAILURE:SECOND_STATE,
                'query_empty':'query_empty'});

        if pick_up: 
            smach.StateMachine.add(
                PICK_UP_STATE,
                PickUpObjectState(),
                transitions={
                    SUCCESS:SUCCESS,
                    FAILURE:PICK_UP_STATE,
                    REPEAT_FAILURE:FAILURE},
                remapping={
                    'object_name':'obj_type'});
        else:
            smach.StateMachine.add(
                PUT_DOWN_STATE,
                PlaceNextTo(dims=dims, max_height=height, radius=radius),
                transitions={
                    SUCCESS:SUCCESS,
                    FAILURE:FAILURE});

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
