#!/usr/bin/env python3

import smach; 
from state_machines.Reusable_States.include_all import *;

def navigate_within_distance_of_pose_input(execute_nav_commands):
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
def navigate_within_distance_of_som_input(execute_nav_commands):
    """
    Navigates to a close distance from the first element that comes up from the query.

    outcomes:
        SUCCESS         : Self explanatory
        FAILURE         : Self explanatory
        'query_empty'   : If the query has nothing in it, then this is the response.
    inputs:
        som_query       : The query we will give the SOM system.
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

    sub_sm = smach.StateMachine(
        outcomes=[SUCCESS, FAILURE, 'query_empty'],
        input_keys=['obj_type'],
        output_keys=[]);

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
                'output_param':'obj_position'});

        smach.StateMachine.add(
            'NavToLoc',
            # OrientRobot(),
            navigate_within_distance_of_pose_input(execute_nav_commands),
            transitions={
                SUCCESS:SUCCESS,
                FAILURE:FAILURE
            },
            remapping={'orient_towards':'target_pose'});
    return sub_sm;



def nav_and_pick_up(execute_nav_commands):
    sub_sm = smach.StateMachine(
        outcomes=[SUCCESS, FAILURE, 'query_empty'],
        input_keys=['obj_type'],
        output_keys=[]);

    with sub_sm:
        smach.StateMachine.add(
            'nav_to_object',
            nav_within_reaching_distance_of(execute_nav_commands),
            transitions={
                SUCCESS:'PickUpObject',
                FAILURE:'PickUpObject',
                'query_empty':'query_empty'});
        
        smach.StateMachine.add(
            'PickUpObject',
            PickUpObjectState(),
            transitions={
                SUCCESS:SUCCESS,
                FAILURE:'PickUpObject',
                REPEAT_FAILURE:FAILURE},
            remapping={
                'object_name':'obj_type'});

    return sub_sm;


def nav_and_place_next_to(execute_nav_commands):
    sub_sm = smach.StateMachine(
        outcomes=[SUCCESS, FAILURE, 'query_empty'],
        input_keys=['obj_type'],
        output_keys=[]);

    with sub_sm:
        smach.StateMachine.add(
            'nav_to_object',
            nav_within_reaching_distance_of(execute_nav_commands),
            transitions={
                SUCCESS:'PutObjectDown',
                FAILURE:'PutObjectDown',
                'query_empty':'query_empty'});
        
        smach.StateMachine.add(
            'PutObjectDown',
            PutObjectOnSurfaceState(),
            transitions={
                SUCCESS:SUCCESS,
                FAILURE:'PutObjectDown',
                REPEAT_FAILURE:FAILURE},
            remapping={
                'object_name':'obj_type'});

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

    sub_sm = nav_and_pick_up(True);
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
