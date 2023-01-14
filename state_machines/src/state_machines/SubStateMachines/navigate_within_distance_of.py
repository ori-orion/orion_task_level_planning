import smach; 
from state_machines.Reusable_States.include_all import *;

def navigate_within_distance_of_pose_input():
    sub_sm = smach.StateMachine(
        outcomes=[SUCCESS, FAILURE],
        input_keys=['target_pose']);

    sub_sm.userdata.number_of_failures = 0;
    sub_sm.userdata.failure_threshold =3;

    with sub_sm:
        # wait for the start signal - this has been replaced by the WAIT_FOR_HOTWORD state
        #   TODO - fix and test the check door state for future competitions
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
            SimpleNavigateState(),
            transitions={
                SUCCESS:SUCCESS,
                FAILURE:'NavToGoal',
                REPEAT_FAILURE: FAILURE},
            remapping={
                'pose':'nav_target'
            });

    return sub_sm;

def navigate_within_distance_of_som_input():
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
                SUCCESS:'FindMyMatesOperatorDetection',
                FAILURE:FAILURE},
            remapping={});

        # Here we have som_query_results:List[...]. We want to get the first item in this query, and then
        # the position out of that as a node to navigate to. We can then navigate to that location. 

        smach.StateMachine.add(
            'PerformQuery',
            GetPropertyAtIndex(property_getting='obj_position', index=0),
            transitions={
                SUCCESS:'FindMyMatesOperatorDetection',
                'index_out_of_range':'query_empty'},
            remapping={
                'input_list':'som_query_results',
                'output_param':'target_pose'});

        smach.StateMachine.add(
            'NavToLoc',
            navigate_within_distance_of_pose_input(),
            transitions={
                SUCCESS:SUCCESS,
                FAILURE:FAILURE});

        


    return sub_sm;