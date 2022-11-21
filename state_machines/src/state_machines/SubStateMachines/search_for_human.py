#!/usr/bin/env python3

from state_machines.Reusable_States.include_all import *;

import numpy as np;
from geometry_msgs.msg import Pose

"""
It would be really useful to order the people found from left to right.
This state does this.
"""
class OrderGuestsFound(smach.State):
    """
    Inputs/Outputs:
        guest_list:Human[]      The list of guests found. 
    
    Order the guests from left to right.

    The aim of this is to minimise the angles between consecutive members,
    as well as to make all the cross products align.
    Now, if A is to the left of B from the robot's perspective, then AxB should point 
        vaguely downwards.
    This gets slightly more confusing because of the rotational aspect. 
    Luckily we only expect a maximum of 4 guests, so we can brute force it. 
    """

    DOWNWARDS = np.asarray([0,0,-1]);

    def __init__(self):
        smach.State.__init__(self, 
            outcomes=[SUCCESS],
            input_keys=['guest_list'],
            output_keys=['guest_list']);


    def testState(self):
        print("Ordering guests test.");
        robot_location = np.asarray([0,0,0]);

        guest_list = [];

        names = ["n1", "n2", "n3", "n4"];

        def create_human(name, x,y,z) -> Human:
            output = Human();
            output.obj_position.position.x = x;
            output.obj_position.position.y = y;
            output.obj_position.position.z = z;
            output.name = name;
            return output;

        guest_list.append(create_human(names[1], 0, 1, 0));
        guest_list.append(create_human(names[3], 1, 0, 0));
        guest_list.append(create_human(names[0], -1, 0, 0));
        guest_list.append(create_human(names[2], 1, 1, 0));

        ordered_guests = self.orderFields(guest_list, robot_location);

        for i in range(len(ordered_guests)):
            guest:Human = ordered_guests[i];
            print("\t", guest.name);

        for i in range(len(ordered_guests)):
            assert(type(ordered_guests[i]) is Human);
            guest:Human = ordered_guests[i];
            assert(guest.name == names[i]);

        print("\tOrdering guest tests passed");

    def orderFields(self, guest_list:list, robot_location:np.ndarray) -> list:
        print("Number of guests found is", len(guest_list));

        respective_to_vecs = [];
        for guest in guest_list:
            guest:Human;
            appending = point_to_numpy(guest.obj_position.position) - robot_location;
            # We want these vectors to be normalised because we're going to be comparing the magnitude of them.
            print(appending);
            if np.linalg.norm(appending) > 3:   # If the human is greater than 3m away from the human...
                continue;
            appending /= np.linalg.norm(appending);
            respective_to_vecs.append(appending);
            
        respective_next_to = [];
        for i in range(len(guest_list)):
            print(i);

            best_cos_angle_diff = -1;
            best_match = -1;
            for j in range(len(guest_list)):
                if i==j:
                    continue;

                if np.dot(np.cross(respective_to_vecs[i], respective_to_vecs[j]), self.DOWNWARDS) < 0:
                    cos_angle = np.dot(respective_to_vecs[i], respective_to_vecs[j]);
                    print("\t", j, cos_angle);
                    if cos_angle > best_cos_angle_diff:
                        best_cos_angle_diff = cos_angle;
                        best_match = j;
            
            if best_match == -1:
                respective_next_to.append(None);
            else:
                respective_next_to.append(best_match);

        print(respective_next_to);

        guest_list_new = [];
        try:
            next_index = respective_next_to.index(None);
        except ValueError:
            next_index = 0;

        for i in range(len(guest_list)):
            print(next_index);
            guest_list_new.append(guest_list[next_index]);
            if (i < len(guest_list) - 1):
                next_index = respective_next_to.index(next_index);
                
        
        return guest_list_new;


    def execute(self, userdata):
        robot_location:np.ndarray = point_to_numpy(get_current_pose().position);

        guest_list:list = userdata.guest_list;
            
        userdata.guest_list = self.orderFields(guest_list, robot_location);

        return SUCCESS;


"""
Spin on the spot and then query for the humans you saw since you started spinning.
"""
def create_search_for_human(execute_nav_commands:bool, start_with_nav:bool = True):
    """
    For searching for humans in a given room.
    This will prioritise humans that haven't been spoken to, and then go to the operators that are closer to you.
    Note that this doesn't speak to the humans, and so doesn't fill in any of the more interesting information (such as name).
        Nor does it input anything into the SOM system. It just gives the Human record generated by the recognition system.
    Note also that there are two navigational systems in place, only one of which is currently in use. 
        centre_of_room_pose and room_node_uid correspond to the move_base and topological node navigation respectively.
    Inputs:
        centre_of_room_pose:Pose        - The pose we are navigating to in the middle of the room to perform the search.
        room_node_uid:str               - The room node id for the room we want to search in.
        failure_threshold               - the number of cumulative failures required to return the repeat_failure outcome
        prev_node_nav_to                - This is something used by any of the topological nodes. Unimportant for the purposes of this specifically.
        approximate_operator_pose:Pose  - What is the approximate operator pose? This will be used to identify the operator out of all the humans observed.
    Outputs:
        operator_pose:Pose        - The position of the operator.
        guest_list:Human[]        - An array with all the guests on it.        
    """

    sub_sm = smach.StateMachine(
        outcomes=[SUCCESS, FAILURE, 'one_person_found'],
        input_keys=[
            'centre_of_room_pose', 'room_node_uid', 'failure_threshold', 'prev_node_nav_to',
            'approximate_operator_pose'],
        output_keys=[
            'operator_pose', 'guest_list']);
                        
    sub_sm.userdata.number_of_failures = 0;

    sub_sm.userdata.nearest_to = None;

    with sub_sm:

        if start_with_nav:
            smach.StateMachine.add(
                'NavToCentreOfRoom',
                SimpleNavigateState(execute_nav_commands=execute_nav_commands),
                transitions={
                    SUCCESS:'CreateHumanQuery',
                    FAILURE:'NavToCentreOfRoom',
                    REPEAT_FAILURE: FAILURE},
                remapping={'pose':'centre_of_room_pose'});

            #region Assumes existence of the topological nodes.
            # smach.StateMachine.add(
            #     'NavToNearestNode',
            #     TopologicalNavigateState(stop_repeat_navigation=True),
            #     transitions={
            #         SUCCESS:'CreateHumanQuery',
            #         FAILURE:'NavToNearestNode',
            #         'repeat_failure':FAILURE},
            #     remapping={'node_id':'room_node_uid'});
            #endregion

        smach.StateMachine.add(
            'CreateHumanQuery',
            CreateSOMQuery(
                CreateSOMQuery.HUMAN_QUERY, 
                save_time=True),
            transitions={
                SUCCESS: 'SpinOnSpot'},
            remapping={});
        
        smach.StateMachine.add(
            'SpinOnSpot',
            SpinState(),
            transitions={
                SUCCESS:'QueryForHumans'},
            remapping={});

        smach.StateMachine.add(
            'QueryForHumans',
            PerformSOMQuery(),
            transitions={
                SUCCESS:'FindMyMatesOperatorDetection',
                FAILURE:FAILURE},
            remapping={});

        smach.StateMachine.add(
            'FindMyMatesOperatorDetection',
            FindMyMates_IdentifyOperatorGuests(),
            transitions={
                SUCCESS:'OrderGuests',
                FAILURE:FAILURE,
                'one_person_found':'one_person_found'},
            remapping={});

        smach.StateMachine.add(
            'OrderGuests',
            OrderGuestsFound(),
            transitions={SUCCESS:SUCCESS},
            remapping={});

    return sub_sm;


"""
Looks at all the guests in sequence.
"""
def create_talk_to_guests():
    """
    Points to all the guests in sequence.
    Inputs:
        guest_list:Human[]      - An array giving all the guests.
    Outputs:
        responses_arr:dict[]    - An array of all the raw responses.
        output_speech_arr:str[] - An array of the things to say.
    """

    sub_sm = smach.StateMachine(
        outcomes=[SUCCESS, FAILURE],
        input_keys=[
            'guest_list'],
        output_keys=[
            "responses_arr",
            "output_speech_arr"
        ]);

    sub_sm.userdata.index = 0;

    sub_sm.userdata.responses_arr = [];
    sub_sm.userdata.output_speech_arr = [];

    with sub_sm:
        smach.StateMachine.add(
            'GetGuestPosition',
            GetPropertyAtIndex('obj_position'),
            transitions={
                SUCCESS:'LookAtGuest',         
                'index_out_of_range':SUCCESS},
            remapping={
                'input_list':'guest_list',
                'output_param':'ith_guest_pose'});
        
        smach.StateMachine.add(
            'LookAtGuest',
            LookAtPoint(),
            transitions={SUCCESS:'TalkToGuest'},
            remapping={'pose':'ith_guest_pose'});

        smach.StateMachine.add(
            'TalkToGuest',
            AskFromSelection(append_result_to_array=True),
            transitions={
                SUCCESS:'IncrementGuestIndex',
                "no_response":'IncrementGuestIndex'},
            remapping={
                "responses_arr" : "responses_arr",
                "output_speech_arr" : "output_speech_arr"
            });

        smach.StateMachine.add(
            'IncrementGuestIndex',
            IncrementValue(increment_by=1),
            transitions={SUCCESS:'GetGuestPosition'},
            remapping={'val':'index'});

    return sub_sm;


"""
This then does both the searching and the talking.
For testing purposes mainly given that there's no navigation 
to where the operator is.
"""
def create_search_talk_and_report(execute_nav_commands, start_with_nav):
    sub_sm = smach.StateMachine(
        outcomes=[SUCCESS, FAILURE],
        input_keys=[],
        output_keys=[]);

    with sub_sm:
        smach.StateMachine.add(
            'SearchForGuests',
            create_search_for_human(execute_nav_commands=execute_nav_commands, start_with_nav=start_with_nav),
            transitions={
                SUCCESS:'TalkToGuests',
                FAILURE:FAILURE,
                'one_person_found':FAILURE},
            remapping={
                'operator_pose':'operator_pose', 
                'guest_list':'guest_list'
            });
        
        smach.StateMachine.add(
            'TalkToGuests',
            create_talk_to_guests(),
            transitions={
                SUCCESS:'LookAtOperator',
                FAILURE:FAILURE},
            remapping={
                "responses_arr":"responses_arr",
                "output_speech_arr":"output_speech_arr"
            });

        smach.StateMachine.add(
            'LookAtOperator',
            LookAtPoint(),
            transitions={SUCCESS:'ReportBackToOperator'},
            remapping={'pose':'operator_pose'});
        
        smach.StateMachine.add(
            'ReportBackToOperator',
            ReportBackToOperator(),
            transitions={SUCCESS:SUCCESS});
        
    return sub_sm;


if __name__ == '__main__':

    # state = OrderGuestsFound();
    # state.testState();

    rospy.init_node('search_for_human_test');

    # sub_sm = create_search_for_human(False);
    # sub_sm.userdata.approximate_operator_pose = Pose();
    # sub_sm.execute();

    sub_sm = create_search_talk_and_report(execute_nav_commands=False, start_with_nav=False);
    sub_sm.userdata.approximate_operator_pose = Pose();
    sub_sm.execute();

    rospy.spin();

