from state_machines.Reusable_States.procedural_states import *;
from state_machines.Reusable_States.include_all import *;

from smach import Concurrence


"""
Speaking to a guest to learn about who they are.
"""
def create_learn_guest_sub_state_machine():

    # create the sub state machine
    sub_sm = smach.StateMachine(outcomes=[SUCCESS, FAILURE],
                                input_keys=['guest_som_human_ids',
                                            'closest_human',
                                            'guest_som_obj_ids',
                                            'person_names'],
                                output_keys=['guest_som_human_ids',
                                            'guest_som_obj_ids'])

    # speech defaults
    sub_sm.userdata.speak_and_listen_params_empty = []
    sub_sm.userdata.speak_and_listen_failures = 0
    sub_sm.userdata.speak_and_listen_failure_threshold = 2

    # speaking to guests
    # sub_sm.userdata.introduction_to_guest_phrase = "";
    sub_sm.userdata.ask_name_phrase = "What is your name?"
    # sub_sm.userdata.no_one_there_phrase = "Hmmm. I don't think anyone is there. It's time for me to move on."
    # sub_sm.userdata.speech_recognition_failure_phrase = "I'm sorry but I didn't understand. Let's try that again."

    sub_sm.userdata.ask_gender_phrase = "What is your gender?"
    sub_sm.userdata.ask_gender_candidates = GENDERS

    sub_sm.userdata.ask_pronouns_phrase = "What are your preferred pronouns?"
    sub_sm.userdata.ask_pronouns_candidates = PRONOUNS

    # TODO - add age fields into SOM person observation data (and expand CreateGuestAttributesDict state)
    # sub_sm.userdata.ask_age_phrase = "How old are you?"                     # assume response is given in years
    # sub_sm.userdata.ask_age_candidates = [str(x) for x in range(1,101)]     # TODO - test recognition of numbers

    # sub_sm.userdata.start_face_registration_phrase = "Please sit still."
    sub_sm.userdata.finish_face_registration_phrase = ""
    # sub_sm.userdata.save_to_som_phrase = "I am saving your details to memory."
    # sub_sm.userdata.farewell_guest = "Thank you."

    sub_sm.userdata.guest_name = ""
    sub_sm.userdata.guest_gender = ""
    sub_sm.userdata.guest_pronouns = ""
    sub_sm.userdata.guest_attributes = {}  # need to initialise it here because it needs to be an input into the CreateGuestAttributesDict state
    sub_sm.userdata.guest_face_attributes = {}  # need to initialise it here because it needs to be an input into the CreateGuestAttributesDict state

    sub_sm.userdata.failure_threshold = 3;

    # Open the container
    with sub_sm:
        # introduction to guest
        smach.StateMachine.add('ANNOUNCE_GUEST_INTRO',
                                SpeakState(phrase=""),
                                transitions={SUCCESS:'ASK_GUEST_NAME'},
                                remapping={})

        # ask for guest's name - New ask guest name action server
        smach.StateMachine.add('ASK_GUEST_NAME',
                               AskPersonNameState(timeout=5, question="What is your name?"),
                                transitions={SUCCESS: 'ANNOUNCE_GUEST_FACE_REGISTRATION_START',
                                # transitions={SUCCESS: 'CREATE_GUEST_ATTRIBUTES_DICT',   # Skip other sub machine states, for testing
                                            FAILURE:'ANNOUNCE_MISSED_GUEST_NAME',
                                            'repeat_failure':'ANNOUNCE_GUEST_FACE_REGISTRATION_START'},
                                remapping={'recognised_name': 'guest_name',
                                           'number_of_failures': 'speak_and_listen_failures',
                                           'failure_threshold': 'speak_and_listen_failure_threshold'})

        # announce that we missed the name, and that we will try again
        smach.StateMachine.add('ANNOUNCE_MISSED_GUEST_NAME',
                                SpeakState(phrase="I'm sorry but I didn't understand. Let's try that again."),
                                transitions={SUCCESS:'ASK_GUEST_NAME'},
                                remapping={})

        # announce that we think there is no-one there & end sub state machine
        smach.StateMachine.add('ANNOUNCE_NO_ONE_THERE',
                                SpeakState(phrase="Hmmm. I don't think anyone is there. It's time for me to move on."),
                                transitions={SUCCESS:FAILURE},
                                remapping={})

        # tell guest face registration is starting
        smach.StateMachine.add('ANNOUNCE_GUEST_FACE_REGISTRATION_START',
                                SpeakState(phrase="Please sit still."),
                                transitions={SUCCESS:'DETECT_OPERATOR_FACE_ATTRIBUTES_BY_DB'},
                                # transitions={SUCCESS:'ANNOUNCE_GUEST_FACE_REGISTRATION_FINISH'},
                                remapping={})

        # detect guest face attributes
        smach.StateMachine.add('DETECT_OPERATOR_FACE_ATTRIBUTES_BY_DB',
                                DetectFaceAttributes(),
                                transitions={SUCCESS:'ANNOUNCE_GUEST_FACE_REGISTRATION_FINISH'},
                                remapping={'face_id':'guest_name',
                                            'face_attributes':'guest_face_attributes',
                                            'num_attributes':'guest_num_attributes'  })

        # tell guest face registration is finished
        smach.StateMachine.add('ANNOUNCE_GUEST_FACE_REGISTRATION_FINISH',
                                SpeakState(phrase=""),
                                transitions={SUCCESS:'CREATE_GUEST_ATTRIBUTES_DICT'},
                                remapping={})

        # create the guest_attributes dictionary
        smach.StateMachine.add('CREATE_GUEST_ATTRIBUTES_DICT',
                                CreateGuestAttributesDict(),
                                transitions={SUCCESS:'SAVE_GUEST_TO_SOM'},
                                remapping={'name':'guest_name','gender':'guest_gender','pronouns':'guest_pronouns','face_id':'guest_name','face_attributes':'guest_face_attributes',
                                        'guest_attributes':'guest_attributes'})

        # save the guest info to the SOM (requires at least one entry in SOM object DB with class_=='person')
        smach.StateMachine.add('SAVE_GUEST_TO_SOM',
                                SaveGuestToSOM(),
                                transitions={SUCCESS:'ANNOUNCE_GUEST_FAREWELL',
                                            FAILURE:FAILURE},
                                remapping={'guest_attributes':'guest_attributes'})

        # farewell guest
        smach.StateMachine.add('ANNOUNCE_GUEST_FAREWELL',
                                SpeakState(phrase="Thank you."),
                                transitions={SUCCESS:SUCCESS},
                                remapping={})

        sub_sm = setupErrorStates(sub_sm, FAILURE);

    return sub_sm

"""
Goes through a list of topological nodes, navigating to each node and checking to see if there's a human there.
"""
def create_search_for_guest_sub_state_machine():
    """ Smach sub state machine to search for guests (non-operator people) not yet spoken to

    Returns SUCCESS if a non-operator person is found, `failure` otherwise.

    input_keys:
        nodes_not_searched: list of topological node ids to visit during search, in search order
        operator_uid: the operator unique id in the SOM object collection, used to ignore detections from the operator
        failure_threshold: number of allowed failed attempts for each topological navigation action
    output_keys:
        nodes_not_searched: list of topological node ids that were not visited during search (does not include the final node because there may be another person there)
        found_guest_uid: the uid of the found guest, if any
    """

    # gets called when ANY child state terminates
    def child_term_cb(outcome_map):
        # terminate all running states if CHECK_FOR_NEW_GUEST_SEEN finished with outcome SUCCESS
        if outcome_map['CHECK_FOR_NEW_GUEST_SEEN']:
            if outcome_map['CHECK_FOR_NEW_GUEST_SEEN'] == SUCCESS:
                rospy.loginfo("Concurrence child_term_cb: outcome_map['CHECK_FOR_NEW_GUEST_SEEN'] == SUCCESS. Terminating")
                return True

        # terminate all running states if NAV_SUB finished with outcome FAILURE
        if outcome_map['NAV_SUB'] == FAILURE:
            rospy.loginfo("Concurrence child_term_cb: outcome_map['NAV_SUB'] == FAILURE. Terminating")
            return True

        # in all other case, just keep running, don't terminate anything
        return False

    # gets called when ALL child states are terminated
    def out_cb(outcome_map):
        if outcome_map['CHECK_FOR_NEW_GUEST_SEEN']:
            if outcome_map['CHECK_FOR_NEW_GUEST_SEEN'] == SUCCESS:
                return SUCCESS

        return FAILURE

    # creating the concurrence state machine
    sm_con = Concurrence(outcomes=[SUCCESS, FAILURE],
                    default_outcome=SUCCESS,
                    input_keys=['nodes_not_searched',
                                 'operator_uid',
                                 'failure_threshold'],
                    output_keys=['nodes_not_searched',
                                    'found_guest_uid'],
                    child_termination_cb = child_term_cb,
                    outcome_cb = out_cb)

    # Open the concurrence container
    with sm_con:
        sub_sm_nav = smach.StateMachine(outcomes=[FAILURE, 'preempted'],
                                input_keys=['nodes_not_searched',
                                            'operator_uid',
                                            'failure_threshold'],
                                output_keys=['nodes_not_searched'])
        # Open the container
        with sub_sm_nav:
            # nav to next top node
            smach.StateMachine.add('NAV_TO_NEXT_TOP_NODE',
                                    SearchForGuestNavToNextNode(),
                                    transitions={'searched':'NAV_TO_NEXT_TOP_NODE',
                                                'exhausted_search':FAILURE,
                                                FAILURE:'NAV_TO_NEXT_TOP_NODE',
                                                'preempted':'preempted'},
                                    remapping={'nodes_not_searched':'nodes_not_searched',
                                                'failure_threshold':'failure_threshold'})

        # Add states to the container
        smach.Concurrence.add('NAV_SUB', sub_sm_nav)
        smach.Concurrence.add('CHECK_FOR_NEW_GUEST_SEEN', CheckForNewGuestSeen())

    return sm_con

"""
Navigation state machine where you first navigate to the closest topological node, and then to the final location.
"""
def create_topo_nav_state_machine():
    sub_sm = smach.StateMachine(outcomes=[SUCCESS, FAILURE],
                            input_keys=['goal_pose'],
                            output_keys=[]);

    

    with sub_sm:
        smach.StateMachine.add('GetClosestNode',
                                GetClosestNodeState(),
                                transitions={SUCCESS:'NavToNearestNode'});
        
        smach.StateMachine.add(
            'NavToNearestNode',
            TopologicalNavigateState(),
            transitions={
                SUCCESS:'NavToFinalGoal',
                FAILURE:'NavToNearestNode',
                'repeat_failure':'NavToFinalGoal'},
            remapping={'node_id':'closest_node'});

        smach.StateMachine.add(
            'NavToFinalGoal',
            SimpleNavigateState(),
            transitions={
                SUCCESS:SUCCESS,
                FAILURE:'NavToFinalGoal',
                'repeat_failure':FAILURE},
            remapping={'pose':'goal_pose'});

    return sub_sm;



"""
Spin on the spot and then query for the humans you saw since you started spinning.
"""
def create_search_for_human():
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

        smach.StateMachine.add(
            'NavToCentreOfRoom',
            SimpleNavigateState(),
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
                SUCCESS:SUCCESS,
                FAILURE:FAILURE,
                'one_person_found':'one_person_found'},
            remapping={});

    return sub_sm;


"""
Looks at all the guests in sequence.
"""
def create_point_to_all_guests():
    """
    Points to all the guests in sequence.
    Inputs:
        guest_list:Human[]  - An array giving all the guests.
    """

    sub_sm = smach.StateMachine(
        outcomes=[SUCCESS, FAILURE],
        input_keys=[
            'guest_list'],
        output_keys=[]);

    sub_sm.userdata.index = 0;

    with sub_sm:
        smach.StateMachine.add(
            'GetGuestPosition',
            GetPropertyAtIndex('obj_position'),
            transitions={
                SUCCESS:'PointAtGuest',         # LookAtGuest
                'index_out_of_range':SUCCESS},
            remapping={
                'input_list':'guest_list',
                'output_param':'ith_guest_pose'});
        
        #region For merely looking at the guests
        smach.StateMachine.add(
            'LookAtGuest',
            LookAtPoint(),
            transitions={SUCCESS:'CommentOnGuestExistence'},
            remapping={'pose':'ith_guest_pose'});

        smach.StateMachine.add(
            'CommentOnGuestExistence',
            SpeakState(phrase="There's a guest here."),
            transitions={SUCCESS:'IncrementGuestIndex'});
        #endregion

        #region For actually pointing at the guests.
        smach.StateMachine.add(
            'PointAtGuest',
            PointAtEntity("Here is a guest."),
            transitions={
                SUCCESS:"IncrementGuestIndex",
                FAILURE:FAILURE},
            remapping={'point_at_loc':'ith_guest_pose'});
        #endregion

        smach.StateMachine.add(
            'IncrementGuestIndex',
            IncrementValue(increment_by=1),
            transitions={SUCCESS:'GetGuestPosition'},
            remapping={'val':'index'});

    return sub_sm;


"""
We now need to go through guest by guest asking name and another detail.
We need description and location for findMyMates.py
"""
def create_get_guest_details():
    """
    Asks all the guests details about them.
    Inputs:
        guest_list:Human[]  - An array giving all the guests.
    Outputs:
        speaking_phrase:str - A string to report back to the operator.
    """

    sub_sm = smach.StateMachine(
        outcomes=[SUCCESS, FAILURE],
        input_keys=[
            'guest_list'],
        output_keys=[]);

    sub_sm.userdata.index = 0;

    with sub_sm:
        smach.StateMachine.add(
            'GetGuestPosition',
            GetPropertyAtIndex('obj_position'),
            transitions={
                SUCCESS:'PointAtGuest',         # LookAtGuest
                'index_out_of_range':SUCCESS},
            remapping={
                'input_list':'guest_list',
                'output_param':'ith_guest_pose'});
        
        smach.StateMachine.add(
            'GetGuestInformation',
            create_learn_guest_sub_state_machine(),
            transitions={
                SUCCESS:'IncrementGuestIndex',
                FAILURE:FAILURE});

        smach.StateMachine.add(
            'IncrementGuestIndex',
            IncrementValue(increment_by=1),
            transitions={SUCCESS:'GetGuestPosition'},
            remapping={'val':'index'});

    return sub_sm;    


"""
Introduction to operator/asking operator name
"""
def create_intro_to_operator(operator_pose:Pose):
    sub_sm = smach.StateMachine(
        outcomes=[SUCCESS, FAILURE],
        input_keys=[],
        output_keys=['operator_name']);
                        
    sub_sm.userdata.ask_operator_name_phrase = "What is your name?"                
    
    sub_sm.userdata.number_of_failures = 0;

    sub_sm.userdata.nearest_to = None;

    with sub_sm:
        # ask for operator's name - New ask guest name action server - TODO - test        
        smach.StateMachine.add('ASK_OPERATOR_NAME',
                                AskPersonNameState(timeout=5),
                                transitions={SUCCESS: 'SearchForOperator',
                                            FAILURE:'ANNOUNCE_MISSED_NAME',
                                            REPEAT_FAILURE:'SearchForOperator'},
                                remapping={'question':'ask_operator_name_phrase',
                                            'recognised_name': 'operator_name',
                                            'number_of_failures': 'speak_and_listen_failures',
                                            'failure_threshold': 'speak_and_listen_failure_threshold'})

        # announce that we missed the name, and that we will try again
        smach.StateMachine.add(
            'ANNOUNCE_MISSED_NAME',
            SpeakState(phrase="I'm sorry but I didn't understand. Let's try that again."),
            transitions={
                SUCCESS:'ASK_OPERATOR_NAME'},
            remapping={})

        smach.StateMachine.add(
            'SearchForOperator',
            GetNearestHuman(),
            transitions={
                'new_human_found':'SAVE_OPERATOR_INFO_TO_SOM',
                'human_not_found':'SAVE_OPERATOR_INFO_TO_SOM_HARDCODED_BACKUP',
                'existing_human_found':'SAVE_OPERATOR_INFO_TO_SOM'},
            remapping={'nearest_to':'operator_pose'});

        # save the operator info to the SOM
        smach.StateMachine.add('SAVE_OPERATOR_INFO_TO_SOM',
                               SaveOperatorToSOM(),
                               transitions={SUCCESS:'CREATE_PHRASE_START_SEARCH',
                                            FAILURE:TASK_FAILURE},
                                remapping={'operator_name':'operator_name', 
                                            'operator_som_id':'operator_som_id'})

        smach.StateMachine.add('SAVE_OPERATOR_INFO_TO_SOM_HARDCODED_BACKUP',
                       SaveOperatorToSOM(operator_pose=operator_pose),
                       transitions={SUCCESS:'CREATE_PHRASE_START_SEARCH',
                                    FAILURE:TASK_FAILURE},
                        remapping={'operator_name':'operator_name', 
                                    'operator_som_id':'operator_som_id'})


"""
Drop off the bin bag.
"""
def create_drop_off_bin_bag():
    sub_sm = smach.StateMachine(
        outcomes=[SUCCESS, FAILURE],
        input_keys=['drop_off_location'],
        output_keys=[]);
                        
    sub_sm.userdata.number_of_failures = 0;

    sub_sm.userdata.nearest_to = None;

    with sub_sm:
        smach.StateMachine.add(
            'NavToDropOff',
            SimpleNavigateState(),
            transitions={
                SUCCESS:'DropBinBag',
                FAILURE:'NavToDropOff',
                REPEAT_FAILURE:'DropBinBag'},
            remapping={'pose':'drop_off_location'});
        
        smach.StateMachine.add(
            'DropBinBag',
            DropEntity(),
            transitions={
                SUCCESS:'NAV_TO_OPERATOR',
                FAILURE:'NAV_TO_OPERATOR',
                REPEAT_FAILURE:'NAV_TO_OPERATOR'},
            remapping={'pose':'drop_off_location'});
    return sub_sm;


def create_repeated_trials(state:type, outcomes, input_keys, output_keys, attempts_to_failure=3):

    
        
    pass;