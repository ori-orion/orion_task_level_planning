from state_machines.Reusable_States.include_all import *;

from smach import Concurrence

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
    sub_sm.userdata.speak_and_listen_timeout = 5
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
                               AskPersonNameState(),
                                transitions={SUCCESS: 'ANNOUNCE_GUEST_FACE_REGISTRATION_START',
                                # transitions={SUCCESS: 'CREATE_GUEST_ATTRIBUTES_DICT',   # Skip other sub machine states, for testing
                                            FAILURE:'ANNOUNCE_MISSED_GUEST_NAME',
                                            'repeat_failure':'ANNOUNCE_GUEST_FACE_REGISTRATION_START'},
                                remapping={'question':'ask_name_phrase',
                                            'recognised_name': 'guest_name',
                                            'timeout':'speak_and_listen_timeout',
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

        # ask for guest's gender
        # smach.StateMachine.add('ASK_GUEST_GENDER',
        #                        SpeakAndListenState(),
        #                         transitions={SUCCESS: 'ASK_GUEST_PRONOUNS',
        #                                     FAILURE:'ASK_GUEST_GENDER',
        #                                     'repeat_failure':'ASK_GUEST_PRONOUNS'},
        #                         remapping={'question':'ask_gender_phrase',
        #                                     'operator_response': 'guest_gender',
        #                                     'candidates':'ask_gender_candidates',
        #                                     'params':'speak_and_listen_params_empty',
        #                                     'timeout':'speak_and_listen_timeout',
        #                                     'number_of_failures': 'speak_and_listen_failures',
        #                                     'failure_threshold': 'speak_and_listen_failure_threshold'})

        # ask for guest's pronouns
        # smach.StateMachine.add('ASK_GUEST_PRONOUNS',
        #                        SpeakAndListenState(),
        #                         transitions={SUCCESS: 'ANNOUNCE_GUEST_FACE_REGISTRATION_START',
        #                                     FAILURE:'ASK_GUEST_PRONOUNS',
        #                                     'repeat_failure':'ANNOUNCE_GUEST_FACE_REGISTRATION_START'},
        #                         remapping={'question':'ask_pronouns_phrase',
        #                                     'operator_response': 'guest_pronouns',
        #                                     'candidates':'ask_pronouns_candidates',
        #                                     'params':'speak_and_listen_params_empty',
        #                                     'timeout':'speak_and_listen_timeout',
        #                                     'number_of_failures': 'speak_and_listen_failures',
        #                                     'failure_threshold': 'speak_and_listen_failure_threshold'})

        # tell guest face registration is starting
        smach.StateMachine.add('ANNOUNCE_GUEST_FACE_REGISTRATION_START',
                                SpeakState(phrase="Please sit still."),
                                transitions={SUCCESS:'DETECT_OPERATOR_FACE_ATTRIBUTES_BY_DB'},
                                # transitions={SUCCESS:'ANNOUNCE_GUEST_FACE_REGISTRATION_FINISH'},
                                remapping={})

        # capture guest's face
        # smach.StateMachine.add('CAPTURE_GUEST_FACE',
        #                         RegisterFace(),
        #                         transitions={SUCCESS:'DETECT_OPERATOR_FACE_ATTRIBUTES_BY_DB',
        #                                     FAILURE:'DETECT_OPERATOR_FACE_ATTRIBUTES_BY_DB'},
        #                         remapping={'face_id':'guest_name'})

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

def create_search_for_human():
    """
    For searching for humans in a given room.
    This will prioritise humans that haven't been spoken to, and then go to the operators that are closer to you.
    Note that this doesn't speak to the humans, and so doesn't fill in any of the more interesting information (such as name).
        Nor does it input anything into the SOM system. It just gives the Human record generated by the recognition system.
    Inputs:
        room_node_uid:str       - The room node id for the room we want to search in.
        failure_threshold       - the number of cumulative failures required to return the repeat_failure outcome
    Outputs:
        closest_human:Human     - The human that was found.
        human_object_uid:str    - What is the object uid of the human in question. (Makes it slightly more general for later logic)
        human_pose:Pose         - Returns the pose at which the human was found.        
    """

    sub_sm = smach.StateMachine(
        outcomes=[SUCCESS, FAILURE],
        input_keys=[
            'room_node_uid', 'failure_threshold', 'prev_node_nav_to'],
        output_keys=[
            'closest_human', 'human_object_uid', 'human_pose', 'prev_node_nav_to']);
                        
    sub_sm.userdata.number_of_failures = 0;

    sub_sm.userdata.nearest_to = None;

    with sub_sm:
        smach.StateMachine.add(
            'NavToNearestNode',
            TopologicalNavigateState(stop_repeat_navigation=True),
            transitions={
                SUCCESS:'SearchForHuman_1',
                FAILURE:'NavToNearestNode',
                'repeat_failure':FAILURE},
            remapping={'node_id':'room_node_uid'});

        smach.StateMachine.add(
            'SearchForHuman_1',
            GetNearestHuman(),
            transitions={
                'new_human_found':'LookAtHuman',
                'human_not_found':'SpinOnSpot',
                'existing_human_found':'SpinOnSpot'},
            remapping={});
        
        smach.StateMachine.add(
            'SpinOnSpot',
            SpinState(),
            transitions={
                SUCCESS:'SearchForHuman_2'},
            remapping={});

        smach.StateMachine.add(
            'SearchForHuman_2',
            GetNearestHuman(),
            transitions={
                'new_human_found':'LookAtHuman',
                'human_not_found':FAILURE,
                'existing_human_found':FAILURE},
            remapping={});

        smach.StateMachine.add(
            'GoToSafePoseFromHuman',
            NavigateDistanceFromGoalSafely(),
            transitions={SUCCESS:'LookAtHuman'},
            remapping={'pose':'human_pose'});

        smach.StateMachine.add(
            'LookAtHuman',
            LookAtHuman(),
            transitions={SUCCESS:SUCCESS});

    return sub_sm;


def create_repeated_trials(state:type, outcomes, input_keys, output_keys):

    
        
    pass;