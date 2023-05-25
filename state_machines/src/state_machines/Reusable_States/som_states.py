from state_machines.Reusable_States.utils import *;
from state_machines.Reusable_States.procedural_states import *;
from state_machines.Reusable_States.misc_states import *;

import smach;

from orion_actions.msg import *;
from orion_actions.srv import *;

import rospy;

import math;

from geometry_msgs.msg import Pose, PoseStamped;

from typing import List;

"""
Overall interface into SOM:
    - First you make a query.
    - Then you input the query into the system. 
    - Note that you can get it to set up the query as well, but that will be an input argument.
"""


class CreateSOMQuery(smach.State):
    """
    Creates a query for the SOM system.

    inputs:
    outputs:
        som_query:SOMQueryHumansRequest|SOMQueryObjectsRequest
                        : The output query.
    """

    HUMAN_QUERY = 1;
    OBJECT_QUERY = 2;
    
    def __init__(self, query_type:int, save_time:bool=False, duration_back:rospy.Duration=None):
        smach.State.__init__(self, 
            outcomes=[SUCCESS],
            input_keys=[],
            output_keys=['som_query'])

        self.query_type = query_type;
        self.save_time = save_time;
        self.duration_back = duration_back;


    def execute(self, userdata):        
        if self.query_type == self.HUMAN_QUERY:
            output = SOMQueryHumansRequest();
        elif self.query_type == self.OBJECT_QUERY:
            output = SOMQueryObjectsRequest();
            # if hasattr(userdata, "class_"):
            #     output.query.class_ = userdata.class_.replace(' ', '_');
            # if hasattr(userdata, "category"):
            #     output.query.category = userdata.category

        if self.duration_back != None:
            output.query.last_observed_at = rospy.Time.now() - self.duration_back;
        elif self.save_time:
            output.query.last_observed_at = rospy.Time.now();

        # print("Checking to see if 'object_class' is within the userdata field.")
        # print(dir(userdata));
        # print(type(userdata));
        # print(userdata.keys());
        # if 'object_class' in userdata.keys() and hasattr(output.query, 'class_'):
        #     print("setting class");
        #     class_:str = userdata.object_class;
        #     class_ = class_.replace(' ', '_');
        #     output.query.class_ = class_;

        userdata.som_query = output;
        return SUCCESS;


class AddSOMEntry(smach.State):
    """
    Optional parameters are not a thing within smach. This will add 
    parameters to the SOM query. 
    It thus edits som_query.
    Inputs:
        field_adding_default:str    - The field we are adding. 
        som_query:
    """
    def __init__(self, field_adding_default):
        smach.State.__init__(self, 
            outcomes=[SUCCESS],
            input_keys=['som_query', 'value'],
            output_keys=['som_query']);
        
        self.field_adding_default:str = field_adding_default;
    
    def execute(self, userdata):
        query = userdata.som_query;
        if self.field_adding_default == "class_":
            query.query.class_ = userdata.value;
        elif self.field_adding_default == "category":
            query.query.category = userdata.value;
        userdata.som_query = query;
        return SUCCESS;


class PerformSOMQuery(smach.State):
    """
    Performs a SOM query.
    distance_filter - We may want to filter observations by distance from the robot. 
        If this is non-zero, this will do that filtering.

    Inputs:
        som_query:<query_type>                  : The query we will give the SOM system. This does type checking for the correct query.
    Outputs:
        som_query_results:List[<response_type>] : The response in the form of a raw array.
    """
    def __init__(self, distance_filter:float=0):
        smach.State.__init__(self, 
            outcomes=[SUCCESS, FAILURE],
            input_keys=['som_query'],
            output_keys=['som_query_results']);
            
        self.distance_filter = distance_filter;

    def execute(self, userdata):

        query = userdata.som_query;

        print(query);
        print(rospy.Time.now());

        output:List[SOMObject] = [];

        if type(query) == SOMQueryHumansRequest:
            rospy.wait_for_service('/som/humans/basic_query');
            human_query_srv = rospy.ServiceProxy('/som/humans/basic_query', SOMQueryHumans);
            result:SOMQueryHumansResponse = human_query_srv(query);
            output = result.returns;
        elif type(query) == SOMQueryObjectsRequest:
            rospy.wait_for_service('/som/objects/basic_query');
            object_query_srv = rospy.ServiceProxy('/som/objects/basic_query', SOMQueryObjects);
            result:SOMQueryObjectsResponse = object_query_srv(query);
            output = result.returns;
        else:
            return FAILURE;

        if self.distance_filter != 0:
            current_pose = get_current_pose();
            output_carry = [];
            for element in output:
                if distance_between_poses(current_pose, element.obj_position) < self.distance_filter:
                    output_carry.append(element);
            output = output_carry
            pass;

        userdata.som_query_results = output;
        rospy.loginfo('\t\t' + str(len(output)) + " entities found matching the query.")
        print(output);
        return SUCCESS;


class FindMyMates_IdentifyOperatorGuests(smach.State):
    """
    Inputs:
        som_query_results:Human[]       - What guests were found in the last query.
        approximate_operator_pose:Pose  - Where is the operator roughly?
                                        - Note that if the operator is on one side of the room, then a point over on this side of the room should be sufficient.
    Outputs:
        operator_pose:Pose  - What is the position of the operator?
        guest_list:Human[]  - Returns a list of the guests.
    """
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=[SUCCESS, FAILURE, 'one_person_found'],
            input_keys=['som_query_results', 'approximate_operator_pose'],
            output_keys=['operator_pose', 'guest_list']);

    def execute(self, userdata):
        results:list = userdata.som_query_results;

        userdata.guest_list = [];
        userdata.operator_pose = userdata.approximate_operator_pose;

        if len(results) == 0:
            return FAILURE;
        elif len(results) == 1:
            return 'one_person_found';
        else:
            approximate_op_pose:Pose = userdata.approximate_operator_pose;
            closest_distance = math.inf;
            closest_pose = Pose();
            closest_human = None;
            guest_list = [];
            for element in results:
                element:Human;
                dist = distance_between_poses(approximate_op_pose, element.obj_position);
                if dist < closest_distance:
                    if (closest_human != None):
                        guest_list.append(closest_human);
                    closest_human = element;
                    closest_pose = element.obj_position;
                    closest_distance = dist;
                else:
                    guest_list.append(element);
                
            userdata.guest_list = guest_list;
            userdata.operator_pose = closest_pose;
            
            return SUCCESS;


def has_seen_object(time_interval:rospy.Duration=None, wait_before_querying:bool=False):
    """
    Checks whether the robot has seen a given object.
    If `time_interval != None` then it will query back an interval of time_interval in time. 
    Else, the query will be across all time.
    Inputs:
        time_interval:rospy.Duration            - The duration back in time over which we are querying.
        wait_before_querying:bool               - Should we do nothing over the course of time_interval before querying, 
                                                or should we just query back immediately.

        class_:str                              - A string giving the object class.
    Outputs:
        item_not_found:bool                     - Whether the list returned is empty or not.
        som_query_results:List[SOMObject|Human] - The results from the query. 
    """

    sub_sm = smach.StateMachine(
        outcomes=['object_seen', 'object_not_seen', FAILURE],
        input_keys=['class_'],
        output_keys=['item_not_found', 'som_query_results']);

    sub_sm.userdata.index = 0;

    with sub_sm:
        smach.StateMachine.add(
            'CreateQuery',
            CreateSOMQuery(
                query_type=CreateSOMQuery.OBJECT_QUERY, 
                duration_back=None if wait_before_querying==True else time_interval,
                save_time=wait_before_querying),
            transitions={
                SUCCESS:'AddEntryToSOMQuery'});
        
        smach.StateMachine.add(
            'AddEntryToSOMQuery',
            AddSOMEntry(
                field_adding_default="class_"),
            transitions={
                SUCCESS:'WaitALittle' if wait_before_querying else 'QuerySom'
            },
            remapping={'value' :'class_'});
        
        if wait_before_querying:
            smach.StateMachine.add(
                'WaitALittle',
                WaitForSecs(time_interval),
                transitions={
                    SUCCESS:'QuerySom'},
                remapping={});
        
        smach.StateMachine.add(
            'QuerySom',
            PerformSOMQuery(),
            transitions={
                SUCCESS:'CheckIfFound',
                FAILURE:FAILURE});

        smach.StateMachine.add(
            'CheckIfFound',
            GetListEmpty(),
            transitions={
                'list_empty':'object_not_seen',
                'list_not_empty':'object_seen'},
            remapping={
                'input_list':'som_query_results',
                'list_empty':'item_not_found'
            });

    return sub_sm;


class SortSOMResultsAsPer(smach.State):
    """
    Take the put away my groceries task. We want to pick things 
    up as per a priority list across category. This sorts the 
    entries found by the field `sort_by`, in order of `order_of_preference`.

    NOTE: Error safe, in that it checks that the parameter exists in the 
    object first. This however does mean that this might cause a bug further 
    down the pipeline. 
    """
    def __init__(self, sort_by:str, order_of_preference:List[str]):
        smach.State.__init__(
            self, outcomes=[SUCCESS, 'list_empty'],
            input_keys=['som_query_results'],
            output_keys=['som_query_results', 'first_result']);
        self.sort_by:str = sort_by;
        self.order_of_preference:List[str] = order_of_preference;

    def execute(self, userdata):
        queries:List[object] = userdata.som_query_results;
        queries_output:List[object] = [];

        if len(queries) == 0:
            userdata.som_query_results = queries_output;
            userdata.first_result = 0;
            return 'list_empty';


        num_skipped = 0;

        for element in self.order_of_preference:
            for query in queries:
                if hasattr(query, self.sort_by):
                    if getattr(query, self.sort_by) == element:
                        queries_output.append(query);
                else:
                    num_skipped += 1;
        
        if num_skipped > 0:
            rospy.logwarn(
                "{0} entries out of {1} did not have the parameter in question.".format(
                num_skipped, len(queries)));

        if num_skipped + len(queries_output) < len(queries):
            rospy.loginfo("{0} entries were ignored. They had the correct field, but their values were not found in order of preference".format(
                len(queries) - len(queries_output) - num_skipped));

        userdata.som_query_results = queries_output;
        userdata.first_result = queries_output[0];
        return SUCCESS;
    pass;


class SaveOperatorToSOM(smach.State):
    """ State for robot to log the operator information as an observation in the SOM

    input_keys:
        operator_name:              - the operator's name
        human_object_uid:str        - The uid of the person refrence within the object collection.
    output_keys:
        operator_som_human_id: the operator's UID in the SOM human collection, returned by SOM observation service call
        operator_som_obj_id: the operator's corresponding UID in the SOM object collection
                             (assumed to be the most recently observed human-class object)
    """

    def __init__(self, operator_pose=None):
        smach.State.__init__(self, outcomes=['success', 'failure'],
                                input_keys=['operator_name', 'closest_human'],
                                output_keys=['operator_som_human_id',
                                            'operator_som_obj_id'])

        self.operator_pose_backup = operator_pose;

    def execute(self, userdata):

        # retreive the most recently observed human-class object in the SOM
        # operator_som_obj = get_most_recent_obj_from_som(class_="person")
        # if(operator_som_obj is None):
        #     # couldn't find any "person"-class objects in the SOM
        #     rospy.logwarn("Could not find any SOM object with class_ 'person' - state failed!")
        #     return 'failure'

        closest_human:Human = userdata.closest_human;

        # create the service message
        operator_obs = SOMAddHumanObsRequest()

        operator_obs.adding.observed_at = rospy.Time.now()
        operator_obs.adding.object_uid  = closest_human.object_uid if closest_human != None else "";
        operator_obs.adding.name = userdata.operator_name
        operator_obs.adding.task_role = 'operator'
        if self.operator_pose_backup == None:
            operator_obs.adding.obj_position = closest_human.obj_position    # same as before
        else:
            operator_obs.adding.obj_position = self.operator_pose_backup     # Use backup location.
            print(self.operator_pose_backup);
        operator_obs.adding.spoken_to_state = Human._OPERATOR;

        # todo - check if used
        # pose = rospy.wait_for_message('/global_pose', PoseStamped).pose

        rospy.loginfo('Storing info for operator "{}" in SOM:\n\t{}'.format(operator_obs.adding.name, operator_obs.adding))

        rospy.wait_for_service('som/human_observations/input')
        som_human_obs_input_service_client = rospy.ServiceProxy('som/human_observations/input', SOMAddHumanObs)

        result:SOMAddHumanObsResponse = som_human_obs_input_service_client(operator_obs)

        rospy.loginfo('Operator "{}" successfully stored in SOM with human collection UID: {}'.format(operator_obs.adding.name, result.UID))
        userdata.operator_som_human_id = result.UID
        userdata.operator_som_obj_id = operator_obs.adding.object_uid
        return 'success'

class SaveGuestToSOM(smach.State):
    """ State for robot to log the guest information as an observation in the SOM,
        and update the ongoing list of som ids (both human ids and object ids)

    input_keys:
        guest_attributes: the guest's attributes (e.g., name), to be added to the SOM, represented as a dictionary with keys: ["name","age","gender","pronouns","facial_features"]
        guest_som_human_ids: list of UIDs in the SOM human collection, corresponding to the guests we've met so far
        guest_som_obj_ids: list of UIDs in the SOM object collection, corresponding to the guests we've met so far
    output_keys:
        guest_som_human_ids: as above
        guest_som_obj_ids: as above
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure'],
                                input_keys=['operator_name', 
                                            'closest_human', 
                                            'guest_attributes',
                                            'guest_som_human_ids',
                                            'guest_som_obj_ids'],
                                output_keys=['guest_som_human_ids',
                                            'guest_som_obj_ids'])

    def execute(self, userdata):
        closest_human:Human = userdata.closest_human;

        # create the service message
        guest_obs = SOMAddHumanObsRequest()

        guest_obs.adding.observed_at = rospy.Time.now()
        guest_obs.adding.object_uid  = closest_human.object_uid
        guest_obs.adding.task_role = 'guest';
        guest_obs.adding.obj_position = closest_human.obj_position    # same as before

        # We don't want to speak to the same guest twice.
        guest_obs.adding.spoken_to_state = Human._SPOKEN_TO;

        print(userdata.guest_attributes)

        if("name" in userdata.guest_attributes):
            guest_obs.adding.name = userdata.guest_attributes["name"]
        if("gender" in userdata.guest_attributes):
            guest_obs.adding.gender = userdata.guest_attributes["gender"]
        if("pronouns" in userdata.guest_attributes):
            guest_obs.adding.pronouns = userdata.guest_attributes["pronouns"]
        if("face_id" in userdata.guest_attributes):
            guest_obs.adding.face_id = userdata.guest_attributes["face_id"]
        if("face_attributes" in userdata.guest_attributes):
            guest_obs.adding.face_attributes = userdata.guest_attributes["face_attributes"]

        # todo - check if used
        # pose = rospy.wait_for_message('/global_pose', PoseStamped).pose

        rospy.loginfo('Storing info for guest "{}" in SOM:\n{}'.format(guest_obs.adding.name, guest_obs.adding))

        rospy.wait_for_service('som/human_observations/input');
        som_human_obs_input_service_client = rospy.ServiceProxy('som/human_observations/input', SOMAddHumanObs)

        result:SOMAddHumanObsResponse = som_human_obs_input_service_client(guest_obs)

        rospy.loginfo('Guest "{}" successfully stored in SOM with human collection UID: {}'.format(guest_obs.adding.name, result.UID))
        userdata.guest_som_human_ids.append(result.UID)
        userdata.guest_som_obj_ids.append(guest_obs.adding.object_uid)
        return 'success'

class GetNearestHuman(smach.State):
    """
    Finds the closest operator to the current robot's position.
    NOTE: currently might return a human with a single observation (which has the possibility of being unreliable).
    This also sets the current robot pose (for convenience).
    Inputs:
        nearest_to:Pose|None        If None, then closest to the robot, otherwise closest to the pose given.   
    Outputs:
        closest_human:(Human|None)
        robot_pose:Pose
        human_pose:Pose
    """
    def __init__(self, ignore_operators=True):
        smach.State.__init__(self, outcomes=['new_human_found', 'human_not_found', 'existing_human_found'],
                                input_keys=['nearest_to'],
                                output_keys=['closest_human', 'robot_location', 'human_pose', 'human_object_uid'])

        self.ignore_operators = ignore_operators;

        self.human_query_srv = rospy.ServiceProxy('/som/humans/basic_query', SOMQueryHumans);
    
    def perform_query(self, query:SOMQueryHumansRequest, nearest_to:Pose) -> Human:
        human_query_results:SOMQueryHumansResponse = self.human_query_srv(query);

        min_distance = math.inf;
        closest_human:Human = None;

        for result in human_query_results.returns:
            result:Human;
            distance = distance_between_poses(result.obj_position, nearest_to);
            if distance < min_distance:
                min_distance = distance;
                closest_human = result;
        
        return closest_human;

    def execute(self, userdata):
        
        # What's the current position of the robot?
        robot_pose:PoseStamped = rospy.wait_for_message('/global_pose', PoseStamped);
        userdata.robot_location = robot_pose.pose

        nearest_to:Pose = userdata.nearest_to;
        print(nearest_to);
        
        query = SOMQueryHumansRequest();
        query.query.spoken_to_state = Human._NOT_SPOKEN_TO;

        if nearest_to == None:
            closest_human:Human = self.perform_query(query, robot_pose.pose);
        else:
            closest_human:Human = self.perform_query(query, nearest_to);

        if closest_human == None:
            query.query.spoken_to_state = Human._NULL;

            closest_human = self.perform_query(query, robot_pose.pose);

            if closest_human == None or (self.ignore_operators and closest_human.spoken_to_state == Human._OPERATOR):
                userdata.closest_human = None;
                userdata.human_pose = nearest_to;
                userdata.human_object_uid = "";
                return 'human_not_found';
            else:
                userdata.closest_human = closest_human;
                userdata.human_pose = closest_human.obj_position;
                userdata.human_object_uid = closest_human.object_uid;
                return 'existing_human_found';
        else:
            userdata.closest_human = closest_human;
            userdata.human_pose = closest_human.obj_position;
            userdata.human_object_uid = closest_human.object_uid;
            return 'new_human_found';

class GetHumanRelativeLoc(smach.State):
    """
    We want to be able to give the location of the human relative to other objects around the room.
    This will work this out.
    Inputs:
    Outputs:
        relevant_matches    The list of closest matches in the form of a list of dictionaries with the following parameters:#
            human_obj_uid       The uid within the object collection.
            relational_str      A string giving the relation in a readable form.
    """
    # Gives the objects we want to say we are nearby/next to/...
    RELATIVE_OBJS = [
        "table", "couch", "chair", "potted plant", 
        "bed", "mirror", "dining table", "tv", 
        "door", "sink", "clock", "vase", "desk"];

    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'no_relevant_matches_found'],
                                input_keys=['couch_left','couch_right', 'left_of_couch', 'right_of_couch'],
                                output_keys=['relevant_matches'])

    def get_most_relevant_relation(self, relation:Relation) -> str:
        if relation.left:
            return " was to the left of the ";
        elif relation.right:
            return " was to the right of the ";
        elif relation.frontof:
            return " was infront of the ";
        elif relation.behind:
            return " was behind the ";
        elif relation.near:
            return " was near to the ";
        return None;

    def get_relative_loc_per_human(self, human_obj_uid:str, human_name:str) -> list:
        query = SOMRelObjQueryRequest();
        query.obj1.HEADER.UID = human_obj_uid;
        query.obj2.category = "unknown";        # So anything not in the file of pickupable objects will be given the category of "unknown." We can use this to our advantage.

        response:SOMRelObjQueryResponse = self.relational_query_srv(query);

        def get_relation_dist(rel:Match):
            return rel.distance;
        # In ascending order by distance, so it will return the closest objects first.
        matches_sorted:list = sorted(response.matches, key=get_relation_dist);

        # The relation is [obj1] [relation] [obj2]. Therefore, if left is true for instance
        # then it will be [human] is to the left of [object].

        relevant_matches = [];

        for match in matches_sorted:
            match:Match;            
            if match.obj2.class_ in GetHumanRelativeLoc.RELATIVE_OBJS:
                relevant_relation = self.get_most_relevant_relation(match.relation);
                if relevant_relation != None:
                    relevant_matches.append({
                        'human_obj_uid': human_obj_uid,
                        'relational_str': self.get_most_relevant_relation(match.relation) + match.obj2.class_,
                        'human_name':human_name,
                        'distance_from_obj':match.distance
                    });

        return relevant_matches;

    def execute(self, userdata):

        rospy.wait_for_service('/som/objects/relational_query');
        rospy.wait_for_service('/som/humans/basic_query');
        self.relational_query_srv = rospy.ServiceProxy('/som/objects/relational_query', orion_actions.srv.SOMRelObjQuery);
        self.humans_query = rospy.ServiceProxy('/som/humans/basic_query', orion_actions.srv.SOMQueryHumans);

        
        human_query = SOMQueryHumansRequest();
        human_query.query.spoken_to_state = Human._SPOKEN_TO;
        spoken_to_guests:SOMQueryHumansResponse = self.humans_query(human_query);

        returns = [];
        for guest in spoken_to_guests.returns:
            guest:Human;
            relevant_matches = self.get_relative_loc_per_human(guest.object_uid, guest.name);
            returns.append(relevant_matches);
        
        if len(returns) == 0:
            userdata.relevant_matches = None;
            return "no_relevant_matches_found";
        else:
            userdata.relevant_matches = returns;
            return "success"

class GetOperatorLoc(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure'],
                                input_keys=[],
                                output_keys=['operator_pose'])
        self.human_query_srv = rospy.ServiceProxy('/som/humans/basic_query', SOMQueryHumans);

    def execute(self, userdata):
        query = SOMQueryHumansRequest();
        query.query.spoken_to_state = Human._OPERATOR;
        responses:SOMQueryHumansResponse = self.human_query_srv(query);

        if len(responses.returns) == 0:
            return 'failure';

        operator_entry:Human = responses.returns[0];
        userdata.operator_pose = operator_entry.obj_position;

        return 'success';
    pass;

class CheckForNewGuestSeen(smach.State):
    """ Smach state to check if we have seen a new guest (i.e., a guest we have not spoken to yet)

    Returns 'success' if a new guest is found, otherwise runs indefinitely. Needs to be preempted in a concurrency state.

    input_keys:

    output_keys:
        found_guest_uid: the uid of the found guest, if any
    """

    def __init__(self):
        smach.State.__init__(self,
                                outcomes=['success', 'preempted'],
                                input_keys=[],
                                output_keys=['found_guest_uid'])

    def execute(self, userdata):
        userdata.found_guest_uid = "not_set"   # initialise to empty to prepare for case where state gets preempted before new guest is found
                                        # (SMACH will complain if output key does not exist)

        # we sit in this loop forever, and only terminate with outcome 'success' if a new guest is found, or 'preempted' if preempted in concurrent state machine
        while True:
            # Check for preempt
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'

            # Check for new guest found. If there are multiple, then go to the closest one
            human_query_srv = rospy.ServiceProxy('/som/humans/basic_query', SOMQueryHumans);

            query = SOMQueryHumansRequest();
            query.query.spoken_to_state = Human._NOT_SPOKEN_TO;

            human_query_results:SOMQueryHumansResponse = human_query_srv(query);

            # What's the current position of the robot?
            robot_pose:PoseStamped = rospy.wait_for_message('/global_pose', PoseStamped);

            min_distance = math.inf;
            closest_human:Human = None;

            for result in human_query_results.returns:
                result:Human;
                distance = distance_between_poses(result.obj_position, robot_pose.pose);
                # Don't consider any human results with task role of operator
                if result.task_role == 'operator':
                    continue
                if distance < min_distance:
                    min_distance = distance;
                    closest_human = result;

            if closest_human is not None:
                userdata.found_guest_uid = closest_human.object_uid
                rospy.loginfo("Found guest not yet spoken to, object_uid: {}".format(closest_human.object_uid))
                return 'success'
            else:
                rospy.loginfo("No new guest not found yet")
                rospy.sleep(2)
