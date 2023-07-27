#!/usr/bin/env python3
from state_machines.Reusable_States.utils import *;
from state_machines.Reusable_States.procedural_states import *;
from state_machines.Reusable_States.misc_states import *;

import smach;

from orion_actions.msg import *;
from orion_actions.srv import *;

import rospy;

import math;
import numpy as np;

from geometry_msgs.msg import Pose, PoseStamped;

from typing import List, Tuple;

"""
Overall interface into SOM:
    - First you make a query.
    - Then you input the query into the system. 
    - Note that you can get it to set up the query as well, but that will be an input argument.
"""


class CreateSOMQuery(SmachBaseClass):
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
        SmachBaseClass.__init__(self, 
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


class AddSOMEntry(SmachBaseClass):
    """
    Optional parameters are not a thing within smach. This will add 
    parameters to the SOM query. 
    It thus edits som_query.
    Inputs:
        field_adding_default:str    - The field we are adding. 
        set_to_default              - If this is not None, then the value will be set to this. Otherwise, it will be set to `userdata.value`.
        som_query:                  - The field we are editing.
    """
    def __init__(self, field_adding_default:str, set_to_default=None):
        SmachBaseClass.__init__(self, 
            outcomes=[SUCCESS],
            input_keys=['som_query', 'value'],
            output_keys=['som_query']);
        
        self.field_adding_default:str = field_adding_default;
        self.set_to_default = set_to_default;
    
    def execute(self, userdata):
        query = userdata.som_query;

        set_to = userdata.value if self.set_to_default==None else self.set_to_default;

        query_inner = query.query;
        if hasattr(query_inner, self.field_adding_default):
            setattr(query_inner, self.field_adding_default, set_to);
        # potentially redundant, but I'll do it anyway:
        query.query = query_inner;
        return SUCCESS;


class PerformSOMQuery(SmachBaseClass):
    """
    Performs a SOM query.
    distance_filter - We may want to filter observations by distance from the robot. 
        If this is non-zero, this will do that filtering.

    Inputs:
        distance_filter:float                   : A distance beyond which we will ignore items.
        som_query:<query_type>                  : The query we will give the SOM system. This does type checking for the correct query.
    Outputs:
        som_query_results:List[<response_type>] : The response in the form of a raw array.
    """
    def __init__(self, distance_filter:float=0):
        SmachBaseClass.__init__(self, 
            outcomes=[SUCCESS],
            input_keys=['som_query'],
            output_keys=['som_query_results']);
            
        self.distance_filter = distance_filter;

    def execute(self, userdata):

        query = userdata.som_query;

        # print(query);
        # print(rospy.Time.now());

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
            print("Query type not found.")
            raise Exception("Query type not found.")

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
        for element in output:
            print("\t({0}, {1}, {2})".format(element.class_, element.category, element.num_observations));
        return SUCCESS;


class FindMyMates_IdentifyOperatorGuests(SmachBaseClass):
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
        SmachBaseClass.__init__(self, 
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
                SUCCESS:'CheckIfFound'});

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


class SortSOMResultsAsPer(SmachBaseClass):
    """
    Take the put away my groceries task. We want to pick things 
    up as per a priority list across category. This sorts the 
    entries found by the field `sort_by`, in order of `order_of_preference`.

    NOTE: Error safe, in that it checks that the parameter exists in the 
    object first. This however does mean that this might cause a bug further 
    down the pipeline. 

    Inputs:
        sort_by:str
        order_of_preference:List[str]
        sort_by_num_observations_first:bool
        num_observations_filter_proportion:float
        filter_for_duplicates_distance:float        : If we are sorting by number of observations, then this will discard any elemnts with fewer observations that are within this distance (m).
    """
    def __init__(
            self, 
            sort_by:str, 
            order_of_preference:List[str], 
            sort_by_num_observations_first:bool=False,
            num_observations_filter_proportion=0.001,
            filter_for_duplicates_distance:float=0):
        
        SmachBaseClass.__init__(
            self, outcomes=[SUCCESS, 'list_empty'],
            input_keys=['som_query_results'],
            output_keys=['som_query_results_out', 'first_result']);
        self.sort_by:str = sort_by;
        self.order_of_preference:List[str] = order_of_preference;
    
        self.sort_by_num_observations_first = sort_by_num_observations_first;
        self.num_observations_filter_proportion = num_observations_filter_proportion;
        self.filter_for_duplicates_distance = filter_for_duplicates_distance;

    def execute(self, userdata):
        queries:List[object] = userdata.som_query_results;
        queries_output:List[object] = [];
        
        if len(queries) == 0:
            userdata.som_query_results_out = queries_output;
            userdata.first_result = 0;
            return 'list_empty';

        if self.sort_by_num_observations_first:
            queries:List[SOMObject]
            queries.sort(key=lambda x:-x.num_observations);
            max_num_observations = queries[0].num_observations;
            queries_carry:List[SOMObject] = [];
            for element in queries:
                if element.num_observations > max_num_observations * self.num_observations_filter_proportion:
                    queries_carry.append(element);
            
            if self.filter_for_duplicates_distance != 0:
                indices_removing = [];
                for i in range(len(queries_carry)):
                    for j in range(i+1,len(queries_carry)):
                        if distance_between_poses(queries_carry[i].obj_position, queries_carry[j].obj_position) < self.filter_for_duplicates_distance:
                            indices_removing.append(j);
                queries = [];
                for i,element in enumerate(queries_carry):
                    if i not in indices_removing:
                        queries.append(element);
                print("Filtered for duplicates:");
                for element in queries:
                    print(element.class_, end=", ");
                print();
            else:
                queries = queries_carry;
        


        num_skipped = 0;

        for element in self.order_of_preference:
            for query in queries:
                if hasattr(query, self.sort_by):
                    if getattr(query, self.sort_by) == element:
                        queries_output.append(query);
                else:
                    num_skipped += 1;
        num_skipped = num_skipped // len(self.order_of_preference) 
        if num_skipped > 0:
            rospy.logwarn(
                "{0} entries out of {1} did not have the parameter in question.".format(
                num_skipped, len(queries)));

        if num_skipped + len(queries_output) < len(queries):
            rospy.loginfo("{0} entries were ignored. They had the correct field, but their values were not found in order of preference".format(
                len(queries) - len(queries_output) - num_skipped));

        userdata.som_query_results_out = queries_output;

        print("Sorted elements");
        for element in queries_output:
            print(element.class_, end=", ");
        print();

        if len(queries_output) == 0:
            return 'list_empty'

        userdata.first_result = queries_output[0];
        return SUCCESS;
    pass;


class FilterSOMResultsAsPer(SmachBaseClass):
    """
    Filters in/out a set of results by a given parameter.
    Inputs:
        filter_by               : What parameter are we filtering by
        filter_out              : If filter_out==True, then any matches will be filtered out. Else, filtered in.
        som_query_results       : The set that is being filtered
        filtering_by            : A list of parameters that determine whether a given element stays in the set.
    Output:
        som_query_results       : The filtered list.
        som_query_results_old   : The old version to allow the remembering of things.
    """
    def __init__(self, filter_by:str, filter_out=True):
        SmachBaseClass.__init__(self, 
            outcomes=[SUCCESS],
            input_keys=['som_query_results', 'filtering_by'],
            output_keys=['som_query_results', 'som_query_results_old']);
        
        self.filter_by = filter_by;
        self.filter_out = filter_out;

    def execute(self, userdata):
        som_query_results = userdata.som_query_results;
        userdata.som_query_results_old = som_query_results;
        filtering_by:list = userdata.filtering_by;
        output = [];
        for element in som_query_results:
            if self.filter_out:
                if getattr(element, self.filter_by) not in filtering_by:
                    output.append(element);
            else:
                if getattr(element, self.filter_by) in filtering_by:
                    output.append(element);
        userdata.som_query_results = output;
        return SUCCESS;



class SOMOccupancyMap:
    """
    We have the locations of all the objects seen through SOM. We should be able to create an 
    occupancy map of places and then work out viable placement locations based on that.
    
    This class looks at everything seen within a certain time frame, and, using the sizes of all the objects,
    creates an occupancy map. This occupancy map is then used to find free space which can be used to find
    placement options. 
    
    There is then a class within robocup_2023_hypothesis/put_away_the_groceries.py called 
    FindPlacementLocationBackup that uses this to find a new placement location and then 
    put something down at this location. 
    
    This class, while not configured for humans, could also be used to find free space on a 
    couch (for example). 
    """

    UNOCCUPIED = 0;
    OCCUPIED = 1;
    NOT_SUPPORTED = 2;
    OFF_THE_EDGE = 3;
    GOAL_USING = 4;

    def __init__(self, grid_resolution=0.03, time_horizon:rospy.Duration=None) -> None:
        """
        Input args:
            grid_resolution:float/m         : 
            time_horizon:rospy.Duration     : The time horizon over which we are querying the SOM system.
        """
        self.occupancy_map_created = False;
        self.occupancy_map = None;
        self.cell_resolution = grid_resolution;
        self.origin:Point = Point();
        self.time_horizon:rospy.Duration = time_horizon;
    
        self.object_query_srv = rospy.ServiceProxy('/som/objects/basic_query', SOMQueryObjects);
        
    def coordinatesToPoint(self, i:int, j:int) -> tuple:
        x = i*self.cell_resolution + self.origin.x;
        y = j*self.cell_resolution + self.origin.y;
        return x,y;
    def pointToCoordinates(self, x:float, y:float) -> tuple:
        i = (x-self.origin.x)/self.cell_resolution;
        j = (y-self.origin.y)/self.cell_resolution;
        return int(i),int(j);

    def createOccupancyMap(self, min_z=None, max_z=None, ignore_categories:list=None, xy_buffer_width=0.3, distance_criterion=1):
        """
        Creates the occupancy map to work out locations that might work for placement options.
        Input args:
            min_z:float                 : Min z val below which we ignore.
            max_z:float                 : Max z val above which we ignore.
            ignore_categories:List[str] : There are likely to be categories that we want to ignore.
            xy_buffer_width:float       : How much wider do we want the space.
        """
        if ignore_categories == None:
            ignore_categories = [];
        
        query = SOMQueryObjectsRequest();
        if self.time_horizon != None:
            query.query.last_observed_at = rospy.Time.now() - self.time_horizon;
        query_results_unflitered:List[SOMObject] = self.object_query_srv(query).returns;
        query_results_filtered:List[SOMObject] = [];

        # print(query_results_unflitered);

        #region Filtering based on the criteria we set out in the inputs.
        current_pose = get_current_pose();
        for obj in query_results_unflitered:
            if min_z != None and max_z != None:
                if obj.obj_position.position.z < min_z or obj.obj_position.position.z > max_z:
                    print("Ignoring obj of class by z val", obj.class_, obj.category);
                    continue;
            if obj.category in ignore_categories:
                print("Ignoring obj of class by category", obj.class_, obj.category);
                continue;
            if distance_between_poses(obj.obj_position, current_pose) > distance_criterion:
                print("Ignoring obj of class by distance", obj.class_, obj.category);
                continue;

            query_results_filtered.append(obj);
        #endregion
        
        # print(query_results_filtered);

        if len(query_results_filtered) == 0:
            raise Exception("No objects seen. Cannot form the occupancy map");

        #region Getting the range of inputs.
        max_y = max_x = -math.inf;
        min_x = min_y = math.inf;
        for obj in query_results_filtered:
            # print(obj.obj_position.position);
            if obj.obj_position.position.x < min_x:
                min_x = obj.obj_position.position.x;
            if obj.obj_position.position.x > max_x:
                max_x = obj.obj_position.position.x;
            if obj.obj_position.position.y < min_y:
                min_y = obj.obj_position.position.y;
            if obj.obj_position.position.y > max_y:
                max_y = obj.obj_position.position.y;
        self.origin = Point( min_x-xy_buffer_width, min_y-xy_buffer_width, 0 );
        print(self.origin);
        #endregion

        occupancy_grid_shape = self.pointToCoordinates(max_x+xy_buffer_width, max_y+xy_buffer_width);
        self.occupancy_map = np.full( occupancy_grid_shape, self.UNOCCUPIED, dtype=int );

        #region Filling out the occupancy map. Working out the mean z value.
        z_val_sum = 0;
        for obj in query_results_filtered:
            z_val_sum += obj.obj_position.position.z;

            min_i, min_j = self.pointToCoordinates(
                obj.obj_position.position.x-obj.size.x,
                obj.obj_position.position.y-obj.size.y);
            max_i, max_j = self.pointToCoordinates(
                obj.obj_position.position.x+obj.size.x,
                obj.obj_position.position.y+obj.size.y);
        
            self.occupancy_map[ min_i:max_i+1, min_j:max_j+1 ] = self.OCCUPIED;
        self.mean_z_val = z_val_sum/len(query_results_filtered);
        #endregion

        self.printGrid();
        pass;
        
    
    def findPlacementLocation(self, obj_size:Point) -> Tuple[Point, bool]:
        """
        Finds a location to place an object of obj_size. 
        Returns:
            Point   : The placement location.
            bool    : Whether a placement location was found.
        """
        HAND_HALF_WIDTH = 0.05;

        index_i_width = int((obj_size.x + HAND_HALF_WIDTH*2) / self.cell_resolution);
        index_j_width = int((obj_size.y + HAND_HALF_WIDTH*2) / self.cell_resolution);
        half_i = int(index_i_width/2);
        half_j = int(index_j_width/2);
        occupancy_map_shape = self.occupancy_map.shape;

        def checkLocFree(i,j) -> Tuple[int, int]:
            """
            Checks to see if a given index is free. 
            Note that this looks from the bottomm left corner!
            Returns:
                output[0] : The enum for which it is.
                output[1] : The number of not unoccupied spaces there are.
            """
            if i < 0 or j < 0 or i+index_i_width >= occupancy_map_shape[0] or j+index_j_width >= occupancy_map_shape[1]:
                return self.OFF_THE_EDGE, 0;
        
            # If every space is UNOCCUPIED, then return self.UNOCCUPIED.
            not_unoccupied:np.ndarray = (self.occupancy_map[i:i+index_i_width, j:j+index_j_width] != self.UNOCCUPIED);
            not_unoccupied_sum = np.sum(not_unoccupied);
            if not_unoccupied_sum == 0:
                return self.UNOCCUPIED, not_unoccupied_sum;
            return self.OCCUPIED, not_unoccupied_sum;
    
        ideal_i, ideal_j = int(occupancy_map_shape[0]/2), int(occupancy_map_shape[1]/2);
        def iDist(i,j):
            return (i-ideal_i)**2 + (j-ideal_j)**2;

        spaces_to_try_shape = (
            occupancy_map_shape[0]-index_i_width, 
            occupancy_map_shape[1]-index_j_width);
        
        # Now all we need is some way of checking the entire map.
        
        best_pair = (None, None);
        best_score = math.inf;
        for i in range(spaces_to_try_shape[0]):
            for j in range(spaces_to_try_shape[1]):
                is_free, num_occupied = checkLocFree(i,j);
                i_distance = iDist(i,j);
                if is_free == self.UNOCCUPIED and i_distance < best_score:
                    best_pair = (i,j);
                    best_score = i_distance;
        
        if best_pair[0] == None:
            return Point(), False;
        
        self.occupancy_map[
            best_pair[0]:best_pair[0]+index_i_width, 
            best_pair[1]:best_pair[1]+index_j_width] = self.GOAL_USING;
        
        best_pair = (best_pair[0]+half_i, best_pair[1]+half_j);
        best_coord = self.coordinatesToPoint( *best_pair );
        output = Point( x=best_coord[0], y=best_coord[1], z=self.mean_z_val );
        return output, True;


    def printGrid(self):
        print("Begin map");
        grid_shape = self.occupancy_map.shape;
        # print(self.cell_resolution);
        for i in range(grid_shape[0]):
            for j in range(grid_shape[1]):
                if self.occupancy_map[i,j]==self.OCCUPIED:
                    print("X", end="");
                else:
                    print( "." if self.occupancy_map[i,j]==self.GOAL_USING else " ", end="");
            print();
        print("End map");
        print();




class SaveOperatorToSOM(SmachBaseClass):
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
        SmachBaseClass.__init__(self, outcomes=['success', 'failure'],
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

class SaveGuestToSOM(SmachBaseClass):
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
        SmachBaseClass.__init__(self, outcomes=['success', 'failure'],
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

class GetNearestHuman(SmachBaseClass):
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
        SmachBaseClass.__init__(self, outcomes=['new_human_found', 'human_not_found', 'existing_human_found'],
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

class GetHumanRelativeLoc(SmachBaseClass):
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
        SmachBaseClass.__init__(self, outcomes=['success', 'no_relevant_matches_found'],
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

class GetOperatorLoc(SmachBaseClass):
    def __init__(self):
        SmachBaseClass.__init__(self, outcomes=['success', 'failure'],
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

class CheckForNewGuestSeen(SmachBaseClass):
    """ Smach state to check if we have seen a new guest (i.e., a guest we have not spoken to yet)

    Returns 'success' if a new guest is found, otherwise runs indefinitely. Needs to be preempted in a concurrency state.

    input_keys:

    output_keys:
        found_guest_uid: the uid of the found guest, if any
    """

    def __init__(self):
        SmachBaseClass.__init__(self,
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


def testSOMOccupancyMap():
    occupancy_map = SOMOccupancyMap();
    occupancy_map.createOccupancyMap(
        ignore_categories=["unknown"],
        distance_criterion=3);
    location, location_found = occupancy_map.findPlacementLocation(Point(0.25,0.25,0));
    occupancy_map.printGrid();
    print(location);
    print(location_found);
    pass;

if __name__ == "__main__":
    rospy.init_node('test_som_occupancy_map');
    testSOMOccupancyMap();