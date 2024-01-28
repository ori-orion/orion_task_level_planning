#!/usr/bin/env python3

import rospy
from actionlib import SimpleActionClient
import math
from typing import List

import hsrb_interface

hsrb_interface.robot.enable_interactive()

from state_machines.Reusable_States.utils import SmachBaseClass, SUCCESS, distance_between_poses
from orion_actions.msg import SOMObject
from orion_actions.srv import SOMQueryObjectsRequest
from tmc_msgs.msg import TalkRequestAction, TalkRequestGoal, Voice

from state_machines.robocup_2024.put_away_the_groceries.common import compute_safe_mast_height, look_around, query_objects_from_som


class ChooseObjectToPickUp(SmachBaseClass):
    
    DISTANCE_FROM_POSE = 0.9

    def __init__(self, min_num_observations: int, 
                 table_mast_height: float, 
                 categories_to_pick_up: List[str],
                 distance_filter: float = 2, 
                 num_observations_filter_proportion=0.001,
                 filter_for_duplicates_distance=0.02):
        """
        Choose an object to be picked up from the table.
        Possible outcomes:
        - SUCCESS: an object has been detected

        Output keys:
        - `target_obj`: the chosen object

        Parameters:
        - `min_num_observations`: the minimum num of observations for an object 
                                    to be considered
        - `table_mast_height`: the height of the mast needed to look over the table
        - `categories_to_pick_up`: the categories the robot should choose from
        - `distance_filter`: the maximum distance for an object to be considered
        - `num_observations_filter_proportion`: used to filter out objects with not 
            enough observations
        - `filter_for_duplicates_distance`: we consider pairs of objects closer
            than this value to be duplicates
        """
        SmachBaseClass.__init__(self, outcomes=[SUCCESS], 
                                output_keys=["target_obj"])

        self.min_num_observations = min_num_observations
        self.table_mast_height = table_mast_height
        self.distance_filter = distance_filter
        self.num_observations_filter_proportion = num_observations_filter_proportion
        self.filter_for_duplicates_distance = filter_for_duplicates_distance
        self.categories_to_pick_up = categories_to_pick_up


    def raise_mast(self):
        """Raise the mast to the table height"""
        mast_height = compute_safe_mast_height(self.table_mast_height)
        
        if mast_height == 0:
            return
        
        self.moveToJointPositions({
            self.JOINT_ARM_LIFT   : mast_height,
            self.JOINT_ARM_FLEX   : -100 * math.pi / 180,
            self.JOINT_HEAD_PAN   : 0,
            self.JOINT_HEAD_TILT  : -math.pi / 6,
            self.JOINT_WRIST_FLEX : 0})


    def query_som(self, query: SOMQueryObjectsRequest):
        """Send query to the SOM, filter out objects too distant or with not 
        enough observations, and sort the remaining objects by category and 
        by num of observations (descending).
        
        - `query`: the query to be sent to the SOM
        """

        query_results = query_objects_from_som(query, distance_filter=self.distance_filter)
        if len(query_results) == 0:
            return []

        # Sort by num observations, and remove the objects with not enough observations
        query_results.sort(key=lambda x:-x.num_observations)
        num_observations_threshold = query_results[0].num_observations * self.num_observations_filter_proportion
        filtered_results = [element for element in query_results if 
                            element.num_observations > num_observations_threshold]

        # Remove objects that are duplicates
        results_no_duplicates: List[SOMObject] = []
        duplicate_indices: List[int] = []
        for i, element in enumerate(filtered_results):
            if i in duplicate_indices:
                continue
            results_no_duplicates.append(element)
            # Find duplicates of this object
            for j in range(i+1, len(filtered_results)):
                if distance_between_poses(element.obj_position, filtered_results[j].obj_position) < self.filter_for_duplicates_distance:
                    duplicate_indices.append(j)

        print("Filtered for duplicates:")
        print(list(map(lambda el: el.class_, results_no_duplicates)))
        print()
        
        # Sort objects by category
        queries_output: List[SOMObject] = []
        for element in self.categories_to_pick_up:
            for result in results_no_duplicates:
                if result.category == element:
                    queries_output.append(result)

        if len(queries_output) < len(results_no_duplicates):
            rospy.loginfo(f"{len(results_no_duplicates) - len(queries_output)} entries were ignored. They did not belong to a category to pick up.")

        print("Sorted elements:")
        print(list(map(lambda el: el.class_, results_no_duplicates)))
        print()

        return queries_output
    
    def say_object_to_pick_up(self, obj_to_pick_up: SOMObject):
        """State the class and category of the object.
        
        - `obj_to_pick_up`: the chosen object
        """
        action_goal = TalkRequestGoal()
        action_goal.data.language = Voice.kEnglish
        action_goal.data.sentence = f"Trying to pick up the {obj_to_pick_up.class_} of category {obj_to_pick_up.category}."

        rospy.loginfo("HSR speaking phrase: " + action_goal.data.sentence)
        speak_action_client = SimpleActionClient('/talk_request_action', TalkRequestAction)

        speak_action_client.wait_for_server()
        speak_action_client.send_goal(action_goal)
        speak_action_client.wait_for_result()

    
    def execute(self, userdata):
        objects_to_pick_up: List[SOMObject] = []
        while len(objects_to_pick_up) == 0:
            query = SOMQueryObjectsRequest()
            query.query.last_observed_at = rospy.Time.now()
            query.query.num_observations = self.min_num_observations

            self.raise_mast()
            look_around()

            objects_to_pick_up = self.query_som(query)

        obj_to_pick_up = objects_to_pick_up[0]
        self.say_object_to_pick_up(obj_to_pick_up)
        
        userdata.target_obj = obj_to_pick_up
        return SUCCESS
