#!/usr/bin/env python3

import rospy
from actionlib import SimpleActionClient
import math
from typing import List

import hsrb_interface

hsrb_interface.robot.enable_interactive()

from state_machines.Reusable_States.utils import SmachBaseClass, SUCCESS, distance_between_poses, get_current_pose
from orion_actions.msg import SOMObject
from orion_actions.srv import SOMQueryObjectsRequest, SOMQueryObjects, SOMQueryObjectsResponse
from orion_spin.msg import SpinAction, SpinGoal
from tmc_msgs.msg import TalkRequestAction, TalkRequestGoal, Voice

from state_machines.robocup_2024.put_away_the_groceries.common import look_around, query_objects_from_som


class ChooseObjectToPickUp(SmachBaseClass):
    """
    """
    MAST_JOINT_MAX = 0.69
    MAST_JOINT_MIN = 0
    DISTANCE_FROM_POSE = 0.9

    def __init__(self, min_num_observations: int, 
                 table_mast_height: float, 
                 categories_to_pick_up: List[str],
                 distance_filter = 2, 
                 num_observations_filter_proportion = 0.001,
                 filter_for_duplicates_distance=0.02):
        SmachBaseClass.__init__(self, outcomes=[SUCCESS], 
                                output_keys=["obj_to_pick_up", "objects_to_pick_up"])

        self.min_num_observations = min_num_observations
        self.table_mast_height = table_mast_height
        self.distance_filter = distance_filter
        self.num_observations_filter_proportion = num_observations_filter_proportion
        self.filter_for_duplicates_distance = filter_for_duplicates_distance
        self.categories_to_pick_up = categories_to_pick_up


    def raise_mast(self):
        """Raise the mast to the table height"""
        mast_height = min(self.table_mast_height, self.MAST_JOINT_MAX)
        mast_height = max(mast_height, self.MAST_JOINT_MIN)
        
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
        by num of observations (descending)."""

        queries = query_objects_from_som(query, distance_filter=self.distance_filter)
        if len(queries) == 0:
            return []

        # Sort by num observations, and remove the objects with not enough observations or duplicates
        queries.sort(key=lambda x:-x.num_observations)
        max_num_observations = queries[0].num_observations
        queries_carry: List[SOMObject] = []
        for element in queries:
            if element.num_observations > max_num_observations * self.num_observations_filter_proportion:
                queries_carry.append(element)

        indices_removing = []
        for i in range(len(queries_carry)):
            for j in range(i+1,len(queries_carry)):
                if distance_between_poses(queries_carry[i].obj_position, queries_carry[j].obj_position) < self.filter_for_duplicates_distance:
                    indices_removing.append(j)
        queries = []
        for i, element in enumerate(queries_carry):
            if i not in indices_removing:
                queries.append(element)

        print("Filtered for duplicates:")
        for element in queries:
            print(element.class_, end=", ")
        print()
        
        # Sort objects by category
        queries_output: List[SOMObject] = []
        for element in self.categories_to_pick_up:
            for query in queries:
                if query.category == element:
                    queries_output.append(query)

        if len(queries_output) < len(queries):
            rospy.loginfo("{0} entries were ignored. They had the correct field, but their values were not found in order of preference".format(
                len(queries) - len(queries_output)))

        print("Sorted elements")
        for element in queries_output:
            print(element.class_, end=", ")
        print()

        return queries_output
    
    def say_object_to_pick_up(self, obj_to_pick_up: SOMObject):
        """State the class and category of the object."""
        action_goal = TalkRequestGoal()
        action_goal.data.language = Voice.kEnglish  # enum for value: 1
        action_goal.data.sentence = f"Trying to pick up the {obj_to_pick_up.class_} of category {obj_to_pick_up.category}."

        rospy.loginfo("HSR speaking phrase: '{}'".format(action_goal.data.sentence))
        speak_action_client = SimpleActionClient('/talk_request_action', TalkRequestAction)

        speak_action_client.wait_for_server()
        speak_action_client.send_goal(action_goal)
        speak_action_client.wait_for_result()

    
    def execute(self, userdata):
        objects_to_pick_up: List[SOMObject] = []
        while len(objects_to_pick_up) == 0:
            query = SOMQueryObjectsRequest()
            query.query.last_observed_at = rospy.Time.now()
            query.num_observations = self.min_num_observations

            self.raise_mast()
            look_around()

            objects_to_pick_up = self.query_som(query)

        obj_to_pick_up = objects_to_pick_up[0]
        self.say_object_to_pick_up(obj_to_pick_up)
        
        userdata.obj_to_pick_up = obj_to_pick_up
        userdata.objects_to_pick_up = objects_to_pick_up
        return SUCCESS
