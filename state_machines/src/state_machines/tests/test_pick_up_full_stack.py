#!/usr/bin/env python3
"""
Author: Matthew Munks
Maintainer: Matthew Munks

This is to act as a test for the full pick up stack and a guide as to
how to use the overall perception pipeline in combination with manipulation.

Outline:
    Use SOM (the memory system) to find what has been seen.
    Use manipulation to pick up the object.
"""

import orion_actions.msg;
import orion_actions.srv;
import rospy;
import actionlib;
from typing import List, Tuple;

def getObjectsInSOM(class_querying_for:str="bottle", time_horizon:rospy.Duration=None) -> List[orion_actions.msg.SOMObject]:
    """
    Performs the query to work out what the robot has seen.
    This includes getting the `tf_name` which will be required for manipulation. 
    """
    #region setting up the query.
    query = orion_actions.srv.SOMQueryObjectsRequest();
    query.query.class_ = class_querying_for;

    # This will query for objects seen since `time_horizon`. For instance, if
    # `time_horizon` is 2 minutes, then it will return everything with class
    # `class_querying_for` seen in the last 2 minutes.
    if time_horizon != None:
        query.query.last_observed_at = rospy.Time.now() - time_horizon;
    
    # --- Other query entries go here... ---
    # num_observations are an option.
    # You don't have to query for a class, that can be left blank.
    # Querying instead for categories (as defined by robocup) is possible.
    # ...
    #endregion

    #region Carrying out the query
    rospy.wait_for_service('/som/objects/basic_query');
    object_query_srv = rospy.ServiceProxy('/som/objects/basic_query', orion_actions.srv.SOMQueryObjects);
    result:orion_actions.srv.SOMQueryObjectsResponse = object_query_srv(query);
    output:List[orion_actions.msg.SOMObject] = result.returns;
    #endregion

    return output;
    

def pickUpObject(object_class="bottle") -> Tuple[bool, int]:
    """
    So we are trying to pick up an object of class `object_class`.

    First we query SOM to see what exists/what the robot has seen.
    """

    objects_seen:List[orion_actions.msg.SOMObject] = getObjectsInSOM(object_class, time_horizon=rospy.Duration(2*60));
    
    # Now `objects_seen` is a list of potential candidates.
    # Each entry has the properties given within the message definition.
    # However, just to highlight a couple:
    #   - obj_position      : Gives the position in the global frame... might be useful for sorting based on position.
    #   - num_observations  : You might want to look for the entity with the most observations for the sake of robustness.
    #   - size              : Might be useful for other reasons?
    #   - tf_name           : Name of the tf given. - we will use this later.  
    # One might want to sort based on some of these. For simplicity, we will just take the first entity for now.

    object_picking_up:orion_actions.msg.SOMObject = objects_seen[0];

    # Now for the manipulation component.
    pick_up_object_action_client = actionlib.SimpleActionClient('pick_up_object', orion_actions.msg.PickUpObjectAction)
    pick_up_object_action_client.wait_for_server()
    pick_up_goal = orion_actions.msg.PickUpObjectGoal();
    pick_up_goal.goal_tf = object_picking_up.tf_name;

    pick_up_object_action_client.send_goal(pick_up_goal)
    pick_up_object_action_client.wait_for_result()

    result:orion_actions.msg.PickUpObjectResult = pick_up_object_action_client.get_result();
    # Finally, result: 
    # There are two fields, a boolean for if it's succeeded or not, and a failure_mode enum for the reason why it's failed. 
    #   - result:bool              - Overall success or failure.
    #   - failure_mode: byte       - These are given in the action definition - might not fully be working yet... will keep you updated.
    return result.result, result.failure_mode

if __name__ == '__main__':
    rospy.init_node('som_mainpulation_test');
    pickUpObject("cup");