#!/usr/bin/env python3

import rospy;
import actionlib;
# from orion_actions import *;
from orion_face_recognition.msg import *;

def execute_test_capture_face():
    action_client = actionlib.SimpleActionClient('as_Capface', ActionServer_CapFaceAction)
    action_client.wait_for_server();
    print("Server found");

    goal = ActionServer_CapFaceGoal();
    print(dir(goal));
    print(goal);
    goal.face_id = "Matthew";
    action_client.send_goal(goal);
    action_client.wait_for_result();
    result = action_client.get_result();
    print(result);

def execute_test_get_face_attributes():
    action_client = actionlib.SimpleActionClient('as_Findattrs', ActionServer_FindAttrsAction);
    action_client.wait_for_server();
    print("server found");

    goal = ActionServer_FindAttrsGoal();
    # goal.goal_id needs to be empty if we are to use the camera. Otherwise specify to use stored images.
    action_client.send_goal(goal);
    action_client.wait_for_result();
    result = action_client.get_result();
    print(result);


if __name__ == '__main__':
    rospy.init_node('facial_test');
    execute_test_get_face_attributes();