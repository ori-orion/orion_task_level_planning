#!/usr/bin/env python3

import rospy;
from actionlib import SimpleActionClient

from state_machines.Reusable_States.utils import SmachBaseClass, SUCCESS
from orion_door_pass.msg import DoorCheckGoal, DoorCheckAction


class StartupWaitForDoor(SmachBaseClass):
    def __init__(self, speak_through_console = False):
        """
        Move the robot to the neutral pose, then wait until it detects the door to 
        be open.

        Possible outcomes:
        - SUCCESS

        Output keys:
        - task_start_time: the time at which the task started (after detecting the 
                            door open)
        """
        SmachBaseClass.__init__(self, outcomes=[SUCCESS], output_keys=["task_start_time"])

        self.wait_time = 2
        self.speak_through_console = speak_through_console

        self.speak_function = lambda x : print("SpeakState:", x) if speak_through_console else lambda x: self.speak(x)

    def moveToNeutral(self):
        """Move the robot to neutral pose.
        
        It may fails. We try 3 times before returning anyway.
        """
        for _ in range(3):
            try:
                self.moveToGo()
                return
            except Exception as e:
                rospy.logwarn("Exception raised within self.whole_body.move_to_neutral():\n" + str(e))
                rospy.loginfo("Retrying in 2s")
                rospy.sleep(2)
        
        rospy.logwarn("Could not move to neutral state, tried 3 times.")


    def checkDoorOpen(self) -> bool:
        """
        Check if the door is open.
        """
        is_door_open_goal = DoorCheckGoal()
        is_door_open_goal.n_closed_door = 20

        is_door_open_action_client = SimpleActionClient('door_check',
                                                        DoorCheckAction)
        is_door_open_action_client.wait_for_server()
        is_door_open_action_client.send_goal(is_door_open_goal)
        is_door_open_action_client.wait_for_result()
        
        is_door_open: bool = is_door_open_action_client.get_result().open #type:ignore
        if is_door_open:
            rospy.loginfo("Detected open door")
            return True
        else:
            rospy.loginfo("Detected closed door")
            return False

    def execute(self, userdata):
        rospy.loginfo("Moving to neutral pose")
        self.moveToNeutral()
        rospy.loginfo("Waiting for door to open")
        while not self.checkDoorOpen():
            rospy.sleep(self.wait_time)
        
        self.speak_function("The door is open.")
        now = rospy.Time.now()
        userdata.task_start_time = now
        rospy.loginfo("Retreived current time: %i sec, %i ns", now.secs, now.nsecs)
        return SUCCESS


