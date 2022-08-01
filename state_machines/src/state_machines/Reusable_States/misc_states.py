from utils import *;

import smach;

from orion_actions.msg import *;
from orion_actions.srv import *;

import rospy;

import math;

from geometry_msgs.msg import Pose, PoseStamped;

import actionlib

from orion_door_pass.msg import DoorCheckGoal, DoorCheckAction

import hsrb_interface;
hsrb_interface.robot.enable_interactive();

class CheckDoorIsOpenState(smach.State):
    """ State for robot to check if the door is open. TODO: THE ACTION SERVER NEEDS TESTING!

    This is a common start signal for tasks.

    input_keys:

    output_keys:

    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['open', 'closed'])

    def execute(self, userdata):
        is_door_open_goal = DoorCheckGoal()
        is_door_open_goal.n_closed_door = 20 # Same as Bruno's code

        is_door_open_action_client = actionlib.SimpleActionClient('door_check',
                                                               DoorCheckAction)
        is_door_open_action_client.wait_for_server()
        is_door_open_action_client.send_goal(is_door_open_goal)
        is_door_open_action_client.wait_for_result()

        # Boolean value returned
        is_door_open = is_door_open_action_client.get_result().open
        if is_door_open:
            rospy.loginfo("Detected open door")
            return 'open'
        else:
            rospy.loginfo("Detected closed door")
            rospy.sleep(0.5);
            return 'closed'

#region Look at states
class LookUpState(smach.State):
    def __init__(self, height=1.2):
        smach.State.__init__(self, outcomes=['success']);

        self.height = height;

        self.robot = hsrb_interface.Robot();
        self.whole_body = self.robot.try_get('whole_body');

    def execute(self, userdata):
        self.whole_body.gaze_point(
            point=hsrb_interface.geometry.Vector3(1, 0, self.height), 
            ref_frame_id="base_link");

        return 'success';

class LookAtHuman(smach.State):
    """
    Look at the last human observed.
    
    Inputs:
        closest_human:(Human|None)
    """
    def __init__(self):
        smach.State.__init__(
            self, 
            outcomes=['success'],
            input_keys=['closest_human']);

        self.robot = hsrb_interface.Robot();
        self.whole_body = self.robot.try_get('whole_body');
    
    def execute(self, userdata):
        closest_human:Human = userdata.closest_human;
        human_loc = closest_human.obj_position.position;
        point_look_at = hsrb_interface.geometry.Vector3(human_loc.x, human_loc.y, 0.8);
        
        # NOTE: A very 'elegant' solution (that really needs to be changed at some point)!
        try:
            self.whole_body.gaze_point(
                point=point_look_at,
                ref_frame_id="map");
        except:
            point_look_at = hsrb_interface.geometry.Vector3(human_loc.x, human_loc.y, 0.8);
            try:
                self.whole_body.gaze_point(
                    point=point_look_at,
                    ref_frame_id="map");
            except:
                self.whole_body.gaze_point(
                    point=hsrb_interface.geometry.Vector3(1, 0, 0.8), 
                    ref_frame_id="base_link");
            rospy.logwarn("Error with gaze_point directly at the human.");
        return 'success';

class LookAtPoint(smach.State):
    """
    Look at the last human observed.
    
    Inputs:
        pose:Pose   The point to look at in 3D space.
    """
    def __init__(self):
        smach.State.__init__(
            self, 
            outcomes=['success'],
            input_keys=['pose']);

        self.robot = hsrb_interface.Robot();
        self.whole_body = self.robot.try_get('whole_body');
    
    def execute(self, userdata):
        pose:Pose = userdata.pose;
        point_look_at = hsrb_interface.geometry.Vector3(pose.position.x, pose.position.y, 1.3);
        
        # NOTE: A very 'elegant' solution (that really needs to be changed at some point)!
        try:
            self.whole_body.gaze_point(
                point=point_look_at,
                ref_frame_id="map");
        except:
            point_look_at = hsrb_interface.geometry.Vector3(pose.position.x, pose.position.y, 1.2);
            try:
                self.whole_body.gaze_point(
                    point=point_look_at,
                    ref_frame_id="map");
            except:
                self.whole_body.gaze_point(
                    point=hsrb_interface.geometry.Vector3(1, 0, 0.8), 
                    ref_frame_id="base_link");
            rospy.logwarn("Error with gaze_point directly at the human.");
        return 'success';

#endregion
