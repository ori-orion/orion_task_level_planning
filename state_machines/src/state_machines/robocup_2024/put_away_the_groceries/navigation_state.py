import rospy
from geometry_msgs.msg import Pose

from state_machines.robocup_2024.put_away_the_groceries.common import navigate_to_pose
from state_machines.Reusable_States.include_all import NAVIGATIONAL_FAILURE, SmachBaseClass, SUCCESS

class NavigationState(SmachBaseClass):

    def __init__(self, execute_nav_commands:bool, max_num_failure_repetitions=4):
        """
        [COPY OF SimpleNAvigationState_V2]

        State for navigating directly to a location on the map.
        Version 2 of the navigation stuff. Removes the retrying being a part of the state machine.

        This state is given a pose and navigates there.
        If the robot stays in the same place for more than a second, then assume failed nav.
        If the navigation has failed, we will check to see if the goal location is blocked, and 
        then nav to a second location. 

        Possible outcomes:
        - SUCCESS if the robot reaches the target
        - NAVIGATIONAL_FAILURE otherwise  

        input_keys:
        - pose: pose for the robot to navigate to
        """
        SmachBaseClass.__init__(
            self,
            outcomes=[SUCCESS, NAVIGATIONAL_FAILURE],
            input_keys=['pose'],
            output_keys=[]);

        self.execute_nav_commands = execute_nav_commands;
        self.max_num_failure_repetitions = max_num_failure_repetitions;
    
    def execute(self, userdata):
        target_pose = userdata.pose;
        if navigate_to_pose(target_pose, self.max_num_failure_repetitions, self.execute_nav_commands):
            return SUCCESS
        return NAVIGATIONAL_FAILURE