from state_machines.Reusable_States.utils import *;

import smach;

# from orion_actions.msg import *;
# from orion_actions.srv import *;

# import rospy;

class PrintToConsole(smach.State):
    """ 
    """

    def __init__(self, phrase=None):
        smach.State.__init__(
            self,
            outcomes = [SUCCESS, FAILURE],
            input_keys=["var"], output_keys=[])

        self.phrase=phrase;

    def execute(self, userdata):
        if self.phrase == None:
            print(userdata.var)
        else:
            print(self.phrase);
        return SUCCESS;

class ReadInFromConsole(smach.State):
    """ 
    """

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes = [SUCCESS, FAILURE],
            input_keys=[], output_keys=["var"])

    def execute(self, userdata):
        var = input();
        if len(var) == 0:
            return FAILURE;
        else:
            userdata.var = var;
            return SUCCESS;