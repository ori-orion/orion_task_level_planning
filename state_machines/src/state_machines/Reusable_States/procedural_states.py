# This is for if a number is greater than/less than another number.
# Sometimes useful for different things.

import smach;
from state_machines.Reusable_States.outcomes_enum import *;

TRUE_STR = 'true';
FALSE_STR = 'false';

class lessThanState(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=[TRUE_STR, FALSE_STR],
            input_keys=['left', 'right']);

    def execute(self, userdata):
        if (userdata.left < userdata.right):
            return TRUE_STR;
        else:
            return FALSE_STR;

class incrementValue(smach.State):
    def __init__(self, increment_by=None):
        smach.State.__init__(
            self,
            outcomes=[SUCCESS],
            input_keys=['val', 'increment_by'],
            output_keys=['val']);
    
        self.increment_by = increment_by;

    def execute(self, userdata):
        if (self.increment_by == None):
            userdata.val = userdata.val + userdata.increment_by;
        else:
            userdata.val = userdata.val + self.increment_by;
        
        return SUCCESS;
