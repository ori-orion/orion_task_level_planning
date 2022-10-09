# This is for if a number is greater than/less than another number.
# Sometimes useful for different things.

import smach;
from state_machines.Reusable_States.utils import *;

TRUE_STR = 'true';
FALSE_STR = 'false';

class LessThanState(smach.State):
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

class IncrementValue(smach.State):
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

class GetPropertyAtIndex(smach.State):
    """
    Gets a (set) property at a given index in the array.
    Inputs:
        input_list:list     - What list are we taking the property from?
        index:int           - What index from that list are we interested in?
    Outputs:
        output_param:Any    - Returns the desired parameter. 
    The state of 'index_out_of_range' will be returned if the index is not within the bounds.
    """
    def __init__(self, property_getting:str):
        smach.State.__init__(
            self,
            outcomes=[SUCCESS, 'index_out_of_range'],
            input_keys=['input_list', 'index'],
            output_keys=['output_param']);
    
        self.property_getting = property_getting;

    def execute(self, userdata):
        list_looking_at:list = userdata.input_list;
        index_looking_in:int = userdata.index;
        if index_looking_in < 0 or index_looking_in >= len(list_looking_at):
            return 'index_out_of_range';
        entry = list_looking_at[index_looking_in];
        userdata.output_val = getattr(entry, self.property_getting);
        return SUCCESS;