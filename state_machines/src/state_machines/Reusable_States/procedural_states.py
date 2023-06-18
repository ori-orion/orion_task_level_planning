# This is for if a number is greater than/less than another number.
# Sometimes useful for different things.

import smach;
from state_machines.Reusable_States.utils import *;
from typing import List

TRUE_STR = 'true';
FALSE_STR = 'false';

class LessThanState(smach.State):
    def __init__(self, left=None, right=None):
        smach.State.__init__(
            self,
            outcomes=[TRUE_STR, FALSE_STR],
            input_keys=['left', 'right']);
        self.left = left
        self.right = right

    def execute(self, userdata):
        left = userdata.left if userdata.left is not None else self.left
        right = userdata.right if userdata.right is not None else self.right
        if (left < right):
            return TRUE_STR;
        else:
            return FALSE_STR;

class AppendToArrState(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=[SUCCESS],
            input_keys=['appending_to', 'appending_with'],
            output_keys=['appending_to']);

    def execute(self, userdata):
        appending_to = userdata.appending_to;
        appending_to.append(userdata.appending_with);
        userdata.appending_to = appending_to;
        return SUCCESS;

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
    def __init__(self, properties_getting:List[str], index:int=None):
        
        smach.State.__init__(
            self,
            outcomes=[SUCCESS, 'index_out_of_range'],
            input_keys=['input_list', 'index'],
            output_keys=properties_getting);
    
        self.properties_getting:List[str] = properties_getting;
        self.index = index;

    def execute(self, userdata):
        if self.index == None:
            index_looking_in:int = userdata.index
        else:
            index_looking_in = self.index;

        list_looking_at:list = userdata.input_list;

        if index_looking_in < 0 or index_looking_in >= len(list_looking_at):
            return 'index_out_of_range';
        
        entry = list_looking_at[index_looking_in];
        for property in self.properties_getting:
            setattr(userdata, property, getattr(entry, property));
        return SUCCESS;

class GetListEmpty(smach.State):
    """
    Returns whether a list is empty or not.
    input_keys:
        input_list:list  - A list to check
    output_keys:
        output_keys:bool = len(input_list) == 0. True if it's empty
    Results:
        list_not_emtpy   - len(input_list) != 0
        list_empty       - len(input_list) == 0
    """
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['list_not_empty', 'list_empty'],
            input_keys=['input_list'],
            output_keys=['list_empty']);

    def execute(self, userdata):
        list_looking_at:list = userdata.input_list;
        if len(list_looking_at) == 0:
            userdata.list_empty = True;
            return 'list_empty';
        else:
            userdata.list_empty = False;
            return 'list_not_empty';

