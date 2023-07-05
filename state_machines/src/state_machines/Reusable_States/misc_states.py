#!/usr/bin/env python3

from state_machines.Reusable_States.utils import *;

import smach;

from orion_actions.msg import *;
from orion_actions.srv import *;
from orion_spin.msg import SpinAction, SpinGoal;
from orion_door_pass.msg import DoorCheckGoal, DoorCheckAction
from geometry_msgs.msg import WrenchStamped;

import rospy;

import math;

from geometry_msgs.msg import Pose, PoseStamped;

import actionlib

import hsrb_interface;
import hsrb_interface.geometry as geometry
hsrb_interface.robot.enable_interactive();

#region Temporal states
class GetTime(smach.State):
    """ Smach state for current time using ROS clock.

    This state will get the current time and return it in the userdata dict.
    """

    def __init__(self):
        smach.State.__init__(self,
                                outcomes = [SUCCESS],
                                output_keys=['current_time'])

    def execute(self, userdata):
        # fetch the time and return
        now = rospy.Time.now()
        userdata.current_time = now
        rospy.loginfo("Retreived current time: %i sec, %i ns", now.secs, now.nsecs)
        return SUCCESS

class WaitForSecs(smach.State):
    """
    Waits for a given number of seconds, parameterised by an input argument.
    num_secs can be either of type int/float or of rospy.Duration (given that rospy.sleep(.) supports both of those types). 
    """
    def __init__(self, num_secs:float):
        smach.State.__init__(
            self,
            outcomes = [SUCCESS]);
        self.num_secs = num_secs;

    def execute(self, userdata):
        rospy.sleep(self.num_secs);
        return SUCCESS;
#endregion


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
        smach.State.__init__(self, outcomes=[SUCCESS]);

        self.height = height;

        self.robot = hsrb_interface.Robot();
        self.whole_body = self.robot.try_get('whole_body');

    def execute(self, userdata):
        self.whole_body.gaze_point(
            point=hsrb_interface.geometry.Vector3(1, 0, self.height), 
            ref_frame_id="base_link");

        return SUCCESS;

class LookAtHuman(smach.State):
    """
    Look at the last human observed.
    
    Inputs:
        closest_human:(Human|None)
    """
    def __init__(self):
        smach.State.__init__(
            self, 
            outcomes=[SUCCESS],
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
        return SUCCESS;

class LookAtPoint(smach.State):
    """
    Look at the last human observed.
    
    Inputs:
        pose:Pose   The point to look at in 3D space.
    """
    def __init__(self, z_looking_at = 1.3, set_head_to_neutral:bool=False, wait_duration_afterwards:rospy.Duration=rospy.Duration(0)):
        """
        Inputs:
            z_looking_at: Gives the z-parameter. If None, then it will take it from userdata.pose.
        """
        smach.State.__init__(
            self, 
            outcomes=[SUCCESS],
            input_keys=['pose']);

        self.robot = hsrb_interface.Robot();
        self.whole_body = self.robot.try_get('whole_body');7

        self.z_looking_at = z_looking_at;
        self.set_head_to_neutral = set_head_to_neutral;
        self.wait_duration = wait_duration_afterwards;
    
    def execute(self, userdata):
        pose:Pose = userdata.pose;

        if self.z_looking_at != None:
            self.z_looking_at = pose.position.z;
        point_look_at = hsrb_interface.geometry.Vector3(pose.position.x, pose.position.y, self.z_looking_at);
        
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

        if self.set_head_to_neutral:
            self.whole_body.move_to_joint_positions({'head_tilt_joint':0})

        rospy.sleep(self.wait_duration);
        
        return SUCCESS;

#endregion


class RaiseMastState(smach.State):
    """
    Raises the mast to a given height.

    Outputs:
        body_rotated:bool   : Returns whether the base was rotated 90 degrees relative to the nav goal given.
    """
    MAST_JOINT_NAME = 'arm_lift_joint';
    MAST_JOINT_MAX = 0.69;
    MAST_JOINT_MIN = 0;
    def __init__(self, mast_height=None, rotate_body=True):
        input_keys = ['mast_height'] if mast_height==None else [];
            
        smach.State.__init__(
            self,
            outcomes=[SUCCESS],
            input_keys=input_keys,
            output_keys=['body_rotated']);

        self.robot = hsrb_interface.Robot();
        self.whole_body = self.robot.try_get('whole_body');
        self.omni_base = self.robot.try_get('omni_base');
        self.mast_height = mast_height;
        self.rotate_body = rotate_body;
    
    def execute(self, userdata):
        if self.mast_height == None: 
            mast_height = userdata.mast_height;
        else:
            mast_height = self.mast_height;

        if mast_height > self.MAST_JOINT_MAX:
            mast_height = self.MAST_JOINT_MAX;
        elif mast_height < self.MAST_JOINT_MIN:
            mast_height = self.MAST_JOINT_MIN;
        
        userdata.body_rotated = False;

        if mast_height == 0:
            return SUCCESS;

        userdata.body_rotated = False;
        self.whole_body.move_to_joint_positions({
            'arm_lift_joint':mast_height,
            'arm_flex_joint':-100*math.pi/180,
            'head_pan_joint':0,
            'head_tilt_joint':-math.pi/6,
            'wrist_flex_joint':0});
        return SUCCESS;
    
        # The code for rotating the body as well.
        if self.rotate_body:
            BASE_ROTATION = math.pi/2;
            self.whole_body.move_to_neutral();
            self.whole_body.move_to_joint_positions({
                'arm_lift_joint':mast_height,
                'arm_flex_joint':-0.1*math.pi/2,
                'head_pan_joint':-BASE_ROTATION});
            self.omni_base.follow_trajectory(
                [geometry.pose(ek=BASE_ROTATION)],
                time_from_starts=[10],
                ref_frame_id='base_footprint');
            userdata.body_rotated = True;
        else:
            self.whole_body.move_to_joint_positions({self.MAST_JOINT_NAME:mast_height})
        return SUCCESS;
    pass;

class MoveToNeutralState(smach.State):
    """
    Sets the robot's pose to neutral.
    """
    def __init__(self):    
        smach.State.__init__(
            self,
            outcomes=[SUCCESS]);

        self.robot = hsrb_interface.Robot();
        self.whole_body = self.robot.try_get('whole_body');
    
    def execute(self, userdata):
        for i in range(3):
            try:
                self.whole_body.move_to_neutral();
                return SUCCESS;
            except Exception as e:
                rospy.logwarn("Exception raised within self.whole_body.move_to_neutral().")
                print(e);
                rospy.loginfo("Retrying in 2s");
                rospy.sleep(2);
        return SUCCESS;
    pass;


class SpinState(smach.State):
    """
    Gets the robots head to spin around, probably in an attempt to find something.

    Inputs:
        spin_height:float=1 - So the overall spin action is basically a sequence of
            `look at point` commands in sequence. Each point is 1m from the robot in 
            the horizontal plane. This parameter gives the vertical height off the 
            ground for the robot to look at.
    """
    OFFSET_BY_0 = 0;
    OFFSET_BY_90 = 1;
    TAKE_OFFSET_FROM_USERDATA = 2;

    def __init__(self, spin_height:float=1, only_look_forwards:bool=False, offset_instruction:int=OFFSET_BY_0):
        input_keys = ['body_rotated'] if offset_instruction==self.TAKE_OFFSET_FROM_USERDATA else [];
        smach.State.__init__(
            self, 
            outcomes = [SUCCESS],
            input_keys=input_keys, 
            output_keys=[]);

        self.spin_height = spin_height;
        self.only_look_forwards:bool = only_look_forwards;
        self.offset_instruction = offset_instruction;

    def execute(self, userdata):
        client = actionlib.SimpleActionClient('spin', SpinAction);
        client.wait_for_server();
        goal = SpinGoal();
        goal.only_look_forwards = self.only_look_forwards;
        goal.height_to_look_at = self.spin_height;
        if self.offset_instruction == self.OFFSET_BY_0:
            goal.spin_offset = 0;
        elif self.offset_instruction == self.OFFSET_BY_90:
            goal.spin_offset = -90;
        elif self.offset_instruction == self.TAKE_OFFSET_FROM_USERDATA:
            goal.spin_offset = 0 if userdata.body_rotated==False else -90;
        client.send_goal(goal);
        client.wait_for_result();

        return SUCCESS;


class ExplicitRemap(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, 
            outcomes = [SUCCESS],
            input_keys=['in_key'], 
            output_keys=['out_key']);
    
    def execute(self, userdata):
        userdata.out_key = userdata.in_key;
        return SUCCESS;


class WaitForWristWrench(smach.State):
    """
    Waiting for a force to be applied to the wrist before moving off.
    Can be a replacement for the hotword detector.
    """

    FORCE_THRESHOLD = 5;
    WAIT_BETWEEN_IT = 0.1;
    
    def __init__(self):
        smach.State.__init__(
            self, 
            outcomes=[SUCCESS],
            input_keys=[], 
            output_keys=[]);
        
        self.mag = 0;
        self.first_run = True;
        self.x_offset = self.y_offset = self.z_offset = 0;
    
    
    def wrist_wrench_raw_sub(self, input_msg:WrenchStamped):
        if self.first_run:
            self.x_offset = input_msg.wrench.force.x;
            self.y_offset = input_msg.wrench.force.y;
            self.z_offset = input_msg.wrench.force.z;
            self.first_run = False;
        else:
            mag_sqared = (input_msg.wrench.force.x-self.x_offset)**2 + (input_msg.wrench.force.y-self.y_offset)**2 + (input_msg.wrench.force.z-self.z_offset)**2;
            self.mag = math.sqrt(mag_sqared);
            print(self.mag);

        
    
    def execute(self, userdata):
        sub = rospy.Subscriber('/hsrb/wrist_wrench/raw', WrenchStamped, self.wrist_wrench_raw_sub);
        while(True):
            if self.mag > self.FORCE_THRESHOLD:
                break;
            rospy.sleep(0.1);
        
        sub.unregister();
        print("Force detected");
        self.mag = 0;
        self.first_run = True;
        
        return SUCCESS;
    


class CreateGuestAttributesDict(smach.State):
    """ Smach state to build the guest attributes dictionary from userdata values.

    This state will return the built dictionary it in the userdata dict.
    """

    def __init__(self):
        smach.State.__init__(self,
                                outcomes = [SUCCESS],
                                input_keys=['guest_attributes','name','gender','pronouns','face_id','face_attributes'],
                                output_keys=['guest_attributes'])

    def execute(self, userdata):
        userdata.guest_attributes = {}
        userdata.guest_attributes["name"] = userdata.name
        userdata.guest_attributes["gender"] = userdata.gender
        userdata.guest_attributes["pronouns"] = userdata.pronouns
        userdata.guest_attributes["face_id"] = userdata.face_id
        userdata.guest_attributes["face_attributes"] = userdata.face_attributes

        print(userdata.guest_attributes);

        # rospy.loginfo("Created guest_attributes dict: {}".format(userdata.guest_attributes))
        return SUCCESS

class ShouldIContinueGuestSearchState(smach.State):
    """ State determines whether we should continue or go back.

    input_keys:
        max_search_duration: search duration (seconds)
        start_time: time we started the task (ros Time object)
        guest_som_obj_ids: the list of guest som object ids (only the length is used)
    output_keys:

    """

    def __init__(self):
        smach.State.__init__(self,
                                outcomes = ['yes', 'no'],
                                input_keys=['max_search_duration',
                                            'expected_num_guests',
                                            'start_time',
                                            'guest_som_obj_ids'])

    def execute(self, userdata):
        # fetch the current time
        now = rospy.Time.now()
        time_elapsed = now - userdata.start_time

        # if time_elapsed > 210: # 3 and a half minutes - # TODO move to state machine
        if time_elapsed > rospy.Duration(userdata.max_search_duration):
            rospy.loginfo("Exceeded max search time ({}/{} sec) - stop searching!".format(time_elapsed.to_sec(), userdata.max_search_duration))
            return "no"

        num_guests_found = len(userdata.guest_som_obj_ids)
        if num_guests_found >= userdata.expected_num_guests:
            rospy.loginfo("Found all the guests ({}) - stop searching!".format(num_guests_found))
            return "no";

        rospy.loginfo("I'll keep searching! Time: {}/{} sec, Found: {}/{} guests".format(time_elapsed.to_sec(), userdata.max_search_duration, num_guests_found,userdata.expected_num_guests))
        return "yes";

class AnnounceGuestDetailsToOperator(smach.State):
    """ State for the robot to give the operator info about mates

    Always succeeds.

    input_keys:
        guest_som_human_ids: TODO
        guest_som_obj_ids: TODO

    output_keys:

    """

    def __init__(self):
        smach.State.__init__(self, outcomes=[SUCCESS],
                                input_keys=['guest_som_human_ids', 'guest_som_obj_ids'])

        self.couch_left = Point();
        self.couch_left.x = 2.8248822689056396;
        self.couch_left.y = -2.577892541885376;
    
        self.couch_right = Point();
        self.couch_right.x = 1.879967212677002;
        self.couch_right.y = -2.7639691829681396;

        self.left_of_couch = Point();
        self.left_of_couch.x = 3.812685966491699;
        self.left_of_couch.y = -1.1837384700775146;

        self.right_of_couch = Point();
        self.right_of_couch.x = 0.9689993858337402;
        self.right_of_couch.y = -1.6282684803009033;

    def get_room_loc(self, person_loc:Point) -> str:
        dist = distance_between_points(person_loc, self.couch_left);
        output = " was seated on the left of the couch."
        trial_dist = distance_between_points(person_loc, self.couch_right);
        if trial_dist < dist:
            dist = trial_dist;
            output = " was seated on the right of the couch."
        trial_dist = distance_between_points(person_loc, self.left_of_couch);
        if trial_dist < dist:
            dist = trial_dist;
            output = " was seated to the left of the couch."
        trial_dist = distance_between_points(person_loc, self.right_of_couch);
        if trial_dist < dist:
            dist = trial_dist;
            output = " was seated to the right of the couch."
        return output;

    def execute(self, userdata):
        rospy.wait_for_service('som/humans/basic_query')
        som_humans_query_service_client = rospy.ServiceProxy('som/humans/basic_query', SOMQueryHumans);
        
        query = SOMQueryHumansRequest();
        query.query.task_role = "guest";
        responses:SOMQueryHumansResponse = som_humans_query_service_client(query);

        number_of_guests_found = len(responses.returns);

        if number_of_guests_found == 0:
            talk_phrase = "I could not find any of your mates. Don't worry, I'm sure they will arrive soon!"
            call_talk_request_action_server(phrase=talk_phrase)
            return SUCCESS

        if number_of_guests_found > 0:
            guest_names = [];
            for human_record in responses.returns:
                human_record:Human;
                if human_record.name:
                    guest_names.append(human_record.name);

            if len(guest_names) == 0:
                name_pl_marker = "names" if len(number_of_guests_found) > 1 else "name";
                talk_phrase = "I found {} of your mates but couldn't hear any of their {}!".format(number_of_guests_found, name_pl_marker)
            else:
                talk_phrase = "I found {} of your mates and could hear {} of their names!".format(number_of_guests_found, len(guest_names));
            call_talk_request_action_server(phrase=talk_phrase);

            guest_prefixes = ["One of the guests", "Another of the guests"];
            guest_num = 0;
            for human_record in responses.returns:
                human_record:Human;

                talk_phrase = "";

                if human_record.name:
                    talk_phrase += human_record.name;
                else:
                    talk_phrase += guest_prefixes[0] if guest_num == 0 else guest_prefixes[1];

                talk_phrase += self.get_room_loc(human_record.obj_position.position);

                if human_record.face_attributes:
                    all_are_attributes = ['Bald', 'Wearing_Necklace', 'Wearing_Necktie']
                    all_have_attributes = ['Bangs', 'Black_Hair', 'Blond_Hair', 'Brown_Hair', 'Eyeglasses', 'Gray_Hair', 'Sideburns', 'Straight_Hair', 'Wavy_Hair']

                    are_attributes = []
                    have_attributes = []

                    for attribute in human_record.face_attributes:
                        if(attribute in all_are_attributes):
                            are_attributes.append(attribute)
                        elif(attribute in all_have_attributes):
                            have_attributes.append(attribute)

                    if(len(are_attributes)>1):
                        # Making sure it can pronounce things like Wearing_Necklace
                        list1 = []
                        for attribute in are_attributes[:-1]:
                            list1.append(attribute_to_sentence(attribute))
                        talk_phrase += " They are {} and {}.".format(list1, attribute_to_sentence(are_attributes[-1]))
                    elif(len(are_attributes)==1):
                        talk_phrase += " They are {}.".format(attribute_to_sentence(are_attributes))

                    if(len(have_attributes)>1):
                        list2 = []
                        for attribute in have_attributes[:-1]:
                            list2.append(attribute_to_sentence(attribute))
                        talk_phrase += " They have {} and {}.".format(list2, attribute_to_sentence(have_attributes[-1]))
                    elif(len(have_attributes)==1):
                        talk_phrase += " They have {}.".format(attribute_to_sentence(have_attributes))
                
                call_talk_request_action_server(phrase=talk_phrase);
                guest_num += 1;

            """
            guest_num = 0;
            for human_record in responses.returns:
                human_record:Human;

                # build the string to tell the operator about the mate
                person_talk_phrase = ""
                if human_record.name:
                    person_talk_phrase += "The {} person I met was {}.".format(positional_to_cardinal(guest_num+1), human_record.name)
                else:
                    if guest_num == 0:
                        person_talk_phrase += "I met the {} person.".format(positional_to_cardinal(guest_num+1))
                    else:
                        person_talk_phrase += "I met a {} person.".format(positional_to_cardinal(guest_num+1))

                # TODO - build in information about person location (maybe query the SOM to find out?)

                if not human_record.gender:
                    pass
                else:
                    if human_record.gender == "prefer not to say":
                        pass
                    else:
                        person_talk_phrase += " They identify as {}.".format(human_record.gender)

                if not human_record.pronouns:
                    pass
                else:
                    if human_record.pronouns == "prefer not to say":
                        pass
                    else:
                        person_talk_phrase += " Their pronouns are '{}'.".format(human_record.pronouns)

                if human_record.face_attributes:
                    all_are_attributes = ['Bald', 'Wearing_Necklace', 'Wearing_Necktie']
                    all_have_attributes = ['Bangs', 'Black_Hair', 'Blond_Hair', 'Brown_Hair', 'Eyeglasses', 'Gray_Hair', 'Sideburns', 'Straight_Hair', 'Wavy_Hair']

                    are_attributes = []
                    have_attributes = []

                    for attribute in human_record.face_attributes:
                        if(attribute in all_are_attributes):
                            are_attributes.append(attribute)
                        elif(attribute in all_have_attributes):
                            have_attributes.append(attribute)

                    if(len(are_attributes)>1):
                        #" Making sure it can pronounce things like Wearing_Necklace""
                        list1 = []
                        "" Making sure it can pronounce things like Wearing_Necklace""
                        string1 = ""
                        for attribute in are_attributes[:-1]:
                            list1 = attribute_to_sentence(attribute)
                            string1 = string1 +" " + " ".join(e for e in list1)
                        string2 = " ".join(e for e in attribute_to_sentence(are_attributes[-1]))
                        person_talk_phrase += " They are {} and {}.".format(string1, string2)
                    elif(len(are_attributes)==1):
                        person_talk_phrase += " They are {}.".format(" ".join(e for e in attribute_to_sentence(are_attributes)))

                    if(len(have_attributes)>1):
                        string3 = ""
                        for attribute in have_attributes[:-1]:
                            list3 = attribute_to_sentence(attribute)
                            string3 = string3 +" " + " ".join(e for e in list3)
                        string4 = " ".join(e for e in attribute_to_sentence(have_attributes[-1]))
                        person_talk_phrase += " They have {} and {}.".format(string3, string4)
                    elif(len(have_attributes)==1):
                        person_talk_phrase += " They have {}.".format(" ".join(e for e in attribute_to_sentence(have_attributes)))
                    #person_talk_phrase += " They have the following facial attributes: {}.".format(human_record.face_attributes)

                # speak the details for this person
                call_talk_request_action_server(phrase=person_talk_phrase)

                guest_num += 1;
            """

            # wrap up
            talk_phrase = "That's everyone I met!"
            call_talk_request_action_server(phrase=talk_phrase)

        # relevant_matches = userdata.relevant_matches;
        # if relevant_matches != None:
        #     talk_phrase = "";
        #     for guest in relevant_matches:
        #         guest:list;
        #         if len(guest) != 0:
        #             guest_sorted = sorted(guest, key=lambda x:x["distance_from_obj"]);
        #             speak_relation:dict = guest_sorted[0];
        #             talk_phrase += speak_relation['human_name'] + speak_relation['relational_str'] + ".";
        #         pass
        #     call_talk_request_action_server(phrase=talk_phrase)

        return SUCCESS


def testForceSensorState():
    """
    Goal is in collision within the hsrb_megaweb2015world map.
    """
    sub_sm = smach.StateMachine(outcomes=[SUCCESS]);

    with sub_sm:
        smach.StateMachine.add(
            "WaitForForceSensor",
            WaitForWristWrench(),
            transitions={
                SUCCESS:SUCCESS});
        pass;
    
    sub_sm.execute();
    print("Force given through force sensor.")
    pass;

if __name__ == '__main__':
    rospy.init_node('misc_states_test');
    testForceSensorState();