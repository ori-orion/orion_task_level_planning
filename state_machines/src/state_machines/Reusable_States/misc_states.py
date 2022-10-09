from state_machines.Reusable_States.utils import *;

import smach;

from orion_actions.msg import *;
from orion_actions.srv import *;
from orion_spin.msg import SpinAction, SpinGoal;
from orion_door_pass.msg import DoorCheckGoal, DoorCheckAction

import rospy;

import math;

from geometry_msgs.msg import Pose, PoseStamped;

import actionlib

import hsrb_interface;
hsrb_interface.robot.enable_interactive();

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
    def __init__(self, z_looking_at = 1.3):
        smach.State.__init__(
            self, 
            outcomes=[SUCCESS],
            input_keys=['pose']);

        self.robot = hsrb_interface.Robot();
        self.whole_body = self.robot.try_get('whole_body');7

        self.z_looking_at = z_looking_at;
    
    def execute(self, userdata):
        pose:Pose = userdata.pose;
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
        return SUCCESS;

#endregion

class SpinState(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                                outcomes = [SUCCESS, FAILURE],
                                input_keys=[], output_keys=[]);

    def execute(self, userdata):
        client = actionlib.SimpleActionClient('spin', SpinAction);
        client.wait_for_server();
        client.send_goal(SpinGoal());
        client.wait_for_result();

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
