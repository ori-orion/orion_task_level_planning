from state_machines.Reusable_States.utils import *;

import smach;

from orion_actions.msg import *;
from orion_actions.srv import *;

import rospy;

import math;
import random;
import copy;

from geometry_msgs.msg import Pose, PoseStamped;

import actionlib;

from tmc_msgs.msg import TalkRequestAction, TalkRequestGoal, Voice

NAMES = ['Gemma', 'Acacia', 'Ollie', 'Nick', 'Hollie',
          'Charlie', 'Matt', 'Daniele', 'Chris', 'Paul', 'Lars', 'John',
          'Michael', 'Matthew', 'Clarissa', 'Ricardo', 'Mia', 'Shu', 'Owen',
          'Jianeng', 'Kim', 'Liam', 'Kelvin', 'Benoit', 'Mark']

COLOURS = ["Red", "Orange", "Yellow", "Green", "Blue", "Purple",
           "Black", "White", "Grey", "Brown", "Beige"]

FRUITS = ['apple', 'banana', 'orange', 'mango', 'strawberry', 'kiwi', 'plum',
          'nectarine'] # TODO: Fill in with the YCB benchmark

DRINKS = ['Coke', 'Beer', 'Water', 'Orange Juice', 'Champagne', 'Absinthe']


class SpeakState(smach.State):
    """ Smach state for the robot to speak a phrase.

    This class has the robot say something and return success.

    input_keys:
        phrase: What we want the robot to say
    
    Note that if self.phrase==None, then we take the input directly from input_keys['phrase'].
    """
    def __init__(self, phrase=None):
        smach.State.__init__(self,
                                outcomes=[SUCCESS],
                                input_keys=['phrase'])

        self.phrase = phrase;

    def execute(self, userdata):
        if (self.phrase == None):
            phrase_speaking = userdata.phrase;
        else:
            phrase_speaking = self.phrase;

        action_goal = TalkRequestGoal()
        action_goal.data.language = Voice.kEnglish  # enum for value: 1
        action_goal.data.sentence = phrase_speaking

        rospy.loginfo("HSR speaking phrase: '{}'".format(phrase_speaking))
        speak_action_client = actionlib.SimpleActionClient('/talk_request_action',
                                        TalkRequestAction)

        speak_action_client.wait_for_server()
        speak_action_client.send_goal(action_goal)
        speak_action_client.wait_for_result()

        # rospy.loginfo("Speaking complete")

        # Can only succeed
        return SUCCESS


class SpeakAndListenState(smach.State):
    """ Smach state for speaking and then listening for a response.

    This state will calll the speak and listen action server,
    to get the robot to say something, wait for a response,
    parse the response, and return it as an output key.

    input_keys:
        question: the question to ask
        candidates: candidate sentences
        params: optional parameters for candidate sentences
        timeout: the timeout for listening
        number_of_failures: an external counter keeping track of the cumulative failure count (incremented in this state upon failure & reset upon success and repreat failure)
        failure_threshold: the number of cumulative failures required to return the repeat_failure outcome
    output_keys:
        operator_response: the recognised response text
        number_of_failures: the updated failure counter upon state exit
    """

    def __init__(self, question=None):
        smach.State.__init__(self,
                                outcomes=[SUCCESS,FAILURE,REPEAT_FAILURE],
                                input_keys=['question', 'candidates','params','timeout','number_of_failures','failure_threshold'],
                                output_keys=['operator_response', 'number_of_failures'])

        self.question = question;

    def execute(self, userdata):
        if self.question==None:
            question_speaking = userdata.question;
        else:
            question_speaking = self.question;

        speak_listen_goal = SpeakAndListenGoal()
        speak_listen_goal.question = question_speaking
        speak_listen_goal.candidates = userdata.candidates
        speak_listen_goal.params = userdata.params
        speak_listen_goal.timeout = userdata.timeout

        speak_listen_action_client = actionlib.SimpleActionClient('speak_and_listen', SpeakAndListenAction)
        speak_listen_action_client.wait_for_server()
        # rospy.loginfo("Pre sending goal");
        speak_listen_action_client.send_goal(speak_listen_goal)

        rospy.loginfo("HSR asking phrase: '{}'".format(question_speaking));

        speak_listen_action_client.wait_for_result()
        # rospy.loginfo("Post wait for result");

        result = speak_listen_action_client.get_result()

        if result is not None and result.succeeded:
            userdata.operator_response = result.answer
            userdata.number_of_failures = 0
            return SUCCESS
        else:
            userdata.number_of_failures+= 1
            if userdata.number_of_failures >= userdata.failure_threshold:
                # reset number of failures because we've already triggered the repeat failure
                userdata.number_of_failures = 0
                return REPEAT_FAILURE
            return FAILURE

class AskPersonNameState(smach.State):
    """ Smach state for the robot to ask for the person's name, executed by the ask_person_name action server.

    This state will call the ask_person_name action server,
    wait for a response, parse the response, and return it as an output key.

    input_keys:
        question: the question to ask
        timeout: the timeout for listening
        number_of_failures: an external counter keeping track of the cumulative failure count (incremented in this state upon failure & reset upon success and repreat failure)
        failure_threshold: the number of cumulative failures required to return the repeat_failure outcome
    output_keys:
        recognised_name: the recognised name response
        number_of_failures: the updated failure counter upon state exit
    """

    def __init__(self, timeout=None, question=None):
        smach.State.__init__(self,
                                outcomes=[SUCCESS,FAILURE,REPEAT_FAILURE],
                                input_keys=['question','timeout','number_of_failures','failure_threshold'],
                                output_keys=['recognised_name', 'number_of_failures'])
        
        self.timeout = timeout;
        self.question = question;

    def execute(self, userdata):
        if (self.timeout == None):
            timeout = userdata.timeout;
        else:
            timeout = self.timeout;
        if self.question == None:
            question = userdata.question;
        else:
            question = self.question;

        ask_name_goal = AskPersonNameGoal()
        rospy.loginfo(f"Asking question {question} with timeout {timeout}")
        ask_name_goal.question = question
        ask_name_goal.timeout = timeout

        ask_name_action_client = actionlib.SimpleActionClient('ask_person_name', AskPersonNameAction)
        ask_name_action_client.wait_for_server()
        # rospy.loginfo("Pre sending goal");
        ask_name_action_client.send_goal(ask_name_goal)
        # rospy.loginfo("Pre wait for result");
        ask_name_action_client.wait_for_result()
        # rospy.loginfo("Post wait for result");

        result = ask_name_action_client.get_result()

        if result is not None and result.answer:
            userdata.recognised_name = result.answer
            userdata.number_of_failures = 0
            return SUCCESS
        else:
            # action server failed
            userdata.number_of_failures += 1
            userdata.recognised_name = "";
            if userdata.number_of_failures >= userdata.failure_threshold:
                # reset number of failures because we've already triggered the repeat failure
                userdata.number_of_failures = 0
                return REPEAT_FAILURE
            return FAILURE

class WaitForHotwordState(smach.State):
    """ Smach state for waiting for the hotword detector to publish a detection message.

    Terminates with SUCCESS outcome if hotword detection message is received within the timeout (if used),
    otherwise FAILURE.

    input_keys:
        timeout: timeout time in seconds (set to None to wait indefinitely)
    """

    def __init__(self):
        smach.State.__init__(self,
                                outcomes = [SUCCESS, FAILURE],
                                input_keys=['timeout'])

    def execute(self, userdata):
        # call_talk_request_action_server(phrase="I'm ready and waiting for the hotword")
        rospy.loginfo("Waiting for hotword...")
        try:
            # Wait for one message on topic
            hotword_msg = rospy.wait_for_message('/hotword', Hotword, timeout=userdata.timeout)
            rospy.loginfo("Hotword '{}' received at time: {}".format(hotword_msg.hotword, hotword_msg.stamp.to_sec()))
            # call_talk_request_action_server(phrase="Hotword received")
            return SUCCESS
        except rospy.ROSException as e:
            rospy.logwarn("Hotword not received within timeout")
            return FAILURE


class AskFromSelection(smach.State):
    """
    A state for asking a selection of questions.
    Outcomes: [SUCCESS, "no_response"]
    Inputs:
        responses_arr       - The array of dictionary responses that we are appending to.
        output_speech_arr   - The array of output speeches that we are appending to.
    Outputs:
        responses           - The individual response.
        output_speech       - The individual speech output.
        responses_arr       - The array of dictionary responses that we are appending to.
        output_speech_arr   - The array of output speeches that we are appending to.

    Note that the ..._arr variables are only accessed if self.append_result_to_array==True
    """

    NO_RESPONSE_RESPONSES = [
        "Sorry, I didn't quite catch that.",
        "Please could you repeat that."
    ];

    # Note the overall architecture here:
    # Each entry is of type ([tag], [question], [candidates?])
    # We will go in sequence over the larger list and then 
    # with the inner lists, that represents a choice over a set of options.
    DEFAULT_QUESTIONS = [
        ("name", "Hello, What's your name?", NAMES),
        [
            ("age", "How old are you?"),
            ("place", "Where are you from?")
            ("drink", "What's your favourite drink?", DRINKS)
            ("colour", "What's your favourite colour?", COLOURS)
        ]
    ];

    END_PHRASES = [
        "Nice to meet you."
    ];

    def __init__(self, questions = None, append_result_to_array=True):
        smach.State.__init__(self,
            outcomes=[SUCCESS, "no_response"],
            input_keys=["responses_arr", "output_speech_arr"],
            output_keys=[
                "responses", "output_speech", 
                "responses_arr", "output_speech_arr"]);

        self.use_default_questions = (questions == None);
        if self.use_default_questions:
            self.questions = AskFromSelection.DEFAULT_QUESTIONS;
        else:
            self.questions = questions;
        self.append_result_to_array = append_result_to_array;

    def ask_question(self, asking:tuple) -> str:  # -> str, bool
        speak_listen_goal = SpeakAndListenGoal()
        speak_listen_goal.question = asking[1];
        if len(asking) == 3:
            speak_listen_goal.candidates = asking[2];
        else:
            speak_listen_goal.candidates = [];
        speak_listen_goal.params = [];
        speak_listen_goal.timeout = 5;

        # rospy.loginfo("Pre sending goal");
        self.speak_listen_action_client.send_goal(speak_listen_goal)

        rospy.loginfo("HSR asking phrase: '{}'".format(speak_listen_goal.question));

        self.speak_listen_action_client.wait_for_result()
        rospy.loginfo("Post wait for result");

        result = self.speak_listen_action_client.get_result();

        information = result.answer;
        succeeded = result.succeeded;
        transcription = result.transcription;
        if len(transcription) == 0:
            rospy.loginfo("Nothing was said");
            speak_listen_goal.question = AskFromSelection.NO_RESPONSE_RESPONSES[random.randrange(len(AskFromSelection.NO_RESPONSE_RESPONSES))];
        elif succeeded == False:
            speak_listen_goal.question = "Yes, but " + speak_listen_goal.question;

        if succeeded == False or len(transcription) == 0:
            self.speak_listen_action_client.send_goal(speak_listen_goal)

            rospy.loginfo("HSR asking phrase: '{}'".format(speak_listen_goal.question));

            self.speak_listen_action_client.wait_for_result()
            rospy.loginfo("Post wait for result");

            result = self.speak_listen_action_client.get_result();
            information = result.answer;
            succeeded = result.succeeded;
            transcription = result.transcription;

            if len(transcription) == 0 or succeeded == False:
                rospy.loginfo("No logical response gained. Moving on.");
                return "", False;
        else:
            return information, True;

    def tag_to_speech(self, tag:str, information:str):
        output = "";
        if tag == "age":
            output += " age is " + information;
        elif tag == "place":
            output += " from " + information;
        elif tag == "drink":
            output += " favorite drink is " + information;
        elif tag == "colour":
            output += " favourite colour is " + information;
        output += ". ";

    def execute(self, userdata):
        
        self.speak_listen_action_client = actionlib.SimpleActionClient('speak_and_listen', SpeakAndListenAction)
        self.speak_listen_action_client.wait_for_server()

        output_dict = {};

        for question_set in self.questions:
            if type(question_set) is list:
                question_set:list;
                question = question_set[random.randrange(len(question_set))];
            else:
                question = question_set;

            information, succeeded = self.ask_question(question);

            if succeeded:
                output_dict[question[0]] = information;

        output_dict_copy = copy.deepcopy(output_dict);

        output_speech = "";
        if "name" in output_dict_copy:
            POSSIBLE_NAME_PREFIXES = ["we have ", "we have someone called "];
            output_speech += POSSIBLE_NAME_PREFIXES[random.randrange(len(POSSIBLE_NAME_PREFIXES))] + output_dict_copy["name"];
            del(output_dict_copy["name"]);
        else:
            output_speech += "we have someone who didn't give their name"
         
        
        if len(output_dict_copy.keys) != 0:
            entry_tag = output_dict_copy.keys[0];
            POSSIBLE_SECOND_PREFIXES = [". Their", " who's", " and their"]
            output_speech += POSSIBLE_SECOND_PREFIXES[random.randrange(len(POSSIBLE_SECOND_PREFIXES))];
            output_speech += self.tag_to_speech(entry_tag, output_dict_copy[entry_tag]);

        for entry_tag in output_dict_copy.keys():
            output_speech += " Their"
            output_speech += self.tag_to_speech(entry_tag, output_dict_copy[entry_tag]);
                
        userdata.output_speech = output_speech;
        userdata.responses = output_dict;
    
        if self.append_result_to_array:
            responses_arr:list = userdata.responses_arr;
            output_speech_arr:list = userdata.output_speech_arr;
            responses_arr.append(output_dict);
            output_speech_arr.append(output_speech);
            userdata.responses_arr = responses_arr;
            userdata.output_speech_arr = output_speech_arr;

        
        #region END PHRASE
        action_goal = TalkRequestGoal()
        action_goal.data.language = Voice.kEnglish  # enum for value: 1
        action_goal.data.sentence = AskFromSelection.END_PHRASES[random.randrange(len(AskFromSelection.END_PHRASES))];

        rospy.loginfo("HSR speaking phrase: '{}'".format(action_goal.data.sentence))
        speak_action_client = actionlib.SimpleActionClient('/talk_request_action', TalkRequestAction)

        speak_action_client.wait_for_server()
        speak_action_client.send_goal(action_goal)
        speak_action_client.wait_for_result()
        #endregion
                
        if len(output_dict.keys()) > 0:
            return SUCCESS;
        else:
            return "no_response";
    pass;


#region Create Phrase stuff.
# This seems to set `userdata.phrase` for subsequent speaking.
# Note that the `SpeakState` then speaks the phrase. Thus `SpeakState` should probably normally follow
# one of these.
class CreatePhraseAnnounceRetrievedItemToNamedOperatorState(smach.State):
    """ Smach state to create the phrase to announce the retreival of an item to a named operator

    This class always returns success.

    input_keys:
        operator_name: the name of the operator
        object_name: the name of the retrieved object
    output_keys:
        phrase: the returned phrase
    """
    def __init__(self):
        smach.State.__init__(self,
                                outcomes=[SUCCESS],
                                input_keys=['operator_name', 'object_name'],
                                output_keys=['phrase'])

    def execute(self, userdata):
        userdata.phrase = "Hi, " + userdata.operator_name + ", I've brought you the " + userdata.object_name

        # Can only succeed
        return SUCCESS

class CreatePhraseAskForHelpPickupObjectState(smach.State):
    """ Smach state to create the phrase to ask for help to pick up an object

    This class always returns success.

    input_keys:
        object_name: the name of the object to be asked to be picked up
    output_keys:
        phrase: the returned phrase
    """
    def __init__(self):
        smach.State.__init__(self,
                                outcomes=[SUCCESS],
                                input_keys=['object_name'],
                                output_keys=['phrase'])

    def execute(self, userdata):
        userdata.phrase = ("Can someone please help me pick up the " + userdata.object_name +
                                " and say ready when they are ready?")

        # Can only succeed
        return SUCCESS

class CreatePhraseStartSearchForPeopleState(smach.State):
    """ Smach state to create the phrase to announce the start of the search for people

    This class always returns success.

    input_keys:
        operator_name: the name of the operator
    output_keys:
        phrase: the returned phrase
    """
    def __init__(self):
        smach.State.__init__(self,
                                outcomes=[SUCCESS],
                                input_keys=['operator_name'],
                                output_keys=['phrase'])

    def execute(self, userdata):
        userdata.phrase = "Ok, " + userdata.operator_name + ", I am now going to search for your friends. I'll be back soon!"

        # Can only succeed
        return SUCCESS
#endregion