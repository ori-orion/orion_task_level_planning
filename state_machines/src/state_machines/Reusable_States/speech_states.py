#!/usr/bin/env python3

from state_machines.Reusable_States.utils import *;

import smach;

from orion_actions.msg import *;
from orion_actions.srv import *;

import rospy;

import math;
import random;
import copy;
import time;

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

COUNTRIES = ["United Kingdom", "America"];

AGE_STRS = [""];

SPEAK_THROUGH_CONSOLE = False;


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

        if SPEAK_THROUGH_CONSOLE:
            print("SpeakState:", phrase_speaking);
            return SUCCESS;

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

        if SPEAK_THROUGH_CONSOLE:
            print("SpeakAndListenState:", question_speaking);
            userdata.operator_response = input(">>>");
            if len(userdata.operator_response) == 0:
                return FAILURE;
            return SUCCESS;


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


#region Ask From Selection framework
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
            ("age", "How old are you?", AGE_STRS),
            ("place", "What country are you from?", COUNTRIES),
            ("drink", "What's your favourite drink?", DRINKS),
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

    def run_action_client(self, speak_listen_goal) -> SpeakAndListenResult:
        """
        Actually runs the client and gets the result.
        This will "speak" through the console if SPEAK_THROUGH_CONSOLE is selected. 
            (Should be useful for debugging.)
        """

        if SPEAK_THROUGH_CONSOLE:
            print("AskFromSelection:", speak_listen_goal.question);
            output_speech = input(">>>");
            output = SpeakAndListenResult();
            output.transcription = output_speech;
            output.answer = output_speech;
            output.succeeded = True;
            return output;

        print("Pre sending goal");

        self.speak_listen_action_client.send_goal(speak_listen_goal)

        rospy.loginfo("HSR asking phrase: '{}'".format(speak_listen_goal.question));

        self.speak_listen_action_client.wait_for_result()
        rospy.loginfo("Post wait for result");

        return self.speak_listen_action_client.get_result();


    def ask_question(self, asking:tuple) -> str:  # -> str, bool
        speak_listen_goal = SpeakAndListenGoal()
        speak_listen_goal.question = asking[1];
        if len(asking) == 3:
            speak_listen_goal.candidates = asking[2];
        else:
            speak_listen_goal.candidates = [];
        speak_listen_goal.params = [];
        speak_listen_goal.timeout = 5;
        
        result:SpeakAndListenResult = self.run_action_client(speak_listen_goal);

        information = result.answer;
        succeeded = result.succeeded;
        transcription = result.transcription;

        if succeeded == False or len(transcription) == 0:

            if len(transcription) == 0:
                rospy.loginfo("Nothing was said");
                speak_listen_goal.question = random.choice(AskFromSelection.NO_RESPONSE_RESPONSES);
            elif succeeded == False:
                speak_listen_goal.question = "Yes, but " + speak_listen_goal.question;

            result = self.run_action_client(speak_listen_goal);

            information:str = result.answer;
            succeeded:bool = result.succeeded;
            transcription:str = result.transcription;

            if len(transcription) == 0 or succeeded == False:
                rospy.loginfo(
                    "No logical response gained. transcription=" + transcription + ", succeeded=" + str(succeeded) + ". Moving on.");
                return "", False;
        
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
        return output;

    def execute(self, userdata):
        
        if SPEAK_THROUGH_CONSOLE == False:
            self.speak_listen_action_client = actionlib.SimpleActionClient('speak_and_listen', SpeakAndListenAction)
            print("Waiting for speak and listen server");
            self.speak_listen_action_client.wait_for_server()
            print("Speak and listen server found");

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

        #region Constructing the bit where we put together the output speech.
        # For this whole section, we will be using a copy of the dictionary.
        # When an item is used, it will be removed. This way we can keep track
        # of the tags that have been used up.
        output_speech = "";
        # Let's first address the person's name.
        if "name" in output_dict_copy:
            POSSIBLE_NAME_PREFIXES = ["we have ", "we have someone called "];
            output_speech += POSSIBLE_NAME_PREFIXES[random.randrange(len(POSSIBLE_NAME_PREFIXES))] + output_dict_copy["name"];
            del(output_dict_copy["name"]);
        else:
            output_speech += "we have someone who didn't give their name"
         
        # Then, if there's something else.
        if len(output_dict_copy.keys()) != 0:
            entry_tag = list(output_dict_copy.keys())[0];
            POSSIBLE_SECOND_PREFIXES = [". They're", " who's", " and their"]
            output_speech += POSSIBLE_SECOND_PREFIXES[random.randrange(len(POSSIBLE_SECOND_PREFIXES))];
            output_speech += self.tag_to_speech(entry_tag, output_dict_copy[entry_tag]);
            del(output_dict_copy[entry_tag]);
        else:
            output_speech += ". "

        # Then if there's anything after that.
        for entry_tag in output_dict_copy.keys():
            output_speech += " Their"
            output_speech += self.tag_to_speech(entry_tag, output_dict_copy[entry_tag]);
        #endregion
                
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
        end_phrase = random.choice(AskFromSelection.END_PHRASES);

        if SPEAK_THROUGH_CONSOLE:
            print("AskFromSelection:", end_phrase);
        else:
            action_goal = TalkRequestGoal()
            action_goal.data.language = Voice.kEnglish  # enum for value: 1
            action_goal.data.sentence = end_phrase;

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


class AskFromSelectionHardCoded(smach.State):
    def __init__(self, append_result_to_array):
        smach.State.__init__(self,
            outcomes=[SUCCESS, "no_response"],
            input_keys=["responses_arr", "output_speech_arr", "index"],
            output_keys=[
                "responses", "output_speech", 
                "responses_arr", "output_speech_arr"]);

        self.questions = ["Hello, what's your name?", 
            [
                "Hello Sam, what's your favorite quisine?",
                "Nice to meet you John. What's your favorite hobby?",
                "What's your favorite colour?"
            ]]

        self.answers_0 = ["Sam", "John", "Anna"];
        self.answers_1 = ["chinese", "sleeping and video gaming", "black"];

        self.speaking_phrases = [
            "we have someone called Sam. They're favorite quisine is chinese. ",
            "we have John who's hobbies are sleeping and video gaming. ",
            "we have Anna who's favorite colour is black. "
        ]

    def speak(self, speaking):
        if SPEAK_THROUGH_CONSOLE:
            print("AskFromSelectionHardCoded: ", speaking);
        else:
            action_goal = TalkRequestGoal()
            action_goal.data.language = Voice.kEnglish  # enum for value: 1
            action_goal.data.sentence = speaking;
            
            rospy.loginfo("HSR speaking phrase: '{}'".format(action_goal.data.sentence));

            self.speak_action_client.send_goal(action_goal)
            self.speak_action_client.wait_for_result()

    def execute(self, userdata):
        if SPEAK_THROUGH_CONSOLE == False:
            self.speak_action_client = actionlib.SimpleActionClient('/talk_request_action', TalkRequestAction)
            self.speak_action_client.wait_for_server()
        
        index:int = userdata.index;

        for element in self.questions:
            if type(element) is list:
                self.speak(element[index]);
            else:
                self.speak(element);
            
            time.sleep(3);
        
        time.sleep(3);
        self.speak("Nice to meet you.");

        userdata.output_speech = self.speaking_phrases[index];
        output_speech_arr:list = userdata.output_speech_arr;
        output_speech_arr.append(self.speaking_phrases[index]);

        return SUCCESS;


class ReportBackToOperator(smach.State):
    """
    So the AskFromSelection state returns a set of things to say. We then need to say them.
    We will assume the guests are ordered from left to right. 
    """

    def __init__(self):
        smach.State.__init__(self,
            outcomes=[SUCCESS],
            input_keys=["responses_arr", "output_speech_arr"],
            output_keys=[]);

    def execute(self, userdata):
        PREFIXES = ["First ", "Then, to their left, ", "To their left, "];
        output_speech_arr:list = userdata.output_speech_arr;

        prefix_index = 0;

        phrase_speaking = "";

        for human_speech in output_speech_arr:
            human_speech:str;
            if len(human_speech) == 0:
                continue;

            phrase_speaking += PREFIXES[prefix_index] + human_speech;

            prefix_index = (prefix_index+1 if prefix_index < len(PREFIXES)-1 else prefix_index);
        
        if SPEAK_THROUGH_CONSOLE:
            print();
            print("ReportBackToOperator:", phrase_speaking);
            print();
        else:
            action_goal = TalkRequestGoal()
            action_goal.data.language = Voice.kEnglish  # enum for value: 1
            action_goal.data.sentence = phrase_speaking

            rospy.loginfo("HSR speaking phrase: '{}'".format(phrase_speaking))
            speak_action_client = actionlib.SimpleActionClient('/talk_request_action',
                                            TalkRequestAction)

            speak_action_client.wait_for_server()
            speak_action_client.send_goal(action_goal)
            speak_action_client.wait_for_result()

        return SUCCESS;
#endregion




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


def askFromSelectionTest():
    from procedural_states import IncrementValue;

    sm = smach.StateMachine(
        outcomes=[SUCCESS, FAILURE],
        input_keys=[],
        output_keys=[]);

    sm.userdata.responses_arr = [];
    sm.userdata.output_speech_arr = [];
    sm.userdata.index = 0;

    with sm:
        smach.StateMachine.add(
            'TalkToGuest1',
            # AskFromSelection(append_result_to_array=True),
            AskFromSelectionHardCoded(append_result_to_array=True),
            transitions={
                SUCCESS:'IncrementGuestIndex1',
                "no_response":'IncrementGuestIndex1'},
            remapping={
                "responses_arr" : "responses_arr",
                "output_speech_arr" : "output_speech_arr"
            });

        smach.StateMachine.add(
            'IncrementGuestIndex1',
            IncrementValue(increment_by=1),
            transitions={SUCCESS:'TalkToGuest2'},
            remapping={'val':'index'});

        smach.StateMachine.add(
            'TalkToGuest2',
            # AskFromSelection(append_result_to_array=True),
            AskFromSelectionHardCoded(append_result_to_array=True),
            transitions={
                SUCCESS:'IncrementGuestIndex2',
                "no_response":'IncrementGuestIndex2'},
            remapping={
                "responses_arr" : "responses_arr",
                "output_speech_arr" : "output_speech_arr"
            });

        smach.StateMachine.add(
            'IncrementGuestIndex2',
            IncrementValue(increment_by=1),
            transitions={SUCCESS:'TalkToGuest3'},
            remapping={'val':'index'});

        smach.StateMachine.add(
            'TalkToGuest3',
            # AskFromSelection(append_result_to_array=True),
            AskFromSelectionHardCoded(append_result_to_array=True),
            transitions={
                SUCCESS:'IncrementGuestIndex3',
                "no_response":'IncrementGuestIndex3'},
            remapping={
                "responses_arr" : "responses_arr",
                "output_speech_arr" : "output_speech_arr"
            });

        smach.StateMachine.add(
            'IncrementGuestIndex3',
            IncrementValue(increment_by=1),
            transitions={SUCCESS:'ReportBack'},
            remapping={'val':'index'});

        smach.StateMachine.add(
            'ReportBack',
            ReportBackToOperator(),
            transitions={SUCCESS:SUCCESS});
    
    sm.execute();

    print(sm.userdata.responses_arr);
    print(sm.userdata.output_speech_arr);


if __name__ == '__main__':

    # state = OrderGuestsFound();
    # state.testState();

    rospy.init_node('test_speech');

    askFromSelectionTest();

    # rospy.spin();