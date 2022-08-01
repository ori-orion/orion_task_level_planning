from utils import *;

import smach;

from orion_actions.msg import *;
from orion_actions.srv import *;

import rospy;

import math;

from geometry_msgs.msg import Pose, PoseStamped;

import actionlib;

from tmc_msgs.msg import TalkRequestAction, TalkRequestGoal, Voice

class SpeakState(smach.State):
    """ Smach state for the robot to speak a phrase.

    This class has the robot say something and return success.

    input_keys:
        phrase: What we want the robot to say
    """
    def __init__(self):
        smach.State.__init__(self,
                                outcomes=['success'],
                                input_keys=['phrase'])

    def execute(self, userdata):
        action_goal = TalkRequestGoal()
        action_goal.data.language = Voice.kEnglish  # enum for value: 1
        action_goal.data.sentence = userdata.phrase

        rospy.loginfo("HSR speaking phrase: '{}'".format(userdata.phrase))
        speak_action_client = actionlib.SimpleActionClient('/talk_request_action',
                                        TalkRequestAction)

        speak_action_client.wait_for_server()
        speak_action_client.send_goal(action_goal)
        speak_action_client.wait_for_result()

        # rospy.loginfo("Speaking complete")

        # Can only succeed
        return 'success'


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
                                outcomes=['success'],
                                input_keys=['operator_name', 'object_name'],
                                output_keys=['phrase'])

    def execute(self, userdata):
        userdata.phrase = "Hi, " + userdata.operator_name + ", I've brought you the " + userdata.object_name

        # Can only succeed
        return 'success'

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
                                outcomes=['success'],
                                input_keys=['object_name'],
                                output_keys=['phrase'])

    def execute(self, userdata):
        userdata.phrase = ("Can someone please help me pick up the " + userdata.object_name +
                                " and say ready when they are ready?")

        # Can only succeed
        return 'success'

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
                                outcomes=['success'],
                                input_keys=['operator_name'],
                                output_keys=['phrase'])

    def execute(self, userdata):
        userdata.phrase = "Ok, " + userdata.operator_name + ", I am now going to search for your friends. I'll be back soon!"

        # Can only succeed
        return 'success'
#endregion