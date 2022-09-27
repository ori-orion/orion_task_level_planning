

from state_machines.Reusable_States.manipulation_states import *;
from state_machines.Reusable_States.misc_states import *;
from state_machines.Reusable_States.navigational_states import *;
from state_machines.Reusable_States.perception_states import *;
from state_machines.Reusable_States.som_states import *;
from state_machines.Reusable_States.speech_states import *;
import state_machines.Reusable_States.utils as utils

from state_machines.Reusable_States.outcomes_enum import *;


# People
NAMES = ['Gemma', 'Acacia', 'Ollie', 'Nick', 'Hollie',
          'Charlie', 'Matt', 'Daniele', 'Chris', 'Paul', 'Lars', 'John',
          'Michael', 'Matthew', 'Clarissa', 'Ricardo', 'Mia', 'Shu', 'Owen',
          'Jianeng', 'Kim', 'Liam', 'Kelvin', 'Benoit', 'Mark']

GENDERS = ['female', 'male', 'gender fluid', 'poly-gender', 'pangender', 'agender', 'non-binary', 'prefer not to say']
PRONOUNS = ['she her', 'he him', 'they them', 'prefer not to say']

# Commands
READY = ['ready']#['I am ready', 'ready', "let's go", "I'm ready"]

# Descriptors
COLOURS = ["Red", "Orange", "Yellow", "Green", "Blue", "Purple",
           "Black", "White", "Grey", "Brown", "Beige"]
RELATIONS = ['left', 'right', 'above', 'below', 'front', 'behind', 'near']
AR_MARKERS = {'bottle': 151}


# Objects & Things
#  Be careful of space/underscore representations of 'cleaning stuff'
OBJECT_CATEGORIES = ['cleaning stuff', 'containers', 'cutlery', 'drinks', 'food', 'fruits', 'snacks', 'tableware']
FRUITS = ['apple', 'banana', 'orange', 'mango', 'strawberry', 'kiwi', 'plum',
          'nectarine'] # TODO: Fill in with the YCB benchmark
DRINKS = ['Coke', 'Beer', 'Water', 'Orange Juice', 'Champagne', 'Absinthe']
OBJECTS = ['potted plant', 'bottle', 'cup', 'cereal', 'bowl', 'cloth'] # TODO: YCB benchmark
OBJECTS += FRUITS + DRINKS




def setupErrorStates(state_machine, failure_mapping=TASK_FAILURE):
    with state_machine:
        #region Failure states.

        # announce nav repeat failure
        smach.StateMachine.add('ANNOUNCE_REPEAT_NAV_FAILURE',
                                SpeakState(phrase="Navigation failed too many times, terminating task."),
                                transitions={
                                    SUCCESS:failure_mapping},
                                remapping={})

        # announce speech recognition repeat failure
        smach.StateMachine.add('ANNOUNCE_REPEAT_SPEECH_RECOGNITION_FAILURE',
                                SpeakState(phrase="Speech recognition failed too many times, terminating task."),
                                transitions={
                                    SUCCESS:failure_mapping},
                                remapping={});

        #endregion
    return state_machine;