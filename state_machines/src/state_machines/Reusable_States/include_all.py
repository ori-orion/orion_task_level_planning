

from state_machines.Reusable_States.manipulation_states import *;
from state_machines.Reusable_States.misc_states import *;
from state_machines.Reusable_States.navigational_states import *;
from state_machines.Reusable_States.perception_states import *;
from state_machines.Reusable_States.som_states import *;
from state_machines.Reusable_States.speech_states import *;

NAMES = ['Gemma', 'Acacia', 'Ollie', 'Nick', 'Hollie',
          'Charlie', 'Matt', 'Daniele', 'Chris', 'Paul', 'Lars', 'John',
          'Michael', 'Matthew', 'Clarissa', 'Ricardo', 'Mia', 'Shu', 'Owen',
          'Jianeng', 'Kim', 'Liam', 'Kelvin', 'Benoit', 'Mark']

OBJECT_CATEGORIES = ['cleaning stuff', 'containers', 'cutlery', 'drinks', 'food', 'fruits', 'snacks', 'tableware']
FRUITS = ['apple', 'banana', 'orange', 'mango', 'strawberry', 'kiwi', 'plum',
          'nectarine'] # TODO: Fill in with the YCB benchmark
DRINKS = ['Coke', 'Beer', 'Water', 'Orange Juice', 'Champagne', 'Absinthe']
OBJECTS = ['potted plant', 'bottle', 'cup', 'cereal', 'bowl', 'cloth'] # TODO: YCB benchmark
OBJECTS += FRUITS + DRINKS

AR_MARKERS = {'bottle': 151}

# commands
READY = ['ready']#['I am ready', 'ready', "let's go", "I'm ready"]