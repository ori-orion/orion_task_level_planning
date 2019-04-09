#!/usr/bin/env python
""" File for generating grako grammar from GPSRCmdGen grammar.

This file contains code to take the grammar definition given by the
GPSRCmdGen repository and re-format it into a grammar suitable for grako
so we can autogenerate state machines for the GPSR task.

Author: Charlie Street

"""

from xml.dom import minidom


# This is an annoying thing I have to do because the repo
# Doesn't understand consistency apparently...
PRONOUNS = ['I', 'you', 'he', 'she', 'it', 'we', 'you', 'they', 
            'me', 'you', 'him', 'her', 'it', 'us', 'you', 'them',
            'mine', 'yours', 'his', 'hers', 'its', 'ours', 'yours', 'theirs',
            'my', 'your', 'his', 'her', 'its', 'our', 'your', 'their']

COMMON = '../GPSRCmdGen/CommonFiles/'

GESTURES = COMMON + 'Gestures.xml'
LOCATIONS = COMMON + 'Locations.xml'
NAMES = COMMON + 'Names.xml'
OBJECTS = COMMON + 'Objects.xml'

def remove_u(str):
    """ Removes weird formatting from xml minidom.

    xml mindom returns all values as u'val' it seems, so this function
    removes that if applicable. Use on any string returned from xml minidom.

    Args:
        str: The string to be reformatted
    
    Returns:
        reformatted: The reformatted string

    """
    if str[0:2] == "u'" and str[-1] == "'":
        return str[2:-1]
    else:
        return str


def form_pronoun_rule(pronoun_list):
    """ Generates grammar rule for pronouns.

    This function generates a grako (EBNF) rule for pronouns.

    Args:
        pronoun_list: A list of pronouns which could appear in the grammar
    
    Returns:
        pron_rule: The grako grammar rule for pronouns as a string
    """
    
    unique_pronouns = list(set(pronoun_list))

    pron_rule = "pron\n\t=\n"

    for pron in unique_pronouns:
        pron_rule += "\t| '" + pron + "'\n"

    pron_rule += "\t;\n\n"

    return pron_rule

def form_gesture_rule(input_file):
    """ Generates grammar rule for gestures (e.g. waving).

    This function generates a grako (EBNF) rule for gestures, such as
    waving and pointing.

    Args:
        input_file: The .xml file containing the gesture information
    
    Returns:
        gest_rule: The grako grammar rule for gestures as a string
    """

    xml_file = minidom.parse(input_file)

    gest_rule = "gesture\n\t=\n"

    gestures = xml_file.getElementsByTagName('gesture')
    
    for gest in gestures:
        gest_name = remove_u(gest.attributes['name'].value.lower())
        gest_rule += "\t| '" + gest_name + "'\n"
    
    gest_rule += "\t;\n\n"

    return gest_rule 


def form_location_rules(input_file):
    """ Generates grammar rules for locations.

    This function generates grako (EBNF) rules for locations including
    rules for rooms, placements and beacons.

    Args:
        input_file: The .xml file containing the location information
    
    Returns:
        loc_rules: The grako grammar rules for locations as a string
    """

    xml_file = minidom.parse(input_file)

    loc_rule = "location\n\t=\n\t| room\n\t| placement\n\t| beacon\n\t;\n\n"
    room_rule = "room\n\t=\n"
    placement_rule = "placement\n\t=\n"
    beacon_rule = "beacon\n\t=\n"
    
    rooms = xml_file.getElementsByTagName('room')

    for room in rooms:
        room_name = remove_u(room.attributes['name'].value.lower()) 
        room_rule += "\t| '" + room_name + "'\n"
        
        locs = room.getElementsByTagName('location')
        for loc in locs:
            loc_name = remove_u(loc.attributes['name'].value)

            try:
                beacon = loc.attributes['isBeacon'].value.lower()
                if 'true' in beacon:
                    beacon_rule += "\t| '" + loc_name + "'\n"
            except:
                pass

            try:
                placement = loc.attributes['isPlacement'].value.lower()
                if 'true' in placement:
                    placement_rule += "\t| '" + loc_name + "'\n"
            except:
                pass


    room_rule += "\t;\n\n"
    placement_rule += "\t;\n\n"
    beacon_rule += "\t;\n\n"
    
    loc_rules = loc_rule + room_rule + placement_rule + beacon_rule

    return loc_rules
    

def form_name_rules(input_file):
    """ Generates grammar rules for names.

    This function generates grako (EBNF) rules for names, split into male
    and female names.

    Args:
        input_file: The .xml file containing the name information
    
    Returns:
        name_rules: The grako grammar rules for names as a string
    
    Raises:
        bad_name: Raised if a bad name element is present in the xml file
    """

    xml_file = minidom.parse(input_file)

    name_rule = "name\n\t=\n\t| male\n\t| female\n\t;\n\n"
    male_rule = "male\n\t=\n"
    female_rule = "female\n\t=\n"

    names = xml_file.getElementsByTagName('name')

    for name in names:
        
        person_name = remove_u(name.firstChild.data.lower())
        gender = remove_u(name.attributes['gender'].value.lower())

        if gender == 'male':
            male_rule += "\t| '" + person_name + "'\n"
        elif gender == 'female':
            female_rule += "\t| '" + person_name + "'\n"
        else:
            raise Exception('Invalid Name Element in XML file.')
    
    male_rule += "\t;\n\n"
    female_rule += "\t;\n\n"
    
    name_rules = name_rule + male_rule + female_rule

    return name_rules

def form_object_rules(input_file):
    """ Generate grammar rules for objects.

    This function generates grako (EBNF) rules for objects, split into
    categories, and then known, alike and special objects.

    Args:
        input_file: The .xml file containing the object information
    
    Returns:
        object_rules: The grako grammar riles for objects as a string
    
    Raises:
        unknown_type: Raised if an unknown object type is found
    """

    xml_file = minidom.parse(input_file)

    category_rule = "category\n\t=\n"
    object_rule = "object\n\t=\n\t| kobject\n\t| aobject \n\t| sobject\n\t;\n\n"
    kobject_rule = "kobject\n\t=\n"
    aobject_rule = "aobject\n\t=\n"
    sobject_rule = "sobject\n\t=\n"

    categories = xml_file.getElementsByTagName('category')
    for cat in categories:
        cat_name = remove_u(cat.attributes['name'].value.lower())
        category_rule += "\t| '" + cat_name + "'\n"
        objects = cat.getElementsByTagName('object')

        for obj in objects:
            name = remove_u(obj.attributes['name'].value.lower())
            obj_type = remove_u(obj.attributes['type'].value.lower())
            if obj_type == 'known':
                kobject_rule += "\t| '" + name + "'\n"
            elif obj_type == 'special':
                sobject_rule += "\t| '" + name + "'\n"
            elif obj_type == 'alike':
                aobject_rule += "\t| '" + name + "'\n"
            else:
                raise Exception('Unknown Object Type Found.')

    category_rule += "\t;\n\n"
    kobject_rule += "\t;\n\n"
    aobject_rule += "\t;\n\n"
    sobject_rule += "\t;\n\n"

    object_rules = (category_rule + 
                    object_rule + 
                    kobject_rule + 
                    aobject_rule +
                    sobject_rule)
    

    return object_rules

def form_question_rule():
    """ Generate grammar rule for questions.

    This function creates the grako (EBNF) grammar rule for questions.
    This is simply the word 'question'.

    Returns:
        q_rule: The grammar rule for questions as a string
    """

    q_rule = "question\n\t=\n\t| 'question'\n\t;\n\n"
    return q_rule

def form_grammar(output_file, enhanced):
    """ Forms grako grammar for (E)GPSR task.

    This function forms the grako usable grammar for the GPSR task. It does
    this by reading in all basic xml files and also the two main GPSR grammar
    files in the official repository.

    Args:
        output_file: The file to output the grammar to
        enhanced: True for EGPSR, False for GPSR
    
    Raises:
        bad_extension: Raised if a bad file extension is used
    """

    extension = output_file[-4:]
    if extension != '.txt':
        raise Exception('Invalid File Extension Given. Use .txt')
    
    with open(output_file, 'w') as grammar:

        # naming the grammar
        if enhanced:
            grammar.write('@@grammar::EGPSR\n\n') 
        else:
            grammar.write('@@grammar::GPSR\n\n')
        
        # Add stuff from xml files etc.
        grammar.write(form_pronoun_rule(PRONOUNS))
        grammar.write(form_gesture_rule(GESTURES))
        grammar.write(form_location_rules(LOCATIONS))
        grammar.write(form_name_rules(NAMES))
        grammar.write(form_object_rules(OBJECTS))
        grammar.write(form_question_rule())

        
if __name__ == '__main__':
    form_grammar('../grammars/test_gpsr.txt', False)
