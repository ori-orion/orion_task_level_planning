#!/usr/bin/env python
""" File for generating grako grammar from GPSRCmdGen grammar.

This file contains code to take the grammar definition given by the
GPSRCmdGen repository and re-format it into a grammar suitable for grako
so we can autogenerate state machines for the GPSR task.

Author: Charlie Street

"""

from xml.dom import minidom
import re

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

GPSR_COMMON = '../GPSRCmdGen/GPSRCmdGen/Resources/CommonRules.txt'
GPSR_GRAMMAR = '../GPSRCmdGen/GPSRCmdGen/Resources/GPSRGrammar.txt'

EGPSR_COMMON = '../GPSRCmdGen/EPGSRCmdGen/Resources/CommonRules.txt'
EGPSR_GRAMMAR = '../GPSRCmdGen/EPGSRCmdGen/Resources/EGPSRGrammar.txt'

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


def process_wildcard(wildcard):
    """ Processes the GPSR wildcard token.

    This function takes a wildcard, i.e. {<stuff>} from the GPSR grammar.
    It returns back an appropriate string for the new grako grammar.

    Args:
        wildcard: The string encapsulated in {}
    
    Returns:
        grako_non_terminal: The appropriate non terminal for our new grammar.
    
    Raises:
        bad_obfuscation: Raised if ? used in wrong place
        unknown_wildcard: Raised if unknown wildcard found
    """
    wildcard_data = wildcard[1:-1]

    # Do we want to move up a level?
    obfuscate = '?' in wildcard_data

    if (wildcard_data.starts_with('location beacon') or
       wildcard_data.starts_with('beacon')):
        if obfuscate:
            return 'room'
        else:
            return 'beacon'

    elif (wildcard_data.starts_with('object alike') or 
          wildcard_data.starts_with('aobject')):
        if obfuscate:
            return 'category'
        else:
            return 'aobject'

    elif (wildcard_data.starts_with('name female') or 
          wildcard_data.starts_with('female')):
        if obfuscate:
            raise Exception("Can't Obfuscate With Names: " + wildcard)
        else:
            return 'female'

    elif (wildcard_data.starts_with('object known') or 
          wildcard_data.starts_with('kobject')):
        if obfuscate:
            return 'category'
        else:
            return 'kobject'

    elif (wildcard_data.starts_with('name male') or 
          wildcard_data.starts_with('male')):
        if obfuscate:
            raise Exception("Can't Obfuscate With Names: " + wildcard)
        else:
            return 'male'

    elif (wildcard_data.starts_with('location placement') or 
          wildcard_data.starts_with('placement')):
        if obfuscate:
            return 'room'
        else:
            return 'placement'

    elif (wildcard_data.starts_with('location room') or 
          wildcard_data.starts_with('room')):
        if obfuscate:
            return "'room'"
        else:
            return 'room'

    elif (wildcard_data.starts_with('object special') or 
          wildcard_data.starts_with('sobject')):
        if obfuscate:
            return 'category'
        else:
            return 'sobject'

    elif wildcard_data.starts_with('category'):
        if obfuscate:
            return "'objects'"
        else:
            return 'category'

    elif wildcard_data.starts_with('gesture'):
        if obfuscate:
            raise Exception("Can't Obfuscate With Gestures: " + wildcard)
        else:
            return 'gesture'
    
    elif wildcard_data.starts_with('location'):
        if obfuscate:
            return ("('room' | room)")
        else:
            return 'location'

    elif wildcard_data.starts_with('name'):
        if obfuscate:
            raise Exception("Can't Obfuscate With Names: " + wildcard)
        else:
            return 'name'

    elif wildcard_data.starts_with('object'):
        if obfuscate:
            return 'category'
        else:
            return 'object'
    
    elif wildcard_data.starts_with('question'):
        if obfuscate:
            raise Exception("Can't Obfuscate With Questions: " + wildcard)
        else:
            return 'question'

    elif wildcard_data.starts_with('void'):
        if obfuscate:
            raise Exception("Can't Obfuscate With Void: " + wildcard)
        else:
            # TODO: Find a better solution to this!
            return "'VOID'"

    elif wildcard_data.starts_with('pron'):
        if obfuscate:
            raise Exception("Can't Obfuscate With Pronouns: " + wildcard)
        else:
            return 'pron'
    else:
        raise Exception('Unknown Wildcard Found: ' + wildcard)
    

def consume_non_terminal(str, start):
    """ Consumes a non-terminal in the grammar.

    This function consumes a non-terminal in the GPSRCmdGen grammar, starting
    at start until an invalid character for a nonterminal name is found.

    Args:
        str: The string to consume from
        start: The start of the non-terminal, i.e. the $ character
    
    Returns:
        non_terminal: The full non-terminal string
        pointer: The index in str of the first character of the non-terminal

    Raises:
        invalid_non_terminal: Raised when an invalid non terminal is passed in
        end_of_string: Raised if we reach the end of the string without stopping
    """
    
    non_terminal = "$"

    if str[start] != "$":
        raise Exception("Invalid Non-Terminal Passed In: " + str[start:])

    for i in range(start+1, len(str)):
        if bool(re.match("[a-z]|[A-Z]|[0-9]|_",str[i])):
            non_terminal += str[i]
        else:
            return non_terminal, i
    
    raise Exception('Reached End Of String Consuming Non-Terminal: ' +
                    str[start:])


def consume_bracket(str, start, brace):
    """ Consumes a bracketed expression in the grammar.

    This function consumes a string until the correct closing bracket is
    matched. It then returns the bracketed expression as well as the index
    of the first character not related to the bracketed expression.

    Args:
        str: The string being examined
        start: The start of the expression, i.e. the position of the (/{
        brace: The type of brace to use, i.e. (/{ in a pair, e.g. ('{','}')

    Returns: 
        expression: The bracketed expression
        pointer: The index of the first character after the expression
    
    Raises:
        invalid_expression: Raised if an invalid expression is passed in
        end_of_string: Raised if we reach the end of the string without stopping
    """
    
    expression = brace[0]
    open_braces = 1

    if str[start] != brace[0]:
        raise Exception("Invalid Bracket Expression Passed In: " + str[start:])

    for i in range(start+1, len(str)-1):
        expression += str[i]

        if str[i] == brace[0]:
            open_braces += 1
        elif str[i] == brace[1]:
            open_braces -= 1
        
        if open_braces == 0:
            return expression, i+1

    raise Exception('Reached End Of String Consuming Bracket Expression: ' + 
                    str[start:])

def parse_line(line, root):
    """ Parses a single line of the GPSRCmdGen grammar.

    This function takes a line from the GPSRCmdGen version of the grammar
    and outputs a line for the grako grammar, and also returns the nonterminals
    seen on the rhs of this line.

    Args:
        line: The string to be parsed
        root: Is the root call to the function (boolean)

    Returns:
        non_terminals_seen: The nonterminal seen in this line
        grako_line: The line to be used in the grako grammar
    """
    non_terminals_seen = []
    grako_line = ""

    # Keeping track of occurrences of literals
    literal = False
    add_outer_brackets = False

    # Accounting for new line characters
    length = len(line)
    if line[length-1] == "\n":
        length -= 1
    
    i = 0

    while i < length:

        if line[i] == "$": # Start non-terminal

            # Stop the literal
            if literal:
                grako_line += "' "
                literal = False
            
            non_terminal, pointer = consume_non_terminal(line, i)
            i = pointer
            non_terminals_seen.append(non_terminal)
            grako_line += non_terminal[1:].lower() # Remove the $ and lower

        elif line[i] == "{": # Start non-terminal
            
            # Stop the literal
            if literal:
                grako_line += "' "
                literal = False

            wildcard, pointer = consume_bracket(line, i, ('{','}'))
            grako_line += process_wildcard(wildcard)
            i = pointer

        elif line[i] == "(": # Start non-terminal
            
            # Stop the literal
            if literal:
                grako_line += "' "
                literal = False

            expression, pointer = consume_bracket(line, i, ('(', ')'))
            grako_line += "(" + parse_line(expression[1:-1], False) + ")"
            i = pointer

        elif line[i:i+2] == " |":
            
            # Stop the literal 
            if literal:
                grako_line += "' "
                literal = False

            grako_line += " | "
            i += 3

            if root:
                add_outer_brackets = True

        else:
            if not literal:
                grako_line += "'"
                literal = True
            grako_line += line[i]
            i += 1


    if add_outer_brackets:
        grako_line = "(" + grako_line + ")"

    return non_terminals_seen, grako_line

def parse_GPSR_grammar(input_files):
    """ Parses files of the GPSRCmdGen grammar.

    This function parses GPSRCmdGen grammar files and converts it into
    a grako (EBNF) grammar for later use.

    Args:
        input_files: The .txt file containing the GPSRCmdGen grammar

    Returns:
        grako_grammar: A string containing the parsed grako grammar
    
    Raises:
        unknown_non_terminal: Raised if an unknown non terminal is found
    """

    # Read in lines of all files
    lines = []
    for in_file in input_files:
        with open(in_file) as to_read:
            lines += to_read.readlines()
    
    # Remove all useless lines from the input files
    no_comments = filter((lambda x: len(x) > 0 and x[0] == '$'), lines)

    grammar = ""

    # Start our grammar building with just
    non_terminals = ["$Main"]
    completed_non_terminals = []

    while non_terminals != []:

        # Adjust non_terminals queue
        new_nt = non_terminals[0]
        non_terminals = non_terminals[1:]

        # If we've already made the rule for this we don't worry about it
        if new_nt in completed_non_terminals:
            continue

        # The grako non-terminal name
        new_rule = new_nt[1].lower()

        # Account for initial case
        if new_nt == "$Main":
            new_rule = "start"
        
        new_rule += "\n\t=\n"

        # All lines starting with that nonterminal
        filter_by_nt = filter((lambda x: x.starts_with(new_nt)), no_comments)

        # If this non-terminal doesn't have a definitin
        if filter_by_nt == []:
            raise Exception('Unknown Non-Terminal Found: ' + new_nt)

        # Compute the new grako lines and add for that rule
        for line in filter_by_nt:

            # the rhs of the line is all we care about
            rhs = line[line.find('=')+2:]

            non_terminals_seen, grako_line = parse_line(rhs, True)
            new_rule += "\t| " + grako_line + "\n"
            non_terminals += non_terminals_seen

        new_rule += "\t;\n\n"
        
        grammar += new_rule

        completed_non_terminals.append(new_nt)

    return grammar






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
        
        # Create a new start symbol with explicit terminator
        grammar.write('s = start $\n\n')

        # Now do the bulk of the parser conversion
        grammar_files = [GPSR_GRAMMAR, GPSR_COMMON]
        
        if enhanced:
            grammar_files = [EGPSR_GRAMMAR, EGPSR_COMMON]

        grammar.write(parse_GPSR_grammar(grammar_files))

        # Add stuff from xml files etc.
        grammar.write(form_pronoun_rule(PRONOUNS))
        grammar.write(form_gesture_rule(GESTURES))
        grammar.write(form_location_rules(LOCATIONS))
        grammar.write(form_name_rules(NAMES))
        grammar.write(form_object_rules(OBJECTS))
        grammar.write(form_question_rule())

        
if __name__ == '__main__':
    form_grammar('../grammars/test_gpsr.txt', False)
