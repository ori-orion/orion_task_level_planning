#!/usr/bin/env python
""" Utilities for the GPSR task.

This file contains utilities for the GPSR task, including filtering
characters out of the input strings, and creating parser objects.

Author: Charlie Street
Owner: Charlie Street
"""

from GPSR_semantic_actions import GPSRSemanticActions
from GPSR_parser import GPSRParser

# Grako doesn't seem to deal with whitespace characters the way I imagined
BAD_CHARACTERS = ["'",".",",","\n","\t"]

def filter_string(string):
    """ Removes bad characters from string. 

    This function filters out bad characters (in BAD_CHARACTERS) from
    the string passed in.

    Args:
        string: The string to filter
    
    Returns:
        filtered_string: The filtered string
    """

    filtered_string = ""
    for char in string:
        if char not in BAD_CHARACTERS:
            filtered_string += char
    
    return filtered_string


def parse_string(parser, string):
    """ Wrapper for parsing string with GPSRParser. """

    filtered = filter_string(string)
    # Could you please causes an issue, probably with lookahead...
    if filtered.lower().find('could you please') == 0:
        filtered = filtered[10:]
    return parser.parse(filtered, 
                        rule_name='s', 
                        semantics=GPSRSemanticActions())
