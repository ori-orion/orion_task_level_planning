#!/usr/bin/env python3
""" Module for dummy behaviours for state machines.

This file contains dummy behaviours used within our
state machine behaviours, i.e. we use these while we
don't have real behaviours to insert.

Author: Charlie Street

"""

import numpy as np
import rospy


def roulette_wheel(probs):
    """Roulette wheel selection.

    Carries out roulette wheel selection over 
    a distribution to sample an index.

    Args:
        probs: A list of probabilities
    
    Returns:
        index: The index in probs that was selected
    
    Raises:
        invalid_dist: Raised if an invalid distribution is passed in
    """

    if not np.isclose(np.sum(np.array(probs)), 1.0):
        raise Exception('Invalid Distribution Passed in')
    
    selected_value = np.random.rand()
    prob_sum = 0.0
    for i in range(len(probs)):
        prob_sum += probs[i]
        if selected_value < prob_sum:
            return i

def dummy_behaviour(outcomes, probs, messages=[]):
    """ Dummy behaviour for state machine.

    This function mimics the work done in a SMACH
    state. Given probabilities of different outcomes,
    it selects one at random, prints a message and returns
    the appropriate outcome.

    Args:
        outcomes: A list of outcomes from a state in the state machine
        probs: A list of probabilities for each outcome
        messages: A list of string messages, one for each outcome.
                  If not specified, we just print the outcome.

    Returns:
        sampled_outcome: The outcome sampled from the probability distribution

    """

    assert len(outcomes) == len(probs)

    index = roulette_wheel(probs)
    if messages == []:
        rospy.loginfo(outcomes[index])
    else:
        rospy.loginfo(messages[index])
    
    return outcomes[index]
