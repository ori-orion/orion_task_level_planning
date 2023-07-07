# State Machine Overview

The purpose of this document is to explain how state machines work, and potentially give the information required to port over to behaviour trees.

## Overall architecture.

### The basics.

A state machine consists of a set of states. 
Each state has a set of outcomes ("success", "failure", "no_objects_seen"...).
Given a state being run, these outcomes are returned, and then the next state is chosen as a result of these outcomes.
So, for example, say you want to navigate to a table and then pick up an object, the state machine might be:
```
Nav to table
    | -> failure        -> Nav to table
    | -> repeat failure -> task failure
    | -> success -> Pick up obj.
```
Thus the overall state machine can be represented as a graph where each node is a given action (Pick up an object, navigate, perform SOM query, look round, ...) and the arrows go from state to state. The path that is taken then depends on the outcome from the particular state. 
All of our states are given within `orion_task_level_planning/state_machines/src/state_machines/ReusableStates`. They are then categorised by subsystem, in a manner that should be relatively intuitive. (An overview of this will be given later).

We then have the ideas of sub-state machines.
These act in a similar manner to functions, in that they are structures that can be reused multiple times.
[Example state machine](src/state_machines/robocup_2023_hypothesis/Put away the groceries 2.png)