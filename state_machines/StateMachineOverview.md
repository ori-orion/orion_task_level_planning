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

## Sub State Machines

We then have the ideas of sub-state machines.
These act in a similar manner to functions, in that they are structures that can be reused multiple times.

For example, see `orion_task_level_planning/state_machines/src/state_machines/robocup_2023_hypothesis/Put away the groceries 2.png`
Every box in this is a sub state machine. 
Our sub state machines can be found within `orion_task_level_planning/state_machines/src/state_machines/SubStateMachines`

## Our architecture - the states

As previously mentioned, we have broken down the system into multiple sub systems. 
These sub systems are as follows:
 - Manipulation
 - Navigation
 - Perception
 - Procedural
 - SOM
 - Speech
 - Training
 - Miscellaneous

The other thing to note is that we use config files extensively.
These can all be found under `/orion_task_level_planning/state_machines/config` and the names often match the name of the state machine in question.


### Manipulation

- Picking up an object (`PickUpObjectState_v2`).
    - Note that here, `PickUpObjectState_v2` should be used instead of `PickUpObjectState` which is deprecated. The newer state attempts retrys within the state and has more checks for success. It also returns `MANIPULATION_FAILURE` rather than `FAILURE` which can be useful for constructing the state machine.
- Putting an object down (`PutObjectOnSurfaceState` and `PlaceNextTo`).
- Handing an object to a human (`HandoverObjectToOperatorState`).
- Receiving an object from a human (`ReceiveObjectFromOperatorState`).
- Pointing at an object (`PointAtEntity`).

5 states

### Navigation

- Getting the current location of the robot (`GetRobotLocationState`).
- Navigating using `tmc_move_base` (`SimpleNavigateState_v2`).
    - Here `SimpleNavigateState_v2` should be used instead of `SimpleNavigateState` which is deprecated. The reasoning is basically the same as above, where `NAVIGATIONAL_FAILURE` is returned instead of `FAILURE`.
    - Note that the `SimpleNavigateState_v2` will see if the robot is not moving, and if this is the case, it will try to replan a goal. If this goal is blocked, it will read in the occupancy map (using `NavigationalListener`) to find the closest point that is free.
- Navigating using the ORI navigational packages (`TopologicalNavigateState`).
- Finding a nav goal that is a set distance from a given point (`NavigateDistanceFromGoalSafely`). 
    - The distance here is hardcoded to 0.9m within the class. Potentially could be made an input. 

4 states

### Perception

This is where facial recognition would go, but this is not working at present.

### Procedural

These states are supprisingly useful. They are as follows:
- Checking to see if `left < right` (`LessThanState`)
- Executing `appending_to.append(appending_with)` (`AppendToArrState`)
- Executing `val += increment_by` (`IncrementValue`)
- Executing `return input_list[index].property` (`GetPropertyAtIndex`)
    - This can return multiple properties, and can often be used to get specific parameters from the results of a given SOM query.
- Working out if a list is empty or not (`GetListEmpty`).
- Setting a given variable to an empty list (`SetToEmptyList`). 

6 states

### SOM


This pertains to accessing the objects in the memory system. The states allow for
- Creating a SOM query (`CreateSOMQuery`).
- Adding a parameter to the SOM query to refine the search (`AddSOMEntry`).
- Performing the query (`PerformSOMQuery`).
- Sorting the entries with respect to a given list (`SortSOMResultsAsPer`).
    - SOM entries are automatically sorted by the time the objects were last seen upon being outputted from SOM. 
    - This allows us to sort based on a different criterion.
    - For example, say you want all the entries of category `fruits` to be first, then of `drinks` etc. You would then use this state with the input argument of `order_of_preference=["fruits", "drinks",...]`. 
- Filtering out certain entries (`FilterSOMResultsAsPer`).
    - Let's say you've tried and failed to pick up an apple with tf `apple_0`. You could then filter out all instances of objects with entry `tf_name == apple_0` from the outputs by using this state.
    - For this, we would want this state with the arguments `filter_by=tf_name`, and the userdata including `filtering_by=["apple_0",...]`. 
    - Note that if you wanted to only get entries with `tf_name=="apple_0"`, you would need to set `filter_out=False`.

There then a few states that pertain to the (old) find my mates task. These save details of humans to SOM for querying later. These haven't been tested fully.

Finally, there is a standalone class for creating an occupancy map using the entries found within SOM called `SOMOccupancyMap` (allowing us to, for instance, find potential placement locations, or empty chairs. This works fairly well, and might be useful. However, it is specific to objects at present.)

5 states


#### An explanation of how SOM works...

The SOM system is the memory system on the robot. 
Each entry in the system should match a given ROS message type.
You get information from the memory system by sending a query into the system. 
The results that are returned are then those that match the query.

So, let's say that the message type is as follows:
```
string class_
string category
time last_observed_at
geometry_msgs/Pose obj_position
```
 - If we send in a query with `class_="bottle` and leave all other entries at their  default values, then all entries with `class_=="bottle"` will be returned.
 - If we send in a query with `last_observed_at=rospy.Time.now()-rospy.Duration(5)`, then all entries observed in the last 5 seconds will be returned.
 - If we send in a query with `class_="bottle"` and `last_observed_at=rospy.Time.now()-rospy.Duration(5)`, then all entries with `class_=="bottle"` that were observed in the last 5 seconds will be returned.
Thus filling in multiple attributes within the query acts as an AND gate within the query.

#### How this works within the state machine system.

- `CreateSOMQuery` creates a given empty query. The query type is passed in by argument.
    - For instance `CreateSOMQuery(CreateSOMQuery.OBJECT_QUERY)`, creates a query into the object collection.
    - `CreateSOMQuery(CreateSOMQuery.HUMAN_QUERY)` creates a query into the human collection.
    - `CreateSOMQuery(CreateSOMQuery.OBJECT_QUERY, save_time=True)`, creates a query into the object collection that fills the current time in, thus restricting the query to items seen since the time this state was active.
- `AddSOMEntry` then fills out entries within the query.
    - `AddSOMEntry("class_")` will fill the `class_` field with `userdata.value`.
    - `AddSOMEntry("class_", "bottle")` will fill the `class_` field with `"bottle"`.
    - This uses the python functions `hasattr(...)` and `setattr(...)` so any fields within the query can be passed in.
- `PerformSOMQuery` then performs the query and fills out `userdata.som_query_results` with the array of results returned.
    - For instance, when performing an object query, `userdata.som_query_results` will be of type `List[SOMObject]`.


### Speech

- Saying an arbitrary phrase (`SpeakState`).
- Speaking and listening for a response (`SpeakAndListenState`).
- Asking specifically for a person's name (`AskPersonNameState`).
- Waiting for a hotword (`WaitForHotwordState`).
- Asking from a selection of questions (`AskFromSelection` and `ReportBackToOperator`).
    - See documentation within `speech_states.py`
- Saying a phrase with userdata fields added automatically (`SayArbitraryPhrase`).
    - This is extremely useful. There is an example of this within `orion_task_level_planning/state_machines/src/state_machines/robocup_2023_hypothesis/put_away_the_groceries.py`. Within this file, search for the state `TellOperatorClassCategory`.

6 states

### Training

These states are mainly used in the training state machines `orion_task_level_planning/state_machines/src/state_machines/Training`.
- Printing to the console (`PrintToConsole`).
- Reading in from the console (`ReadInFromConsole`).

2 states

### Miscellaneous

This is the set of states that don't seem to fit into any given category.
- Getting the current time (`GetTime`).
- Waiting for a set number of seconds (`WaitForSecs`).
- Checking to see if the door is open for the competition (`CheckDoorIsOpenState`).
- Looking up (`LookUpState`).
- Looking at a human (`LookAtHuman`).
- Looking at a point (`LookAtPoint`).
- Raising the mast (`RaiseMastState`).
- Moving to neutral (`MoveToNeutralState`).
- Spinning the head around (`SpinState`).
- Remapping between variables (`ExplicitRemap`).
    - Despite having the remapping argument, this can still be useful if there are variables that you want to pass as input into a state that outputs a variable of the same name, but you need to remap upon input.
- Waiting for someone to touch the hand (`WaitForWristWrench`).
    - This could do with a little fine tuning. You need to push harder than necessary for this.

11 states

## Our architecture - the sub state machines.

We then have the sub state machines for more developed and refined functionality.

These are as follows:

### create_sub_state_machines.py

A lot of these are using older states, so, if you want to use these, it is recommended to rewrite some of the functionality, especially using the `SayArbitraryPhrase` state.

- `create_learn_guest_sub_state_machine`
- `create_search_for_guest_sub_state_machine`
- `create_topo_nav_state_machine`
- `create_point_to_all_guests`
- `create_get_guest_details`
- `create_intro_to_operator`

### navigate_within_distance_of.py

This is mainly for local navigation. You want to navigate to a pose from which you can pick up an object or from which you can talk to someone.

- `navigate_within_distance_of_pose_input`
    - Navigate within a certain distance (NavigateDistanceFromGoalSafely::DISTANCE_FROM_POSE) of a given pose.
    - It looks at the goal, finds a nav goal, navigates, and then looks at the goal again.
- `navigate_within_distance_of_som_input`
    - Performs a SOM query (which is inputted). 
    - It then navigates to a point close to the first result from this query (using `navigate_within_distance_of_pose_input`)
- `search_for_entity`
    - This has two different modes.
        - Spin and query where it uses ORIon spin to look round, and then queries for the things it saw during the spin.
        - All time query where it queries for all the objects it's seen ever.
    - One of these is then run after the other one, the behaviour being toggled using the input argument `spin_first`. 
    - `som_query_results`, the results from the query, are then returned.
- `nav_within_reaching_distance_of`
    - This combines `search_for_entity` with `navigate_within_distance_of_pose_input`, first searching for an object of a given class or category (toggleable using the flag `find_same_category`), to navigate within reaching distance of a given type of object.
- `nav_and_pick_up_or_place_next_to`
    - Finally this uses `nav_within_reaching_distance_of` to either pick up an object matching a given class or category, or to put down an object in the gripper that matches a given class or category.
    - The put down infrastructure still needs to include a backup for if a placement location is not found.
    - This is then essentially the entry point for pick and place infrastructure.

### startup.py

- `create_wait_for_startup`
    - This waits for the door to open, waits a little bit of time, and then returns `SUCCESS`.
    - For starting each round in the competition.

