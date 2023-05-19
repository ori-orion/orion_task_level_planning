# Running the introspection server.

Some of the state machines (but not all) are linked up to the introspection server. 
Linking them up is easy:
```
sm = create_state_machine();
sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
sis.start()
sm.execute();
```
Viewing the state machine is then possible through the command.
```
rosrun smach_viewer smach_viewer
```
This then loads a real time visual display of the state machine and its current state.

# State Machines for ORION.

## Stage I

### Clean Up (housekeeper)

Find six misplaced objects in a room and bring them to their predefined locations.

### Storing Groceries

Move 5 objects from a table to the cabinet, grouping them by category.
This feels like the most feasable next task. 
Manipulation needs to know what's empty beforehand. 

### Serve Breakfast

Has to set the table for one person and prepare cereal for them. 
 - Place breakfast items on the table.
    - Bowl, spoon, cereal box and milk carton. 
 - Big goal: Pour cereal into the bowl.

### Take out the trash. 

This has now been written, but the manipulation actions are not finalised. Neither are the locations.



--------------------

### Carry My Luggage

Very hard.

### Farewell

Have to show 2 guests out (including a doctor who's urgent). 
Perception (again).

### Find my mates

The state of the system at present is it can identify everyone but doesn't then try to learn their names.
This is insufficient as per the rule book, but I would say that talking to them is slightly risky given the problems we had last time.

### Receptionist

Task is to take two new guests into the living room to introduce them and offer a free place to sit.
Again, we'd need to identify the free location. (Not sure what we might have to do that yet).
"Maintain appropriate gaze direction."
Would also therefore need person tracking which we theoretically have... 

### Serving drinks

Take the order of a drink from all people without one, and deliver the correct drink to the person who ordered it. 
Requires person identification.
Drink identification.
Picking up a drink

## Stage II

### Hand me that.

Task is to identify stuff that the operator points at. 
Perception heavy.