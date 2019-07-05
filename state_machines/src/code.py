diff --git a/state_machines/src/GPSR.py b/state_machines/src/GPSR.py
index de3b8d7..d530f6c 100644
--- a/state_machines/src/GPSR.py
+++ b/state_machines/src/GPSR.py
@@ -71,6 +71,7 @@ def create_state_machine(action_dict):
     global_store = {}
     global_store['start_time'] = time.time()
     global_store['tasks_completed'] = 0
+    global_store['people_found'] = []
 
     sm = smach.StateMachine(outcomes=['TASK_SUCCESS', 'TASK_FAILURE'])
 
diff --git a/state_machines/src/carry_my_luggage.py b/state_machines/src/carry_my_luggage.py
index 06bf283..3311383 100644
--- a/state_machines/src/carry_my_luggage.py
+++ b/state_machines/src/carry_my_luggage.py
@@ -32,6 +32,7 @@ def create_state_machine(action_dict):
 
     # Initialise global store
     global_store = {}
+    global_store['people_found'] = []
 
     # Create the state machine
     sm = smach.StateMachine(outcomes=['TASK_SUCCESS', 'TASK_FAILURE'])
diff --git a/state_machines/src/choose_task.py b/state_machines/src/choose_task.py
index aa2aa84..85feefd 100755
--- a/state_machines/src/choose_task.py
+++ b/state_machines/src/choose_task.py
@@ -130,7 +130,34 @@ def choose_task():
             choose_task()
 
     elif stage == 2:
-        print("Not Implemented Yet!")
+        print("Please enter the number (1-9) of the task you want to run:")
+        print("[1]: Clean The Table (Housekeeper)")
+        print("[2]: Enhanced GPSR (Housekeeper)")
+        print("[3]: Find My Disk (Housekeeper)")
+        print("[4]: Hand Me That (Party Host)")
+        print("[5]: Set The Table (Housekeeper)")
+        print("[6]: Restaurant (Party Host)")
+        print("[7]: Smoothie Chef (Party Host)")
+        print("[8]: Stickler For The Rules (Party Host)")
+        print("[9]: Where Is This? (Party Host)")
+        task = input()
+
+        if task == 1:
+            print("Clean The Table Selected.")
+            # TODO: Sort out
+        elif task == 4:
+            print("Hand Me That Selected.")
+            # TODO: Sort out
+        elif task == 5:
+            print("Set The Table Selected.")
+            # TODO: Sort out
+        elif task == 7:
+            print("Smoothie Chef Selected.")
+            # TODO: Sort out
+        else:
+            print("Invalid Task Entered. Please try again.")
+            choose_task()
+
     elif stage == 3:
         print("Not Implemented Yet!")
     else:
diff --git a/state_machines/src/clean_the_table.py b/state_machines/src/clean_the_table.py
new file mode 100644
index 0000000..c4c6c65
--- /dev/null
+++ b/state_machines/src/clean_the_table.py
@@ -0,0 +1,363 @@
+#!/usr/bin/env python
+""" Code for the Clean The Table Task.
+
+This file contains the state machine code for the Clean The Table task.
+
+Author: Charlie Street
+Owner: Charlie Street
+"""
+
+import rospy
+import smach
+import actionlib
+import time
+
+from reusable_states import * # pylint: disable=unused-wildcard-import
+from set_up_clients import create_stage_2_clients
+from orion_actions.msg import SOMObservation, Relation
+
+def go_to_dishwasher(action_dict):
+    """ Gets location of dishwasher. """
+    obj1 = SOMObservation()
+    obj1.type = 'clean_the_table_point_of_interest_dishwasher'
+
+    return get_location_of_object(action_dict, obj1, 
+                                  Relation(), SOMObservation())
+
+
+def go_to_table(action_dict):
+    """ Gets location of table. """
+    obj1 = SOMObservation()
+    obj1.type = 'clean_the_table_point_of_interest_table'
+
+    return get_location_of_object(action_dict, obj1, 
+                                  Relation(), SOMObservation())
+
+
+class DecideNextItemState(ActionServiceState):
+    """ Decides the next item to put in the tray. """
+    def __init__(self, action_dict, global_store):
+        outcomes = ['ITEM', 'NONE_LEFT']
+        super(DecideNextItemState, self).__init__(action_dict=action_dict,
+                                                  global_store=global_store,
+                                                  outcomes=outcomes)
+    
+    def execute(self, userdata):
+        # TODO: Fill in! and organise how to do this
+        self.global_store['rel_pos'] = ('tray', 0.0, 0.0, 0.2)
+        return self._outcomes[1]
+
+
+def create_state_machine(action_dict):
+    """ This function creates and returns the state machine for this task. """
+
+    # Initialise global_store
+    global_store = {}
+    global_store['drawer_handle'] = 'dishwasher'
+    global_store['furniture_door'] = 'dishwasher'
+
+    # Create the state machine
+    sm = smach.StateMachine(outcomes=['TASK_SUCCESS', 'TASK_FAILURE'])
+
+    with sm:
+        
+        # Store initial location
+        smach.StateMachine.add('StoreLocation',
+                               GetRobotLocationState(action_dict, global_store),
+                               transitions={'STORED':'StartTalking'})
+        
+        # Start Talking
+        phrase = "Hi, I'm Bam Bam, lets tidy up!"
+        smach.StateMachine.add('StartTalking',
+                               SpeakState(action_dict, global_store, phrase),
+                               transitions={'SUCCESS':'SetNavToDishwasher',
+                                            'FAILURE':'SetNavtoDishwasher'})
+        
+        # Set nav to dishwasher
+        func = lambda : go_to_dishwasher(action_dict)
+        smach.StateMachine.add('SetNavToDishwasher',
+                               SetNavGoalState(action_dict, global_store, func),
+                               transitions={'SUCCESS':'NavToDishwasher'})
+        
+        # Nav to dishwasher
+        smach.StateMachine.add('NavToDishwasher',
+                               NavigateState(action_dict, global_store),
+                               transitions={'SUCCESS':'OpenDishwasher',
+                                            'FAILURE':'NavToDishwasher',
+                                            'REPEAT_FAILURE':'TASK_FAILURE'})
+        
+        # Open dishwasher door
+        smach.StateMachine.add('OpenDishwasher',
+                               OpenFurnitureDoorState(action_dict,global_store),
+                               transitions={'SUCCESS':'OpenRacks',
+                                            'FAILURE':'AskForDishwasherHelp'})
+        
+        # Ask for help opening dishwasher
+        question = ("Can someone open the dishwasher for me please and tell me"+
+                   " when it is open?")
+        smach.StateMachine.add('AskForDishwasherHelp',
+                               SpeakAndListenState(action_dict, 
+                                                   global_store,
+                                                   question,
+                                                   ['open'],
+                                                   [],
+                                                   30),
+                               transitions={'SUCCESS':'OpenRacks',
+                                            'FAILURE':'AskForDishwasherHelp',
+                                            'REPEAT_FAILURE':'TASK_FAILURE'})
+        
+        # Open dishwasher racks
+        smach.StateMachine.add('OpenRacks',
+                               OpenDrawerState(action_dict, global_store),
+                               transitions={'SUCCESS':'SetPickUpTray',
+                                            'FAILURE':'AskForRackHelp'})
+        
+        # Ask for help opening dishwasher racks
+        question = ("Can someone open the dishwasher racks for me please " +
+                    "and tell me when they are open?")
+        smach.StateMachine.add('AskForRackHelp',
+                               SpeakAndListenState(action_dict, 
+                                                   global_store,
+                                                   question,
+                                                   ['open'],
+                                                   [],
+                                                   30),
+                               transitions={'SUCCESS':'SetPickUpTray',
+                                            'FAILURE':'AskForRackHelp',
+                                            'REPEAT_FAILURE':'TASK_FAILURE'})
+        
+        # set pick up to tray
+        smach.StateMachine.add('SetPickUpTray',
+                               SetPickupState(action_dict, 
+                                              global_store, 
+                                              'tray'),
+                               transitions={'SUCCESS':'PickUpTray'})
+        
+        # Pick up the tray
+        smach.StateMachine.add('PickUpTray',
+                               PickUpObjectState(action_dict, global_store),
+                               transitions={'SUCCESS':'SetNavToTable',
+                                            'FAILURE':'AskForTrayHelp'})
+        
+        # Ask for help picking up tray
+        question = ("Can someone please hand me the dishwasher tray and " +
+                    "let me know when they're ready to hand it over?")
+        smach.StateMachine.add('AskForTrayHelp',
+                               SpeakAndListenState(action_dict, 
+                                                   global_store,
+                                                   question,
+                                                   READY,
+                                                   [],
+                                                   30),
+                               transitions={'SUCCESS':'ReceiveTray',
+                                            'FAILURE':'AskForTrayHelp',
+                                            'REPEAT_FAILURE':'TASK_FAILURE'})
+        
+        # Receive tray
+        smach.StateMachine.add('ReceiveTray',
+                               ReceiveObjectFromOperatorState(action_dict,
+                                                              global_store),
+                               transitions={'SUCCESS':'SetNavToTable',
+                                            'FAILURE':'AskForTrayHelp'})
+        
+        # Set nav to table
+        func = lambda : go_to_table(action_dict)
+        smach.StateMachine.add('SetNavToTable',
+                               SetNavGoalState(action_dict, global_store, func),
+                               transitions={'SUCCESS':'NavToTable'})
+        
+        # Nav to table
+        smach.StateMachine.add('NavToTable',
+                               NavigateState(action_dict, global_store),
+                               transitions={'SUCCESS':'PlaceTray',
+                                            'FAILURE':'NavToTable',
+                                            'REPEAT_FAILURE':'TASK_FAILURE'})
+        
+        # Try placing tray down
+        smach.StateMachine.add('PlaceTray',
+                               PutObjectOnSurfaceState(action_dict, 
+                                                       global_store),
+                               transitions={'SUCCESS':'DecideNextItem',
+                                            'FAILURE':'AskForTrayPlaceHelp'})
+        
+        # Ask for help picking up tray
+        question = ("Can someone please take the tray from me? If so, can you" +
+                    "let me know when you're ready to hand it over?")
+        smach.StateMachine.add('AskForTrayPlaceHelp',
+                               SpeakAndListenState(action_dict, 
+                                                   global_store,
+                                                   question,
+                                                   READY,
+                                                   [],
+                                                   30),
+                               transitions={'SUCCESS':'HandoverTray',
+                                            'FAILURE':'AskForTrayPlaceHelp',
+                                            'REPEAT_FAILURE':'TASK_FAILURE'})
+        
+        # Handover tray
+        smach.StateMachine.add('HandoverTray',
+                               HandoverObjectToOperatorState(action_dict,
+                                                             global_store),
+                               transitions={'SUCCESS':'DecideNextItem',
+                                            'FAILURE':'AskForTrayPlaceHelp'})
+        
+        # Decide next item
+        smach.StateMachine.add('DecideNextItem',
+                               DecideNextItemState(action_dict, global_store),
+                               transitions={'ITEM':'PickupItem',
+                                            'NONE_LEFT':'SetTakeTray'})
+        
+        # Pick up an item
+        smach.StateMachine.add('PickupItem',
+                               PickUpObjectState(action_dict, global_store),
+                               transitions={'SUCCESS':'PlaceInTray',
+                                            'FAILURE':'AskForHelpPickupItem'})
+        
+        # Ask for help picking up item
+        question = ("Can someone please help me pick this up? If so, can you" +
+                    "let me know when you're ready to hand it over?")
+        smach.StateMachine.add('AskForHelpPickupItem',
+                               SpeakAndListenState(action_dict, 
+                                                   global_store,
+                                                   question,
+                                                   READY,
+                                                   [],
+                                                   30),
+                               transitions={'SUCCESS':'ReceiveItem',
+                                            'FAILURE':'AskForHelpPickupItem',
+                                            'REPEAT_FAILURE':'TASK_FAILURE'})
+        
+        # Receive item
+        smach.StateMachine.add('ReceiveItem',
+                               ReceiveObjectFromOperatorState(action_dict,
+                                                              global_store),
+                               transitions={'SUCCESS':'PlaceInTray',
+                                            'FAILURE':'AskForHelpPickupItem'})
+        
+        # Place item in tray
+        smach.StateMachine.add('PlaceInTray',
+                               PlaceObjectRelativeState(action_dict, 
+                                                        global_store),
+                               transitions={'SUCCESS':'DecideNextItem',
+                                            'FAILURE':'AskForHelpItemPlace'})
+        
+        # Ask for help putting item in tray
+        question = ("Can someone please put this in the tray for me? If so, " +
+                    "can you let me know when you're ready to hand over?")
+        smach.StateMachine.add('AskForHelpItemPlace',
+                               SpeakAndListenState(action_dict, 
+                                                   global_store,
+                                                   question,
+                                                   READY,
+                                                   [],
+                                                   30),
+                               transitions={'SUCCESS':'HandoverItem',
+                                            'FAILURE':'AskForHelpItemPlace',
+                                            'REPEAT_FAILURE':'TASK_FAILURE'})
+        
+        # Handover item
+        smach.StateMachine.add('HandoverItem',
+                               HandoverObjectToOperatorState(action_dict,
+                                                             global_store),
+                               transitions={'SUCCESS':'DecideNextItem',
+                                            'FAILURE':'AskForHelpItemPlace'})
+        
+        # Set pick up to tray
+        smach.StateMachine.add('SetTakeTray',
+                               SetPickupState(action_dict, global_store,'tray'),
+                               transitions={'SUCCESS': 'TakeTray'})
+
+        # Take the tray and run!
+        smach.StateMachine.add('TakeTray',
+                               PickUpObjectState(action_dict, global_store),
+                               transitions={'SUCCESS':'SetNavBackToDishwasher',
+                                            'FAILURE':'AskForHelpTakeTray'})
+        
+        # Ask for help picking up tray
+        question = ("Can someone please help me pick up the tray? If so, " +
+                    "can you let me know when you're ready to hand it over?")
+        smach.StateMachine.add('AskForHelpTakeTray',
+                               SpeakAndListenState(action_dict, 
+                                                   global_store,
+                                                   question,
+                                                   READY,
+                                                   [],
+                                                   30),
+                               transitions={'SUCCESS':'ReceiveTakeTray',
+                                            'FAILURE':'AskForHelpTakeTray',
+                                            'REPEAT_FAILURE':'TASK_FAILURE'})
+        
+        # Receive tray
+        smach.StateMachine.add('ReceiveTakeTray',
+                               ReceiveObjectFromOperatorState(action_dict,
+                                                              global_store),
+                               transitions={'SUCCESS':'SetNavBackToDishwasher',
+                                            'FAILURE':'AskForHelpTakeTray'})
+        
+        # Set nav back to dishwasher
+        func = lambda : go_to_dishwasher(action_dict)
+        smach.StateMachine.add('SetNavBackToDishwasher',
+                               SetNavGoalState(action_dict, global_store, func),
+                               transitions={'SUCCESS':'NavBackToDishwasher'})
+        
+        # Nav back to dishwasher
+        smach.StateMachine.add('NavBackToDishwasher',
+                               NavigateState(action_dict, global_store),
+                               transitions={'SUCCESS':'PutTrayInDishwasher',
+                                            'FAILURE':'NavBackToDishwasher',
+                                            'REPEAT_FAILURE':'TASK_FAILURE'})
+        
+        # Try to put tray back in dishwasher
+        smach.StateMachine.add('PutTrayInDishwasher',
+                               PutObjectOnSurfaceState(action_dict, 
+                                                       global_store),
+                               transitions={'SUCCESS':'FinishSpeak',
+                                            'FAILURE':'AskForHelpTrayPlace'})
+        
+        # Ask for help putting tray in dishwasher
+        question = ("Can someone please put the tray in the dishwasher? If " +
+                    "so, can you let me know when you're ready to hand over?")
+        smach.StateMachine.add('AskForHelpTrayPlace',
+                               SpeakAndListenState(action_dict, 
+                                                   global_store,
+                                                   question,
+                                                   READY,
+                                                   [],
+                                                   30),
+                               transitions={'SUCCESS':'HandoverTrayToPlace',
+                                            'FAILURE':'AskForHelpTrayPlace',
+                                            'REPEAT_FAILURE':'TASK_FAILURE'})
+        
+        # Handover tray
+        smach.StateMachine.add('HandoverTrayToPlace',
+                               HandoverObjectToOperatorState(action_dict,
+                                                             global_store),
+                               transitions={'SUCCESS':'FinishSpeak',
+                                            'FAILURE':'AskForHelpTrayPlace'})
+        
+        # Finish speech
+        phrase = "Looks like my job here is done! See ya!"
+        smach.StateMachine.add('FinishSpeak',
+                               SpeakState(action_dict, global_store, phrase),
+                               transitions={'SUCCESS':'SetNavBackToStart',
+                                            'FAILURE':'SetNavBackToStart'})
+        
+        # Set nav back to location
+        func = lambda : global_store['stored_location']
+        smach.StateMachine.add('SetNavBackToStart',
+                               SetNavGoalState(action_dict, global_store, func),
+                               transitions={'SUCCESS':'NavBackToStart'})
+        
+        # Nav back to start
+        smach.StateMachine.add('NavBackToStart',
+                               NavigateState(action_dict, global_store),
+                               transitions={'SUCCESS':'TASK_SUCCESS',
+                                            'FAILURE':'NavBackToStart',
+                                            'TASK_FAILURE':'TASK_FAILURE'})
+
+    return sm
+
+if __name__ == '__main__':
+    action_dict = create_stage_2_clients(1)
+    sm = create_state_machine(action_dict)
+    sm.execute()
\ No newline at end of file
diff --git a/state_machines/src/hand_me_that.py b/state_machines/src/hand_me_that.py
new file mode 100644
index 0000000..2b3a7d0
--- /dev/null
+++ b/state_machines/src/hand_me_that.py
@@ -0,0 +1,140 @@
+#!/usr/bin/env python
+""" Code for the Hand Me That Task.
+
+This file contains the state machine code for the Hand Me That task.
+
+Author: Charlie Street
+Owner: Charlie Street
+"""
+
+import rospy
+import smach
+import actionlib
+import time
+
+from reusable_states import * # pylint: disable=unused-wildcard-import
+from set_up_clients import create_stage_2_clients
+from orion_actions.msg import SOMObservation, Relation
+
+
+class DetectPointedObjectsState(ActionServiceState):
+    """ Detects Pointed objects in vicinity of point. """
+    def __init__(self, action_dict, global_store):
+        outcomes = ['SUCCESS', 'FAILURE']
+        super(DetectPointedObjectsState, self).__init__(action_dict=action_dict,
+                                                        global_store=
+                                                        global_store,
+                                                        outcomes=outcomes)
+
+    def execute(self, userdata):
+        # TODO: Fill in!
+        # Needs to call action server, but should return array of things?
+        return self._outcomes[0] 
+
+
+class NextQuestionState(ActionServiceState):
+    """ Determines what the next action by the robot should be and does it. """
+    def __init__(self, action_dict, global_store):
+        """ The outcomes mean the following:
+            * CORRECT: Guessed the object and is correct
+            * INCORRECT: Guessed the object and is incorrect
+            * ANSWERED: Asked a question and got an answer
+            * GIVE_UP: Gave up with an object
+            * FINISHED: Finished all 5 objects
+        """
+        outcomes = ['CORRECT', 'INCORRECT', 'ANSWERED', 'GIVE_UP', 'FINISHED']
+        super(NextQuestionState, self).__init__(action_dict=action_dict,
+                                                global_store=global_store,
+                                                outcomes=outcomes)
+    
+    def execute(self, userdata):
+        # TODO: Fill in!
+        # Should do the following:
+        # Look at the list of pointed objects
+        # If only one possibility, say that this is the object
+        # Speak and listen, yes and no,
+        # If yes, correct, if no, incorrect
+        # If multiple possibilities, generate question and ask it
+        # Get yes or no response.
+        # If too many guesses, give up
+        # If all 5 objects done, return finished
+        return self._outcomes[0]
+
+
+
+def create_state_machine(action_dict):
+    """ This function creates and returns the state machine for this task. """
+
+    # Initialise global_store
+    global_store = {}
+    global_store['question'] = []
+
+    # Create the state machine
+    sm = smach.StateMachine(outcomes=['TASK_SUCCESS', 'TASK_FAILURE'])
+
+    with sm:
+
+        # Get name off the operator
+        question = "Hi, I'm Bam Bam, what is your name?"
+        smach.StateMachine.add('StartTalking',
+                               SpeakAndListenState(action_dict,
+                                                   global_store,
+                                                   question,
+                                                   NAMES,
+                                                   [],
+                                                   20),
+                               transitions={'SUCCESS': 'DetectOperator',
+                                            'FAILURE': 'StartTalking',
+                                            'REPEAT_FAILURE': 'TASK_FAILURE'})
+        
+        # Detect and memorise the operator
+        smach.StateMachine.add('DetectOperator',
+                               OperatorDetectState(action_dict, global_store),
+                               transitions={'SUCCESS': 'WhatDoYouWant',
+                                            'FAILURE': 'StartTalking'})
+        
+        # Ask what does the operator want
+        phrase = ("What do you need? I will follow you to what you want. " +
+                 "Please say Bam Bam when you have arrived")
+        smach.StateMachine.add('WhatDoYouWant',
+                               SpeakState(action_dict, global_store, phrase),
+                               transitions={'SUCCESS':'Follow',
+                                            'FAILURE':'Follow'})
+        
+        # Follow the operator until they arrive
+        smach.StateMachine.add('Follow',
+                               make_follow_hotword_state(action_dict,
+                                                         global_store),
+                               transitions={'SUCCESS': 'AskToPoint',
+                                            'FAILURE': 'Follow',
+                                            'REPEAT_FAILURE': 'TASK_FAILURE'})
+        
+        # Ask the operator to point to the object
+        phrase = "Please point at the object you need."
+        smach.StateMachine.add('AskToPoint',
+                               SpeakState(action_dict, global_store, phrase),
+                               transitions={'SUCCESS': 'DetectPointedObjects',
+                                            'FAILURE': 'DetectPointedObjects'})
+        
+        # Detect Pointed Objects
+        smach.StateMachine.add('DetectPointedObjects',
+                               DetectPointedObjectsState(action_dict, 
+                                                         global_store),
+                               transitions={'SUCCESS': 'NextQuestion',
+                                            'FAILURE': 'AskToPoint'})
+
+        smach.StateMachine.add('NextQuestion',
+                               NextQuestionState(action_dict, global_store),
+                               transitions={'CORRECT': 'WhatDoYouWant',
+                                            'INCORRECT': 'NextQuestion',
+                                            'ANSWERED': 'NextQuestion',
+                                            'GIVE_UP': 'WhatDoYouWant',
+                                            'FINISHED': 'TASK_SUCCESS'})
+    
+    return sm
+
+
+if __name__ == '__main__':
+    action_dict = create_stage_2_clients(4)
+    sm = create_state_machine(action_dict)
+    sm.execute()
\ No newline at end of file
diff --git a/state_machines/src/reusable_states.py b/state_machines/src/reusable_states.py
index 1414e1b..49ceac2 100644
--- a/state_machines/src/reusable_states.py
+++ b/state_machines/src/reusable_states.py
@@ -22,10 +22,12 @@
             PutObjectOnSurfaceGoal, CheckForBarDrinksGoal, SpeakAndListenGoal, \
                 HotwordListenGoal, GetPointedObjectGoal, PickUpObjectGoal, \
                     NavigateGoal, FollowGoal, OpenBinLidGoal, OpenDrawerGoal, \
-                        PlaceObjectRelativeGoal, PourIntoGoal, PointToObjectGoal
+                        PlaceObjectRelativeGoal, PourIntoGoal, \
+                            PointToObjectGoal, OpenFurnitureDoorGoal
 
-from orion_actions.msg import SOMObservation
-from geometry_msgs.msg import Pose
+from orion_actions.msg import DetectionArray, FaceDetectionArray
+from orion_actions.msg import SOMObservation, Relation
+from geometry_msgs.msg import Pose, PoseStamped
 from move_base_msgs.msg import MoveBaseGoal
 from actionlib_msgs.msg import GoalStatus
 from tmc_msgs.msg import TalkRequestGoal, Voice
@@ -45,6 +47,8 @@
 
 RELATIONS = ['left', 'right', 'above', 'below', 'front', 'behind', 'near']
 OBJECTS = ['apple', 'banana', 'cereal', 'bowl', 'cloth'] # TODO: Fill in
+FRUITS = ['apple', 'banana', 'orange', 'mango', 'strawberry', 'kiwi', 'plum',
+          'nectarine'] # TODO: Fill in!
 
 class ActionServiceState(smach.State):
     """ A subclass of Smach States which gives access to actions/services.
@@ -206,6 +210,26 @@ def execute(self, userdata):
             return self._outcomes[0]
 
 
+class OpenFurnitureDoorState(ActionServiceState):
+    """ Smach state to open furniture door. """
+    def __init__(self, action_dict, global_store):
+        outcomes = ['SUCCESS', 'FAILURE']
+        super(OpenFurnitureDoorState, self).__init__(action_dict=action_dict,
+                                                     global_store=global_store,
+                                                     outcomes=outcomes)
+    
+    def execute(self, userdata):
+        goal = OpenFurnitureDoorGoal()
+        goal.goal_tf = self.global_store['furniture_door']
+        self.action_dict['OpenFurnitureDoor'].send_goal(goal)
+        self.action_dict['OpenFurnitureDoor'].wait_for_result()
+
+        if self.action_dict['OpenFurnitureDoor'].get_result().result:
+            return self._outcomes[0]
+        else:
+            return self._outcomes[1]
+
+
 class HandoverObjectToOperatorState(ActionServiceState):
     """ Smach state for handing a grasped object to an operator.
 
@@ -341,9 +365,9 @@ def __init__(self, action_dict, global_store):
         
     def execute(self, userdata):
         # Wait for one message on topic and then set as the location
-        pose = rospy.wait_for_message('/robot_pose', Pose) # TODO: Weird?
-        self.global_store['stored_location'] = pose
-
+        pose = rospy.wait_for_message('/global_pose', PoseStamped)
+        self.global_store['stored_location'] = pose.pose
+        rospy.loginfo(pose)
         return self._outcomes[0]
 
 
@@ -397,8 +421,7 @@ def execute(self, userdata):
         result = self.action_dict['SpeakAndListen'].get_result()
         if result.succeeded:
             self.global_store['last_response'] = result.answer
-
-            del self.global_store['speak_listen_failure']
+            self.global_store['speak_listen_failure'] = 0
             return self._outcomes[0]
         else:
             self.global_store['speak_listen_failure'] += 1
@@ -425,13 +448,23 @@ def execute(self, userdata):
         hotword_goal = HotwordListenGoal()
         hotword_goal.timeout = self.timeout
         self.action_dict['HotwordListen'].send_goal(hotword_goal)
-        self.action_dict['HotwordListen'].wait_for_result()
+
+        goal_finished = False
+        while not self.preempt_requested() and not goal_finished:
+            timeout = rospy.Duration(secs=2)
+            goal_finished = \
+                self.action_dict['HotwordListen'].wait_for_result(timeout=
+                                                                  timeout)
         
-        result = self.action_dict['HotwordListen'].get_result()
+        if goal_finished:
+            result = self.action_dict['HotwordListen'].get_result()
 
-        if result.succeeded:
-            return self._outcomes[0]
+            if result.succeeded:
+                return self._outcomes[0]
+            else:
+                return self._outcomes[1]
         else:
+            self.action_dict['HotwordListen'].cancel_all_goals()
             return self._outcomes[1]
 
 
@@ -458,12 +491,12 @@ def execute(self, userdata):
         pickup_goal.goal_tf = obj""" # TODO: Change later when we can do pointed objects
 
         pickup_goal = PickUpObjectGoal()
-        puckup_goal.goal_tf = 'plant'
+        pickup_goal.goal_tf = 'potted plant'
 
         self.action_dict['PickUpObject'].send_goal(pickup_goal)
         self.action_dict['PickUpObject'].wait_for_result()
 
-        result = self.action_dict['PickUpObject'].get_result().goal_complete
+        result = self.action_dict['PickUpObject'].get_result().result
 
         if result:
             return self._outcomes[0]
@@ -485,11 +518,13 @@ def __init__(self, action_dict, global_store):
                                                   outcomes=outcomes)
     
     def execute(self, userdata):
+        failed = 0
         operator = SOMObservation()
         operator.type = 'person'
         operator.task_role = 'operator'
         # TODO: Pose observation of person
-        operator.robot_pose = rospy.wait_for_message('/robot_pose', Pose) # TODO: pose
+        operator.robot_pose = rospy.wait_for_message('/global_pose', 
+                                                     PoseStamped).pose
         # TODO: Room name (what room are we in)
         
         for name in NAMES:
@@ -497,9 +532,30 @@ def execute(self, userdata):
                 operator.name = name
                 break
 
-        # TODO: Age
-        # TODO: Gender
-        # TODO: Shirt Colour 
+        try:
+            person_msg = rospy.wait_for_message('/vision/bbox_detections', 
+                                                DetectionArray, timeout=5)
+            for detection in person_msg.detections:
+                if 'person' in detection.label.name:
+                    operator.shirt_colour = detection.colour
+                    break
+        except:
+            failed += 1
+        
+        try:
+            face_msg = rospy.wait_for_message('/vision/face_bbox_detections', 
+                                              FaceDetectionArray, 
+                                              timeout=5)
+            
+            face = face_msg.detections[0]
+            operator.age = face.age
+            operator.gender = face.gender
+        except:
+            failed += 1
+
+        if failed >= 3:
+            return self._outcomes[1]
+
         result = self.action_dict['SOMObserve'](operator)
         if not result.result:
             return self._outcomes[1]
@@ -519,18 +575,42 @@ def __init__(self, action_dict, global_store):
                                                   outcomes=outcomes)
     
     def execute(self, userdata):
+        failed = 0
         person = SOMObservation()
         person.type = 'person'
         # TODO: Pose observation of person
-        person.robot_pose = rospy.wait_for_message('/robot_pose', Pose) # TODO: Fix
+        person.robot_pose = rospy.wait_for_message('/global_pose', 
+                                                   PoseStamped).pose
         # TODO: Room name (what room are we in)
         for name in NAMES:
             if name in self.global_store['last_response']:
                 person.name = name
                 break
-        # TODO: Age
-        # TODO: Gender
-        # TODO: Shirt colour
+        
+        try:
+            person_msg = rospy.wait_for_message('/vision/bbox_detections', 
+                                                DetectionArray, timeout=5)
+            for detection in person_msg.detections:
+                if 'person' in detection.label.name:
+                    person.shirt_colour = detection.colour
+                    break
+        except:
+            failed += 1
+        
+        try:
+            face_msg = rospy.wait_for_message('/vision/face_bbox_detections', 
+                                              FaceDetectionArray, 
+                                              timeout=5)
+            
+            face = face_msg.detections[0]
+            person.age = face.age
+            person.gender = face.gender
+        except:
+            failed += 1
+
+        if failed >= 3:
+            return self._outcomes[1]
+
         result = self.action_dict['SOMObserve'](person)
         if not result.result:
             return self._outcomes[1]
@@ -554,13 +634,23 @@ def __init__(self, action_dict, global_store):
     
     def execute(self, userdata):
         follow_goal = FollowGoal()
-        follow_goal.object_name = 'person' # TODO: Fix later!
+
+        obs = SOMObservation()
+        obs.type = 'person'
+        obs.task_role = 'operator'
+        matches = self.action_dict['SOMQuery'](obs,Relation(),SOMObservation())
+        op = matches[0].obj1
+        colour = op.shirt_colour
+
+        follow_goal.object_name = 'person_' + colour
         self.action_dict['Follow'].send_goal(follow_goal)
 
         current_result = True
-        while not self.preempt_requested() and current_result != False:
-            time.sleep(1)
-            current_result = self.action_dict['Follow'].get_result().succeeded
+        while not self.preempt_requested():
+            finished = self.action_dict['Follow'].wait_for_result(timeout=rospy.Duration(secs=1))
+            if finished:
+                current_result = self.action_dict['Follow'].get_result().succeeded
+                break
         
         self.action_dict['Follow'].cancel_all_goals()
     
@@ -570,15 +660,19 @@ def execute(self, userdata):
                 return self._outcomes[2]
             return self._outcomes[1]
         else:
-            del self.global_store['follow_failure']
+            self.global_store['follow_failure'] = 0
             return self._outcomes[0]
 
 
-def follow_child_cb(outcome_map):
+def follow_child_cb(outcome_map, global_store):
     """Executed whenever a child in the concurrent state is terminated."""
     if outcome_map['Hotword'] == 'SUCCESS':
         return True
     if outcome_map['Hotword'] == 'FAILURE':
+        if 'follow_failure' in global_store:
+            global_store['follow_failure'] += 1
+        else:
+            global_store['follow_faulure'] = 1
         return True
     if outcome_map['Follow'] == 'FAILURE':
         return True
@@ -587,11 +681,13 @@ def follow_child_cb(outcome_map):
     
     return False
 
-def follow_out_cb(outcome_map):
+def follow_out_cb(outcome_map, global_store):
     if outcome_map['Hotword'] == 'SUCCESS':
         return 'SUCCESS'
     elif outcome_map['Follow'] == 'REPEAT_FAILURE':
         return 'REPEAT_FAILURE'
+    elif ('follow_failure' in global_store and global_store['follow_failure'] >= FAILURE_THRESHOLD):
+        return 'REPEAT_FAILURE'
     else:
         return 'FAILURE'
 
@@ -601,16 +697,17 @@ def make_follow_hotword_state(action_dict, global_store):
     This concurrent state machine follows someone while waiting for a hot
     word to be spoken.
     """
+    # TODO: Fix, this still isn't quite right
     con = Concurrence(outcomes=['SUCCESS', 'FAILURE', 'REPEAT_FAILURE'],
                       default_outcome='FAILURE',
-                      child_termination_cb=follow_child_cb,
-                      outcome_cb=follow_out_cb)
+                      child_termination_cb=(lambda om: (follow_child_cb(om, global_store))),
+                      outcome_cb=(lambda om: follow_out_cb(om, global_store)))
     
     with con:
         Concurrence.add('Follow', FollowState(action_dict, global_store))
         Concurrence.add('Hotword', HotwordListenState(action_dict, 
                                                       global_store,
-                                                      120))
+                                                      180))
     
     return con
 # --- End of follow code
@@ -637,14 +734,16 @@ def execute(self, userdata):
         goal.target_pose.header.frame_id = "map"
         goal.target_pose.header.stamp = rospy.Time.now()
         goal.target_pose.pose = pose
-
+        #goal.target_pose.pose.orientation.z = 0.0
+        rospy.loginfo(goal.target_pose.pose)
         self.action_dict['Navigate'].send_goal(goal)
         self.action_dict['Navigate'].wait_for_result()
 
-        result = self.action_dict['Navigate'].get_result().status
+        status = self.action_dict['Navigate'].get_state()
+        rospy.loginfo('status = ' + str(status))
 
-        if result == GoalStatus.SUCCEEDED:
-            del self.global_store['nav_failure']
+        if status == GoalStatus.SUCCEEDED:
+            self.global_store['nav_failure'] = 0
             return self._outcomes[0]
         else:
             self.global_store['nav_failure'] += 1
diff --git a/state_machines/src/serve_the_breakfast.py b/state_machines/src/serve_the_breakfast.py
index 4d8b8bf..7d02f02 100644
--- a/state_machines/src/serve_the_breakfast.py
+++ b/state_machines/src/serve_the_breakfast.py
@@ -97,8 +97,8 @@ def create_state_machine(action_dict):
         # Wait for door to open
         smach.StateMachine.add('WaitForDoor',
                                CheckDoorIsOpenState(action_dict, global_store),
-                               transitions={'SUCCESS':'SetNavToKitchen',
-                                            'FAILURE':'WaitForDoor'})
+                               transitions={'OPEN':'SetNavToKitchen',
+                                            'CLOSED':'WaitForDoor'})
                     
         # Set nav goal to kitchen
         func = lambda : None # TODO: fix!
diff --git a/state_machines/src/serving_drinks.py b/state_machines/src/serving_drinks.py
index 8c41674..5ca195a 100644
--- a/state_machines/src/serving_drinks.py
+++ b/state_machines/src/serving_drinks.py
@@ -77,6 +77,7 @@ def create_state_machine(action_dict):
     """ Function creates and returns the state machine for this task. """
 
     global_store = {}
+    global_store['people_found'] = []
 
 
     sm = smach.StateMachine(outcomes=['TASK_SUCCESS', 'TASK_FAILURE'])
diff --git a/state_machines/src/set_the_table.py b/state_machines/src/set_the_table.py
new file mode 100644
index 0000000..528a0ec
--- /dev/null
+++ b/state_machines/src/set_the_table.py
@@ -0,0 +1,300 @@
+#!/usr/bin/env python
+""" Code for the Set The Table task.
+
+This file contains the state machine code for the Set The Table task.
+
+Author: Charlie Street
+Owner: Charlie Street
+"""
+
+import rospy
+import smach
+import actionlib
+import time
+
+from reusable_states import * # pylint: disable=unused-wildcard-import
+from set_up_clients import create_stage_2_clients
+from orion_actions.msg import SOMObservation, Relation
+
+def go_to_cupboard(action_dict):
+    """ Gets location of cupboard. """
+    obj1 = SOMObservation()
+    obj1.type = 'set_the_table_point_of_interest_cupboard'
+
+    return get_location_of_object(action_dict, obj1, 
+                                  Relation(), SOMObservation())
+
+
+def go_to_table(action_dict):
+    """ Gets location of table. """
+    obj1 = SOMObservation()
+    obj1.type = 'set_the_table_point_of_interest_table'
+
+    return get_location_of_object(action_dict, obj1, 
+                                  Relation(), SOMObservation())
+
+
+class ObserveCupboardState(ActionServiceState):
+    """ State observes the cupboard and what is in it. """
+    def __init__(self, action_dict, global_store):
+        outcomes = ['SUCCESS']
+        super(ObserveCupboardState, self).__init__(action_dict=action_dict,
+                                                   global_store=global_store,
+                                                   outcomes=outcomes)
+        
+    def execute(self, userdata):
+        # TODO: Fill in!
+        # Should set a list of observed items, make sure placemat, then dish!
+        # Should also populate semantic map
+        return self._outcomes[0]
+
+
+class ChooseItemState(ActionServiceState):
+    """ State chooses te next item to pick up. """
+    def __init__(self, action_dict, global_store):
+        outcomes = ['PLACEMAT', 'ITEM', 'NONE_LEFT']
+        super(ChooseItemState, self).__init__(action_dict=action_dict,
+                                              global_store=global_store,
+                                              outcomes=outcomes)
+    
+    def execute(self, userdata):
+        # TODO: Fill in!
+        # Should set pick_up, rel_pos appropriately etc.
+        # Remember, everything relative to bowl, except bowl
+        # bowl is relative to placemat!
+        return self._outcomes[2]
+
+
+def create_state_machine(action_dict):
+    """ This function creates and returns the state machine for this task. """
+
+    # Initialise global_store
+    global_store = {}
+    global_store['to_place'] = []
+    global_store['drawer_handle'] = 'cupboard' # TODO: Sort out!
+    global_store['next_item'] = 0
+    global_store['furniture_door'] = 'cupboard'
+
+    # Create the state machine
+    sm = smach.StateMachine(outcomes=['TASK_SUCCESS', 'TASK_FAILURE'])
+
+    with sm:
+        
+        # Get the robot to start talking
+        phrase = "Hi, I'm Bam Bam and I'm here to set the table!"
+        smach.StateMachine.add('StartTalking',
+                               SpeakState(action_dict, global_store, phrase),
+                               transitions={'SUCCESS':'SetNavToCupboard',
+                                            'FAILURE':'SetNavToCupboard'})
+                    
+        # Set nav to cupboard
+        func = lambda: go_to_cupboard(action_dict)
+        smach.StateMachine.add('SetNavToCupboard',
+                               SetNavGoalState(action_dict, global_store, func),
+                               transitions={'SUCCESS':'NavToCupboard'})
+        
+        # Navigate to cupboard
+        smach.StateMachine.add('NavToCupboard',
+                               NavigateState(action_dict, global_store),
+                               transitions={'SUCCESS':'OpenCupboard',
+                                            'FAILURE':'NavToCupboard',
+                                            'REPEAT_FAILURE':'TASK_FAILURE'})
+        
+        # Open the cupboard
+        smach.StateMachine.add('OpenCupboard',
+                               OpenFurnitureDoorState(action_dict, 
+                                                      global_store),
+                               transitions={'SUCCESS':'ObserveCupboard',
+                                            'FAILURE':'AskForHelpCupboard'})
+        
+        # If can't open cupboard
+        question = ("Can someone open the cupboard for me and " +
+                   "say my name when they have?")
+        smach.StateMachine.add('AskForHelpCupboard',
+                               SpeakState(action_dict, global_store, question),
+                               transitions={'SUCCESS':'WaitTillOpen',
+                                            'FAILURE':'WaitTillOpen'})
+        
+        # Wait till name said
+        smach.StateMachine.add('WaitTillOpen',
+                               HotwordListenState(action_dict, global_store,30),
+                               transitions={'SUCCESS':'ObserveCupboard',
+                                            'FAILURE':'TASK_FAILURE'})
+        
+        # Observe Cupboard
+        smach.StateMachine.add('ObserveCupboard',
+                               ObserveCupboardState(action_dict, global_store),
+                               transitions={'SUCCESS':'ChooseNextItem'})
+        
+        # Choose Item
+        smach.StateMachine.add('ChooseNextItem',
+                               ChooseItemState(action_dict, global_store),
+                               transitions={'PLACEMAT':'PickUpPlacemat',
+                                            'ITEM':'PickUpItem',
+                                            'NONE_LEFT':'Finish'})
+        
+        # If no items left, finish
+        phrase = "Your table is set for dinner. Bon appetite!"
+        smach.StateMachine.add('Finish',
+                               SpeakState(action_dict, global_store, phrase),
+                               transitions={'SUCCESS':'TASK_SUCCESS',
+                                            'FAILURE':'TASK_SUCCESS'})
+
+        # Pick up the placemat
+        smach.StateMachine.add('PickUpPlacemat',
+                               PickUpObjectState(action_dict, global_store),
+                               transitions={'SUCCESS':'SetNavToTable',
+                                            'FAILURE':'AskForPlacematHelp'})
+        
+        # If failed, ask for help
+        question = ("I can't pick up the placemat. Can you hand it to me " +
+                   "please? Let me know when you're ready to hand over!")
+        smach.StateMachine.add('AskForPlacematHelp',
+                               SpeakAndListenState(action_dict,
+                                                   global_store,
+                                                   question,
+                                                   READY,
+                                                   [],
+                                                   30),
+                               transitions={'SUCCESS':'ReceivePlacemat',
+                                            'FAILURE':'AskForPlacematHelp',
+                                            'REPEAT_FAILURE':'TASK_FAILURE'})
+        
+        # Receive Placemat
+        smach.StateMachine.add('ReceivePlacemat',
+                               ReceiveObjectFromOperatorState(action_dict,
+                                                              global_store),
+                               transitions={'SUCCESS':'SetNavToTable',
+                                            'FAILURE':'AskForPlacematHelp'})
+        
+        # Set nav To table
+        func = lambda : go_to_table(action_dict)
+        smach.StateMachine.add('SetNavToTable',
+                               SetNavGoalState(action_dict, global_store, func),
+                               transitions={'SUCCESS':'NavToTable'})
+        
+        # Navigate to table
+        smach.StateMachine.add('NavToTable',
+                               NavigateState(action_dict, global_store),
+                               transitions={'SUCCESS':'PlacePlacemat',
+                                            'FAILURE':'NavToTable',
+                                            'REPEAT_FAILURE':'TASK_FAILLURE'})
+        
+        # Place down placemat
+        smach.StateMachine.add('PlacePlacemat',
+                               PutObjectOnSurfaceState(action_dict, 
+                                                       global_store),
+                               transitions={'SUCCESS':'SetNavBackToCupboard',
+                                            'FAILURE':'AskForPlacingMatHelp'})
+        
+        # Get help placing place mat
+        question = ("I'm struggling to put this place mat down. Can you take " +
+                   "from me and put it down on the table for me? Let me know " +
+                   "when you're ready for me to hand it to you.")
+        smach.StateMachine.add('AskForPlacingMatHelp',
+                               SpeakAndListenState(action_dict,
+                                                   global_store,
+                                                   question,
+                                                   READY,
+                                                   [],
+                                                   20),
+                               transitions={'SUCCESS':'GivePlacemat',
+                                            'FAILURE':'AskForPlacingMatHelp',
+                                            'REPEAT_FAILURE':'TASK_FAILURE'})
+        
+        # Give placemat to person
+        smach.StateMachine.add('GivePlacemat',
+                               HandoverObjectToOperatorState(action_dict,
+                                                             global_store),
+                               transitions={'SUCCESS':'SetNavBackToCupboard',
+                                            'FAILURE':'AskForPlacingMatHelp'})
+        
+        # Go back to the cupboard
+        func = lambda : go_to_cupboard(action_dict)
+        smach.StateMachine.add('SetNavBackToCupboard',
+                               SetNavGoalState(action_dict, global_store, func),
+                               transitions={'SUCCESS':'NavBackToCupboard'})
+        
+        # Nav back to cupboard
+        smach.StateMachine.add('NavBackToCupboard',
+                               NavigateState(action_dict, global_store),
+                               transitions={'SUCCESS':'ChooseNextItem',
+                                            'FAILURE':'NavBackToCupboard',
+                                            'REPEAT_FAILURE':'TASK_FAILURE'})
+
+
+        # Pick up object
+        smach.StateMachine.add('PickUpItem',
+                               PickUpObjectState(action_dict, global_store),
+                               transitions={'SUCCESS':'SetNavToTable',
+                                            'FAILURE':'AskForItemHelp'})
+        
+        # If failed, ask for help
+        question = ("I can't pick up this item. Can you hand it to me " +
+                   "please? Let me know when you're ready to hand over!")
+        smach.StateMachine.add('AskForItemHelp',
+                               SpeakAndListenState(action_dict,
+                                                   global_store,
+                                                   question,
+                                                   READY,
+                                                   [],
+                                                   30),
+                               transitions={'SUCCESS':'ReceiveItem',
+                                            'FAILURE':'AskForItemHelp',
+                                            'REPEAT_FAILURE':'TASK_FAILURE'})
+        
+        # Receive item
+        smach.StateMachine.add('ReceiveItem',
+                               ReceiveObjectFromOperatorState(action_dict,
+                                                              global_store),
+                               transitions={'SUCCESS':'SetNavToTableItem',
+                                            'FAILURE':'AskForItemHelp'})
+
+         # Set nav To table
+        func = lambda : go_to_table(action_dict)
+        smach.StateMachine.add('SetNavToTableItem',
+                               SetNavGoalState(action_dict, global_store, func),
+                               transitions={'SUCCESS':'NavToTableItem'})
+        
+        # Navigate to table
+        smach.StateMachine.add('NavToTableItem',
+                               NavigateState(action_dict, global_store),
+                               transitions={'SUCCESS':'PlaceItem',
+                                            'FAILURE':'NavToTableItem',
+                                            'REPEAT_FAILURE':'TASK_FAILLURE'})
+
+        # Place down placemat
+        smach.StateMachine.add('PlaceItem',
+                               PlaceObjectRelativeState(action_dict, 
+                                                       global_store),
+                               transitions={'SUCCESS':'SetNavBackToCupboard',
+                                            'FAILURE':'AskForPlacingItemHelp'})
+        
+        # Get help placing place mat
+        question = ("I'm struggling to put this item down. Can you take " +
+                   "from me and put it down in the right place for me? " +
+                   "Let me know when you're ready for me to hand it to you.")
+        smach.StateMachine.add('AskForPlacingItemHelp',
+                               SpeakAndListenState(action_dict,
+                                                   global_store,
+                                                   question,
+                                                   READY,
+                                                   [],
+                                                   20),
+                               transitions={'SUCCESS':'GiveItem',
+                                            'FAILURE':'AskForPlacingItemHelp',
+                                            'REPEAT_FAILURE':'TASK_FAILURE'})
+        
+        # Give itemto person
+        smach.StateMachine.add('GiveItem',
+                               HandoverObjectToOperatorState(action_dict,
+                                                             global_store),
+                               transitions={'SUCCESS':'SetNavBackToCupboard',
+                                            'FAILURE':'AskForPlacingItemHelp'})
+
+    return sm
+
+if __name__ == '__main__':
+    action_dict = create_stage_2_clients(5)
+    sm = create_state_machine(action_dict)
+    sm.execute()
\ No newline at end of file
diff --git a/state_machines/src/set_up_clients.py b/state_machines/src/set_up_clients.py
index 5177765..18c281b 100644
--- a/state_machines/src/set_up_clients.py
+++ b/state_machines/src/set_up_clients.py
@@ -49,7 +49,7 @@ def create_stage_1_clients(task_number):
     rospy.loginfo('SOM service proxies set up...')
     # Now add common action clients
     rospy.loginfo('Setting up Move Base client...')
-    action_dict['Navigate'] = actionlib.SimpleActionClient('move_base', 
+    action_dict['Navigate'] = actionlib.SimpleActionClient('/move_base/move', 
                                                            MoveBaseAction)
     action_dict['Navigate'].wait_for_server() # TODO: Change if necessary
     rospy.loginfo('Move Base ready...')
@@ -148,7 +148,6 @@ def create_stage_1_clients(task_number):
         action_dict['IsDoorOpen'] = actionlib.SimpleActionClient('is_door_open',
                                                                IsDoorOpenAction)
         action_dict['IsDoorOpen'].wait_for_server()
-        # TODO: Loads more to add here later
 
     elif task_number == 6: # Receptionist
         action_dict['IsDoorOpen'] = actionlib.SimpleActionClient('is_door_open',
@@ -259,7 +258,21 @@ def create_stage_1_clients(task_number):
 
 def create_stage_2_clients(task_number):
     """ Same as create_stage_1_clients but for stage 2. """
-    return {}
+    if task_number == 1: # Clean the table
+        # TODO: Sort out
+        return {}
+    elif task_number == 4:
+        # TODO: Sort out
+        return {}
+    elif task_number == 5:
+        # TODO: Sort out
+        return {}
+    elif task_number == 7:
+        # TODO: Sort out
+        return {}
+    else:
+        raise Exception("Invalid Task Number Passed In!")
+
 
 
 def create_final_clients(task_number):
diff --git a/state_machines/src/smoothie_chef.py b/state_machines/src/smoothie_chef.py
new file mode 100644
index 0000000..a6397bc
--- /dev/null
+++ b/state_machines/src/smoothie_chef.py
@@ -0,0 +1,363 @@
+#!/usr/bin/env python
+""" Code for the Smoothie Chef task.
+
+This file contains the state machine code for the Smoothie Chef task.
+
+Author: Charlie Street
+Owner: Charlie Street
+"""
+
+import rospy
+import smach
+import actionlib
+import time
+
+from reusable_states import * # pylint: disable=unused-wildcard-import
+from set_up_clients import create_stage_2_clients
+from orion_actions.msg import SOMObservation, Relation
+
+def go_to_kitchen_counter(action_dict):
+    """ Gets location of kitchen counter. """
+    obj1 = SOMObservation()
+    obj1.type = 'smoothie_chef_point_of_interest'
+
+    return get_location_of_object(action_dict, obj1, 
+                                  Relation(), SOMObservation())
+
+
+class MemoriseRecipeState(ActionServiceState):
+    """ This state watches the operator make a smoothie. 
+        Returns the sequence of fruits."""
+
+    def __init__(self, action_dict, global_store):
+        outcomes = ['SUCCESS']
+        super(MemoriseRecipeState, self).__init__(action_dict=action_dict,
+                                                  global_store=global_store,
+                                                  outcomes=outcomes)
+        
+    def execute(self, userdata):
+
+        # TODO : Call action server and get response
+        return self._outcomes[0]
+
+
+class ChooseIngredientState(ActionServiceState):
+    """ This state chooses the next ingredient to add to the smoothie. """
+
+    def __init__(self, action_dict, global_store):
+        outcomes = ['FRUIT','SUGAR','MILK','NO_MORE','DONT_KNOW']
+        super(ChooseIngredientState, self).__init__(action_dict=action_dict,
+                                                    global_store=global_store,
+                                                    outcomes=outcomes)
+        self.asked_for_help = False
+        self.global_store['rel_pos'] = ('blender', 0.0, 0.0, 0.2) # TODO: Check
+        self.global_store['pour_into'] = 'blender'
+    
+    def execute(self, userdata):
+        
+        if self.global_store['next_item'] < 3:
+            fruits = self.global_store['recipe']
+            if len(fruits) != 3: # Bad information
+                if not self.asked_for_help:
+                    return self._outcomes[4]
+                else:
+                    self.global_store['pick_up'] = \
+                        self.global_store['last_response']
+                    self.global_store['next_item'] += 1
+                    return self._outcomes[0]
+            else:
+                self.global_store['pick_up'] = \
+                    fruits[self.global_store['next_item']]
+                self.global_store['next_item'] += 1
+                return self._outcomes[0]
+
+        elif self.global_store['next_item'] == 3: # sugar
+            self.global_store['pick_up'] = 'spoon'
+            self.global_store['next_item'] += 1
+            return self._outcomes[1]
+
+        elif self.global_store['next_item'] == 4: # milk
+            self.global_store['pick_up'] = 'milk'
+            self.global_store['next_item'] += 1
+            return self._outcomes[2]
+        else: # Done
+            return self._outcomes[3]
+
+
+class PourSugarState(ActionServiceState):
+
+    def __init__(self, action_dict, global_store):
+        outcomes = ['SUCCESS', 'FAILURE']
+        super(PourSugarState, self).__init__(action_dict=action_dict,
+                                             global_store=global_store,
+                                             outcomes=outcomes)
+    
+    def execute(self, userdata):
+        # TODO: Fill in!
+        return self._outcomes[1]
+
+
+def create_state_machine(action_dict):
+    """ This function creates and returns the state machine for this task. """
+
+    # Initialise global store
+    global_store = {}
+    global_store['next_item'] = 0
+    global_store['recipe'] = []
+
+    # Create the state machine
+    sm = smach.StateMachine(outcomes=['TASK_SUCCESS', 'TASK_FAILURE'])
+
+    with sm:
+        
+        # Get the robot to start talking
+        phrase = "Hi, I'm Bam Bam, lets make some smoothies!"
+        smach.StateMachine.add('StartTalking',
+                               SpeakState(action_dict, global_store, phrase),
+                               transitions={'OPEN':'WaitForDoor',
+                                            'CLOSED':'WaitForDoor'})
+
+        # Wait for the door to open
+        smach.StateMachine.add('WaitForDoor',
+                               CheckDoorIsOpenState(action_dict, global_store),
+                               transitions={'OPEN':'SetNavToKitchen',
+                                            'CLOSED':'WaitForDoor'})
+        
+        # Set nav to kitchen counter 
+        func = lambda : go_to_kitchen_counter(action_dict)
+        smach.StateMachine.add('SetNavToKitchen',
+                               SetNavGoalState(action_dict, global_store, func),
+                               transitions={'SUCCESS':'NavToKitchen'})
+
+        # Navigate to kitchen counter
+        smach.StateMachine.add('NavToKitchen',
+                               NavigateState(action_dict, global_store),
+                               transitions={'SUCCESS':'AskForDemo',
+                                            'FAILURE':'NavToKitchen',
+                                            'REPEAT_FAILURE':'TASK_FAILURE'})
+
+        # Ask for a demo of the smoothie recipe
+        phrase = ("Hello, please could you show me how to make a smoothie? " +
+                 "Please say my name, Bam Bam, when you're finished.")
+        smach.StateMachine.add('AskForDemo',
+                               SpeakState(action_dict, global_store, phrase),
+                               transitions={'SUCCESS':'Demo',
+                                            'FAILURE':'Demo'})
+            
+        # Watch the demo
+        smach.StateMachine.add('Demo',
+                               MemoriseRecipeState(action_dict, global_store),
+                               transitions={'SUCCESS':'WaitForName'})
+        
+        # wait for name
+        smach.StateMachine.add('WaitForName',
+                               HotwordListenState(action_dict, 
+                                                  global_store, 120),
+                               transitions={'SUCCESS': 'ChooseIngredient',
+                                            'FAILURE': 'WaitForName'})
+        
+        # Choose next ingredient
+        smach.StateMachine.add('ChooseIngredient',
+                               ChooseIngredientState(action_dict, global_store),
+                               transitions={'FRUIT': 'PickUpFruit',
+                                            'SUGAR': 'PickUpSpoon',
+                                            'MILK': 'PickUpMilk',
+                                            'NO_MORE': 'SpeakFinish',
+                                            'DONT_KNOW':'AskForIngredientHelp'})
+
+        # If no more items
+        phrase = ("Looks like I've just made a delicious smoothie! " + 
+                 "Thanks for showing me how, good bye!")
+        smach.StateMachine.add('SpeakFinish',
+                               SpeakState(action_dict, global_store, phrase),
+                               transitions={'SUCCESS':'TASK_SUCCESS',
+                                            'FAILURE':'TASK_SUCCESS'})
+        
+        # If don't know
+        phrase = ("I'm not sure what I should be adding next, could you tell " +
+                  "what I should add please?")
+        smach.StateMachine.add('AskForIngredientHelp',
+                               SpeakAndListenState(action_dict,
+                                                   global_store,
+                                                   phrase,
+                                                   FRUITS + ['sugar', 'milk'],
+                                                   [],
+                                                   20),
+                               transitions={'SUCCESS':'ChooseIngredient',
+                                            'FAILURE':'AskForIngredientHelp',
+                                            'REPEAT_FAILURE':'TASK_FAILURE'})
+        
+        # If fruit, pick up
+        smach.StateMachine.add('PickUpFruit',
+                               PickUpObjectState(action_dict, global_store),
+                               transitions={'SUCCESS':'PlaceFruitInBlender',
+                                            'FAILURE':'AskForFruitPickHelp'})
+        
+        # get help with fruit
+        question = ("I can't seem to pick up this fruit, could you please help"+
+                   " me and let me know when you're ready to hand it over?")
+        smach.StateMachine.add('AskForFruitPickHelp',
+                               SpeakAndListenState(action_dict,
+                                                   global_store,
+                                                   question,
+                                                   READY,
+                                                   [],
+                                                   20),
+                               transitions={'SUCCESS':'ReceiveFruit',
+                                            'FAILURE':'AskForFruitPickHelp',
+                                            'REPEAT_FAILURE':'TASK_FAILURE'})
+        
+        # receive fruit
+        smach.StateMachine.add('ReceiveFruit',
+                               ReceiveObjectFromOperatorState(action_dict,
+                                                              global_store),
+                               transitions={'SUCCESS':'PlaceFruitInBlender',
+                                            'FAILURE':'AskForFruitPickHelp'})
+                                        
+
+        # Place Fruit In Blender
+        smach.StateMachine.add('PlaceFruitInBlender',
+                               PlaceObjectRelativeState(action_dict, 
+                                                        global_store),
+                               transitions={'SUCCESS':'ChooseIngredient',
+                                            'FAILURE':'AskForFruitPlaceHelp'})
+        
+        # Ask for fruit based help
+        question = ("I can't get this fruit in the blender. Can you help me " +
+                   "please and let me know when you're ready to receive " +
+                   "the fruit?")
+        smach.StateMachine.add('AskForFruitPlaceHelp',
+                               SpeakAndListenState(action_dict,
+                                                   global_store,
+                                                   question,
+                                                   READY,
+                                                   [],
+                                                   20),
+                               transitions={'SUCCESS':'HandoverFruit',
+                                            'FAILURE':'AskForFruitPlaceHelp',
+                                            'REPEAT_FAILURE':'TASK_FAILURE'})
+        
+        # Handover fruit to operator
+        smach.StateMachine.add('HandoverFruit',
+                               HandoverObjectToOperatorState(action_dict, 
+                                                             global_store),
+                               transitions={'SUCCESS':'ChooseIngredient',
+                                            'FAILURE':'AskForFruitPlaceHelp'})
+        
+        # If spoon/sugar
+        smach.StateMachine.add('PickUpSpoon',
+                               PickUpObjectState(action_dict, global_store),
+                               transitions={'SUCCESS':'PourSugar',
+                                            'FAILURE':'AskForSpoonHelp'})
+        
+
+        # get help with spoon
+        question = ("I can't seem to pick up this spoon, could you please help"+
+                   " me and let me know when you're ready to hand it over?")
+        smach.StateMachine.add('AskForSpoonHelp',
+                               SpeakAndListenState(action_dict,
+                                                   global_store,
+                                                   question,
+                                                   READY,
+                                                   [],
+                                                   20),
+                               transitions={'SUCCESS':'ReceiveSpoon',
+                                            'FAILURE':'AskForSpoonHelp',
+                                            'REPEAT_FAILURE':'TASK_FAILURE'})
+        
+        # receive spoon
+        smach.StateMachine.add('ReceiveSpoon',
+                               ReceiveObjectFromOperatorState(action_dict,
+                                                              global_store),
+                               transitions={'SUCCESS':'PourSugar',
+                                            'FAILURE':'AskForSpoonHelp'})
+
+        # Pour some sugar on me...
+        smach.StateMachine.add('PourSugar',
+                               PourSugarState(action_dict, global_store),
+                               transitions={'SUCCESS':'ChooseIngredient',
+                                            'FAILURE':'AskForSugarPourHelp'})
+        
+        # Ask for sugar based help
+        question = ("I can't get this sugar in the blender. Can you do it for "+
+                   "me please and let me know when you're ready so I can give "+
+                   "you the spoon?")
+        smach.StateMachine.add('AskForSugarHelp',
+                               SpeakAndListenState(action_dict,
+                                                   global_store,
+                                                   question,
+                                                   ['Done'],
+                                                   [],
+                                                   20),
+                               transitions={'SUCCESS':'HandoverSpoon',
+                                            'FAILURE':'AskForSugarHelp',
+                                            'REPEAT_FAILURE':'TASK_FAILURE'})
+        
+        # Handover the spoon
+        smach.StateMachine.add('HandoverSpoon',
+                               HandoverObjectToOperatorState(action_dict, 
+                                                             global_store),
+                               transitions={'SUCCESS':'ChooseIngredient',
+                                            'FAILURE':'AskForSugarHelp'})
+
+        # Pick up the milk
+        smach.StateMachine.add('PickUpMilk',
+                               PickUpObjectState(action_dict, global_store),
+                               transitions={'SUCCESS':'PourMilk',
+                                            'FAILURE':'AskForMilkHelp'})
+        
+         # get help with milk
+        question = ("I can't seem to pick up this milk, could you please help"+
+                   " me and let me know when you're ready to hand it over?")
+        smach.StateMachine.add('AskForMilkHelp',
+                               SpeakAndListenState(action_dict,
+                                                   global_store,
+                                                   question,
+                                                   READY,
+                                                   [],
+                                                   20),
+                               transitions={'SUCCESS':'ReceiveMilk',
+                                            'FAILURE':'AskForMilkHelp',
+                                            'REPEAT_FAILURE':'TASK_FAILURE'})
+        
+        # receive milk
+        smach.StateMachine.add('ReceiveMilk',
+                               ReceiveObjectFromOperatorState(action_dict,
+                                                              global_store),
+                               transitions={'SUCCESS':'PourMilk',
+                                            'FAILURE':'AskForMilkHelp'})
+        
+        # Pour milk
+        smach.StateMachine.add('PourMilk',
+                               PourIntoState(action_dict, global_store),
+                               transitions={'SUCCESS':'ChooseIngredient',
+                                            'FAILURE':'AskForMilkPourHelp'})
+
+        # Ask for milk based help
+        question = ("I can't pour this milk in the blender. Can you help me " +
+                   "please and let me know when you're ready to receive " +
+                   "the milk?")
+        smach.StateMachine.add('AskForMilkPourHelp',
+                               SpeakAndListenState(action_dict,
+                                                   global_store,
+                                                   question,
+                                                   READY,
+                                                   [],
+                                                   20),
+                               transitions={'SUCCESS':'HandoverMilk',
+                                            'FAILURE':'AskForMilkPourHelp',
+                                            'REPEAT_FAILURE':'TASK_FAILURE'})
+        
+        # Handover milk to operator
+        smach.StateMachine.add('HandoverMilk',
+                               HandoverObjectToOperatorState(action_dict, 
+                                                             global_store),
+                               transitions={'SUCCESS':'ChooseIngredient',
+                                            'FAILURE':'AskForMilkPourHelp'})
+
+    return sm
+
+if __name__ == '__main__':
+    action_dict = create_stage_2_clients(7)
+    sm = create_state_machine(action_dict)
+    sm.execute()
diff --git a/state_machines/src/storing_groceries.py b/state_machines/src/storing_groceries.py
index e01ead9..679290e 100644
--- a/state_machines/src/storing_groceries.py
+++ b/state_machines/src/storing_groceries.py
@@ -35,7 +35,7 @@ class PickUpClosestItemState(ActionServiceState):
     """ State for picking up the closest item to us. """
     
     def __init__(self, action_dict, global_store):
-        outcomes = ['SUCCESS', 'FAILURE']
+        outcomes = ['SUCCESS', 'FAILURE', 'NO_ITEMS']
         super(PickUpClosestItemState, self).__init__(action_dict=action_dict,
                                                      global_store=global_store,
                                                      outcomes=outcomes)
@@ -217,8 +217,8 @@ def create_state_machine(action_dict):
                                SpeakAndListenState(action_dict,
                                                    global_store,
                                                    question,
-                                                   map(RELATIONS, 
-                                                   (lambda x: x + ' <param>')),
+                                                   map((lambda x: x+' <param>'),
+                                                   RELATIONS),
                                                    OBJECTS,
                                                    20),
                                transitions={'SUCCESS':'UpdateItemInfo',
diff --git a/state_machines/src/take_out_the_garbage.py b/state_machines/src/take_out_the_garbage.py
index 4237412..51d954d 100644
--- a/state_machines/src/take_out_the_garbage.py
+++ b/state_machines/src/take_out_the_garbage.py
@@ -199,7 +199,7 @@ def create_state_machine(action_dict):
                                             'FAILURE':'AskForHandover'})
         
         # Grab the Garbage
-        smach.StateMachine.add('GrabGrabage',
+        smach.StateMachine.add('GrabGarbage',
                                PickUpObjectState(action_dict, global_store),
                                transitions={'SUCCESS':'SetCollectZone',
                                             'FAILURE':'AskForHelpWithGarbage'})
