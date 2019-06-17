#!/usr/bin/env python
""" File to let user choose which RoboCup task to execute.

This file contains a script to let the user choose from all tasks which 
to execute.

Author: Charlie Street
Owner: Charlie Street
"""

import rospy
from set_up_clients import create_stage_1_clients
from set_up_clients import create_stage_2_clients
from set_up_clients import create_final_clients
import carry_my_luggage
import clean_up
import farewell
import find_my_mates
import GPSR
import receptionist
import serving_drinks 
import serve_the_breakfast
import storing_groceries
import take_out_the_garbage

def choose_task():
    print("Please enter the number (1-3) for the stage you are in:")
    print("[1]: Stage 1")
    print("[2]: Stage 2")
    print("[3]: Final")

    stage = input()
    
    if stage == 1:
        print("Please enter the number (1-10) of the task you want to run:")
        print("[1]: Carry My Luggage (Party Host)")
        print("[2]: Clean Up (Housekeeper)")
        print("[3]: Farewell (Party Host)")
        print("[4]: Find My Mates (Party Host)")
        print("[5]: General Purpose Service Robot (Housekeeper)")
        print("[6]: Receptionist (Party Host)")
        print("[7]: Serving Drinks (Party Host)")
        print("[8]: Serve The Breakfast (Housekeeper)")
        print("[9]: Storing Groceries (Housekeeper)")
        print("[10]: Take Out The Garbage (Housekeeper)")
        task = input()

        if task == 1:
            print("Carry My Luggage Selected.")
            rospy.init_node('carry_my_luggage_state_machine')
            action_dict = create_stage_1_clients(task)
            sm = carry_my_luggage.create_state_machine(action_dict)
            outcome = sm.execute()
            print("OUTCOME: " + str(outcome))

        elif task == 2:
            print("Clean Up Selected.")
            rospy.init_node('clean_up_state_machine')
            action_dict = create_stage_1_clients(task)
            sm = clean_up.create_state_machine(action_dict)
            outcome = sm.execute()
            print("OUTCOME: " + str(outcome))

        elif task == 3:
            print("Farewell Selected.")
            rospy.init_node('farewell_state_machine')
            action_dict = create_stage_1_clients(task)
            sm = farewell.create_state_machine(action_dict)
            outcome = sm.execute()
            print("OUTCOME: " + str(outcome))

        elif task == 4:
            print("Find My Mates Selected.")
            rospy.init_node('find_my_mates_state_machine')
            action_dict = create_stage_1_clients(task)
            sm = find_my_mates.create_state_machine(action_dict)
            outcome = sm.execute()
            print("OUTCOME: " + str(outcome))

        elif task == 5:
            print("General Purpose Service Robot Selected.")
            rospy.init_node('GPSR_state_machine')
            action_dict = create_stage_1_clients(task)
            sm = GPSR.create_state_machine(action_dict)
            outcome = sm.execute()
            print("OUTCOME: " + str(outcome))

        elif task == 6:
            print("Receptionist Selected.")
            rospy.init_node('receptionist_state_machine')
            action_dict = create_stage_1_clients(task)
            sm = receptionist.create_state_machine(action_dict)
            outcome = sm.execute()
            print("OUTCOME: " + str(outcome))

        elif task == 7:
            print("Serving Drinks Selected.")
            rospy.init_node('serving_drinks_state_machine')
            action_dict = create_stage_1_clients(task)
            sm = serving_drinks.create_state_machine(action_dict)
            outcome = sm.execute()
            print("OUTCOME: " + str(outcome))

        elif task == 8:
            print("Serve The Breakfast Selected.")
            rospy.init_node('serve_the_breakfast_state_machine')
            action_dict = create_stage_1_clients(task)
            sm = serve_the_breakfast.create_state_machine(action_dict)
            outcome = sm.execute()
            print("OUTCOME: " + str(outcome))

        elif task == 9:
            print("Storing Groceries Selected.")
            rospy.init_node('storing_groceries_state_machine')
            action_dict = create_stage_1_clients(task)
            sm = storing_groceries.create_state_machine(action_dict)
            outcome = sm.execute()
            print("OUTCOME: " + str(outcome))

        elif task == 10:
            print("Take Out The Garbage Selected.")
            rospy.init_node('take_out_the_garbage_state_machine')
            action_dict = create_stage_1_clients(task)
            sm = take_out_the_garbage.create_state_machine(action_dict)
            outcome = sm.execute()
            print("OUTCOME: " + str(outcome))
            
        else:
            print("Invalid Task Entered. Please try again.")
            choose_task()

    elif stage == 2:
        print("Please enter the number (1-9) of the task you want to run:")
        print("[1]: Clean The Table (Housekeeper)")
        print("[2]: Enhanced GPSR (Housekeeper)")
        print("[3]: Find My Disk (Housekeeper)")
        print("[4]: Hand Me That (Party Host)")
        print("[5]: Set The Table (Housekeeper)")
        print("[6]: Restaurant (Party Host)")
        print("[7]: Smoothie Chef (Party Host)")
        print("[8]: Stickler For The Rules (Party Host)")
        print("[9]: Where Is This? (Party Host)")
        task = input()

        if task == 1:
            print("Clean The Table Selected.")
            # TODO: Sort out
        elif task == 4:
            print("Hand Me That Selected.")
            # TODO: Sort out
        elif task == 5:
            print("Set The Table Selected.")
            # TODO: Sort out
        elif task == 7:
            print("Smoothie Chef Selected.")
            # TODO: Sort out
        else:
            print("Invalid Task Entered. Please try again.")
            choose_task()

    elif stage == 3:
        print("Not Implemented Yet!")
    else:
        print("Incorrect Stage Entered! Please try again.")
        choose_task()

if __name__ == '__main__':
    choose_task()
