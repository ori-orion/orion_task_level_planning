
import smach; 
from state_machines.Reusable_States.include_all import *;

def createWaitForStartup():
    sub_sm = smach.StateMachine(
        outcomes=[TASK_SUCCESS],
        output_keys=['task_start_time']);

    with sub_sm:
        # wait for the start signal - this has been replaced by the WAIT_FOR_HOTWORD state
        #   TODO - fix and test the check door state for future competitions
        smach.StateMachine.add(
            'WAIT_FOR_START_SIGNAL',
            CheckDoorIsOpenState(),
            transitions={
                'open':'StartSpeech', 
                'closed':'WAIT_FOR_START_SIGNAL'});

        smach.StateMachine.add(
            'StartSpeech',
            SpeakState(phrase="The door is open."),
            transitions={SUCCESS:'SAVE_START_TIM'});

        # save the start time
        smach.StateMachine.add(
            'SAVE_START_TIME',
            GetTime(),
            transitions={SUCCESS:TASK_SUCCESS},
            remapping={'current_time':'task_start_time'});

    return sub_sm;

