#!/usr/bin/env python

'''
CLI Node
Author: Curt Henrichs
Date: 5-22-19

Command-Line Interace for task runer.

This module provides a simple CLI to enter defined tasks for the ITER runner.

Commands include:
    - help
            "Provides list of commands"
    - task <filepath>
            "Runs plan specified in the filepath"
    - get_mode
            "Gets the current mode"
            ['capture','replay']
    - set_mode <mode>
            "Sets the current mode"
            ['capture','replay']

Note: the modes 'capture' and 'replay' are used for the RAD subsystem. Capture
will record the time per primitive in plan. Replay will use pre-recorded time to
output timing estimates to the RAD display.
'''

import time
import json
import rospy

from iter_app_tools.time_mode_enum import TimeModeEnum
from iter_app.srv import Task, TaskResponse, ModeGet, ModeSet, ModeGetResponse, ModeSetResponse

print 'Waiting for runner'
rospy.wait_for_service('/runner/task_input')
print 'Found runner'

task_srv = rospy.ServiceProxy('/runner/task_input', Task)
mode_set_srv = rospy.ServiceProxy('/runner/set/mode',ModeSet)
mode_get_srv = rospy.ServiceProxy('/runner/get/mode',ModeGet)


def set_mode(mode):
    msg = ModeSet()
    msg.mode = mode
    mode_set_srv(mode)

def get_mode():
    rxMsg = mode_get_srv()
    return rxMsg.mode

def load_task(task_file_name):
    early_stop = False

    # Load JSON file
    try:
        f = open(task_file_name,'r')
        txStr = f.read()
        f.close()
    except:
        print '[-] Error handling file : Read'
        early_stop = True

    if early_stop:
        return

    # Request
    response = task_srv(txStr)

    if response.end_status and response.recorded_task and get_mode() == TimeModeEnum.CAPTURE.value:
        try:
            f = open(task_file_name,'w')
            try:
                rxMsg = json.loads(response.recorded_task)
                json.dump(rxMsg,f,indent=4)
            except:
                print '[-] Error formatting new JSON'
            f.close()
        except:
            print '[-] Error handling file : Write'

    print response.msg
    return response.end_status

if __name__ == "__main__":
    time.sleep(2) # wait for other things to launch / setup

    # Print initial prompt
    print('Interdependence Task Experiment Runner')

    while True:

        # Get user command and parse
        inStr = raw_input('ITER: ')
        args = inStr.split()

        if len(args) < 1:
            print '[-] Error must supply command'
        elif args[0].lower() == 'task':
            if len(args) < 2:
                print '[-] Error must supply filepath arguement'
            else:
                try:
                    print 'End status: ', load_task(args[1])
                except Exception, e:
                    print '[-] Error:', e
        elif args[0].lower() == 'get_mode':
            print 'Current mode: ', get_mode()
        elif args[0].lower() == 'set_mode':
            if len(args) < 2:
                print '[-] Error must supply mode to set'
            elif TimeModeEnum.from_str(args[1]) == None:
                print '[-] Error invalid mode suplied'
            else:
                print 'Set mode to: ', args[1]
                set_mode(args[1])
        elif args[0].lower() == 'help':
            print 'The following commands may be used'
            print '* help'
            print '* task <your_task_file_path>'
            print '* get_mode'
            print '* set_mode <new_mode>'
        else:
            print '[-] Error invalid command supplied'
