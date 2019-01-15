#!/usr/bin/env python

import json
import rospy

from time_mode_enum import TimeModeEnum
from iter.srv import Task, TaskResponse, ModeGet, ModeSet, ModeGetResponse, ModeSetResponse


rospy.wait_for_service('/runner/task_input')

task_srv = rospy.ServiceProxy('/runner/task_input', Task)
mode_set_srv = rospy.ServiceProxy('/runner/set/mode',ModeSet)
mode_get_srv = rospy.ServiceProxy('/runner/get/mode',ModeGet)


def set_mode(mode):
    msg = ModeSet()
    msg.mode = mode
    mode_set_srv(mode)

def get_mode():
    msg = mode_get_srv(ModeGet())
    return msg.mode

def load_task(task_file_name):
    early_stop = False

    # Load JSON file
    try:
        f = open(task_file_name,'r')
        try:
            txStr = json.dump(f)
        except:
            print '[-] Error parsing JSON'
            early_stop = True
        f.close()
    except:
        print '[-] Error handling file : Read'
        early_stop = True

    if early_stop:
        return

    # Request
    response = task_srv(txStr)

    if response.recorded_task:
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

    return response.end_status

if __name__ == "__main__":
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
                    load_task(args[1])
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
        else:
            print '[-] Error invalid command supplied'
