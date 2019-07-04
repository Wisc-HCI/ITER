#!/usr/bin/env python

'''
Command Node
Author: Curt Henrichs
Date: 7-4-19

Terminal primitive to command runner with preexisting plan.

Input args format:
    command.py <mode> <filepath>
        <mode> = ['capture', 'replay']
        <filepath> = string path to plan
'''

import sys
import json
import rospy

from iter_app_tools.time_mode_enum import TimeModeEnum
from iter_app.srv import Task, TaskResponse, ModeGet, ModeSet, ModeGetResponse, ModeSetResponse


rospy.init_node('command_node')

print 'Waiting for runner'
rospy.wait_for_service('/runner/task_input')
print 'Found runner'


task_srv = rospy.ServiceProxy('/runner/task_input', Task)
mode_set_srv = rospy.ServiceProxy('/runner/set/mode',ModeSet)
mode_get_srv = rospy.ServiceProxy('/runner/get/mode',ModeGet)


# Check valid number of args
if len(sys.argv) != 3:
    rospy.logerr('Invalid number of arguements')
    sys.exit()

# Extract mode arg
mode = sys.argv[1]
if not mode in ['capture','replay']:
    rospy.logerr('Mode is not valid')
    sys.exit()

# extract string from file specified by filepath arg
filepath = sys.argv[2]
txStr = ''
try:
    fin = open(filepath,'r')
    txStr = fin.read()
    fin.close()
except:
    rospy.logger('File failed to read in data')
    sys.exit()

# update runner state
mode_set_srv(mode)
response = task_srv(txStr)

# writeback if needed
if response.end_status and response.recorded_task and get_mode() == 'capture':
    try:
        fout = open(filepath,'w')
        try:
            rxMsg = json.loads(response.recorded_task)
            json.dump(rxMsg,fout,indent=4)
        except:
            rospy.logger('Error formatting new JSON plan on capture writeback')
        fout.close()
    except:
        rospy.logger('Error handling file on capture writeback')

rospy.loginfo('Complete')
