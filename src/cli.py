#!/usr/bin/env python

import json
import rospy

from iter.srv import Task, TaskResponse


"""
{
    'task': [
        {
            'name': <string ['wait, move, grasp, release']>
            ... <params>
            'rad': {
                'negelect_time': <number>
                'is_interaction': <boolean>
            }
        }
    ]
}
"""

msg = {
    'task': [
        {
            'name': 'release'
        },
        {
            'name': 'grasp',
            'effort': 2
        },
        {
            'name': 'wait',
            'condition': 'button'
        },
        {
            'name': 'wait',
            'condition': 'time',
            'value': 5
        },
        {
            'name': 'move',
            'position': {
                'x': 0.25,
                'y': 0.1,
                'z': 0.25
            },
            'orientation': {
                'x': 0,
                'y': 0,
                'z': 0
            }
        }
    ]
}

if __name__ == "__main__":
    print 'wait for service'
    rospy.wait_for_service('/runner/task_input')

    print 'create proxy'
    task_srv = rospy.ServiceProxy('/runner/task_input', Task)

    print 'generate message'
    txStr = json.dumps(msg)
    print txStr

    print 'send message'
    response = task_srv(txStr)

    print 'response'
    print response.end_status
