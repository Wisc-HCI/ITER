#!/usr/bin/env python

import json
import rospy

from iter.srv import Task, TaskResponse

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
        }
    ]
}

if __name__ == "__main__":
    print 'wait for service'
    rospy.wait_for_service('task_input')

    print 'create proxy'
    task_srv = rospy.ServiceProxy('task_input', Task)

    print 'generate message'
    txStr = json.dumps(msg)
    print txStr

    print 'send message'
    response = task_srv(txStr)

    print 'response'
    print response.end_status
