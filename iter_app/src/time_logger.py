#!/usr/bin/env python

'''
TODO documentation
'''

import time
import json
import rospy

from std_msgs.msg import String, Int32, Bool


class TimeLogger:

    def __init__(self, filepath):
        self._path = filepath
        self._data = {}

        self.start_timing_topic = rospy.Subscriber('time_node/start', String, self._start_timing)
        self.stop_timing_topic = rospy.Subscriber('time_node/stop', Bool, self._stop_timing)
        self.sync_timing_topic = rospy.Subscriber('time_node/sync', Int32, self._sync_timing)
        self.interaction_topic = rospy.Subscriber('time_node/interaction', Bool, self._interaction_event_trigger)

    def spin(self):
        file = open(self._path + '/{}.json'.format(time.time()), 'w')
        
        try:
            while not rospy.is_shutdown():
                rospy.sleep(0.1)
        except:
            pass

        json.dump(self._data,file)
        file.close()

    def _start_timing(self, msg):
        self._data['{}'.format(time.time())] = {'event': 'start', 'timeline': json.loads(msg.data)}

    def _stop_timing(self, msg):
        self._data['{}'.format(time.time())] = {'event': 'stop'}

    def _sync_timing(self, msg):
        self._data['{}'.format(time.time())] = {'event': 'sync', 'index', msg.data}

    def _interaction_event_trigger(self, msg):
        self._data['{}'.format(time.time())] = {'event': 'interaction'}


if __name__ == "__main__":
    rospy.init_node('time_logger')

    filepath = rospy.get_param('~filepath')

    node = TimeLogger(filepath)
    node.spin()
