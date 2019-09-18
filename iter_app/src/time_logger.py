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
        self._running_data = {}

        self.start_timing_topic = rospy.Subscriber('time_node/start', String, self._start_timing)
        self.stop_timing_topic = rospy.Subscriber('time_node/stop', Bool, self._stop_timing)
        self.sync_timing_topic = rospy.Subscriber('time_node/sync', Int32, self._sync_timing)
        self.interaction_topic = rospy.Subscriber('time_node/interaction', Bool, self._interaction_event_trigger)

        self.save_topic = rospy.Subscriber('save',String, self._save_cb)

    def _save_cb(self, msg):
        file = open(self._path + '/{}_{}.json'.format(msg.data,time.time()),'w')
        json.dump(self._data,file)
        self._data = {}
        file.close()

    def spin(self):
        file = open(self._path + '/temp_{}.json'.format(time.time()), 'w')

        try:
            while not rospy.is_shutdown():
                rospy.sleep(0.1)
        except:
            pass

        json.dump(self._running_data,file)
        file.close()

    def _start_timing(self, msg):
        key = '{}'.format(time.time())
        value = {'event': 'start', 'timeline': json.loads(msg.data)}

        self._data[key] = value
        self._running_data[key] = value

    def _stop_timing(self, msg):
        key = '{}'.format(time.time())
        value = {'event': 'stop'}

        self._data[key] = value
        self._running_data[key] = value

    def _sync_timing(self, msg):
        key = '{}'.format(time.time())
        value = {'event': 'sync', 'index': msg.data}

        self._data[key] = value
        self._running_data[key] = value

    def _interaction_event_trigger(self, msg):
        key = '{}'.format(time.time())
        value = {'event': 'interaction'}

        self._data[key] = value
        self._running_data[key] = value


if __name__ == "__main__":
    rospy.init_node('time_logger')

    filepath = rospy.get_param('~filepath')

    node = TimeLogger(filepath)
    node.spin()
