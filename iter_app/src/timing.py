#!/usr/bin/env python

import json
import rospy

from iter_app.msg import RADTime, TimeInterval
from std_msgs.msg import Float32, String, Int32, Bool


class TimingServer:

    SIGNAL_PUBLISH_TIME_STEP = 0.01

    def __init__(self):

        self.start_timing_topic = rospy.Subscriber('time_node/start', String, self._start_timing)
        self.stop_timing_topic = rospy.Subscriber('time_node/stop', Bool, self._stop_timing)
        self.sync_timing_topic = rospy.Subscriber('time_node/sync', Int32, self._sync_timing)

        self.pub_signal = rospy.Publisher('/rad/signal', RADTime, queue_size=10)
        self.pub_neglect_time = rospy.Publisher('/rad/neglect_time', TimeInterval, queue_size=10)
        self.pub_interaction_time = rospy.Publisher('rad/interaction_time', TimeInterval, queue_size=10)

        self._neglect_time_list = []
        self._run = False
        self._index = 0
        self._interaction = False

    def _start_timing(self,json_string):
        self._neglect_time_list = json.loads(json_string.data)
        self._run = True

    def _stop_timing(self, noop):
        # set neglect time to null
        self._run = False

    def _sync_timing(self, syncIndex):
        # call this once a primitive is done running
        self._index = syncIndex

    def loop(self):

        while not rospy.is_shutdown():
            rospy.spin()

        '''
        try:
            neglect_time = TimeInterval()
            interaction_time = 0

            for t in self._neglect_time_list:

                if 'interaction' in t.keys() and t['interaction']:
                    # Wait for interaction event
                    interaction_time = 0
                    current = base = time.time()

                    while not button_state and self._thread_alive:

                        interaction_time = current - base
                        self.pub_interaction_time.publish(interaction_time)

                        time.sleep(self.SIGNAL_PUBLISH_TIME_STEP)
                        current = time.time()

                    button_state = False
                else:
                    # Run through the prerecorded timing
                    neglect_time.current = t["time"]
                    neglect_time.initial = t["time"]

                    current = base = time.time()

                    while current < base + t["time"] and self._thread_alive:

                        neglect_time.current = t["time"] - (current - base)
                        self.pub_neglect_time.publish(neglect_time)

                        time.sleep(self.SIGNAL_PUBLISH_TIME_STEP)
                        current = time.time()

                if not self._thread_alive:
                    break

        except Exception, e:
            print e
        '''


if __name__ == '__main__':
    rospy.init_node('timing node')
    rad = TimingServer()
    rad.loop()
