#!/usr/bin/env python

import json
import rospy

from iter_app.msg import TimeInterval
from std_msgs.msg import String, Int32, Bool


class TimingServer:

    SIGNAL_PUBLISH_TIME_STEP = 0.01
    FAKE_TIME_INITIAL = 60

    def __init__(self):

        self.start_timing_topic = rospy.Subscriber('time_node/start', String, self._start_timing)
        self.stop_timing_topic = rospy.Subscriber('time_node/stop', Bool, self._stop_timing)
        self.sync_timing_topic = rospy.Subscriber('time_node/sync', Int32, self._sync_timing)
        self.interaction_topic = rospy.Subscriber('time_node/interaction', Bool, self._interaction_event_trigger)

        self.pub_neglect_time = rospy.Publisher('/rad/neglect_time', TimeInterval, queue_size=10)
        self.pub_interaction_time = rospy.Publisher('rad/interaction_time', TimeInterval, queue_size=10)

        self._neglect_time_list = []
        self._run = False
        self._sync_index = 0
        self._interaction_event = False

        self._fake_time = 60
        self._fake_mode = 'neglect'

    def _start_timing(self,json_string):
        self._neglect_time_list = json.loads(json_string.data)
        self._run = True

    def _stop_timing(self, noop):
        # set neglect time to null
        self._run = False

    def _sync_timing(self, syncIndex):
        # call this once a primitive is done running
        self._sync_index = syncIndex

    def _interaction_event_trigger(self, noop):
        self._interaction_event = True

    def _fake_time_publisher(self):
        timeInterval = TimeInterval()
        timeInterval.initial = self.FAKE_TIME_INITIAL
        timeInterval.current = self._fake_time

        if self._fake_mode == 'neglect':
            self.pub_neglect_time.publish(timeInterval)
        else:
            self.pub_interaction_time.publish(timeInterval)

        rospy.sleep(self.SIGNAL_PUBLISH_TIME_STEP)
        self._fake_time -= self.SIGNAL_PUBLISH_TIME_STEP

        if self._fake_time < 0:
            self._fake_time = self.FAKE_TIME_INITIAL
            if self._fake_mode == 'neglect':
                self._fake_mode = 'interaction'
            else:
                self._fake_mode = 'neglect'

    def _real_time_publisher(self):
        #TODO use indexing method for task loop. Use this index with the sync index
        #TODO provide ideal interaction time as an output not the actual time currently taken

        timeInterval = TimeInterval()
        interaction_time = 0

        for index in range(0,len(self._neglect_time_list)):
            t = self._neglect_time_list[index]

            if 'interaction' in t.keys() and t['interaction']:
                # Wait for interaction event
                timeInterval.current = t["time"]
                timeInterval.initial = t["time"]

                current = base = time.time()

                while self._run and not self._interaction_event:

                    timeInterval.current = current - base
                    if timeInterval.current > timeInterval.initial:
                        timeInterval.current = timeInterval.initial

                    self.pub_interaction_time.publish(timeInterval)

                    time.sleep(self.SIGNAL_PUBLISH_TIME_STEP)
                    current = time.time()

                self._interaction_event = False
            else:
                # Run through the prerecorded timing
                timeInterval.current = t["time"]
                timeInterval.initial = t["time"]

                current = base = time.time()

                while self._run and current < base + t["time"]:

                    timeInterval.current = t["time"] - (current - base)
                    self.pub_neglect_time.publish(timeInterval)

                    time.sleep(self.SIGNAL_PUBLISH_TIME_STEP)
                    current = time.time()

    def loop(self):
        fakeTime = rospy.get_param("fake_time",False)

        while not rospy.is_shutdown():
            rospy.spin()

            if fakeTime:
                self._fake_time_publisher()
            elif self._run:
                self._real_time_publisher()


if __name__ == '__main__':
    rospy.init_node('timing_node')
    rad = TimingServer()
    rad.loop()
