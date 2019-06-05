#!/usr/bin/env python

'''
Timing Node
Author: Curt Henrichs
Date: 5-22-19

Provides RAD timing updates to ITER network.

Inspired by fanout and robot-attention-demand (RAD) research with the concept of
neglect and interaction time, this node attempt to convey the necessary signal
as the task progresses. Thus a plan in the standard runner json format is provided,
iterated over, and timing information published. Note: this plan must have
pre-recorded timing information in order for the timing node to generate meaningful
estimations.

Timing information conveyed:
    - neglect time
            "Time that the robot is working independent of a human"
    - interaction time
            "Time that human is working with the robot or while the robot is idle"
    - rad-signal
            "Theoretically defined by RAD = (interaction time) / [(interaction time) + (neglect time)]"
            "If provided as a real-time estimate, provides better understanding of task / subtask"
'''

import time
import json
import rospy

from iter_app.msg import RADSignal, TimeInterval
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
        self.pub_rad_signal = rospy.Publisher('/rad/signal', RADSignal, queue_size=10)

        self._neglect_time_list = []
        self._run = False
        self._sync_index = 0
        self._interaction_event = False
        self._in_interaction_task = False

        self._fake_time = 60
        self._fake_mode = 'neglect'

    def _start_timing(self,message):
        self._neglect_time_list = json.loads(message.data)
        self._sync_index = 0
        self._interaction_event = False
        self._in_interaction_task = False
        self._run = True

    def _stop_timing(self, noop):
        # set neglect time to null
        self._run = False

    def _sync_timing(self, message):
        # call this once a primitive is done running
        self._sync_index = message.data

    def _interaction_event_trigger(self, noop):
        if self._in_interaction_task:
            self._interaction_event = True

    def _fake_time_publisher(self):

        signal = RADSignal()

        timeInterval = TimeInterval()
        timeInterval.initial = self.FAKE_TIME_INITIAL
        timeInterval.current = self._fake_time

        if self._fake_mode == 'neglect':
            signal.neglect_time = timeInterval
            signal.mode = RADSignal.NEGLECT_MODE
            self.pub_neglect_time.publish(timeInterval)
        else:
            signal.interaction_time = timeInterval
            signal.mode = RADSignal.INTERACTION_MODE
            self.pub_interaction_time.publish(timeInterval)
        self.pub_rad_signal.publish(signal)

        rospy.sleep(self.SIGNAL_PUBLISH_TIME_STEP)

        if self._fake_mode == 'neglect':
            self._fake_time -= self.SIGNAL_PUBLISH_TIME_STEP
            if self._fake_time < 0:
                self._fake_time = 0
                self._fake_mode = 'interaction'
        else:
            self._fake_time += self.SIGNAL_PUBLISH_TIME_STEP
            if self._fake_time > self.FAKE_TIME_INITIAL:
                self._fake_time = self.FAKE_TIME_INITIAL
                self._fake_mode = 'neglect'

    def _real_time_publisher(self):
        #TODO use indexing method for task loop. Use this index with the sync index

        timeInterval = TimeInterval()
        interaction_time = 0

        index = 0
        while index < len(self._neglect_time_list):
            signal = RADSignal()

            # Re-sync
            if index > self._sync_index:
                # out of sync - ahead of runner
                index = self._sync_index
            elif index < self._sync_index:
                # out of sync - behind runner
                index = self._sync_index

            if index >= len(self._neglect_time_list):
                break

            t = self._neglect_time_list[index]

            # run timing for task primitive
            if 'interaction' in t.keys() and t['interaction']:
                signal.mode = RADSignal.INTERACTION_MODE
                self._in_interaction_task = True

                # Wait for interaction event
                timeInterval.current = 0
                timeInterval.initial = t["time"]
                current = base = time.time()

                while self._run and not self._interaction_event:
                    if index != self._sync_index:
                        # sync event means moving to next primitive in runner
                        # done with this task
                        break

                    timeInterval.current = current - base

                    self.pub_interaction_time.publish(timeInterval)

                    signal.interaction_time = timeInterval
                    self.pub_rad_signal.publish(signal)

                    time.sleep(self.SIGNAL_PUBLISH_TIME_STEP)
                    current = time.time()

                self._in_interaction_task = False
                self._interaction_event = False

            else:
                signal.mode = RADSignal.NEGLECT_MODE

                # Run through the prerecorded timing
                timeInterval.current = t["time"]
                timeInterval.initial = t["time"]

                current = base = time.time()

                while self._run and current < base + t["time"]:
                    if index != self._sync_index:
                        # sync event means moving to next primitive in runner
                        # done with this task
                        break

                    timeInterval.current = t["time"] - (current - base)
                    self.pub_neglect_time.publish(timeInterval)

                    signal.neglect_time = timeInterval
                    self.pub_rad_signal.publish(signal)

                    time.sleep(self.SIGNAL_PUBLISH_TIME_STEP)
                    current = time.time()

            # increment loop counter
            index += 1

    def loop(self):
        fakeTime = rospy.get_param("~fake_time",False)
        print 'Timing running in', 'fake' if fakeTime else 'real', 'time'

        while not rospy.is_shutdown():

            if fakeTime:
                self._fake_time_publisher()
            elif self._run:
                self._real_time_publisher()


if __name__ == '__main__':
    rospy.init_node('timing_node')
    rad = TimingServer()
    rad.loop()
