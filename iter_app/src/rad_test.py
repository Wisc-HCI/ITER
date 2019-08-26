#!/usr/bin/env python

'''
TODO documenation
'''

import json
import rospy

from std_msgs.msg import String, Int32, Bool


class RadTest:

    def __init__(self):
        self.time_start_topic = rospy.Publisher('/time_node/start', String, queue_size=10)
        self.time_stop_topic = rospy.Publisher('/time_node/stop', Bool, queue_size=10)
        self.time_sync_topic = rospy.Publisher('/time_node/sync', Int32, queue_size=10)
        self.interaction_topic = rospy.Publisher('/time_node/interaction', Bool, queue_size=10)

    def _small_timeline(self):
        return [
            {
                'time': 20,
                'interaction': False
            },
            {
                'time': 37,
                'interaction': True
            },
            {
                'time': 55,
                'interaction': False
            }
        ]

    def _large_timeline(self):
        return [
            {
                'time': 20,
                'interaction': False
            },
            {
                'time': 37,
                'interaction': True
            },
            {
                'time': 55,
                'interaction': False
            },
            {
                'time': 55,
                'interaction': True
            },
            {
                'time': 86,
                'interaction': False
            },
            {
                'time': 5,
                'interaction': True
            }
        ]

    def loop(self):

        while not rospy.is_shutdown():

            str = raw_input('Start?')

            timeline = rospy.get_param('~timeline_slct','short')
            if timeline == 'long':
                data = self._large_timeline()
            else:
                data = self._small_timeline()

            self.time_start_topic.publish(String(json.dumps(data)))

            for index in range(0,len(data)):
                if rospy.is_shutdown():
                    break

                self.time_sync_topic.publish(Int32(index))

                if data[index]['interaction']:
                    str = raw_input('Interaction')
                    #self.interaction_topic.publish(True)
                else:
                    rospy.sleep(data[index]['time'])

            self.time_stop_topic.publish(Bool(True))


if __name__ == '__main__':
    rospy.init_node('rad_test_node')
    rad = RadTest()
    rad.loop()
