#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from experiment_therblig_task_runner.msg import NeglectTime

# RAD = IT / (IT + NT)

if __name__ == '__main__':
    try:
        pub_signal = rospy.Publisher('/rad/signal', Float32, queue_size=10)
        pub_time = rospy.Publisher('/rad/neglect_time', NeglectTime, queue_size=10)
        rospy.init_node('runner', anonymous=True)

        rate = rospy.Rate(1)

        signal = 1
        neglect_time = NeglectTime()
        neglect_time.current = 65
        neglect_time.initial = 65
        interaction_time = 60
        countUp = False

        while not rospy.is_shutdown():

            # grab therblig

            # robot action + button input

            # compute signals
            if neglect_time.current > 0:
                neglect_time.current -= 1
            else:
                neglect_time.current = neglect_time.initial

            signal = interaction_time * 1.0 / (interaction_time + neglect_time.current)

            print signal
            #rospy.loginfo(('RAD: %s' % signal) + (' Time: %s') % neglect_time)
            pub_signal.publish(signal)
            pub_time.publish(neglect_time)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
