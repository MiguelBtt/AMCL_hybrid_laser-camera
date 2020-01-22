#!/usr/bin/env python

import rospy
from random import random, uniform
from sensor_msgs.msg import LaserScan
import numpy as np

#print(uniform(0, 0.1))
class MakeNoise:

    def receiveScan(self, data):
        new_ranges = list()
        for i in range (0, len(data.ranges)):
            #print("before: " + str(data.ranges[i]))
            if not data.ranges[i] == np.inf:
                new_ranges.append(data.ranges[i] + uniform(-self.noise, self.noise))
            else:
                new_ranges.append(data.ranges[i])
            #print("After: " + str(new_ranges[i]))
        data.ranges = new_ranges
        self.pub.publish(data)

    def __init__ (self):
        rospy.init_node('makeNoise', anonymous=True)
        self.noise = rospy.get_param("~noise", 0.01)
        topic_sub = rospy.get_param("~topic_sub", "/camera/scan")
        topic_pub = rospy.get_param("~topic_pub", "/camera/scan/noise")

        rospy.Subscriber(topic_sub, LaserScan, self.receiveScan)
        self.pub = rospy.Publisher(topic_pub, LaserScan, queue_size=1)
        rospy.spin()

if __name__ == '__main__':
    MakeNoise()