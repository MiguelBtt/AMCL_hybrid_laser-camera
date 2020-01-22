#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan

class listener:


    def callback0(self, data):
        print("0-> " + str(len(data.ranges)))
        self.scan.header.stamp = rospy.Time.now()
        if self.info_init_msg_0 == False:
            self.scan.angle_min = -0.785399
            self.scan.angle_max =  2.7488935
            self.scan.angle_increment = data.angle_increment
            self.scan.range_max = data.range_max
            self.scan.range_min = data.range_min
            self.info_init_msg_0 = True
            self.scan.header.frame_id = "camera_link"
            print("receive init scan msg")

        if self.info_init_msg_0 == True  and  self.info_init_msg_90 == True  and  self.info_init_msg_180 == True  and  self.info_init_msg_270 == True:
            self.scan.ranges = data.ranges + self.scan_90 + self.scan_180 + self.scan_270
            self.pub.publish(self.scan)
            print("Pub scan with len: " + str(len(self.scan.ranges)))
            self.info_init_msg_90 = False
            self.info_init_msg_180 = False
            self.info_init_msg_270 = False

        print(self.info_init_msg_90)

    def callback90(self, data):
        print("90-> " + str(len(data.ranges)))
        self.info_init_msg_90 = True
        self.scan_90 = data.ranges


    def callback180(self, data):
        print("180-> " + str(len(data.ranges)))
        self.info_init_msg_180 = True
        self.scan_180 = data.ranges

    def callback270(self, data):
        print("270-> " + str(len(data.ranges)))
        self.info_init_msg_270 = True
        self.scan_270 = data.ranges

    def __init__(self):
        rospy.init_node('scan_listener', anonymous=True)
        rospy.Subscriber("/camera_0/scan", LaserScan, self.callback0)
        rospy.Subscriber("/camera_90/scan", LaserScan, self.callback90)
        rospy.Subscriber("/camera_180/scan", LaserScan, self.callback180)
        rospy.Subscriber("/camera_270/scan", LaserScan, self.callback270)
        self.pub = rospy.Publisher("/merged_scan", LaserScan, queue_size=1)

        self.scan = LaserScan()
        self.scan_90 = list()
        self.scan_180 = list()
        self.scan_270 = list()
        self.info_init_msg_0 = False
        self.info_init_msg_90 = False
        self.info_init_msg_180 = False
        self.info_init_msg_270 = False
        rospy.spin()



if __name__ == '__main__':
    listener()