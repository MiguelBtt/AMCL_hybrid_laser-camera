#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from math import pow


class ListenerPath:

    def gazeboCallback(self, twist):
        #cmd_vel = twist
        now_time = rospy.Time.now().secs + (rospy.Time.now().nsecs * pow(10, -9))
        print("\n Time: " + str(now_time))
        print(twist)
        self.file.write(str(now_time) + ", " + str(twist.linear.x) + ", " + str(twist.linear.y) + ", " + str(twist.linear.z) + ", " + str(twist.angular.x) + ", " + str(twist.angular.y) + ", " + str(twist.angular.z) + "\n")   
        
    def __init__(self, name_file):
        rospy.init_node('save_robot_path', anonymous=True)
        rospy.Subscriber("/cmd_vel", Twist, self.gazeboCallback)
        self.file = open (name_file, "w")
        self.file.write("time, linear.x, linear.y, linear.z, angular.x, angular.y, angular.z\n")
        rospy.spin()
        self.file.close()


if __name__ == '__main__':

    listenerPath = ListenerPath("/home/miguel/catkin_ws/src/simulation_files/saved_files_and_results/test_1/motion_path.txt")