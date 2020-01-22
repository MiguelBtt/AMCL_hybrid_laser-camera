#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import time
import threading


class PathTalker:

    rospy.init_node("read_robot_path", anonymous=True)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
    time_before = 0.0

    def myThread(self, time_sleep):
        time.sleep(time_sleep)

    
    def threadFunction(self):
        cont_line = 0
        twist = Twist()

        for line in self.file:
            value = line.strip().split(",")
            if cont_line == 0:
                twist.linear.x = 0
                twist.linear.y = 0
                twist.linear.z = 0
                twist.angular.x = 0
                twist.angular.y = 0
                twist.angular.z = 0
                self.pub.publish(twist)
                my_thread = threading.Thread(target=self.myThread, args=(1.0,))
                my_thread.start()
                my_thread.join() 
            if cont_line > 0:
                if cont_line > 1:
                    print("time: " + str(value[0]) + "  prior: " + str(self.time_before))
                    print("time sleep: " + str(float(value[0]) - self.time_before))
                    my_thread = threading.Thread(target=self.myThread, args=(float(value[0])-self.time_before, ))
                    my_thread.start()
                    my_thread.join()                                                         
                    #time.sleep(0.5)
                self.time_before = float(value[0])
                twist.linear.x = float(value[1])
                twist.linear.y = float(value[2])
                twist.linear.z = float(value[3])
                twist.angular.x = float(value[4])
                twist.angular.y = float(value[5])
                twist.angular.z = float(value[6])
                self.pub.publish(twist)
                print("\ncommand: linear[" + str(twist.linear.x) + ", " + str(twist.linear.y) + ", " + str(twist.linear.z) + " ], angular["+ str(twist.angular.x) + ", " + str(twist.angular.y) + ", " + str(twist.angular.z) + " ]")
                

            cont_line+=1

        else:
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            self.pub.publish(twist)
            print("End commands")
            self.file.close()    

    def __init__(self, name_file):
        self.file = open (name_file, "r")
        print("Open file")        
        my_thread = threading.Thread(target = self.threadFunction)
        my_thread.start()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()
        else:
            print("Shutdown")
            self.file.close()



if __name__ == '__main__':
    try:
        path_talker = PathTalker("/home/miguel/catkin_ws/src/simulation_files/saved_files_and_results/test_1/motion_path.txt")
    except rospy.ROSInterruptException:
        pass