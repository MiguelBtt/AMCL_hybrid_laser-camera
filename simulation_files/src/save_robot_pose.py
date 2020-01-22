#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Twist, PoseArray
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker
from simulation_files.msg import num_markers, pixels_cloud, coeff_sensor, marker_error
import math

class Listener:

    gazebo_pose = Pose()
    start_time = 0.0
    marker_time = Marker()
    count = -1
    pub = rospy.Publisher('marker_time', Marker, queue_size = 1)
    prior_time = rospy.Time()
    coeff_marker = 0
    coeff_laser = 0

    def numPointsCloudCallback(self, particles):
        if self.start_time > 0.0:
            now_time = float(rospy.Time.now().secs) + (float(rospy.Time.now().nsecs) * pow(10, -9))
            run_time = now_time - self.start_time
            #print(str(now_time) + " - " + str(self.start_time) + " = " + str(run_time))
            self.file_num_points.write(str(run_time) + ",")
            self.file_num_points.write(str(len(particles.poses)) + "\n")
            print("Num particles: " + str(len(particles.poses)))
        else:
            self.file_num_points.write("0.0,")
            self.file_num_points.write(str(len(particles.poses)) + "\n")
            print("Incial value numParticles= " + str(len(particles.poses)))

    def numMarkersCallback(self, markers):
        #print(self.start_time)
        if self.start_time > 0.0:
            now_time = float(rospy.Time.now().secs) + (float(rospy.Time.now().nsecs) * pow(10, -9))
            run_time = now_time - self.start_time
            #print(str(now_time) + " - " + str(self.start_time) + " = " + str(run_time))
            self.file_num_markers.write(str(run_time) + ",")
            self.file_num_markers.write(str(markers.number.data) + "\n")
            #print("Num markers: " + str(markers.number.data) + "\n")

    def gazeboPosCallback(self, model_state):

        #print("gazeboCallback")
        i = model_state.name.index(self.name_robot)
        #print("pos turtle: " + str(i))
        self.gazebo_pose = model_state.pose[i]
        #print(self.gazebo_pose)

    def amclPosCallback(self, pose_msg):

        #print("amclCallback")

        """if self.zero_time == rospy.Time():
            self.zero_time = rospy.Time.now()
            self.start_time = float(self.zero_time.secs) + (float(self.zero_time.nsecs) * pow(10, -9))
            print("start_time:" + str(self.start_time) + "\n")

        else:"""
        if self.start_time > 0.0:
            now_time = float(rospy.Time.now().secs) + (float(rospy.Time.now().nsecs) * pow(10, -9))
            run_time = now_time - self.start_time
            print(str(now_time) + " - " + str(self.start_time) + " = " + str(run_time))
            self.file.write(str(run_time) + ",")

            quat = (self.gazebo_pose.orientation.x, self.gazebo_pose.orientation.y, self.gazebo_pose.orientation.z, self.gazebo_pose.orientation.w)
            euler = euler_from_quaternion(quat)
            gazebo_yaw = (euler[2] * 180)/math.pi
            if gazebo_yaw < 0:
                gazebo_yaw = gazebo_yaw + 360 * ((gazebo_yaw//-360) + 1)
            else:
                gazebo_yaw = gazebo_yaw - 360 * (gazebo_yaw//360)
  
            #self.file.write(str(self.gazebo_pose.position.x) + ", " + str(self.gazebo_pose.position.y) + ", " + str(gazebo_yaw) + ", ")
            print("Gazebo position: " + str(self.gazebo_pose.position.x) + ", " + str(self.gazebo_pose.position.y) + ", " + str(gazebo_yaw))

            quat = (pose_msg.pose.pose.orientation.x, pose_msg.pose.pose.orientation.y, pose_msg.pose.pose.orientation.z, pose_msg.pose.pose.orientation.w)
            euler = euler_from_quaternion(quat)
            amcl_yaw = (euler[2] * 180)/math.pi 
            if amcl_yaw < 0:
                amcl_yaw = amcl_yaw + 360 * ((amcl_yaw//-360) + 1)
            else:
                amcl_yaw = amcl_yaw - 360 * (amcl_yaw//360)

            diff_yaw = amcl_yaw - gazebo_yaw
            if diff_yaw > 180:
                diff_yaw = diff_yaw - 360
            elif diff_yaw < -180: 
                diff_yaw = diff_yaw + 360 

            #self.file.write(str(pose_msg.pose.pose.position.x) + ", " + str(pose_msg.pose.pose.position.y) + ", " + str(amcl_yaw) + ", ")
            print("Amcl position: " + str(pose_msg.pose.pose.position.x) + ", " + str(pose_msg.pose.pose.position.y) + ", " + str(amcl_yaw))
            #self.file.write(str(pose_msg.pose.covariance[0]) + ", " + str(pose_msg.pose.covariance[7]) + ", " + str(pose_msg.pose.covariance[35]) + "\n")
            print("Amcl covariance: " + str(pose_msg.pose.covariance[0]) + ", " + str(pose_msg.pose.covariance[7]) + ", " + str(pose_msg.pose.covariance[35]))
            print("\n")
            
            #x_error = abs(self.gazebo_pose.position.x - (pose_msg.pose.pose.position.x))
            #y_error = abs(self.gazebo_pose.position.y - (pose_msg.pose.pose.position.y))
            x_error = self.gazebo_pose.position.x - (pose_msg.pose.pose.position.x)
            y_error = self.gazebo_pose.position.y - (pose_msg.pose.pose.position.y)
            self.file.write(str(x_error) + ",")
            self.file.write(str(y_error) + ",")
            self.file.write(str(math.sqrt((x_error*x_error) + (y_error*y_error))) + ",")
            self.file.write(str(diff_yaw) + ",")
            self.file.write(str(math.sqrt(pose_msg.pose.covariance[0])) + "," + str(math.sqrt(pose_msg.pose.covariance[7])))
            #self.file.write(str(pose_msg.pose.covariance[0]) + "," + str(pose_msg.pose.covariance[7]))
            self.file.write("\n")
            self.file_ATR.write(str(self.gazebo_pose.position.x) + "," + str(self.gazebo_pose.position.y ) + "," + str(pose_msg.pose.pose.position.x) + "," + str(pose_msg.pose.pose.position.y) + "\n")

            if int(rospy.Time.now().secs) >= (int(self.prior_time.secs) + 2):
                self.prior_time = rospy.Time.now()
                self.marker_time.pose = pose_msg.pose.pose
                self.marker_time.text = "__" + str(int(run_time))
                self.count += 1
                self.marker_time.id = self.count
                self.pub.publish(self.marker_time)
            

    def velCallback(self, vel):
        if self.start_time == 0.0:
            zero_time = rospy.Time.now()
            self.start_time = float(zero_time.secs) + (float(zero_time.nsecs) * pow(10, -9))
            print("start_time:" + str(self.start_time) + "\n")


    def __init__ (self):

        rospy.init_node('save_robot_pose', anonymous=True)
        rospy.Subscriber(rospy.get_param("~topic_pose"), PoseWithCovarianceStamped, self.amclPosCallback)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.gazeboPosCallback)
        rospy.Subscriber("num_detecte_markers", num_markers, self.numMarkersCallback)
        rospy.Subscriber("/particlecloud", PoseArray, self.numPointsCloudCallback)
        rospy.Subscriber("/cmd_vel", Twist, self.velCallback)
        
        self.name_robot = rospy.get_param("~robot_name")
        self.file = open (rospy.get_param("~save_file_path") + ".txt", "w")
        self.file_num_markers = open (rospy.get_param("~save_file_path") + "_num_markers.txt", "w")
        self.file_num_points = open (rospy.get_param("~save_file_path") + "_num_particles.txt", "w")
        self.file_ATR = open (rospy.get_param("~save_file_path") + "_ATR.txt", "w")
        self.file.write("Time,X position error,Y position error,Distance error,YAW error,X deviation,Y deviation\n")
        self.file_num_markers.write("Time,Num Markers\n")
        self.file_num_points.write("Time,Num Particles\n")
        self.file_ATR.write("X_gazebo,Y_gazebo,X_amcl,Y_amcl\n")
        
        
        self.marker_time.header.frame_id = "map"
        self.marker_time.ns = "path_time"
        self.marker_time.type = 9
        self.marker_time.action = 0
        self.marker_time.scale.x = 0.15
        self.marker_time.scale.y = 0.15
        self.marker_time.scale.z = 0.15
        self.marker_time.color.b = 1.0
        self.marker_time.color.a = 1.0
        self.marker_time.lifetime = rospy.Duration()

        rospy.spin()
        self.file.close()
        self.file_num_markers.close()
        self.file_num_points.close()
        self.file_ATR.close()

if __name__ == '__main__':
    listener = Listener()