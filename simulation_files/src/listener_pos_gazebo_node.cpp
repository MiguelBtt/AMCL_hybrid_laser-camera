/*
This node read the robot infomation that it is publish for gazebo.
After publish two topic:
- pub_path_gazebo: publish infomation for using like in rviz for painting the robot path
- pub_pose_gazebo: publish robot position
author: Miguel Beteta
@email: betetamiguel@hotmail.com
*/
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "gazebo_msgs/ModelStates.h"
#include "iostream"
#include "tf/tf.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"

nav_msgs::Path  pub_path;
std::string robot_name;
ros::Publisher pub_path_gazebo, pub_pose_gazebo;

void posModelCallback(const gazebo_msgs::ModelStates::ConstPtr &msg);

int main(int argc, char **argv){

    ros::init(argc, argv, "listener_pos_gazebo");
    ros::NodeHandle nh_("~"); // without "~" get.Param does not work
    nh_.param("robot_name", robot_name, std::string("turtlebot3_burger"));
    ROS_INFO( "Listener_pos_gazebo node  is running -> Robot Name is  %s", robot_name);
    std::cout << "listener_pos_gazebo node  is running -> Robot Name is " << robot_name << std::endl;
    ros::Rate loop_rate(10);

    //robot_name.data = "turtlebot3_burger";
    pub_path.header.frame_id = "map";

    ros::Subscriber sub = nh_.subscribe("/gazebo/model_states", 1, posModelCallback);
    pub_path_gazebo = nh_.advertise<nav_msgs::Path>("/listener_pos_gazebo/pub_path_gazebo", 1);
    pub_pose_gazebo = nh_.advertise<geometry_msgs::PoseStamped>("/listener_pos_gazebo/pub_pose_gazebo", 1);

    while(ros::ok()){

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

void posModelCallback(const gazebo_msgs::ModelStates::ConstPtr &msg)
{  
    geometry_msgs::PoseStamped path_pose_stamp;
    path_pose_stamp.header.frame_id = "map";
    path_pose_stamp.header.stamp = ros::Time::now();
    for(int i=0; i<msg->name.size(); i++){
        if(msg->name[i] == robot_name){
            /*tf::Quaternion quat;
            quat.setX(msg->pose[i].orientation.x);
            quat.setY(msg->pose[i].orientation.y);
            quat.setZ(msg->pose[i].orientation.z);
            quat.setW(msg->pose[i].orientation.w);
            tf::Matrix3x3 m(quat);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            //Show info in terminal
            std::cout << "/nPos robot in gazebo is: " << std::endl;
            std::cout << "   Pos-> X:" << msg->pose[i].position.x <<  "  Y:" << msg->pose[i].position.y <<  "  Z:" << msg->pose[i].position.z << std::endl;
            std::cout << "   Ori-> X:" << msg->pose[i].orientation.x <<  "  Y:" << msg->pose[i].orientation.y <<  "  Z:" << msg->pose[i].orientation.z <<  "  W:" << msg->pose[i].orientation.w << std::endl;
            std::cout << "   Ori-> roll:" << roll <<  "  pitch:" << pitch <<  "  yaw:" << yaw << std::endl;*/

            path_pose_stamp.pose.position = msg->pose[i].position;
            path_pose_stamp.pose.orientation = msg->pose[i].orientation;
            pub_path.poses.push_back(path_pose_stamp);
            pub_path_gazebo.publish(pub_path);
            pub_pose_gazebo.publish(path_pose_stamp);
        }
    }
}