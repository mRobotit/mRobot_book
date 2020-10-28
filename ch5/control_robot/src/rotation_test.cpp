#include<iostream>
#include<string>
#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<geometry_msgs/Quaternion.h>
#include<nav_msgs/Odometry.h>
#include<tf/tf.h>
#define UNIT_ANGLE 180 / 3.1149
double rotation_angle, rotation_vel = 0.5;
geometry_msgs::Vector3 rpy;
tf::Quaternion q_msg;
double roll, pitch, yaw, i_yaw, m_yaw=0;
void poseCallback(const nav_msgs::Odometry &odom){
    tf::quaternionMsgToTF(odom.pose.pose.orientation, q_msg);
    tf::Matrix3x3(q_msg).getRPY(roll, pitch, yaw);
}
double T_angle(double a){
    a = fmod(a, 360);
    if(a > 180)a = a - 360;
    else if(a < -180)a = a + 360;
    return a;
}
int main(int argc, char **argv){
    ros::init(argc, argv, "rotation_publisher");
    ros::NodeHandle nh;
    ros::Publisher rotation_pub = nh.advertise<geometry_msgs::Twist> ("/cmd_vel", 10);
    ros::Subscriber pose_sub = nh.subscribe("/odom", 10, poseCallback);
    std::cout << "Please Input the rotation_angle: " << std::endl;
    std::cin >> rotation_angle ;

    rotation_angle = T_angle(rotation_angle);
    double angle = 0;
    double abs_rotation_angle = abs(rotation_angle);
    bool start = true;
    double count = 0;
    while(ros::ok()){
        ros::spinOnce();
        if(start){
            i_yaw = yaw;
            start = false;
        }
        if(angle * UNIT_ANGLE < abs_rotation_angle){
            ros::spinOnce();
            geometry_msgs::Twist r_vel_msgs;
            if(rotation_angle < 0){
                angle = i_yaw - yaw;
                r_vel_msgs.angular.z = -rotation_vel;
                rotation_pub.publish(r_vel_msgs);
            }
            else{
                angle = yaw - i_yaw;
                r_vel_msgs.angular.z = rotation_vel;
                rotation_pub.publish(r_vel_msgs);
            }
            std::cout << "yaw: %.5f" << yaw << "Angle: %.2f " << angle * UNIT_ANGLE << std::endl;
        }
        if(angle *UNIT_ANGLE >= abs_rotation_angle)break;
    }
    return 0;
}
