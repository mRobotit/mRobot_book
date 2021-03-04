#include<iostream>
#include<string>
#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<geometry_msgs/Quaternion.h>
#include<nav_msgs/Odometry.h>
#include<tf/tf.h>

#define UNIT_ANGLE 180/3.1149

double rotation_angle, rotation_vel;
double roll, pitch, yaw, i_yaw;

double T_angle(double angle){
    angle = fmod(angle, 360);
    if(angle > 180)angle  = angle -360;
    else if(angle < -180) angle = angle + 360;
    return angle;
}

void poseCallback(const nav_msgs::Odometry &odom){
    tf::Quaternion q_msg;
    tf::quaternionMsgToTF(odom.pose.pose.orientation, q_msg);
    tf::Matrix3x3(q_msg).getRPY(roll, pitch, yaw);
}

int main(int argc, char ** argv){

    ros::init(argc, argv, "control_rotation");
    ros::NodeHandle nh;
    ros::Publisher rotation_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Subscriber pose_sub = nh.subscribe("/odom", 10, poseCallback);

    std::cout << "Please Input the rotation_angle: "<< std::endl;
    std::cin >> rotation_angle;
    std::cout << "Please Input the rotation_velocity: "<< std::endl;
    std::cin >> rotation_vel;
    
    double angle = 0;

    bool is_start = true;

    while(ros::ok() && angle*UNIT_ANGLE < rotation_angle){
        ros::spinOnce();
        if(is_start){
            i_yaw = yaw;
            is_start = false;
        }
        geometry_msgs::Twist vel_msgs;
        if(T_angle(rotation_angle) < 0){
            angle = i_yaw - yaw;
            vel_msgs.angular.z = -rotation_vel;
            rotation_pub.publish(vel_msgs);
        }
        else{
            angle = yaw - i_yaw;
            vel_msgs.angular.z = rotation_vel;
            rotation_pub.publish(vel_msgs);
        }
        ROS_INFO("Robot rotation_velocity[%.2f rad/s], Angle covered[%.2f ]", rotation_vel, angle*UNIT_ANGLE);

    }

    return 0;
}