#include<iostream>
#include<string>
#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<geometry_msgs/Quaternion.h>
#include<nav_msgs/Odometry.h>
#include<sensor_msgs/Imu.h>
#include<tf/tf.h>
#include"control_test/control_robot.h"

tf::Quaternion q_msg, imu_msg;
double roll, pitch, yaw;
bool init = false;
double imu_ox, imu_oy, imu_oz, imu_ax, imu_ay, imu_az, imu_lx, imu_ly, imu_lz;

void poseCallback(const nav_msgs::Odometry &odom){
    tf::quaternionMsgToTF(odom.pose.pose.orientation, q_msg);
    tf::Matrix3x3(q_msg).getRPY(roll, pitch, yaw);
    init=true;
}

void ImuCallback(const sensor_msgs::Imu &imu_data){
    tf::quaternionMsgToTF(imu_data.orientation, imu_msg);
    tf::Matrix3x3(imu_msg).getRPY(imu_ox, imu_oy, imu_oz);

    imu_ax = imu_data.angular_velocity.x;
    imu_ay = imu_data.angular_velocity.y;
    imu_az = imu_data.angular_velocity.z;

    imu_lx = imu_data.linear_acceleration.x;
    imu_ly = imu_data.linear_acceleration.y;
    imu_lz = imu_data.linear_acceleration.z;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "rotation_publisher");


    double rotation_angle, rotation_vel = 0.3;
    rotation_angle = 90;
    ros::param::get("~rotation_angle",rotation_angle);
    ros::param::get("~rotation_vel",rotation_vel);
    
    //ROS_INFO("rotation_angle:%f rotation_vel:%f",rotation_angle,rotation_vel);
    turnController turn_controller(rotation_vel);
    ROS_INFO("Inital successfully!");
    ros::NodeHandle nh;
    ros::Publisher rotation_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Subscriber pose_sub = nh.subscribe("/odom", 10, poseCallback);
    ros::Subscriber imu_sub = nh.subscribe("/mobile_base/sensors/imu_data", 10, ImuCallback);
	
    sleep(1);
    while(ros::ok()){
        ros::spinOnce();
        if(init){
           // ROS_INFO("imu_oz: %f",imu_oz);
        //     if(turn_controller.TURN(rotation_pub, imu_oz, rotation_angle)){
	 	//         ROS_INFO("rotation finished");
        //     	break;
        //    }
		    if(turn_controller.turn(rotation_pub,imu_oz,rotation_angle)){
	 	        ROS_INFO("rotation finished");
            	break;
           }
    	}
    }
    return 0;

}

