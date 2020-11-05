#include<ros/ros.h>
#include<string>
#include<iostream>
#include<geometry_msgs/Pose.h>
#include<nav_msgs/Odometry.h>
#include<geometry_msgs/Twist.h>
#include<sensor_msgs/Imu.h>
#include<time.h>
#include"control_test/control_robot.h"

double imu_ox, imu_oy, imu_oz, imu_ax, imu_ay, imu_az, imu_lx, imu_ly, imu_lz;//imu_data
double px, py, roll, pitch, yaw;//initation & pose x,y
bool init = false;
tf::Quaternion q_msg, imu_msg;

void poseCallback(const nav_msgs::Odometry &p_msg){
    tf::quaternionMsgToTF(p_msg.pose.pose.orientation, q_msg);
    tf::Matrix3x3(q_msg).getRPY(roll, pitch, yaw);
    px = p_msg.pose.pose.position.x;
    py = p_msg.pose.pose.position.y;
    init = true;
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

int main(int argc,char** argv){
    ros::init(argc, argv, "go_cmd_publisher");
    
    double distance1, go_vel = 0.3;
    ros::param::get("~distance1", distance1);
    ros::param::get("~go_vel", go_vel);
    
    
    goController go_controller(go_vel);
    
    ros::NodeHandle nh;
    ros::Publisher command_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Subscriber pose_sub = nh.subscribe("/odom", 10, poseCallback);
    ros::Subscriber imu_sub = nh.subscribe("/mobile_base/sensors/imu_data", 10, ImuCallback);

    sleep(1);

    while(ros::ok()){
        ros::spinOnce();
        
        if(init){
            //ROS_INFO("INITIAL imu_oz: %f", imu_oz);
            if(go_controller.Go(command_pub, px, py, imu_oz, distance1)){
                ROS_INFO("Go finished!");
                break;
            }


            // if(go_controller.Go(command_pub, px, yaw, distance)){
            //     ROS_INFO("Go finished!");
            //     break;
            // }
        }
    }

    return 0;
}