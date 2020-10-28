#include<ros/ros.h>
#include<string>
#include<iostream>
#include<geometry_msgs/Pose.h>
#include<nav_msgs/Odometry.h>
#include<geometry_msgs/Twist.h>
#include<time.h>
double ix, iy, px, py;//initation & pose x,y
double dis_long;//the distance need to move
double robot_v = 0.3;//the linear_velocity with the robot
void poseCallback(const nav_msgs::Odometry &p_msg){
    px = p_msg.pose.pose.position.x;
    py = p_msg.pose.pose.position.y;
    ROS_INFO("Robot walked %.2f m",px);
}
int main(int argc,char** argv){
    ros::init(argc, argv, "command_publisher");
    ros::NodeHandle nh;
    ros::Publisher command_pub = nh.advertise<geometry_msgs::Twist> ("/cmd_vel", 10);
    ros::Subscriber pose_sub = nh.subscribe("/odom", 10, poseCallback);
    std::cout << "Please Input distance(m) :"<< std::endl;
    std::cin >> dis_long;
    bool is_start = true;
    double count = 0;
    while(ros::ok()){
        ros::spinOnce();
        if(is_start)
        {
            ix = px;
            is_start =false;
        }
        if(count < dis_long){
            ros::spinOnce();
            geometry_msgs::Twist com_msg;
            if(count < dis_long / 5 || count > dis_long * 4 / 5){
                com_msg.linear.x = robot_v / 3;
            }
            else{
                com_msg.linear.x = robot_v;
            }
            ROS_INFO("Publish turtle velocity command[%.2f m/s] distance: %.2f", com_msg.linear.x, count);
            command_pub.publish(com_msg);
            count = px - ix;
        }
        if(count >= dis_long)break;
    }
    return 0;
}   
