#include <ros/ros.h>
#include "turtlesim/Pose.h"
//接收到订阅的消息后，会进入消息回调函数
void poseCallback(const turtlesim::Pose::ConstPtr& msg){
		ROS_INFO("Turtle pose: x:%0.6f,y:%0.6f",msg->x,msg->y);//将接收到的消息打印出来
}
int main(int argc,char** argv){
   		ros::init(argc,argv,"pose_cubscriber");//初始化ROS节点
 		ros::NodeHandle n;//创建节点句柄
 		ros::Subscriber pose_sub=n.subscribe("/turtle1/pose",10,poseCallback);//创建一个Subscriber，订阅名为/turtle1/pose的topic，注册回调函数poseCallback
		ros::spin();//循环等待回调函数
	return 0;
}