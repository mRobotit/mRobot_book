#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
int main(int argc,char **argv){
	ros::init(argc,argv,"velocity_publisher");//ROS节点初始化
	ros::NodeHandle nh;//创建句柄，实例化node
	ros::Publisher turtle_vel_pub=nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",10);//创建publisher，发布名为/turtle1/cmd_vel的topic，消息类型为geometry_msgs::Twist，队列长度10
	ros::Rate loop_rate(10);//定义循环发布频率
	int count=0;	
	while(ros::ok()){
		geometry_msgs::Twist vel_msg;//初始化geometry_msgs::Twist类型的消息
		vel_msg.linear.x=0.5;
        	vel_msg.angular.z=0.2;

       	 	turtle_vel_pub.publish(vel_msg);
		ROS_INFO("Publish turtle velocity command[%0.2f m/s,%0.2 rad/s]",vel_msg.linear.x,vel_msg.angular.z);//发布消息

		loop_rate.sleep();//根据定义的发布频率，sleep
	}
	return 0;
}