#include<ros/ros.h>

int main(int agrc, char ** agrv){
    ros::init(agrc, agrv, "helloworld");
    ROS_INFO("helloworld");
    return 0;
}