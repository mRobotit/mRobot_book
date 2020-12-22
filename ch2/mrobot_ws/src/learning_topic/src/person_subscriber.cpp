#include  <ros/ros.h>
#include "learning_topic/Person.h"

// 接收到订阅的消息后，会进入消息回调函数
void personInfoCallback(const learning_topic::Person::ConstPtr& msg) {

    // 将接收到的消息打印出来
    ROS_INFO("Subscribe Person Info:name:%s age:%d, sex:%d", msg->name.c_str(), msg->age, msg->sex);
}

int main(int argc, char** argv) {
    
    ros::init(argc, argv, "person_subscriber");// 初始化ROS节点

    ros::NodeHandle nh;// 创建节点句柄

    /*subscribe()调用用于告诉ROS您希望接收关于给定主题的消息。
    *这将调用对ROS主节点的调用，该节点保存谁在发布谁在订阅的注册表。
    *消息被传递给一个回调函数，这里叫做chatterCallback。
    *subscribe()返回一个订阅者对象，您必须一直持有该对象，直到您希望取消订阅。当订阅者对象的所有副本超出作用域时，此回调将自动从此主题取消订阅。
    *subscribe()函数的第二个参数是消息队列的大小。如果消息到达的速度快于其被处理的速度，这是在开始丢弃最老的消息之前将被缓冲的消息数量。*/
    ros::Subscriber person_info_sub = nh.subscribe("/person_info", 10, personInfoCallback);// 创建一个Subscriber，订阅名为/person_info的topic，注册回调函数personInfoCallback

    /*spin()将进入一个循环，泵送回调。在这个版本中，所有回调都将从这个线程(主线程)中调用。
    *ros::spin()将在按下Ctrl-C或节点被主服务器关闭时退出。*/
    ros::spin();// 循环等待回调函数
    return 0;
}