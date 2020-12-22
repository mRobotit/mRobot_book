#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "learning_action/ReadbookAction.h"

//服务器发送任务目标后，调用该函数执行任务
void execute(const learning_action::ReadbookGoalConstPtr& goal, actionlib::SimpleActionServer<learning_action::ReadbookAction>* as)
{
    ros::Rate r(1);
    learning_action::ReadbookFeedback feedback;
    ROS_INFO("Begin to read %d pages", goal->total_pages);
    for (int  i=0; i < goal->total_pages; i++)
    {
        feedback.reading_page = i;

        //反馈任务执行的过程
        as->publishFeedback(feedback);
        r.sleep();
    }
    ROS_INFO("All pages is read.");
    as->setSucceeded();
}
int main(int argc, char** argv)
{
    
    //初始化ROS节点
    ros::init(argc, argv, "readbook_server");

    //创建节点句柄
    ros::NodeHandle nh;

    //创建一个aciton服务器，接受名称”read_book”的aciton任务
    actionlib::SimpleActionServer<learning_action::ReadbookAction> server(nh, "read_book", boost::bind(&execute, _1, &server), false);
    
    //服务器启动
    server.start();
    
    ros::spin();
    return 0;
}

