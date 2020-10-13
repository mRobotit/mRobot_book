#include <actionlib/client/simple_action_client.h>
#include "learning_action/readbookAction.h"
 
//action完成后调用此函数
void doneCb(const actionlib::SimpleClientGoalState& state,
            const learning_action::readbookResultConstPtr& result)
{
    ROS_INFO("Finsh Reading!");
    //任务完成就关闭节点
    ros::shutdown();
}
 
//action的目标任务发送给server且开始执行时，调用此函数
void activeCb(){
   ROS_INFO("Goal is active! Begin to Read.");
}
 
//action任务在执行过程中，server对过程有反馈则调用此函数
void feedbackCb(const learning_action::readbookFeedbackConstPtr& feedback)
{
    //将服务器的反馈输出（读到第几页书）
    ROS_INFO("Reading page: %d", feedback->reading_page);
}
 
int main(int argc, char** argv)
{
ros::init(argc, argv, "readbook_client");
 
    //创建一个action的client，指定action名称为”read_book”
    actionlib::SimpleActionClient<learning_action::readbookAction> client("read_book", true);
 
    ROS_INFO("Waiting for action server to start");
    //等待服务器响应
    client.waitForServer();
    ROS_INFO("Action server started");
    
    //创建一个目标：读10页书
    learning_action::readbookGoal goal;
    goal.total_pages = 10;
    
    //把action的任务目标发送给服务器，绑定上面定义的各种回调函数
    client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
 
    ros::spin();
 
    return 0;
}

