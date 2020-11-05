
/*This code is used to plan the trajectory of the robot
 */

#include <math.h>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <tf/transform_listener.h>
#include <fstream>
#include <iostream>
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"

using namespace std;
using namespace tf;

float x_current;
float y_current;

float normeNextGoal;

class quaternion_ros {
public:
    float w;
    float x;
    float y;
    float z;

    quaternion_ros();

    void toQuaternion(float pitch, float roll, float yaw);
};

void quaternion_ros::toQuaternion(float pitch, float roll, float yaw) {
    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);
    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);

    w = cy * cr * cp + sy * sr * sp;
    x = cy * sr * cp - sy * cr * sp;
    y = cy * cr * sp + sy * sr * cp;
    z = sy * cr * cp - cy * sr * sp;
}

quaternion_ros::quaternion_ros() {
    w = 1;
    x = 0;
    y = 0;
    z = 0;
}

class Path_planned {
public:
    struct Goal {
        float x;
        float y;
        bool visited;
    };

    vector<Goal> Path;

    Path_planned();
    // Path_planned(float x, float y, bool visited);
    void addGoal(float X, float Y, bool visit);
};

Path_planned::Path_planned() {}

// Path_planned(float x, float y, bool visited)

void Path_planned::addGoal(float X, float Y, bool visit) {
    Path_planned::Goal newGoal;
    newGoal.x = X;
    newGoal.y = Y;
    newGoal.visited = visit;
    Path.push_back(newGoal);
}

Path_planned planned_path;
nav_msgs::Path passed_path;
ros::Publisher pub_passed_path;
void pose_callback(
    const nav_msgs::Odometry &poses) { //里程计回调函数,用来计算当前机器人位置与前面目标点的距离,判断是否要发新的幕摆点
    x_current = poses.pose.pose.position.x;
    y_current = poses.pose.pose.position.y;
    passed_path.header = poses.header;
    geometry_msgs::PoseStamped p;
    p.header = poses.header;
    p.pose = poses.pose.pose;
    passed_path.poses.emplace_back(p);
    pub_passed_path.publish(passed_path);
}

int taille_last_path = 0;
bool new_path = false;

//接受规划的路径
void path_callback(const nav_msgs::Path &path) {
    //注意为了rviz显示方便 路径一直在发,但是这里只用接受一次就好,当规划的路径发生变化时候再重新装载
    if ((planned_path.Path.size() == 0) || (path.poses.size() != taille_last_path)) {
        planned_path.Path.clear();
        new_path = true;
        //遍历所有的弓形路径点，把这些添加在vector<Goal> Path;中
        for (int i = 0; i < path.poses.size(); i++) {
            planned_path.addGoal(path.poses[i].pose.position.x, path.poses[i].pose.position.y, false);

            cout << path.poses[i].pose.position.x << " " << path.poses[i].pose.position.y << endl;
        }
        cout << "Recv path size:" << path.poses.size() << endl;
        taille_last_path = path.poses.size();
    }
}

int main(int argc, char *argv[]) {
    srand(time(0));
    ros::init(argc, argv, "next_goal");
    ros::NodeHandle next_goal;
    //订阅机器人的里程计，获得机器人当前的姿态信息
    ros::Subscriber sub1 = next_goal.subscribe("/odom", 1000, pose_callback);
    //订阅在pathplanningnoode发布的路径话题cleaning_path，进入回调path_callback
    ros::Subscriber sub2 =
        next_goal.subscribe("/path_planning_node/cleaning_plan_nodehandle/cleaning_path", 1000, path_callback);
    //这个话题发布是给movebase导航包提供路径规划的接口
    ros::Publisher pub1 = next_goal.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1000);
    //这个话题发布是为了给rviz显示用的
    pub_passed_path = next_goal.advertise<nav_msgs::Path>("/clean_robot/passed_path", 1000);

    ros::Rate loop_rate(10);

    geometry_msgs::PoseStamped goal_msgs;
    int count = 0;
    double angle;
    bool goal_reached = false;
    //获取发送下一个点的阈值，也就是机器人离目标点的距离阈值
    if (!next_goal.getParam("/NextGoal/tolerance_goal", normeNextGoal)) {
        ROS_ERROR("Please set your tolerance_goal");
        return 0;
    }
    ROS_INFO("tolerance_goal=%f", normeNextGoal);

    while (ros::ok()) {
        //订阅函数只有遇到ros::spinOnce();才会去调用回调函数，很关键！！
        ros::spinOnce();
        if (new_path) {
            count = 0;
            new_path = false;
        }
        //当前处理的点
        cout << " count : " << count << endl;
        //如果存在路径
        if (!planned_path.Path.empty()) {
            //当前距离达到了
            if (sqrt(pow(x_current - planned_path.Path[count].x, 2) + pow(y_current - planned_path.Path[count].y, 2)) <=
                normeNextGoal) {
                // count++就会在下一次循环跳到下一个导航点
                count++;
                goal_reached = false;
            }
            if (goal_reached == false) {
                goal_msgs.header.frame_id = "map";///////odom
                goal_msgs.header.stamp = ros::Time::now();
                goal_msgs.pose.position.x = planned_path.Path[count].x;
                goal_msgs.pose.position.y = planned_path.Path[count].y;
                goal_msgs.pose.position.z = 0;
                if (count < planned_path.Path.size()) { //计算发布的yaw
                    angle = atan2(planned_path.Path[count + 1].y - planned_path.Path[count].y,
                                  planned_path.Path[count + 1].x - planned_path.Path[count].x);
                } else {
                    angle = atan2(planned_path.Path[0].y - planned_path.Path[count].y,
                                  planned_path.Path[0].x - planned_path.Path[count].x);
                }
                cout << angle << endl;
                quaternion_ros q;
                q.toQuaternion(0, 0, float(angle));
                goal_msgs.pose.orientation.w = q.w;
                goal_msgs.pose.orientation.x = q.x;
                goal_msgs.pose.orientation.y = q.y;
                if (planned_path.Path[count].x < planned_path.Path[count + 1].x) {
                    goal_msgs.pose.orientation.z = 0;
                }
                if (planned_path.Path[count].x > planned_path.Path[count + 1].x) {
                    goal_msgs.pose.orientation.z = 2;
                }

                cout << " NEW GOAL " << endl;
                cout << " x = " << planned_path.Path[count].x << " y = " << planned_path.Path[count].y << endl;

                goal_reached = true;
                //把设定的导航点发布出去
                pub1.publish(goal_msgs);
            }
            cout << x_current << " " << y_current << endl;
            //当前
            cout << planned_path.Path[count].x << " " << planned_path.Path[count].y << endl;
            //目标
            cout << " DISTANCE : "
                 << sqrt((x_current - planned_path.Path[count].x) * (x_current - planned_path.Path[count].x) +
                         (y_current - planned_path.Path[count].y) * (y_current - planned_path.Path[count].y))
                 << endl;
            // 距离公式
        }

        loop_rate.sleep();
    }

    return 0;
}
