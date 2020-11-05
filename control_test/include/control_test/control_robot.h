#include<iostream>
#include<string>
#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<geometry_msgs/Quaternion.h>
#include<nav_msgs/Odometry.h>
#include<tf/tf.h>
#include"common_func.h"
#include<Eigen/Core>
#include<stack>

struct robotStatus
{
    Eigen::Vector2d position;
    double yaw;
};

class controlMode
{
public:
    virtual void init();
    virtual void run();
    virtual void setArgs();
    virtual void quit();
};

class robotController
{

public:
    std::stack<controlMode> modes_;
    robotStatus robot_;
};


class turnController
{
    public:
    typedef enum{TURN_RIGHT,TURN_LEFT} ORIENTATAION;

    turnController(double basic_velocity):basic_velocity_(basic_velocity),initial_(false){}
        bool JudgeInfo(double theta,double current_orientation);
        bool judgeInfo(double theta,double current_orientation);
        void turnBasic(ros::Publisher& pub,double velocity);
        bool turn(ros::Publisher& pub, double current_orientation, double theta);
        bool TURN(ros::Publisher& pub, double current_orientation, double theta);
    private:
        double last_orientation_;
        double orientation_count = 0.0;
        double basic_velocity_;
        bool initial_;
        double aim_orientation_;
        double current_orientation_;
        ORIENTATAION orientation_;
};

class goController
{
    public:
        goController(double go_velocity):go_velocity_(go_velocity),initial_(false){}
        bool Go(ros::Publisher& pub, double current_px, double current_py, double current_imu_oz, double distance);
        void GoBasic(ros::Publisher& pub, double velocity, double angular);
        double PID_set(double current_yaw, double aim_yaw);
        double Imu_PID_set(double current_imu_data, double aim_imu_data);
    private:
        double go_velocity_;
        bool initial_;

        double init_px_;
        double init_py_;

        //IMU_data
        double aim_imu_oz_;
        double current_imu_oz_;

        //PID set
        double y_error = 0;
        double yy_error = 0;

        double KP = 2.0, KI = 0.012, KD = 0.2;
};