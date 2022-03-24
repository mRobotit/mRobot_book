#include<ros/ros.h>
#include<string>
#include<iostream>
#include<nav_msgs/Odometry.h>
#include<nav_msgs/Path.h>
#include<geometry_msgs/Twist.h>
#include<geometry_msgs/Pose.h>
#include<geometry_msgs/Quaternion.h>
#include<geometry_msgs/PoseStamped.h>
#include<sensor_msgs/Imu.h>
#include<tf/tf.h>

double ix, iy, px, py;
double dis_long;
double robot_v;

// double imu_ox, imu_oy, imu_oz, imu_ix, imu_iy, imu_iz;
// double y_error = 0, yy_error = 0;
// tf::Quaternion imu_msg;
// double KP = 0.02, KI = 0.00015, KD = 0.002;

void poseCallback(const nav_msgs::Odometry &p_msg){
    px = p_msg.pose.pose.position.x;
}

// void ImuCallback(const sensor_msgs::Imu &imu_data){
//     tf::quaternionMsgToTF(imu_data.orientation, imu_msg);
//     tf::Matrix3x3(imu_msg).getRPY(imu_ox, imu_oy, imu_oz);
// }

// double Imu_PID_set(double current_imu_oz, double aim_imu_oz){
//     double error = 0;
//     double p_error = 0;
//     double i_error = 0;
//     double d_error = 0;

//     if (fabs(current_imu_oz - aim_imu_oz)>M_PI)current_imu_oz = -current_imu_oz;

//     error = -current_imu_oz + aim_imu_oz;
//     p_error = error;
//     i_error = p_error + i_error;
//     d_error = error - y_error*2 + yy_error;
//     double pub_vel_y = KP * p_error + KI * i_error + KD * d_error;
//     yy_error = y_error;
//     y_error = error;

//     return pub_vel_y;
// }

int main(int argc,char** argv){

    ros::init(argc, argv, "control_aline");
    ros::NodeHandle nh;
    ros::Publisher command_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Subscriber pose_sub = nh.subscribe("/odom", 10, poseCallback);
    // ros::Subscriber imu_sub = nh.subscribe("moblie_base/sensors/imu_data", 10, ImuCallback);
    std::cout << "Please Input distance(m): " << std::endl;
    std::cin >> dis_long;
    std::cout << "Please Input velocity(m/s): " << std::endl;
    std::cin >> robot_v;

    bool is_start = true;
    double count = 0;
    sleep(3);
    while(ros::ok()){
        ros::spinOnce();
        if(is_start){
            ix = px;
            // imu_iz = imu_oz;
            is_start = false;
        }
        //ros::spinOnce();
        geometry_msgs::Twist com_msg;
        com_msg.linear.x = robot_v;
        // com_msg.linear.y = Imu_PID_set(imu_oz, imu_iz);
        com_msg.linear.y = 0;
        ROS_INFO("Robot velocity[%.2f m/s], Distance covered[%.2f m],[%.2f,%.2f]", com_msg.linear.x, count, ix, px);
        count = px - ix;
        if(count >= dis_long){
            com_msg.linear.x = 0;
            com_msg.linear.y = 0;
            command_pub.publish(com_msg);
            return 0;
        }
        command_pub.publish(com_msg);
    }

    return 0;
}