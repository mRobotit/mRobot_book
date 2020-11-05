#include<iostream>
#include<ros/ros.h>
#include<tf/tf.h>
#include<tf/transform_listener.h>
#include<geometry_msgs/TransformStamped.h>
#include<sensor_msgs/Imu.h>
#include"control_test/control_robot.h"

double px, py, roll, pitch, yaw;
double tf_px, tf_py, e_roll, e_pitch, e_yaw, ee_px, ee_py, ee_roll, ee_pitch, ee_yaw;
double imu_ox, imu_oy, imu_oz, imu_ax, imu_ay, imu_az, imu_lx, imu_ly, imu_lz;//imu_data
bool init = true;
tf::Quaternion q_msg, imu_msg, tf_msg;

void poseCallback(const nav_msgs::Odometry &p_msg){
    tf::quaternionMsgToTF(p_msg.pose.pose.orientation, q_msg);
    tf::Matrix3x3(q_msg).getRPY(roll, pitch, yaw);
    px = p_msg.pose.pose.position.x;
    py = p_msg.pose.pose.position.y;
}

void ImuCallback(const sensor_msgs::Imu &imu_data){
    tf::quaternionMsgToTF(imu_data.orientation, imu_msg);
    tf::Matrix3x3(imu_msg).getRPY(imu_ox, imu_oy, imu_oz);

    imu_ax = imu_data.angular_velocity.x;
    imu_ay = imu_data.angular_velocity.y;
    imu_az = imu_data.angular_velocity.z;

    imu_lx = imu_data.linear_acceleration.x;
    imu_ly = imu_data.linear_acceleration.y;
    imu_lz = imu_data.linear_acceleration.z;
}

int main(int argc, char** argv){

    ros::init(argc, argv, "tf_sub_test");

    double rotation_angle = 90, rotation_vel = 0.3;
    double distance1 = 1, go_vel = 0.3, distance2 = 1;
    ros::param::get("~distance1", distance1);
    ros::param::get("~distance2", distance2);
    ros::param::get("~go_vel", go_vel);
    ros::param::get("~rotation_angle",rotation_angle);
    ros::param::get("~rotation_vel",rotation_vel);

    goController go_controller(go_vel);
    turnController turn_controller(rotation_vel);

    ros::NodeHandle nh;
    ros::Publisher command_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Subscriber pose_sub = nh.subscribe("/odom", 10, poseCallback);
    ros::Subscriber imu_sub = nh.subscribe("/mobile_base/sensors/imu_data", 10, ImuCallback);

    tf::TransformListener tf_listener;
    sleep(1);
    while(ros::ok()){
        ros::spinOnce();
        tf::StampedTransform transform;
        try
        {
            tf_listener.lookupTransform("map", "odom_combined", ros::Time(0), transform);
                
            tf_msg = tf::Quaternion(transform.getRotation().getX(), transform.getRotation().getY(), transform.getRotation().getZ(), transform.getRotation().getW());
            tf::Matrix3x3(tf_msg).getRPY(e_roll, e_pitch, e_yaw);
            tf_px = transform.getOrigin().x();
            tf_py = transform.getOrigin().y();    
            ROS_INFO("x,y,z = %f, %f, %f", transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
            // ROS_INFO("z,w = %f, %f", transform.getRotation().getZ(), transform.getRotation().getW());
        }
        catch(const std::exception& e)
        { 
            ROS_ERROR("%s", e.what());
        }

        if(init){
            ee_px = tf_px;
            ee_py = tf_py;
            ee_yaw = e_yaw;
            init = false;
        }
        else{
            px = px + tf_px / 2;
            py = py + tf_py / 2;
            imu_oz = imu_oz + e_yaw;

            ROS_INFO("px %f py: %f tf_px: %f tf_py: %f ee_px: %f ee_py: %f", px, py, tf_px, tf_py, ee_px, ee_py);
            if(go_controller.Go(command_pub, px, py, imu_oz, distance1)){
                ROS_INFO("Go finished!");
                break;
            }
        }
    }
    
    return 0;
}

