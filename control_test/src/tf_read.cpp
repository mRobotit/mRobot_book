#include<iostream>
#include<ros/ros.h>
#include<tf/tf.h>
#include<tf/transform_listener.h>
#include<geometry_msgs/TransformStamped.h>
#include<nav_msgs/Odometry.h>
#include<sensor_msgs/Imu.h>

double px, py, tf_px, tf_py;
double roll, pitch, yaw, imu_ox, imu_oy, imu_oz;
tf::Quaternion q_msg, imu_msg, tf_msg;

void poseCallback(const nav_msgs::Odometry &p_msg){
    tf::quaternionMsgToTF(p_msg.pose.pose.orientation, q_msg);
    tf::Matrix3x3(q_msg).getRPY(roll, pitch, yaw);
    px = p_msg.pose.pose.position.x;
    py = p_msg.pose.pose.position.y;
    // init = true;
}

void ImuCallback(const sensor_msgs::Imu &imu_data){
    tf::quaternionMsgToTF(imu_data.orientation, imu_msg);
    tf::Matrix3x3(imu_msg).getRPY(imu_ox, imu_oy, imu_oz);

    // imu_ax = imu_data.angular_velocity.x;
    // imu_ay = imu_data.angular_velocity.y;
    // imu_az = imu_data.angular_velocity.z;

    // imu_lx = imu_data.linear_acceleration.x;
    // imu_ly = imu_data.linear_acceleration.y;
    // imu_lz = imu_data.linear_acceleration.z;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "tf_readandpub");
    ros::NodeHandle nh;

    ros::Subscriber pose_sub = nh.subscribe("/odom", 10, poseCallback);
    ros::Subscriber imu_sub = nh.subscribe("/mobile_base/sensors/imu_data", 10, ImuCallback);
    ros::Publisher process_odom_pub = nh.advertise<nav_msgs::Odometry>("/process_odom", 50);

    tf::TransformListener tf_listener;
    // ros::Rate rate(1);
    double e_roll, e_pitch, e_yaw;
    
    while(ros::ok()){
        
        tf::StampedTransform transform;
        try
        {
            tf_listener.lookupTransform("map", "odom_combined", ros::Time(0), transform);

            tf_msg = tf::Quaternion(transform.getRotation().getX(), transform.getRotation().getY(), transform.getRotation().getZ(), transform.getRotation().getW());
            tf::Matrix3x3(tf_msg).getRPY(e_roll, e_pitch, e_yaw);
            tf_px = transform.getOrigin().x();
            tf_py = transform.getOrigin().y();
            // ROS_INFO("x: %f y: %f roll: %f pitch: %f yaw: %f",transform.getOrigin().x(),transform.getOrigin().y(), e_roll, e_pitch, e_yaw);
        }
        catch(const std::exception& e)
        {
            ROS_ERROR("%s", e.what());
        }

        nav_msgs::Odometry process_odom;
        process_odom.header.stamp = ros::Time::now();
        process_odom.header.frame_id = "process_odom";

        process_odom.pose.pose.position.x = px - tf_px;
        process_odom.pose.pose.position.y = py - tf_py;
        process_odom.pose.pose.position.z = 0.0;
        process_odom.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll - e_roll, pitch - e_pitch, yaw - e_yaw);

        process_odom_pub.publish(process_odom);

        // rate.sleep();
        ros::spinOnce();
    }
    return 0;

}