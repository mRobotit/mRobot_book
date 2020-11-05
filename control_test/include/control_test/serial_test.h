#include<iostream>
#include<string>
#include<math.h>
#include<string.h>
#include<ros/ros.h>
#include<tf/tf.h>
#include<serial/serial.h>
#include<geometry_msgs/Twist.h>
#include<sensor_msgs/Imu.h>
#include<geometry_msgs/Quaternion.h>
#include<nav_msgs/Odometry.h>

using namespace std;

const double odom_pose_covariance[36] = {1e-3, 0, 0, 0, 0, 0, 
										0, 1e-3, 0, 0, 0, 0,
										0, 0, 1e6, 0, 0, 0,
										0, 0, 0, 1e6, 0, 0,
										0, 0, 0, 0, 1e6, 0,
										0, 0, 0, 0, 0, 1e3};
const double odom_pose_covariance2[36] = {1e-9, 0, 0, 0, 0, 0, 
										0, 1e-3, 1e-9, 0, 0, 0,
										0, 0, 1e6, 0, 0, 0,
										0, 0, 0, 1e6, 0, 0,
										0, 0, 0, 0, 1e6, 0,
										0, 0, 0, 0, 0, 1e-9};
 
const double odom_twist_covariance[36] = {1e-3, 0, 0, 0, 0, 0, 
										0, 1e-3, 0, 0, 0, 0,
										0, 0, 1e6, 0, 0, 0,
										0, 0, 0, 1e6, 0, 0,
										0, 0, 0, 0, 1e6, 0,
										0, 0, 0, 0, 0, 1e3};
const double odom_twist_covariance2[36] = {1e-9, 0, 0, 0, 0, 0, 
										0, 1e-3, 1e-9, 0, 0, 0,
										0, 0, 1e6, 0, 0, 0,
										0, 0, 0, 1e6, 0, 0,
										0, 0, 0, 0, 1e6, 0,
										0, 0, 0, 0, 0, 1e-9};

#pragma pack(1)
typedef struct _Mpu6050_Str_
{
    short X_data;
    short Y_data;
    short Z_data;
    
}Mpu6050_Str;

typedef union _Upload_Data_
{
    unsigned char buffer[33];

    struct _Sensor_Str_
    {
        /* data */
        unsigned int Header;
        float X_speed;
        float Y_speed;
        float Z_speed;

        float Source_Voltage;
        Mpu6050_Str Link_Accelerometer;
        Mpu6050_Str Link_Gyroscope;

        unsigned char End_flag;
    }Sensor_Str;
    
}Upload_Data;
#pragma pack(4)

class Serial_test{
    public:
        Serial_test();
        ~Serial_test();

        bool ReadFromUstra();
        bool ReadAndWrite();
        void OdomPublish();
        void ImudataPublish();
        void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
        void accelateOffest(float gx, float gy, float gz);
        float invSqrt(float number);
        void cmd_velCallback(const geometry_msgs::Twist &twist_aux);

        serial::Serial Robot_Serial;
    private:
        Upload_Data Reciver_Str, Send_Str;

        unsigned short Offest_Count;

        float GXdata_Offest, GYdata_Offest, GZdata_Offest;

        double x, y, th, vx, vy, vth, dt;

        int baud_data;
        sensor_msgs::Imu Mpu6050;
        std::string usart_port;

        ros::NodeHandle n;
        ros::Subscriber cmd_sub;
        ros::Publisher odom_pub, imu_pub;
        ros::Time current_time, last_time;
};