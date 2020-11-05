#include"control_test/serial_test.h"

Serial_test::Serial_test()
{
    memset(&Reciver_Str, 0, sizeof(Reciver_Str));
    memset(&Send_Str, 0, sizeof(Send_Str));
    x = y = th = vx = vy = vth = dt = 0.0;
    ROS_INFO("initial~");

    Send_Str.Sensor_Str.Header = 0XFEFEFEFE;
    Send_Str.Sensor_Str.End_flag = 0XEE;

    ros::NodeHandle nh("~");
    nh.param<std::string>("usart_port", this->usart_port, "/dev/ttyUSB0");
    nh.param<int>("baud_data", this->baud_data, 115200);
    this->cmd_sub = nh.subscribe("/serial_test/cmd_vel", 100 , &Serial_test::cmd_velCallback, this);
    this->odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
    this->imu_pub = nh.advertise<sensor_msgs::Imu>("/serial_test/imu_data",50);

    ROS_INFO("Initial success!");

    try
    {
        Robot_Serial.setPort(this->usart_port);
        Robot_Serial.setBaudrate(this->baud_data);
        serial::Timeout to = serial::Timeout::simpleTimeout(2000);
        Robot_Serial.setTimeout(to);
        Robot_Serial.open();
    }
    catch(serial::IOException& e)
    {
        ROS_INFO("Unable to open port!");
    }
    if(Robot_Serial.isOpen())ROS_INFO("Serial port opened!");
    
    
}
Serial_test::~Serial_test()
{
    Robot_Serial.close();
}

void Serial_test::cmd_velCallback(const geometry_msgs::Twist &twist_aux)
{
	/** process callback function msgs**/
    
	Send_Str.Sensor_Str.X_speed = twist_aux.linear.x;
	Send_Str.Sensor_Str.Z_speed = twist_aux.angular.z;
    ROS_INFO("Send to USTRA! X: %f  Z: %f", Send_Str.Sensor_Str.X_speed, Send_Str.Sensor_Str.Z_speed);
	Robot_Serial.write(Send_Str.buffer, sizeof(Send_Str.buffer));
}

bool Serial_test::ReadFromUstra(){
    unsigned char CheckSumBuffer[1];

    Robot_Serial.read(Reciver_Str.buffer,sizeof(Send_Str.buffer));

    if(Reciver_Str.Sensor_Str.Header == 0XFEFEFEFE){
        if(Reciver_Str.Sensor_Str.End_flag = 0XEE){
            vx = Reciver_Str.Sensor_Str.X_speed * 1.0;
            Mpu6050.linear_acceleration.x = Reciver_Str.Sensor_Str.Link_Accelerometer.X_data / 16384.0;
            Mpu6050.linear_acceleration.y = Reciver_Str.Sensor_Str.Link_Accelerometer.Y_data / 16384.0;
            Mpu6050.linear_acceleration.z = Reciver_Str.Sensor_Str.Link_Accelerometer.Z_data / 16384.0;

            Mpu6050.angular_velocity.x = Reciver_Str.Sensor_Str.Link_Gyroscope.X_data * 0.001064;
            Mpu6050.angular_velocity.y = Reciver_Str.Sensor_Str.Link_Gyroscope.Y_data * 0.001064;
            Mpu6050.angular_velocity.z = Reciver_Str.Sensor_Str.Link_Gyroscope.Z_data * 0.001064;
            
            return true;
        }
    }
    return false;
}

void Serial_test::OdomPublish(){
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();;
    odom.header.frame_id = "odom";

    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x =  this->vx;
    odom.twist.twist.linear.y =  this->vy;
    odom.twist.twist.angular.z = this->vth;		

    if(this->vx == 0)
    {
    	memcpy(&odom.pose.covariance, odom_pose_covariance2, sizeof(odom_pose_covariance2));
    	memcpy(&odom.twist.covariance, odom_twist_covariance2, sizeof(odom_twist_covariance2));
    }
    else
    {
    	memcpy(&odom.pose.covariance, odom_pose_covariance, sizeof(odom_pose_covariance));
    	memcpy(&odom.twist.covariance, odom_twist_covariance, sizeof(odom_twist_covariance));
    }				

    odom_pub.publish(odom);
}

void Serial_test::ImudataPublish(){
    sensor_msgs::Imu ImuSensor;

	ImuSensor.header.stamp = ros::Time::now(); 
	ImuSensor.header.frame_id = "gyro_link"; 

	ImuSensor.orientation.x = 0.0; 
	ImuSensor.orientation.y = 0.0; 
	ImuSensor.orientation.z = Mpu6050.orientation.z;
	ImuSensor.orientation.w = Mpu6050.orientation.w;

	ImuSensor.orientation_covariance[0] = 1e6;
	ImuSensor.orientation_covariance[4] = 1e6;
	ImuSensor.orientation_covariance[8] = 1e-6;

	ImuSensor.angular_velocity.x = 0.0;		
	ImuSensor.angular_velocity.y = 0.0;
	ImuSensor.angular_velocity.z = Mpu6050.angular_velocity.z;

	ImuSensor.angular_velocity_covariance[0] = 1e6;
	ImuSensor.angular_velocity_covariance[4] = 1e6;
	ImuSensor.angular_velocity_covariance[8] = 1e-6;

	ImuSensor.linear_acceleration.x = 0; 
	ImuSensor.linear_acceleration.y = 0; 
	ImuSensor.linear_acceleration.z = 0;  

	imu_pub.publish(ImuSensor); 
}

bool Serial_test::ReadAndWrite(){
    last_time = ros::Time::now();
    while(ros::ok()){
        current_time = ros::Time::now();
        dt = (current_time - last_time).toSec();

        if(ReadFromUstra()){
            double delta_x = (vx * cos(th) + vy * sin(th)) * dt;
            double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
            double delta_th = vth * dt;
            this->x += delta_x;
            this->y += delta_y;
            this->th += delta_th;

            if(Offest_Count < 40){
                Offest_Count ++;
                accelateOffest(Mpu6050.angular_velocity.x, Mpu6050.angular_velocity.y, Mpu6050.angular_velocity.z);
            }
            else{
                Offest_Count = 40;
                Mpu6050.angular_velocity.x = Mpu6050.angular_velocity.x - GXdata_Offest;
                Mpu6050.angular_velocity.y = Mpu6050.angular_velocity.y - GYdata_Offest;
                Mpu6050.angular_velocity.z = Mpu6050.angular_velocity.z - GZdata_Offest;

                MahonyAHRSupdateIMU(Mpu6050.angular_velocity.x, Mpu6050.angular_velocity.y, Mpu6050.angular_velocity.z, Mpu6050.linear_acceleration.x, Mpu6050.linear_acceleration.y, Mpu6050.linear_acceleration.z );

                OdomPublish();
                ImudataPublish();
            }
        }
        last_time = current_time;
        ROS_INFO("Send to USTRA! X: %f  Z: %f", Send_Str.Sensor_Str.X_speed, Send_Str.Sensor_Str.Z_speed);
        ros::spinOnce();
    }

}

float Serial_test::invSqrt(float number)
{
	volatile long i;
    volatile float x, y;
    volatile const float f = 1.5F;

    x = number * 0.5F;
    y = number;
    i = * (( long * ) &y);
    i = 0x5f375a86 - ( i >> 1 );
    y = * (( float * ) &i);
    y = y * ( f - ( x * y * y ) );

	return y;
}

void Serial_test::accelateOffest(float gx, float gy, float gz)
{
	GXdata_Offest += gx; 
  	GYdata_Offest += gy; 
  	GZdata_Offest += gz;

  	if (Offest_Count == 40)
  	{
  		GXdata_Offest = GXdata_Offest / 40;
  		GYdata_Offest = GYdata_Offest / 40;
  		GZdata_Offest = GZdata_Offest / 40;
  	}
}

volatile float twoKp = 1.0f;											// 2 * proportional gain (Kp)
volatile float twoKi = 1.0f;											// 2 * integral gain (Ki)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;					// quaternion of sensor frame relative to auxiliary frame
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;

void Serial_test::MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az)
{
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// 首先把加速度计采集到的值(三维向量)转化为单位向量，即向量除以模
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;        

		// 把四元数换算成方向余弦中的第三行的三个元素
		halfvx = q1 * q3 - q0 * q2;
		halfvy = q0 * q1 + q2 * q3;
		halfvz = q0 * q0 - 0.5f + q3 * q3;
	
		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * (1.0f / 20.2f);	// integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f / 20.2f);
			integralFBz += twoKi * halfez * (1.0f / 20.2f);
			gx += integralFBx;				// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;				// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f / 20.2f));		// pre-multiply common factors
	gy *= (0.5f * (1.0f / 20.2f));
	gz *= (0.5f * (1.0f / 20.2f));
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx); 
	
	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

	Mpu6050.orientation.w = q0;
	Mpu6050.orientation.x = q1;
	Mpu6050.orientation.y = q2;
	Mpu6050.orientation.z = q3;
}


int main(int argc, char **argv){

    ros::init(argc, argv, "serial_test");
    ROS_INFO("serial_test node start");

    Serial_test serial_test;
    serial_test.ReadAndWrite();

    //ros::spin();
    return 0;
}