#include"control_test/control_robot.h"



void turnController::turnBasic(ros::Publisher& pub,double velocity)
{
    geometry_msgs::Twist velocityMsg;
    if(orientation_ == TURN_RIGHT)
        velocityMsg.angular.z = -velocity;
    else
        velocityMsg.angular.z = velocity;
    pub.publish(velocityMsg);
}

bool turnController::TURN(ros::Publisher& pub, double current_orientation, double theta){
    
    //ROS_INFO("last_orientation: %f  ", last_orientation_);
    if(!initial_){
        last_orientation_ = current_orientation;
        initial_ = true;
    }
    double velocity = basic_velocity_;
    if(theta < 0 || fmod(theta, 360.0) > 180){ 
        velocity = - basic_velocity_;
        if(theta > 0){
            theta = theta - 180;
        }
        else{
            theta = fabs(theta);
        }
    }
    // ROS_INFO("orientation_count: %f  current_orientation: %f theta: %f vel: %f", orientation_count, current_orientation, theta, basic_velocity_);
    double delta_orientation = fabs(fabs(current_orientation) - fabs(last_orientation_));
    orientation_count += delta_orientation * 180 / M_PI;
   // ROS_INFO("orientation_count: %f  ", orientation_count);
    if(orientation_count >= theta){
        return true;
    }
    else{
        geometry_msgs::Twist vel_msg;
        if(orientation_count < theta * 4 / 5)
            vel_msg.angular.z = velocity;
        else vel_msg.angular.z = 0.1 * velocity;
        pub.publish(vel_msg);
    }
    last_orientation_ = current_orientation;
    //sleep(0.0001);
}

bool turnController::turn(ros::Publisher& pub, double current_orientation,double theta)
{
    
    if(!initial_)
    {
        if(!judgeInfo(theta, current_orientation))
        {
            ROS_INFO("Rotation Angle is so small");
            return true;
        }
        initial_ = true;
    }
    
    double res_theta = fabs(aim_orientation_ - current_orientation);
    double velocity = basic_velocity_;
    
    // if(res_theta >= M_PI/2)velocity*=1;
    // if(res_theta>= M_PI/4&&res_theta<M_PI/2)velocity*=1;
    if(res_theta<M_PI/6) velocity=0.2;
    ROS_INFO("current_orientation:%f  aim_orientation_:%f  res_theta: %f vel: %f" ,current_orientation, aim_orientation_, res_theta, velocity);
    // ROS_INFO("rotation_vel: %.2f", velocity);
    if(res_theta < 0.05){
        initial_ = false;
        int n = 10;
        while(n--){
            turnBasic(pub, -1);
        }
        return true;
    }else{
        turnBasic(pub, velocity);
        return false;
    }   
}

bool turnController::judgeInfo(double theta, double current_orientation){
    aim_orientation_ = radianNormal(current_orientation + angle2Radian(theta));
    // ROS_INFO("current_orientation:%f theta:%f aim_orientation_:%f",current_orientation, theta, aim_orientation_);
    if(fabs(aim_orientation_ - current_orientation) < 0.001) return false;
    if(radianNormal(aim_orientation_ - current_orientation) > 0) orientation_ = TURN_LEFT; 
    else orientation_ = TURN_RIGHT;
    return true;
}

void goController::GoBasic(ros::Publisher& pub, double velocity, double angular)
{   
    // ROS_INFO("pub_vel: %f",velocity);
    geometry_msgs::Twist gomsg;
    gomsg.linear.x = velocity;
    gomsg.angular.z = angular;
    pub.publish(gomsg);
}

double goController::Imu_PID_set(double current_imu_oz, double aim_imu_oz){
    double error = 0;
    double p_error = 0;
    static double i_error = 0;
    double d_error = 0;
    if(fabs(current_imu_oz - aim_imu_oz) > M_PI) current_imu_oz = - current_imu_oz;
    
    error = - current_imu_oz + aim_imu_oz;
    p_error = error;
    i_error = p_error + i_error;
    d_error = error - y_error * 2 + yy_error;

    ros::param::get("~KP", KP);
    ros::param::get("~KI", KI);
    ros::param::get("~KD", KD);

    ROS_INFO("P_E: %f I_E: %f D_E: %f ", p_error, i_error, d_error);
    double pub_yaw = KP * p_error + KI * i_error + KD * d_error;
    
    yy_error = y_error;
    y_error = error;
    ROS_INFO("current_imu_oz: %f aim_imu_oz: %f pub_yaw :%f", current_imu_oz, aim_imu_oz, pub_yaw);
    return pub_yaw;
}

bool goController::Go(ros::Publisher& pub, double current_px, double current_py, double current_imu_oz, double distance)
{
    if(!initial_){
        aim_imu_oz_ = current_imu_oz;
        init_px_ = current_px;
        init_py_ = current_py;
        initial_ = true;
    }
    current_imu_oz_ = current_imu_oz;
    double velocity = go_velocity_;
    double d_px = fabs(current_px - init_px_);
    double d_py = fabs(current_py - init_py_);
    double ddistance = sqrt(d_px * d_px + d_py * d_py);
    //ROS_ERROR("d_px: %f d_py: %f distance: %f", d_px, d_py, ddistance);
    if(distance - ddistance <= 0.3) velocity = 0.2;
    if(distance - ddistance < 0.02){
        int n = 10;
        while(n--){
            GoBasic(pub, 0.0, 0.0);
        }
        
        initial_ = false;
        return true;
    }
    else{
        // ROS_INFO("current_imu_oz: %f aim_imu_oz: %f vel: %f", current_imu_oz, aim_imu_oz_, velocity);
        GoBasic(pub, velocity, Imu_PID_set(current_imu_oz_, aim_imu_oz_));
        return false;
    }
}
