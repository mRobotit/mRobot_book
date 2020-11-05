#include"control_test/control_robot.h"
#include<sensor_msgs/Imu.h>
#include<tf/transform_listener.h>

double e_px, e_py, e_roll, e_pitch, e_yaw = 0, ee_px = 0, ee_py = 0, ee_yaw = 0;
double imu_ox, imu_oy, imu_oz, imu_ax, imu_ay, imu_az, imu_lx, imu_ly, imu_lz;//imu_data
double px, py, roll, pitch, yaw;//initation & pose x,y
bool init = false, is_init = true;
tf::Quaternion q_msg, imu_msg, tf_msg;

void poseCallback(const nav_msgs::Odometry &p_msg){
    tf::quaternionMsgToTF(p_msg.pose.pose.orientation, q_msg);
    tf::Matrix3x3(q_msg).getRPY(roll, pitch, yaw);
    px = p_msg.pose.pose.position.x;
    py = p_msg.pose.pose.position.y;
    init = true;
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

int main(int argc,char** argv){
    ros::init(argc, argv, "go_cmd_publisher");
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

    sleep(1);
    tf::TransformListener tf_listener;
    while(ros::ok()){
        ros::spinOnce();
        if(is_init){
            ee_px = e_px;
            ee_py = e_py;
            ee_yaw = e_yaw;
        }
        
        if(!is_init){
            // tf::StampedTransform transform;
            // try
            // {
            //     tf_listener.lookupTransform("map", "odom_combined", ros::Time(0), transform);
            //     tf_msg = tf::Quaternion(transform.getRotation().getX(), transform.getRotation().getY(), transform.getRotation().getZ(), transform.getRotation().getW());
            //     tf::Matrix3x3(tf_msg).getRPY(e_roll, e_pitch, e_yaw);
            //     e_px = transform.getOrigin().x() - ee_px;
            //     e_py = transform.getOrigin().y() - ee_py;
                
            //     // ROS_INFO("x,y,z = %f, %f, %f", transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
            //     // ROS_INFO("z,w = %f, %f", transform.getRotation().getZ(), transform.getRotation().getW());
            // }
            // catch(const std::exception& e)
            // { 
            //     ROS_ERROR("%s", e.what());
            // }
            // ROS_INFO("INITIAL imu_oz: %f", imu_oz);
            // if(go_controller.Go(command_pub, px + e_px, py + e_py, imu_oz + e_yaw, distance1)){
            //     ROS_INFO("Go finished!");
            //     break;
            // }
            if(go_controller.Go(command_pub, px, py, imu_oz, distance1)){
                ROS_INFO("Go finished!");
                break;
            }
        }
        is_init = false;
    }
    is_init = true;
    while(ros::ok()){
        ros::spinOnce();
        if(is_init){
            ee_px = e_px;
            ee_py = e_py;
            ee_yaw = e_yaw;
            
        }
        
        if(!is_init){
            // tf::StampedTransform transform;
            // try
            // {
            //     tf_listener.lookupTransform("map", "odom_combined", ros::Time(0), transform);
            //     e_px = transform.getOrigin().x() - ee_px;
            //     e_py = transform.getOrigin().y() - ee_py;
            //     tf_msg = tf::Quaternion(transform.getRotation().getX(), transform.getRotation().getY(), transform.getRotation().getZ(), transform.getRotation().getW());
            //     tf::Matrix3x3(tf_msg).getRPY(e_roll, e_pitch, e_yaw);
            //     // ROS_INFO("x,y,z = %f, %f, %f", transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
            //     // ROS_INFO("z,w = %f, %f", transform.getRotation().getZ(), transform.getRotation().getW());
                
            // }
            // catch(const std::exception& e)
            // { 
            //     ROS_ERROR("%s", e.what());
            // }
            // ROS_INFO("imu_oz: %f",imu_oz);
            // if(turn_controller.turn(command_pub, imu_oz + e_yaw, rotation_angle)){
	 	    //     ROS_INFO("rotation finished");
            // 	break;
            // }
            if(turn_controller.turn(command_pub, imu_oz, rotation_angle)){
	 	        ROS_INFO("rotation finished");
            	break;
            }
    	}
        is_init = false;
    }
    is_init = true;
    while(ros::ok()){
        ros::spinOnce();
        if(is_init){
            ee_px = e_px;
            ee_py = e_py;
            ee_yaw = e_yaw;
        }
        
        if(!is_init){
            // tf::StampedTransform transform;
            // try
            // {
            //     tf_listener.lookupTransform("map", "odom_combined", ros::Time(0), transform);
            //     e_px = transform.getOrigin().x() - ee_px;
            //     e_py = transform.getOrigin().y() - ee_py;
            //     // e_yaw = transform.getRotation().getZ() - ee_yaw;
            //     tf_msg = tf::Quaternion(transform.getRotation().getX(), transform.getRotation().getY(), transform.getRotation().getZ(), transform.getRotation().getW());
            //     tf::Matrix3x3(tf_msg).getRPY(e_roll, e_pitch, e_yaw);
            //     // ROS_INFO("x,y,z = %f, %f, %f", transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
            //     // ROS_INFO("z,w = %f, %f", transform.getRotation().getZ(), transform.getRotation().getW());
                
            // }
            // catch(const std::exception& e)
            // { 
            //     ROS_ERROR("%s", e.what());
            // }
            //ROS_INFO("INITIAL imu_oz: %f", imu_oz);
            // if(go_controller.Go(command_pub, px + e_px, py + e_py, imu_oz + e_yaw, distance2)){
            //     ROS_INFO("Go finished!");
            //     break;
            // }
            if(go_controller.Go(command_pub, px, py, imu_oz, distance2)){
                ROS_INFO("Go finished!");
                break;
            }
        }
        is_init = false;
    }
    is_init = true;
    while(ros::ok()){
        ros::spinOnce();
        if(is_init){
            ee_px = e_px;
            ee_py = e_py;
            ee_yaw = e_yaw;
        }
        
        if(!is_init){
            // tf::StampedTransform transform;
            // try
            // {
            //     tf_listener.lookupTransform("map", "odom_combined", ros::Time(0), transform);
            //     e_px = transform.getOrigin().x() - ee_px;
            //     e_py = transform.getOrigin().y() - ee_py;
            //     tf_msg = tf::Quaternion(transform.getRotation().getX(), transform.getRotation().getY(), transform.getRotation().getZ(), transform.getRotation().getW());
            //     tf::Matrix3x3(tf_msg).getRPY(e_roll, e_pitch, e_yaw);
            //     // ROS_INFO("x,y,z = %f, %f, %f", transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
            //     // ROS_INFO("z,w = %f, %f", transform.getRotation().getZ(), transform.getRotation().getW());
                
            // }
            // catch(const std::exception& e)
            // { 
            //     ROS_ERROR("%s", e.what());
            // }
            // ROS_INFO("imu_oz: %f",imu_oz);
            // if(turn_controller.turn(command_pub, imu_oz + e_yaw, rotation_angle)){
	 	    //     ROS_INFO("rotation finished");
            // 	break;
            // }
            if(turn_controller.turn(command_pub, imu_oz, rotation_angle)){
	 	        ROS_INFO("rotation finished");
            	break;
            }
    	}
        is_init = false;
    }
    return 0;
}