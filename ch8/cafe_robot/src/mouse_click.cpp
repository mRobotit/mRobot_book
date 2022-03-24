#include<ros/ros.h>
#include<visualization_msgs/MarkerArray.h>
#include<geometry_msgs/PointStamped.h>
#include<visualization_msgs/Marker.h>

void Click_Callback(const geometry_msgs::PointStamped click_point){
    ROS_INFO("%f , %f ",click_point.point.x, click_point.point.y);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "multipoint_nav");

    visualization_msgs::Marker getpoint;
    visualization_msgs::MarkerArray pointarray;

    ros::NodeHandle nh;
    ros::Subscriber click_sub = nh.subscribe("/clicked_point", 10, Click_Callback);
    
    ros::spin();

    return 0;
}