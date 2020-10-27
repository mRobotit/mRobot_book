#include<ros/ros.h>
#include<sensor_msgs/CameraInfo.h>
#include<opencv2/opencv.hpp>
#include"camera_manager.h"
using namespace cv;


int main(int argc,char** argv)
{
    ros::init(argc,argv,"save_images");

    ros::NodeHandle nh;

    std::string device_name,save_path;
    bool save,visualization;
    
    ros::param::get("~/image_dev",device_name);
    ros::param::get("~/save_path",save_path);
    ros::param::get("~/visualization",visualization);
    CameraManager cm(nh,device_name,20,save_path);
    
    cm.spin(true,visualization,save);
    
}
