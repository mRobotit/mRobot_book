#include"read_camera.h"

using namespace std;

CameraManager::CameraManager(ros::NodeHandle nh,std::string name,int hz,std::string path):
nh_(nh),device_name_(name),hz_(hz),path_(path)
{
    delay_ = 1000.0/hz_;
    
    cout<<"delay: "<<delay_<<endl;

    capture_ = cv::VideoCapture(0);
    
    if(!capture_.isOpened())
        return;

    cv::Mat frame;
    capture_ >> frame;
    size_[0] = frame.size[0];
    size_[1] = frame.size[1];
    size_[2] = frame.size[2];

    image_pub = nh_.advertise("/sensor_msgs/image",5);
}

cv::Mat* CameraManager::read_image(bool save)
{
    cv::Mat frame,gray;
    capture_ >> frame;
    ros::Time timestamp = ros::Time::now();
    std::string time_second = std::to_string(timestamp.toSec()*1e9);

    std::string image_name(time_second);
    image_name.erase(19);
    image_name.append(".png");

    if(frame.empty())
    {
        //std::cout << "frequency is too high"<<std::endl;
        return nullptr;
    }

    cv::cvtColor(frame, gray, CV_BGR2GRAY);

    RGB_ = frame;
    GRAY_ = gray;
    if(save)
    {
        save_image(image_name, gray);
    }
    return &RGB_;
}


void CameraManager::spin(bool ros_send ,bool save,bool visualization)
{    
    sensor_msgs::ImagePtr msg;
    while(ros::ok())
    {
        cv::Mat* imagePtr = read_image(save);

        if(visualization) imshow("image_gray",*imagePtr);
        if(imagePtr&&ros_send)
        {
            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", *imagePtr).toImageMsg();
            image_pub.publish(msg);
        }
        cv::waitKey(delay_);
    }
}