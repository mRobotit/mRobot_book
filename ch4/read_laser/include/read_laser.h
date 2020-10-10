#include<ros/ros.h>
#include<std_msgs/UInt16.h>
#include<string>
#include<iostream>
#include<sensor_msgs/LaserScan.h>
#include<boost/asio.hpp>
#include<boost/array.hpp>
namespace read_laser_test
{
    class ReadLaser
    {
    public:
        uint16_t rpms;
        ReadLaser(const std::string& port,uint32_t baud_rate,boost::asio::io_service& io);
        ~ReadLaser();
        void poll(sensor_msgs::LaserScan::Ptr scan);
        void close(){
            shutting_down_=true;
        }
    private:
        std::string port_;
        uint32_t baud_rate_;
        bool shutting_down_;
        boost::asio::serial_port serial_;
        uint16_t motor_speed_;
    };
}