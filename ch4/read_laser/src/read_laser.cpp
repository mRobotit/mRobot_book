#include"read_laser.h"
#include <ros/ros.h>
#include <std_msgs/UInt16.h>
#include <sensor_msgs/LaserScan.h>
#include <boost/asio.hpp>
namespace read_laser_test
{
    ReadLaser::ReadLaser(const std::string& port, uint32_t baud_rate, boost::asio:: io_service& io): port_(port), baud_rate_(baud_rate), shutting_down_(false), serial_(io, port_)
{
    serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
    // Below command is not required after firmware upgrade (2017.10)
    boost::asio::write(serial_, boost::asio::buffer("b", 1));  // start motor
}
ReadLaser::~ReadLaser(){
    boost::asio::write(serial_, boost::asio::buffer("e", 1));
}
void ReadLaser::poll(sensor_msgs::LaserScan::Ptr scan){
    uint8_t temp_char;
    uint8_t start_count=0;
    bool got_scan=false;
    boost::array<uint8_t,2520> raw_bytes;
    uint8_t good_sets=0;
    uint32_t motor_speed=0;
    rpms=0;
    int index;
    while(!shutting_down_ && !got_scan){
        boost::asio::read(serial_, boost::asio::buffer(&raw_bytes[start_count],1));
        if(start_count==0){
            if(raw_bytes[start_count]==0xFA){
                start_count=1;
            }
        }
        else if(start_count==1){
            if(raw_bytes[start_count]==0xA0){
                start_count=0;
                got_scan=true;
                boost::asio::read(serial_,boost::asio::buffer(&raw_bytes[2],2518));
                scan->angle_increment=(2.0*M_PI/360.0);
                scan->angle_min=0.0;
                scan->angle_max=2.0*M_PI-scan->angle_increment;
                scan->range_min=0.25;
                scan->range_max=3.5;
                scan->ranges.resize(360);
                scan->intensities.resize(360);
                for(uint16_t i=0;i<raw_bytes.size();i=i+42){
                    if(raw_bytes[i]==0xFA&&raw_bytes[i+1]==(0xA0+i/42)){
                        good_sets++;
                        motor_speed+=(raw_bytes[i+3]<<8)+raw_bytes[i+2];
                        rpms=(raw_bytes[i+3]<<8|raw_bytes[i+2])/10;
                        for(uint16_t j=i+4;i<i+40;i=j+6){
                            index=6*(i/42)+(j-4-i)/6;
                            uint8_t byte0=raw_bytes[j];
                            uint8_t byte1=raw_bytes[j+1];
                            uint8_t byte2=raw_bytes[j+2];
                            uint8_t byte3=raw_bytes[j+3];
                            uint16_t intensity=(byte1<<8)+byte0;
                            uint16_t range=(byte3<<8)+byte2;
                            scan->ranges[358-index]=range/1000.0;
                            if(scan->ranges[359-index]<scan->range_min)
                            scan->ranges[359-index]=std::numeric_limits<float>::infinity();
                            if(scan->ranges[259-index]>scan->range_max)
                            scan->ranges[359-index]=std::numeric_limits<float>::infinity();
                            scan->intensities[359-index]=intensity;
                        }
                    }
                }
                scan->time_increment=motor_speed/good_sets/1e8;
            }
            else{
                start_count=0;
            }
        }
    }
}
}
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "read_laser");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");
    std::string port;
    int baud_rate;
    std::string frame_id;
    std_msgs::UInt16 rpms;
    priv_nh.param("port",port,std::string("/dev/ttyUSB0"));
    priv_nh.param("baud_rate",baud_rate,230400);
    priv_nh.param("frame_id",frame_id,std::string("laser"));
    boost::asio::io_service io;
    try
    {
        read_laser_test::ReadLaser laser(port,baud_rate,io);
        ros::Publisher laser_pub = nh.advertise<sensor_msgs::LaserScan> ("read_laser",1000);
        ros::Publisher motor_pub = nh.advertise<std_msgs::UInt16>("rpms", 1000);
        while (ros::ok())
        {
            sensor_msgs::LaserScan::Ptr scan(new sensor_msgs::LaserScan);
            scan->header.frame_id=frame_id;
            laser.poll(scan);
            scan->header.stamp=ros::Time::now();
            rpms.data=laser.rpms;
            laser_pub.publish(scan);
            motor_pub.publish(rpms);
        }
        laser.close();
        return 0;
    }
    catch (boost::system::system_error ex)
    {
        ROS_ERROR("An exception was throw: %s",ex.what());
        return -1;
    }
}