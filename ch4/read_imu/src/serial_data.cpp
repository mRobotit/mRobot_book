#include <fcntl.h>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

char r_buf[512];
int port_fd = 0;
float acce[3], angular[3], angle[3], quater[4];

/**********************************************
 * Description:打开串口,输入各参数即可.
 *********************************************/
int openSerialPort(const char *path, int baud, int dataBits,
                   const char *parity, int stopBit)
{
    int fd = 0;
    struct termios terminfo;
    bzero(&terminfo, sizeof(terminfo));

    fd = open(path, O_RDWR | O_NOCTTY);
    if (-1 == fd)
    {
        ROS_ERROR("Open imu dev error:%s", path);
        return -1;
    }

    if (isatty(STDIN_FILENO) == 0)
    {
        ROS_ERROR("IMU dev isatty error !");
        return -2;
    }

    /*设置串口通信波特率-bps*/
    switch (baud)
    {
    case 9600:
        cfsetispeed(&terminfo, B9600); //设置输入速度
        cfsetospeed(&terminfo, B9600); //设置输出速度
        break;

    case 19200:
        cfsetispeed(&terminfo, B19200);
        cfsetospeed(&terminfo, B19200);
        break;

    case 38400:
        cfsetispeed(&terminfo, B38400);
        cfsetospeed(&terminfo, B38400);
        break;

    case 57600:
        cfsetispeed(&terminfo, B57600);
        cfsetospeed(&terminfo, B57600);
        break;

    case 115200:
        cfsetispeed(&terminfo, B115200);
        cfsetospeed(&terminfo, B115200);
        break;

    default: //设置默认波特率9600
        cfsetispeed(&terminfo, B9600);
        cfsetospeed(&terminfo, B9600);
        break;
    }

    //设置数据位
    terminfo.c_cflag |= CLOCAL | CREAD;
    terminfo.c_cflag &= ~CSIZE;
    switch (dataBits)
    {
    case 7:
        terminfo.c_cflag |= CS7;
        break;

    case 8:
        terminfo.c_cflag |= CS8;
        break;

    default:
        ROS_ERROR("set dataBit error !");
        return -3;
    }

    //设置奇偶校验位
    switch (*parity)
    {
    case 'o': //设置奇校验
    case 'O':
        terminfo.c_cflag |= PARENB;
        terminfo.c_cflag |= PARODD;
        terminfo.c_iflag |= (INPCK | ISTRIP);
        break;

    case 'e': //设置偶校验
    case 'E':
        terminfo.c_iflag |= (INPCK | ISTRIP);
        terminfo.c_cflag |= PARENB;
        terminfo.c_cflag &= ~PARODD;
        break;

    case 'n': //不设置奇偶校验位
    case 'N':
        terminfo.c_cflag &= ~PARENB;
        break;

    default:
        ROS_ERROR("set parity error !");
        return -4;
    }

    //设置停止位
    switch (stopBit)
    {
    case 1:
        terminfo.c_cflag &= ~CSTOPB;
        break;

    case 2:
        terminfo.c_cflag |= CSTOPB;
        break;

    default:
        ROS_ERROR("set stopBit error !");
        return -5;
    }

    terminfo.c_cc[VTIME] = 10;
    terminfo.c_cc[VMIN] = 0;
    tcflush(fd, TCIFLUSH);

    if ((tcsetattr(fd, TCSANOW, &terminfo)) != 0)
    {
        ROS_ERROR("set imu serial port attr error !");
        return -6;
    }

    return fd;
}

/**************************************
 * Description:关闭串口文件描述符.
 *************************************/
int closeSerialPort()
{
    int ret = close(port_fd);
    return ret;
}

/*****************************************************
 * Description:向串口中发送数据.
 ****************************************************/
int send_data(int fd, char *send_buffer, int length)
{
    length = write(fd, send_buffer, length * sizeof(unsigned char));

    return length;
}

/*******************************************************
 * Description:从串口中接收数据.
 *******************************************************/
int recv_data(int fd, char *recv_buffer, int len)
{
    int length = read(fd, recv_buffer, len);
    return length;
}

/************************************************************
 * Description:根据串口数据协议来解析有效数据,
 ***********************************************************/
void parse_serialData(char chr)
{
    static unsigned char chrBuf[100];
    static unsigned char chrCnt = 0;
    signed short sData[4]; //save 8 Byte valid info
    unsigned char i = 0;
    char frameSum = 0;

    chrBuf[chrCnt++] = chr; //保存当前字节,字节计数加1

    //判断是否满一个完整数据帧11个字节
    if (chrCnt < 11)
        return;

    //计算数据帧的前十个字节的校验和
    for (i = 0; i < 10; i++)
    {
        frameSum += chrBuf[i];
    }

    //找到数据帧第一个字节是0x55,校验和是否正确,若两者有一个不正确,
    //都需要移动到最开始的字节,再读取新的字节进行判断数据帧完整性
    if ((chrBuf[0] != 0x55) || (frameSum != chrBuf[10]))
    {
        memcpy(&chrBuf[0], &chrBuf[1], 10); //将有效数据往前移动1字节位置
        chrCnt--;                           //字节计数减1,需要再多读取一个字节进来,重新判断数据帧
        return;
    }

    //for(i=0;i<11; i++)
    //  printf("0x%x ",chrBuf[i]);
    //printf("\n");

    memcpy(&sData[0], &chrBuf[2], 8);
    switch (chrBuf[1])
    {
    case 0x51: //x,y,z轴 加速度输出
        acce[0] = ((short)(((short)chrBuf[3] << 8) | chrBuf[2])) / 32768.0 * 16.0;
        acce[1] = ((short)(((short)chrBuf[5] << 8) | chrBuf[4])) / 32768.0 * 16.0;
        acce[2] = ((short)(((short)chrBuf[7] << 8) | chrBuf[6])) / 32768.0 * 16.0;
        break;

    case 0x52: //角速度输出
        angular[0] = ((short)(((short)chrBuf[3] << 8) | chrBuf[2])) / 32768.0 * 2000.0;
        angular[1] = ((short)(((short)chrBuf[5] << 8) | chrBuf[4])) / 32768.0 * 2000.0;
        angular[2] = ((short)(((short)chrBuf[7] << 8) | chrBuf[6])) / 32768.0 * 2000.0;
        break;

    case 0x53: //欧拉角度输出, roll, pitch, yaw
        angle[0] = ((short)(((short)chrBuf[3] << 8) | chrBuf[2])) / 32768.0 * 180.0;
        angle[1] = ((short)(((short)chrBuf[5] << 8) | chrBuf[4])) / 32768.0 * 180.0;
        angle[2] = ((short)(((short)chrBuf[7] << 8) | chrBuf[6])) / 32768.0 * 180.0;
        break;

    case 0x59: //四元素输出
        quater[0] = ((short)(((short)chrBuf[3] << 8) | chrBuf[2])) / 32768.0;
        quater[1] = ((short)(((short)chrBuf[5] << 8) | chrBuf[4])) / 32768.0;
        quater[2] = ((short)(((short)chrBuf[7] << 8) | chrBuf[6])) / 32768.0;
        quater[3] = ((short)(((short)chrBuf[9] << 8) | chrBuf[8])) / 32768.0;
        break;
    }
    chrCnt = 0;
}

/*****************************************************************
 * Description:根据串口配置信息来配置打开串口,准备进行数据通信
 ****************************************************************/
int initSerialPort(const char *path, const int baud, const int dataBits,
                   const char *parity, const int stopBit)
{
    bzero(r_buf, 512);
    port_fd = openSerialPort(path, baud, dataBits, parity, stopBit);
    if (port_fd < 0)
    {
        ROS_ERROR("openSerialPort error !");
        return -1;
    }

    return 0;
}

float getAccX()
{
    return acce[0];
}
float getAccY()
{
    return acce[1];
}
float getAccZ()
{
    return acce[2];
}

float getAngularX()
{
    return angular[0];
}

float getAngularY()
{
    return angular[1];
}

float getAngularZ()
{
    return angular[2];
}

float getAngleX()
{
    return angle[0];
}
float getAngleY()
{
    return angle[1];
}
float getAngleZ()
{
    return angle[2];
}

int getImuData()
{
    int ret = 0;
    ret = recv_data(port_fd, r_buf, 44);
    if (ret == -1)
    {
        ROS_ERROR("read serial port data failed !\n");
        closeSerialPort();

        return -1;
    }

    for (int i = 0; i < ret; i++)
    {
        parse_serialData(r_buf[i]);
    }

    return 0;
}
