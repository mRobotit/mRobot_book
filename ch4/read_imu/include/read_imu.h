#ifndef _IMU_DATA_H_
#define _IMU_DATA_H_
int initSerialPort(const char* path, const int baud, const int dataBits, const char* parity, const int stopBit);
int getImuData();
int closeSerialPort();
float getAccX();
float getAccY();
float getAccZ();
float getAngularX();
float getAngularY();
float getAngularZ();
float getAngleX();
float getAngleY();
float getAngleZ();
#endif