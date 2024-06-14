/*! 
 *  @file Robotous_FT.h
 *  @brief Robotous FT Sensor data communication using CAN 2.0
 *  @author Sunhong Kim (tjsghd101@naver.com)
 *  @data Nov. 08. 2023
 *  @Comm
 */

#ifndef ROBOTOUS_FT_H_
#define ROBOTOUS_FT_H_

// CAN Bus Command
#define READ_MODEL_NAME 0x01;
#define READ_SERIAL_NUM 0x02;
#define READ_FIRMWARE_VER 0x03;
#define SET_COMM_ID 0x04;
#define READ_COMM_ID 0x05;
#define READ_BAUDRATE 0x07;
#define SET_FILTER 0x08;
#define READ_FILTER_SETTING 0x09;
#define READ_ONCE 0x0A;
#define READ_START 0x0B;
#define READ_STOP 0x0C;
#define SET_DATA_RATE 0x0F;
#define READ_DATA_RATE 0x10;
#define SET_BIAS 0x11;
#define READ_OVERLOAD_COUNT 0x12;

#define FT_RATE_10HZ 0x01;
#define FT_RATE_20HZ 0x02;
#define FT_RATE_50HZ 0x03;
#define FT_RATE_100HZ 0x04;
#define FT_RATE_200HZ 0x05;
#define FT_RATE_333HZ 0x06;
#define FT_RATE_500HZ 0x07;
#define FT_RATE_1000HZ 0x08;
#endif // ROBOTOUS_FT_H_
