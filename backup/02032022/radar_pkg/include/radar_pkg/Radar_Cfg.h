#ifndef RADAR_CFG_H
#define RADAR_CFG_H

#include <string>
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include "aev_pkg/radar_msg.h"
#include "std_msgs/UInt8MultiArray.h"
#include <vector>
#include <iostream>
#include <cstdlib>
#include <algorithm>
using namespace std;

#define CFG_LOOP_RATE 10 

#define ser_Cfg_Port_Name "/dev/ttyUSB0"
#define ser_Data_Port_Name "/dev/ttyUSB1"

#define MMWDEMO_UART_MSG_DETECTED_POINTS			1
#define MMWDEMO_UART_MSG_RANGE_PROFILE				2
#define MMWDEMO_UART_MSG_NOISE_PROFILE				3
#define MMWDEMO_UART_MSG_DETECTED_POINTS_SIDE_INFO  7

/* Output of Radar */
typedef struct
{
    uint32_t msg_counter = 0;
    bool isObject = false;
    float distance = 10.0f;
} Radar_Output_Struct;

struct structHeader
{
    uint16_t magicWord[8];
    uint32_t version[4];
    uint32_t totalPacketLen;
    uint32_t platform;
    uint32_t frameNumber;
    uint32_t timeCpuCycles;
    uint32_t numDetectedObj;
    uint32_t numTLVs;
    uint32_t subFrameNumber;
    uint32_t numStaticDetectedObj;
    uint32_t idX;
};

struct structTLV
{
    uint32_t type;
    uint32_t length;
    vector<uint8_t> payload;
};

struct pointStruct
{
	vector<float> x;
	vector<float> y ;
	vector<float> z ;
	vector<float> doppler ;
} ;

union byte2float
{
	int str[4];
	vector<uint8_t>  myByte;
	vector<float> myFloat;
	~byte2float() {}
};

class RadarObj
{
    public:
    RadarObj();
    // explicit
    RadarObj(RadarObj &&) {}
    // implicit
    RadarObj(const RadarObj&) = default;
    RadarObj& operator=(const RadarObj&) = default;
    ~RadarObj();

    serial::Serial ser_Cfg_Port;
    serial::Serial ser_Data_Port;

    Radar_Output_Struct Output;
    pointStruct ptCloud;

    bool init_cfg_port(void);
    bool init_data_port(void);
    void start_radar(void);
    void stop_radar(void);
    bool data_handler(std_msgs::UInt8MultiArray data, uint16_t data_len);

    private:
    void send_cfg(std::string msg);
    structHeader getFrameHeader (uint8_t framePacket[], uint16_t dataLen);
    structTLV getTLV (uint8_t framePacket[], uint32_t numTLVs, uint16_t idX);

};

#endif