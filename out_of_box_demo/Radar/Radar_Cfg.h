#ifndef RADAR_CFG_H
#define RADAR_CFG_H

#include <string>
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include "aev_pkg/radar_msg.h"

#define CFG_LOOP_RATE 10 

#define ser_Cfg_Port_Name "/dev/ttyUSB0"
#define ser_Data_Port_Name "/dev/ttyUSB1"

#define BYTES_BUFFER_MAX_SIZE  65536

#define MMWDEMO_UART_MSG_DETECTED_POINTS 1
#define MMWDEMO_UART_MSG_RANGE_PROFILE 2
#define MMWDEMO_UART_MSG_AZIMUT_STATIC_HEAT_MAP 4
#define MMWDEMO_UART_MSG_RANGE_DOPPLER_HEAT_MAP 5
#define MMWDEMO_UART_MSG_STATS 6
#define MMWDEMO_UART_MSG_DETECTED_POINTS_SIDE_INFO 7
#define MMWDEMO_UART_MSG_Azimuth_Elevation_STATIC_HEAT_MAP 8
#define MMWDEMO_UART_MSG_TEMPERATURE_STATISTICS 9


/* Output of Radar */
typedef struct
{
  uint32_t msg_counter = 0;
  bool isObject = false;
  float distance = 10.0f;
} Radar_Output_Struct;

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

    bool init_cfg_port(void);
    bool init_data_port(void);
    void start_radar(void);
    void stop_radar(void);
    bool data_handler(std_msgs::String data, uint16_t data_len);

  private:
    void send_cfg(std::string msg);
    
};


#endif