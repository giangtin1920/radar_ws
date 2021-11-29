#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include "Radar_Cfg.h"

#define DataPort_EN

RadarObj radarObj;
ros::Publisher radar_pub;

int cnt = 0;
aev_pkg::radar_msg radar_output_msg;
void timer_uart_Callback(const ros::TimerEvent& e)
{
  if(radarObj.ser_Data_Port.available())
  {
    uint16_t dataLen = 0;
    dataLen = radarObj.ser_Data_Port.available();
    std_msgs::String raw_data;
    raw_data.data = radarObj.ser_Data_Port.read(radarObj.ser_Data_Port.available());

    // Process the data
    if (true == radarObj.data_handler(raw_data, dataLen))
    {
        // Send ros message
        radar_output_msg.msg_counter = radarObj.Output.msg_counter;
        radar_output_msg.isObject = radarObj.Output.isObject;
        radar_output_msg.distance = radarObj.Output.distance;
        radar_pub.publish(radar_output_msg);
    }
    
    cnt++;
    if (cnt >= 2000)
    {
        radarObj.stop_radar();
    }
  }
}

int main (int argc, char** argv){
    ros::init(argc, argv, "Radar");
    ros::NodeHandle nh;
    radar_pub = nh.advertise<aev_pkg::radar_msg>("Radar_Data", 1000);

    // Timer to receive data from Radar
    ros::Timer timer_uart = nh.createTimer(ros::Duration(0.05), timer_uart_Callback);

    // Connect to COM port of Radar 
    if (true == radarObj.init_cfg_port())
    {
        #ifdef DataPort_EN
        if (true == radarObj.init_data_port())
        {



        }
        else
        {
            return -1;
        }
        #endif
    }
    else
    {
        return -1;
    }
    radarObj.start_radar();

    // While loop do nothing, data received by interrupt
    while(ros::ok())
    {
        ros::spinOnce();
    }
}
