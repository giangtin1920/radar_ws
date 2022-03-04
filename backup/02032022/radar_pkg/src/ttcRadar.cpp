#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include "Radar_Cfg.h"
#include "std_msgs/UInt8MultiArray.h"

#define DataPort_EN

RadarObj radarObj;
ros::Publisher radar_pub;
radar_pkg::radar_msg radar_output_msg;

void timer_uart_Callback(const ros::TimerEvent& )
{
    if(radarObj.ser_Data_Port.available())
    {
        uint16_t dataLen = 0;
        dataLen = radarObj.ser_Data_Port.available();
        std_msgs::UInt8MultiArray raw_data;
        radarObj.ser_Data_Port.read(raw_data.data, radarObj.ser_Data_Port.available());
        ROS_INFO("Read: %u byte ---------------------", dataLen);

        // Process the raw_data
        if (true == radarObj.data_handler(raw_data, dataLen))
        {
            // Send ros message
            radar_output_msg.msg_counter = radarObj.Output.msg_counter;
            radar_output_msg.isObject = radarObj.Output.isObject;
            radar_output_msg.distance = radarObj.Output.distance;
            radar_pub.publish(radar_output_msg);
            ROS_INFO("Public message ok (TTC)");
        }
    }
}

int main (int argc, char** argv){
    ros::init(argc, argv, "ttcRadar");
    ros::NodeHandle nh;
    radar_pub = nh.advertise<radar_pkg::radar_msg>("ttcRadar_Data", 1000);

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
