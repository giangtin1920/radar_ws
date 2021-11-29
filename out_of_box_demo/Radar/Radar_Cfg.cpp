#include "Radar_Cfg.h"

RadarObj::RadarObj()
{

}

RadarObj::~RadarObj()
{
  
}

bool RadarObj::init_cfg_port(void)
{
    // Init Radar Config Port
    try
    {
        ser_Cfg_Port.setPort(ser_Cfg_Port_Name);
        ser_Cfg_Port.setBaudrate(115200);
        serial::Timeout to1 = serial::Timeout::simpleTimeout(1000);
        ser_Cfg_Port.setTimeout(to1);
        ser_Cfg_Port.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open config port ");
        return false;
    }
    if(ser_Cfg_Port.isOpen()){
        ROS_INFO_STREAM("Radar Config Port initialized");
    }else{
        return false;
    }

}

bool RadarObj::init_data_port(void)
{
  // Init Radar Data Port
    try
    {
        ser_Data_Port.setPort(ser_Data_Port_Name);
        ser_Data_Port.setBaudrate(921600);
        serial::Timeout to2 = serial::Timeout::simpleTimeout(1000);
        ser_Data_Port.setTimeout(to2);
        ser_Data_Port.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open data port ");
        return false;
    }
    if(ser_Data_Port.isOpen()){
        ROS_INFO_STREAM("Radar Data Port initialized");
    }else{
        return 1;
    }

}

void RadarObj::send_cfg(std::string msg)
{
  ros::Rate loop_rate1(CFG_LOOP_RATE);

  ser_Cfg_Port.write(msg + "\n");
  ROS_INFO_STREAM("Send: " << msg);
  loop_rate1.sleep();

  if(ser_Cfg_Port.available())
  {
    std_msgs::String result;
    result.data = ser_Cfg_Port.read(ser_Cfg_Port.available());
    ROS_INFO_STREAM("-Read: " << result.data);
    //read_pub.publish(result);
  }
  loop_rate1.sleep();
}

void RadarObj::start_radar(void)
{
    std::string msg;

    msg = "sensorStop";
    send_cfg(msg);

    msg = "flushCfg";
    send_cfg(msg);

    msg = "dfeDataOutputMode 1";
    send_cfg(msg);

    msg = "channelCfg 15 5 0";
    send_cfg(msg);

    msg = "adcCfg 2 1";
    send_cfg(msg);

    msg = "adcbufCfg -1 0 1 1 1";
    send_cfg(msg);

    msg = "lowPower 0 0";
    send_cfg(msg);

    msg = "profileCfg 0 60.25 7 3 24 0 0 156 1 256 12500 0 0 30";
    send_cfg(msg);

    msg = "chirpCfg 0 0 0 0 0 0 0 1";
    send_cfg(msg);

    msg = "chirpCfg 1 1 0 0 0 0 0 4";
    send_cfg(msg);

    // 100 is 100ms delay between 2 output frame
    msg = "frameCfg 0 1 32 0 100 1 0";
    send_cfg(msg);

    msg = "guiMonitor -1 1 1 1 0 0 1";
    send_cfg(msg);

    //Threshold scale [0..100]
    msg = "cfarCfg -1 0 2 8 4 3 0 15 0";
    send_cfg(msg);
    msg = "cfarCfg -1 1 0 4 2 3 1 15 0";
    send_cfg(msg);

    msg = "multiObjBeamForming -1 1 0.5";
    send_cfg(msg);

    msg = "calibDcRangeSig -1 0 -5 8 256";
    send_cfg(msg);

    msg = "clutterRemoval -1 0";
    send_cfg(msg);

    // View config (degrees) : [ -1 <minAzimuthDeg> <maxAzimuthDeg> <minElevationDeg> <maxElevationDeg> ]
    msg = "aoaFovCfg -1 -10 10 -8 8";
    send_cfg(msg);

    // Config point filtering in range direction (meter)
    msg = "cfarFovCfg -1 0 0.25 5.0";
    send_cfg(msg);

    // Config point filtering in Doppler direction (meter/sec)
    msg = "cfarFovCfg -1 1 -10 10";
    send_cfg(msg);

    msg = "compRangeBiasAndRxChanPhase 0.0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0";
    send_cfg(msg);

    msg = "measureRangeBiasAndRxChanPhase 0 1. 0.2";
    send_cfg(msg);

    msg = "extendedMaxVelocity -1 0";
    send_cfg(msg);

    msg = "lvdsStreamCfg  -1 0 0 0";
    send_cfg(msg);

    msg = "bpmCfg -1 0 0 0";
    send_cfg(msg);

    msg = "CQRxSatMonitor 0 3 4 99 0";
    send_cfg(msg);

    msg = "CQSigImgMonitor 0 31 4";
    send_cfg(msg);

    msg = "analogMonitor 0 0";
    send_cfg(msg);

    msg = "sensorStart";
    send_cfg(msg);

}

void RadarObj::stop_radar(void)
{
    std::string msg;
    msg = "sensorStop";
    send_cfg(msg);
}

bool RadarObj::data_handler(std_msgs::String raw_data, uint16_t data_len)
{
    bool is_data_ok = true;
  //ROS_INFO("-Read: %d %d %d %d %d %d", data_len, 
    //raw_data.data[0], raw_data.data[1], raw_data.data[2], raw_data.data[3], raw_data.data[4]);

  if (100 < data_len && 1)
  {
    std_msgs::String msg;
    std::stringstream ss;
    ss << "-Read " << data_len << " byte:";

    for (int i = 0; i < data_len; i++)
    {
      ss << " " << (int)raw_data.data[i];
    }
    msg.data = ss.str();
    ROS_INFO("%s", msg.data.c_str());

    // Processing

    // Update output
    Output.msg_counter++;
    Output.isObject = true;
    Output.distance = 5.0f;
  }

  return is_data_ok;
}