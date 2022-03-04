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

    msg = "profileCfg 0 60 180 7 15 0 0 100 1 64 9142 0 0 158";
    send_cfg(msg);

    msg = "chirpCfg 0 0 0 0 0 0 0 1";
    send_cfg(msg);

    msg = "chirpCfg 1 1 0 0 0 0 0 4";
    send_cfg(msg);

    // 100 is 100ms delay between 2 output frame
    msg = "frameCfg 0 1 128 0 100 1 0";
    send_cfg(msg);

    msg = "lowPower 0 0";
    send_cfg(msg);

    msg = "guiMonitor -1 1 1 0 0 0 0";
    send_cfg(msg);

    //Threshold scale [0..100]
    msg = "cfarCfg -1 0 2 8 4 3 0 15 1";
    send_cfg(msg);
    msg = "cfarCfg -1 1 0 8 4 4 1 15 1";
    send_cfg(msg);

    msg = "multiObjBeamForming -1 1 0.5";
    send_cfg(msg);

    msg = "clutterRemoval -1 0";
    send_cfg(msg);

    msg = "calibDcRangeSig -1 0 -5 8 256";
    send_cfg(msg);

    msg = "extendedMaxVelocity -1 0";
    send_cfg(msg);

    msg = "lvdsStreamCfg  -1 0 0 0";
    send_cfg(msg);

    msg = "compRangeBiasAndRxChanPhase 0.0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0";
    send_cfg(msg);

    msg = "measureRangeBiasAndRxChanPhase 0 1. 0.2";
    send_cfg(msg);

    msg = "CQRxSatMonitor 0 3 4 19 0";
    send_cfg(msg);

    msg = "CQSigImgMonitor 0 31 4";
    send_cfg(msg);

    msg = "analogMonitor 0 0";
    send_cfg(msg);

    // View config (degrees) : [ -1 <minAzimuthDeg> <maxAzimuthDeg> <minElevationDeg> <maxElevationDeg> ]
    msg = "aoaFovCfg -1 -10 10 -8 8";
    send_cfg(msg);

    // Config point filtering in range direction (meter)
    msg = "cfarFovCfg -1 0 0.25 10.0";
    send_cfg(msg);

    // Config point filtering in Doppler direction (meter/sec)
    msg = "cfarFovCfg -1 1 -30 30";
    send_cfg(msg);

    // msg = "bpmCfg -1 0 0 0";
    // send_cfg(msg);

    msg = "calibData 0 0 0";
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

structHeader RadarObj::getFrameHeader (uint8_t framePacket[], uint16_t dataLen)
{
	structHeader frameHeader;

	// check that all packet has been read
    frameHeader.totalPacketLen = framePacket[12] + framePacket[13] * 256.0 + framePacket[14] * 65536.0 + framePacket[15] * 1.6777216E+7;
	uint32_t idX = 0;

	// read the header
	if ((dataLen >= frameHeader.totalPacketLen) && (dataLen != 0))
	{
		// word array to convert 4 bytes to a 32 bit number
        // word = [1, 2**8, 2**16, 2**24]

        // Initialize the pointer index
        for (auto idX = 0; idX < 8; idX++)
		{
			frameHeader.magicWord[idX] = framePacket[idX];
		}
		idX += 8;
		for (auto idX = 0; idX < 4; idX++)
		{
			frameHeader.version[idX] = framePacket[idX + 8];
		}
		idX += 4;
        frameHeader.totalPacketLen = framePacket[idX]*1 + framePacket[idX + 1]*256.0 + framePacket[idX + 2]*65536.0 + framePacket[idX + 3]*1.6777216E+7;
        idX += 4;
        frameHeader.platform = framePacket[idX]*1 + framePacket[idX + 1]*256.0 + framePacket[idX + 2]*65536.0 + framePacket[idX + 3]*1.6777216E+7;
        idX += 4;
        frameHeader.frameNumber = framePacket[idX]*1 + framePacket[idX + 1]*256.0 + framePacket[idX + 2]*65536.0 + framePacket[idX + 3]*1.6777216E+7;
        idX += 4;
        frameHeader.timeCpuCycles = framePacket[idX]*1 + framePacket[idX + 1]*256.0 + framePacket[idX + 2]*65536.0 + framePacket[idX + 3]*1.6777216E+7;
        idX += 4;
        frameHeader.numDetectedObj = framePacket[idX]*1 + framePacket[idX + 1]*256.0 + framePacket[idX + 2]*65536.0 + framePacket[idX + 3]*1.6777216E+7;
        idX += 4;
        frameHeader.numTLVs = framePacket[idX]*1 + framePacket[idX + 1]*256.0 + framePacket[idX + 2]*65536.0 + framePacket[idX + 3]*1.6777216E+7;
        idX += 4;
		frameHeader.subFrameNumber = framePacket[idX]*1 + framePacket[idX + 1]*256.0 + framePacket[idX + 2]*65536.0 + framePacket[idX + 3]*1.6777216E+7;
        idX += 4;
 	}
	frameHeader.idX = idX;

	return frameHeader;
}

structTLV RadarObj::getTLV (uint8_t framePacket[], uint32_t numTLVs, uint32_t idX)
{
	structTLV tlv;

    // clear all the elements of the vector container
    tlv.payload.clear();
    ptCloud.x.clear();
    ptCloud.y.clear();
    ptCloud.z.clear();
    ptCloud.doppler.clear();

	// read all (numTLVs)TLVs to ptCloud
	for (auto tlvIdx = 0; tlvIdx < numTLVs; tlvIdx++)
	{
        // check the header of the TLV message
		tlv.type = framePacket[idX]*1 + framePacket[idX + 1]*256.0 + framePacket[idX + 2]*65536.0 + framePacket[idX + 3]*1.6777216E+7;
		idX += 4;
		tlv.length = framePacket[idX]*1 + framePacket[idX + 1]*256.0 + framePacket[idX + 2]*65536.0 + framePacket[idX + 3]*1.6777216E+7;
		idX += 4;
		for (auto i = 0; i < tlv.length ; i++)
			{
				tlv.payload.push_back(framePacket[idX + i]);
			}
		idX += tlv.length;
        tlv.idX = idX;

		switch (tlv.type)
		{
			case MMWDEMO_UART_MSG_DETECTED_POINTS :
			{
				// getGtrackPtCloud(payload)
				int numDetectedObj = tlv.length/16;
				byte2float data = {0};

				if (numDetectedObj)
				{
                    // Convert 4byte to float
					for (auto i = 0; i < tlv.length; i++)
                    {
                        data.myByte.push_back(tlv.payload[i]);
                    }

					for (auto i = 0; i < numDetectedObj; i++)
					{
						ptCloud.x.push_back(data.myFloat[i * 4]);
						ptCloud.y.push_back(data.myFloat[i * 4 + 1]);
						ptCloud.z.push_back(data.myFloat[i * 4 + 2]);
						ptCloud.doppler.push_back(data.myFloat[i * 4 + 3]);
				    }

                    // ROS_INFO("xyzv = %f, %f, %f, %f", ptCloud.x[0], ptCloud.y[0], ptCloud.z[0], ptCloud.doppler[0]);
				}
			}
			break;

			case MMWDEMO_UART_MSG_RANGE_PROFILE:
			break;
			case MMWDEMO_UART_MSG_NOISE_PROFILE:
			break;
			case MMWDEMO_UART_MSG_DETECTED_POINTS_SIDE_INFO:
			break;
			default:
			break;
		}
	}
    return tlv;
}

float RadarObj::processingOutput (void)
{
    // sorting vector distance in increasing order
    sort(ptCloud.y.begin(), ptCloud.y.end());

    // para config output
    float delta = 0.3;
    float numRangePt = 1;
    float outDistance = 0;
    float rangePt = (float)(ptCloud.y.size())/3;
    ROS_INFO("numDetected = %zu", ptCloud.y.size());

    for (int i = 0; i < ptCloud.y.size(); i++)
        {
        ROS_INFO("y= %f", ptCloud.y[i]);
        }

    // check the numDetected
    if (ptCloud.y.size() > 1)
    {
        for (auto i = 0; i < ptCloud.y.size() - 1; i++)
        {
            outDistance = ptCloud.y[i];
            numRangePt = 1;
            for (auto j = 0; j < ptCloud.y.size() - 1 - i; j++)
            {
                if(abs(ptCloud.y[i + j +1] - ptCloud.y[i]) < delta)
                {
                    outDistance += ptCloud.y[i + j +1];
                    numRangePt++;
                }
            }
            ROS_INFO("numRangePt = %f", numRangePt);

            if(numRangePt >= rangePt)
            {
                outDistance = outDistance/(int)numRangePt;
                break;
            }
            else
            {
                outDistance = ptCloud.y[0];
            }
        }
    }
    else if(ptCloud.y.size() == 1)
    {
        outDistance = ptCloud.y[0];
    }
    else
    {
        outDistance = 10.0;
    }
    ROS_INFO("outDisTmp = %f", outDistance);
    
    // fillter 2
    float delta_2 = 0.8;
    float numRangePt_2 = 1;
    bufDistance.push_back(outDistance); // vector global

    // buffer output distance has 5 elements
    if (bufDistance.size() == 6)
    {
        for (int i = 0; i < bufDistance.size() - 1; i++)
        {
        ROS_INFO("bufDis= %f", bufDistance[i+1]);
        }

        bufDistance.erase(bufDistance.begin());
        float rangePt_2 = (float)(bufDistance.size())/2; // how many elements to fit 

        for (auto i = 0; i < bufDistance.size() - 1; i++)
        {
            outDistance = bufDistance[i];
            numRangePt_2 = 1;
            for (auto j = 0; j < bufDistance.size() - 1 - i; j++)
            {
                if(abs(bufDistance[i + j +1] - bufDistance[i]) < delta_2)
                {
                    outDistance += bufDistance[i + j +1];
                    numRangePt_2++;
                }
            }
            ROS_INFO("numRangePt_2 = %f", numRangePt_2);

            if(numRangePt_2 >= rangePt_2)
            {
                outDistance = outDistance/(int)numRangePt_2;
                break;
            }
            else
            {
                outDistance = bufDistance[0];
                ROS_INFO("no filter 2");
            }
        }
    }

    return outDistance;
}

bool RadarObj::data_handler( std_msgs::UInt8MultiArray raw_data, uint16_t dataLen)
{
    bool is_data_ok = false;
    uint16_t numframesAvailable = 0;
    uint32_t totalPacketLen = 0;

    // Check for all numframesAvailable started by magic word
    // magic word = [2,1,4,3,6,5,8,7]
    for (uint16_t i = 0; i < dataLen - 7; i++)
    {
        if (raw_data.data[i] == 2 && raw_data.data[i+1] == 1 && raw_data.data[i+2] == 4 && raw_data.data[i+3] == 3 && raw_data.data[i+4] == 6 && raw_data.data[i+5] == 5 && raw_data.data[i+6] == 8 && raw_data.data[i+7] == 7)
        {
            numframesAvailable++;
        }
    }

    // Processing
    if (numframesAvailable > 0)
    {
        is_data_ok = true;
        uint32_t startIdx[numframesAvailable + 1];
        uint16_t framesAvailable = 0;

        // Check for all possible locations of the magic word to startIdx
        for (uint16_t i = 0; i < dataLen - 7; i++)
        {
            if (raw_data.data[i] == 2 && raw_data.data[i+1] == 1 && raw_data.data[i+2] == 4 && raw_data.data[i+3] == 3 && raw_data.data[i+4] == 6 && raw_data.data[i+5] == 5 && raw_data.data[i+6] == 8 && raw_data.data[i+7] == 7)
            {
                startIdx[framesAvailable] = i;
                framesAvailable++;
            }
        }

        // Check that startIdx is not empty // framePacket has executed only 1 frame
        startIdx[numframesAvailable] = dataLen;
        uint8_t framePacket[(startIdx[1] - startIdx[0])];
       
        //Remove the data before the first start index
        for (auto i = 0; i < (startIdx[1] - startIdx[0]); i++)
        {
            framePacket[i] = raw_data.data[startIdx[0] + i];
        }
        //update dataLen
        dataLen = startIdx[1] - startIdx[0];
        
        // Read the Header messages
        structHeader frameHeader = getFrameHeader(framePacket, dataLen);
        uint32_t idX = frameHeader.idX;

        // Read the TLV messages
        structTLV tlv = getTLV(framePacket, frameHeader.numTLVs, idX);
        idX = tlv.idX;

        // processing output
        float outDistance = processingOutput();

        // update output
        if (frameHeader.numDetectedObj)
        {
            Output.isObject = true;
            Output.msg_counter++;
            Output.distance = outDistance;
            ROS_INFO("distance ============= %f",outDistance);
        }
        else
        {
            Output.isObject = false;
            Output.distance = outDistance;
            ROS_INFO("distance ============= %f", outDistance);
        }
    }
    return is_data_ok;
    
}
