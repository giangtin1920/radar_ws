"""
 @file  server.py

 @brief
   Packages mmWave UART stream and sends to client over socket

Copyright (C) 2021 Texas Instruments Incorporated - http://www.ti.com/

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:
 
  Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.
 
  Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the
  distribution.
 
  Neither the name of Texas Instruments Incorporated nor the names of
  its contributors may be used to endorse or promote products derived
  from this software without specific prior written permission.
 
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

"""
Client.py opens two serial ports for communication with the mmWave sensor. 
One port is used for sending sensor configuration, while the other is for 
retrieving data, such as point cloud and sensor statistics. This data is then
transmitted to a server via sockets. See server.py. 

This script was tested with Python 3.7, mmWave SDK 3.5, and IWR6843ISK. 

If pySerial is not installed, it can be installed from PyPI with:
`python3 -m pip install pyserial`
"""

import socket
import struct
import time
import serial

HOST = '127.0.0.1'  # The server's hostname or IP address
PORT = 65432        # The port used by the server

""" 
Change the radarCfgFile variable to match the file name of the desired *.cfg file
to load into the TI mmWave sensor. 
"""
radarCfgFile = '6843_2d.cfg'

"""
On Windows machine, serial port variables should be listed as 'COMx'. 
In device manager, the CLI Serial port will be described as the "Enhanced" or "Application" port.
The Data Serial port will be described as the "Standard" or "Data" port.
For example:
    cliSerialPort = 'COM14'
    dataSerialPort = 'COM13'

On linux machines, serial port variables should be listed as '/dev/ttyUSBx' or '/dev/ttyACMx'
For example:
    cliSerialPort = '/dev/ttyUSB0'
    dataSerialPort = '/dev/ttyUSB1'
"""
cliSerialPort = 'COM14'
dataSerialPort = 'COM13'
cliDevice = serial.Serial(cliSerialPort,115200, timeout=3.0)
dataDevice = serial.Serial(dataSerialPort, 921600, timeout=3.0)

headerLength = 40
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    print("Connecting to socket")
    s.connect((HOST, PORT))
    """
    Write *.cfg file to the CLI/User port of the sensor
    """
    cliDevice.write(('\r').encode())
    for i in open(radarCfgFile):
        cliDevice.write((i).encode())
        temp = cliDevice.readline()
        print(temp)
        if b"Ignored" in temp:
            print(cliDevice.readline())    
        print(cliDevice.readline())
        time.sleep(0.05)
    cliDevice.close()
    while True:
        header = dataDevice.read(headerLength)
        magic, version, length, platform, frameNum, cpuCycles, numObj, numTLVs, subFrameNum = struct.unpack('Q8I', header)
        # if magic is wrong, cycle through bytes until data stream is correct
        while magic != 506660481457717506:
            print("Bad magic word.")
            header = header[1:] + dataDevice.read(1)
            magic, version, length, platform, frameNum, cpuCycles, numObj, numTLVs, subFrameNum = struct.unpack('Q8I', header)
        print("FrameNum: " + str(frameNum) + ", NumObj: " + str(numObj))
        data = dataDevice.read(length - headerLength)
        s.sendall(header + data)

dataDevice.close()
    