import serial
import time
import numpy as np

#configFileName = '68xx_traffic_monitoring_70m_MIMO_2D.cfg'
configFileName = 'xwr64xx_profile_OOB.cfg'

CLIport = {}
Dataport = {}
byteBuffer = np.zeros(2**15,dtype = 'uint8')
byteBufferLength = 0;
maxBufferSize = 2**15;

CLIport  = serial.Serial('COM7', 115200)
Dataport = serial.Serial('COM6', 921600)


    
#"""
while True:
# Check that the buffer is not full, and then add the data to the buffer
    readBuffer = Dataport.read(Dataport.in_waiting)
    s = readBuffer
    byteVec = np.frombuffer(readBuffer, dtype = 'uint8')
    byteCount = len(byteVec)
    byteBuffer[byteBufferLength:byteBufferLength + byteCount] = byteVec[:byteCount]
    byteBufferLength = byteBufferLength + byteCount
    print(s)

"""
        
        
while True:
    byteCount=Dataport.inWaiting()   #get the count of bytes in the buffer #read a variable amount of bytes from buffer
    s = Dataport.read(byteCount)
    print(s)

"""