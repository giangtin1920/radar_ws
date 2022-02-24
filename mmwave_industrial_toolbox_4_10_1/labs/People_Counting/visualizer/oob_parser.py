import struct
import sys
import serial
import binascii
import time
import numpy as np
import math

from graphUtilities import rotX

import os
import datetime
 
list = [0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
        0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
        0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
        0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
        0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
        0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
        0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
        0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
        0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
        0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
        0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
        0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
        0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
        0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
        0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
        0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
        0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
        0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
        0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
        0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
        0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
        0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
        0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
        0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
        0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
        0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
        0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
        0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
        0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
        0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
        0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
        0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0]
 
 
def getcrc(buff, list):
    crc = 0
    for i in range(len(buff)):
        crc = list[((crc >> 8) ^ (buff[i])) & 0xFF] ^ (crc << 8)%(0xFFFF+1)
 
    return crc

#Initialize this Class to create a UART Parser. Initialization takes one argument:
# 1: String Lab_Type - These can be:
#   a. 3D People Counting
#   b. SDK Out of Box Demo
#   c. Long Range People Detection
#   d. Indoor False Detection Mitigation
#   e. (Legacy): Overhead People Counting
#   f. (Legacy) 2D People Counting
# Default is (f). Once initialize, call connectComPorts(self, UartComPort, DataComPort) to connect to device com ports.
# Then call readAndParseUart() to read one frame of data from the device. The gui this is packaged with calls this every frame period.
# readAndParseUart() will return all radar detection and tracking information.
class uartParserSDK():
    def __init__(self,type='(Legacy) 2D People Counting'):
        self.headerLength = 52
        self.magicWord = 0x708050603040102
        self.threeD = 0
        self.ifdm = 0
        self.replay = 0
        self.SDK3xPointCloud = 0
        self.SDK3xPC = 0
        self.capon3D = 0
        self.vital_signs = 0
        self.aop = 0
        self.maxPoints = 1150
        if (type=='(Legacy): Overhead People Counting'):
            self.threeD = 1
        elif (type=='Sense and Detect HVAC Control'):
            self.ifdm = 1
        elif (type=='Replay'): # unused
            self.replay = 1
        elif (type=="SDK Out of Box Demo"):
            self.SDK3xPointCloud = 1
        elif (type=="Long Range People Detection"):
            self.SDK3xPC = 1
        elif (type=='3D People Counting'):
            self.capon3D = 1
        elif (type == 'Capon3DAOP'): #unused
            self.capon3D = 1
            self.aop = 1
        elif (type == "Vital Signs"):
            self.vital_signs = 1
            #self.capon3D = 1
        #data storage
        self.pcPolar = np.zeros((5,self.maxPoints))
        self.pcBufPing = np.zeros((5,self.maxPoints))
        self.numDetectedObj = 0
        self.targetBufPing = np.ones((10,20))*-1
        self.indexBufPing = np.zeros((1,self.maxPoints))
        self.classifierOutput = []
        self.frameNum = 0
        self.missedFrames = 0
        self.byteData = bytes(1)
        self.oldData = []
        self.indexes = []
        self.numDetectedTarget = 0
        self.fail = 0
        self.unique = []
        self.savedData = []
        self.saveNum = 0
        self.saveNumTxt = 0
        self.replayData = []
        self.startTimeLast = 0
        self.saveReplay = 0
        self.savefHist = 0
        self.saveBinary = 0
        self.saveTextFile = 0
        self.fHistRT = np.empty((100,1), dtype=np.object)
        self.plotDimension = 0
        self.getUnique = 0
        self.CaponEC = 0
        self.VSinfo = np.zeros((687,2))
        self.ADCRAW = np.zeros((1536,2))
        self.now_time = datetime.datetime.now().strftime('%Y%m%d-%H%M')


        self.printVerbosity = 0 #set 0 for limited logFile printing, 1 for more logging
        
        if (self.capon3D):
            #3D people counting format
            #[frame #][header,pt cloud data,target info]
            #[][header][magic, version, packetLength, platform, frameNum, subFrameNum, chirpMargin, frameMargin, uartSentTime, trackProcessTime, numTLVs, checksum]
            #[][pt cloud][pt index][#elev, azim, doppler, range, snr]
            #[][target][Target #][TID,x,y,z,vx,vy,vz,ax,ay,az]
            self.textStructCapon3D = np.zeros(1000*3*self.maxPoints*10).reshape((1000,3,self.maxPoints,10))#[frame #][header,pt cloud data,target info]

        if (self.ifdm):
            #Sense and direct format
            #[frame #][header,pt cloud data,target info]
            #[][header][magic, version, platform, timestamp, packetLength, frameNum, subFrameNum, chirpMargin, frameMargin, uartSentTime, trackProcessTime, numTLVs, checksum]
            #[][pt cloud][pt index][#range, azim, doppler, snr]
            #[][target][Target #][TID,x,y,vx,vy,ax,ay]
            self.textStruct2D = np.zeros(1000*3*self.maxPoints*7).reshape((1000,3,self.maxPoints,7))#[frame #][header,pt cloud data,target info]
    
    def parseVitalSign(self, data, tlvLength):
        objStruct = '22f4h200f' # '22f4h674f'
        objStruct2 = '22f4h800B' #'22f4h2696B'
        objSize = struct.calcsize(objStruct)
        # print(objSize)
        temp = np.zeros(700)
        try:
            temp = struct.unpack(objStruct,data[:objSize])
            temp2 = struct.unpack(objStruct2,data[:objSize])

            aaaa = data[objSize:]

            crc_device = struct.unpack('1H',aaaa[:2])
            
            for i in range(0,2):

                self.VSinfo[0,i] = temp[i+0];#unwrapped_waveform
                self.VSinfo[1,i] = temp[i+2];#heart_waveform
                self.VSinfo[2,i] = temp[i+4];#breathing_waveform
                self.VSinfo[3,i] = temp[i+6];#heart_rate
                self.VSinfo[4,i] = temp[i+8];#breathing_rate
                self.VSinfo[5,i] = temp[i+10];#x
                self.VSinfo[6,i] = temp[i+12];#y
                self.VSinfo[7,i] = temp[i+14];#z
                self.VSinfo[8,i] = temp[i+16];#id
                self.VSinfo[9,i] = temp[i+18];#range
                self.VSinfo[10,i] = temp[i+20];#angle
                self.VSinfo[11,i] = temp[i+22];#rangeidx
                self.VSinfo[12,i] = temp[i+24];#angleidx
                #self.VSinfo[13,i] = temp[26+i];#angleidx
            for i in range(0,200): #range(0,674):
                self.VSinfo[13+i,0] = temp[26+i];#angleidx
            crc_python = ((getcrc(temp2[26:], list)))
        except:
            print('vitalsign parse error!')

    def WriteFile(self, data):
        filepath=self.now_time + '.bin'
        # print(filepath)
        # data22 = 123
        objStruct = '6144B'
        objSize = struct.calcsize(objStruct)
        binfile = open(filepath, 'ab+') #open binary file for append
        binfile.write(bytes(data))
        # print(3)
        binfile.close()

# for 128 range bin
    def parseADCRAW(self, data, tlvLength):
        objStruct = '3072h'
        objStruct2 = '6144B'
        objSize = struct.calcsize(objStruct)
        objSize2 = struct.calcsize(objStruct2)
        temp = np.zeros(3072,)
        temp2 = np.zeros(6144,dtype='int')
        temp3 = bytes(6144)

        try:
            temp = struct.unpack(objStruct,data[:objSize])
            temp2 = struct.unpack(objStruct2,data[:objSize])
            aaaa = data[objSize:]
            crc_device = struct.unpack('1H',aaaa[:2])
            # print('crc_device '+str(crc_device[0]))
            crc_python = ((getcrc(temp2, list)))
                                   
            if ((crc_python)) == ((crc_device[0])): 
                for i in range(1536):
                    self.ADCRAW[i,0] = temp[2*i];#angleidx
                    self.ADCRAW[i,1] = temp[2*i+1];#angleidx
                self.WriteFile(temp2[0:6144])
            else:
                print('ADC RAW CRC ERROR!\r\n 6843-CRC:')
                print(hex(crc_device[0]))
                print('\r\n GUI-CRC:')
                print (hex(crc_python)) 
                print(len(data))
                self.WriteFile(temp3[0:6144])                

        except Exception as e:
            print(e.args)
            self.WriteFile(temp3[0:6144]) 


#below funtions are used for converting output of labs that do not match SDK 3.x DPIF output
    #convert 2D polar People Counting to 3D Cartesian
    def polar2Cart(self):
        self.pcBufPing = np.empty((5,self.numDetectedObj))
        for n in range(0, self.numDetectedObj):
            self.pcBufPing[1,n] = self.pcPolar[0,n]*math.cos(self.pcPolar[1,n])  #y
            self.pcBufPing[0,n] = self.pcPolar[0,n]*math.sin(self.pcPolar[1,n])  #x
        self.pcBufPing[3,:] = self.pcPolar[2,0:self.numDetectedObj] #doppler
        self.pcBufPing[4,:] = self.pcPolar[3,0:self.numDetectedObj] #snr
        self.pcBufPing[2,:self.numDetectedObj] = 0                                 #Z is zero in 2D case

    #convert 3D people counting polar to 3D cartesian
    def polar2Cart3D(self):
        self.pcBufPing = np.empty((5,self.numDetectedObj))
        for n in range(0, self.numDetectedObj):
            self.pcBufPing[2,n] = self.pcPolar[0,n]*math.sin(self.pcPolar[2,n]) #z
            self.pcBufPing[0,n] = self.pcPolar[0,n]*math.cos(self.pcPolar[2,n])*math.sin(self.pcPolar[1,n]) #x
            self.pcBufPing[1,n] = self.pcPolar[0,n]*math.cos(self.pcPolar[2,n])*math.cos(self.pcPolar[1,n]) #y
        self.pcBufPing[3,:] = self.pcPolar[3,0:self.numDetectedObj] #doppler
        self.pcBufPing[4,:] = self.pcPolar[4,0:self.numDetectedObj] #snr
        #print(self.pcBufPing[:,:10])

    #decode People Counting TLV Header
    def tlvHeaderDecode(self, data):
        #print(len(data))
        tlvType, tlvLength = struct.unpack('2I', data)
        return tlvType, tlvLength

    #decode People Counting Point Cloud TLV
    def parseDetectedObjects(self, data, tlvLength):
        objStruct = '4f'
        objSize = struct.calcsize(objStruct)
        self.numDetectedObj = int((tlvLength)/16)
        for i in range(self.numDetectedObj):
            try:
                self.pcPolar[0,i], self.pcPolar[1,i], self.pcPolar[2,i], self.pcPolar[3,i] = struct.unpack(objStruct,data[:objSize])
                data = data[16:]
            except:
                self.numDectedObj = i
                break
        self.polar2Cart()

    #decode IFDM point Cloud TLV
    def parseDetectedObjectsIFDM(self, data, tlvLength):
        pUnitStruct = '4f'
        pUnitSize = struct.calcsize(pUnitStruct)
        pUnit = struct.unpack(pUnitStruct, data[:pUnitSize])
        data = data[pUnitSize:]
        objStruct = '2B2h'
        objSize = struct.calcsize(objStruct)
        self.numDetectedObj = int((tlvLength-16)/objSize)
        #print('Parsed Points: ', self.numDetectedObj)
        for i in range(self.numDetectedObj):
            try:
                az, doppler, ran, snr = struct.unpack(objStruct, data[:objSize])
                data = data[objSize:]
                #get range, azimuth, doppler, snr
                self.pcPolar[0,i] = ran*pUnit[2]           #range
                if (az >= 128):
                    az -= 256
                self.pcPolar[1,i] = math.radians(az*pUnit[0])   #azimuth
                self.pcPolar[2,i] = doppler*pUnit[1]       #doppler
                self.pcPolar[3,i] = snr*pUnit[3]           #snr
                
                #Sense and direct format
                #[frame #][header,pt cloud data,target info]
                #[][header][magic, version, platform, timestamp, packetLength, frameNum, subFrameNum, chirpMargin, frameMargin, uartSentTime, trackProcessTime, numTLVs, checksum]
                #[][pt cloud][pt index][#range, azim, doppler, snr]
                #[][target][Target #][TID,x,y,vx,vy,ax,ay]
                self.textStruct2D[self.frameNum%1000,1,i,0] = self.pcPolar[0,i] #range
                self.textStruct2D[self.frameNum%1000,1,i,1] = self.pcPolar[1,i] #az
                self.textStruct2D[self.frameNum%1000,1,i,2] = self.pcPolar[2,i] #doppler
                self.textStruct2D[self.frameNum%1000,1,i,3] = self.pcPolar[3,i] #snr
                
            except:
                self.numDetectedObj = i
                break
        self.polar2Cart()

    #decode 3D People Counting Point Cloud TLV
    def parseDetectedObjects3D(self, data, tlvLength):
        objStruct = '5f'
        objSize = struct.calcsize(objStruct)
        self.numDetectedObj = int(tlvLength/20)
        for i in range(self.numDetectedObj):
            try:
                self.pcPolar[0,i], self.pcPolar[1,i], self.pcPolar[2,i], self.pcPolar[3,i], self.pcPolar[4,i] = struct.unpack(objStruct,data[:objSize])
                data = data[20:]
            except:
                self.numDectedObj = i
                print('failed to get point cloud')
                break
        self.polar2Cart3D()

    #support for Capoin 3D point cloud
    #decode Capon 3D point Cloud TLV
    def parseCapon3DPolar(self, data, tlvLength):
        pUnitStruct = '5f'  #elev, azim, doppler, range, snr
        pUnitSize = struct.calcsize(pUnitStruct)
        pUnit = struct.unpack(pUnitStruct, data[:pUnitSize])
        data = data[pUnitSize:]
        objStruct = '2bh2H' #2 int8, 1 int16, 2 uint16
        objSize = struct.calcsize(objStruct)
        self.numDetectedObj = int((tlvLength-pUnitSize)/objSize)
        #if (self.printVerbosity == 1):
        #print('Parsed Points: ', self.numDetectedObj)
        for i in range(self.numDetectedObj):
            try:
                elev, az, doppler, ran, snr = struct.unpack(objStruct, data[:objSize])
                #print(elev, az, doppler, ran, snr)
                data = data[objSize:]
                #get range, azimuth, doppler, snr
                self.pcPolar[0,i] = ran*pUnit[3]           #range
                if (az >= 128):
                    print ('Az greater than 127')
                    az -= 256
                if (elev >= 128):
                    print ('Elev greater than 127')
                    elev -= 256
                if (doppler >= 32768):
                    print ('Doppler greater than 32768')
                    doppler -= 65536
                self.pcPolar[1,i] = az*pUnit[1]  #azimuth
                self.pcPolar[2,i] = elev*pUnit[0] #elevation
                self.pcPolar[3,i] = doppler*pUnit[2]       #doppler
                self.pcPolar[4,i] = snr*pUnit[4]           #snr
                
                #add pt cloud data to textStructCapon3DCapon3D for text file printing
                #self.textStructCapon3DCapon3D[,,,] = [frame #][header,pt cloud data,target info]
                #[][pt cloud = 0][pt index][#elev, azim, doppler, range, snr]
                if(self.capon3D):
                    self.textStructCapon3D[self.frameNum%1000,1,i,0] = self.pcPolar[2,i] #elev
                    self.textStructCapon3D[self.frameNum%1000,1,i,1] = self.pcPolar[1,i] #az
                    self.textStructCapon3D[self.frameNum%1000,1,i,2] = self.pcPolar[3,i] #doppler
                    self.textStructCapon3D[self.frameNum%1000,1,i,3] = self.pcPolar[0,i] #range
                    self.textStructCapon3D[self.frameNum%1000,1,i,4] = self.pcPolar[4,i] #snr
            except:
                data = data[objSize:]
                self.numDetectedObj = i
                print('Point Cloud TLV Parser Failed')
                break
        self.polar2Cart3D()

    #decode 2D People Counting Target List TLV
    def parseDetectedTracks(self, data, tlvLength):
        if (self.plotDimension):
            targetStruct = 'I8f9ff'
        else:
            targetStruct = 'I6f9ff'
        targetSize = struct.calcsize(targetStruct)
        self.numDetectedTarget = int(tlvLength/targetSize)
        targets = np.empty((13,self.numDetectedTarget))
        for i in range(self.numDetectedTarget):
            targetData = struct.unpack(targetStruct,data[:targetSize])
            targets[0,i]=int(targetData[0]) #TID
            targets[1:3,i]=targetData[1:3] #X,Y
            targets[3,i]=0 #Z=0
            targets[4:6,i]=targetData[3:5] #vX,Vy
            targets[6,i]=0#vZ=0
            targets[7:9,i]=targetData[5:7] #aX,aY
            targets[9,i]=0 #az=0
            if (self.plotDimension):
                targets[10:12,i]=targetData[7:9]
                targets[12,i]=1
            else:
                targets[10:12,i]=[0.75,0.75]
                targets[12,i]=1
            data = data[targetSize:]    
            
            if (self.saveTextFile and self.capon3D):
                self.textStruct2D[self.frameNum%1000,2,i,0] = targets[0,i] #TID
                self.textStruct2D[self.frameNum%1000,2,i,1] = targets[1,i] #x
                self.textStruct2D[self.frameNum%1000,2,i,2] = targets[2,i] #y
                
                self.textStruct2D[self.frameNum%1000,2,i,3] = targets[4,i] #vx
                self.textStruct2D[self.frameNum%1000,2,i,4] = targets[5,i] #vy
                
                self.textStruct2D[self.frameNum%1000,2,i,5] = targets[7,i] #ax
                self.textStruct2D[self.frameNum%1000,2,i,6] = targets[8,i] #ay
                
                if (self.printVerbosity == 1):
                    print('target added to textStructCapon3D')
        self.targetBufPing = targets           

    #decode 3D People Counting Target List TLV
    def parseDetectedTracks3D(self, data, tlvLength):
        if(self.vital_signs == 1):
            targetStruct = 'I27f'
        else:
            targetStruct = 'I9f'
        targetSize = struct.calcsize(targetStruct)
        self.numDetectedTarget = int(tlvLength/targetSize)
        targets = np.empty((13,self.numDetectedTarget))
        for i in range(self.numDetectedTarget):
            targetData = struct.unpack(targetStruct,data[:targetSize])
            targets[0:7,i]=targetData[0:7]
            targets[7:10,i]=[0,0,0]
            targets[10:13,i] = targetData[7:10]
            data = data[targetSize:]
        self.targetBufPing = targets

    #decode Target Index TLV
    def parseTargetAssociations(self, data):
        targetStruct = 'B'
        targetSize = struct.calcsize(targetStruct)
        numIndexes = int(len(data)/targetSize)
        self.indexes = []
        self.unique = []
        try:
            for i in range(numIndexes):
                ind = struct.unpack(targetStruct, data[:targetSize])
                self.indexes.append(ind[0])
                data = data[targetSize:]
            if (self.getUnique):
                uTemp = self.indexes[math.ceil(numIndexes/2):]
                self.indexes = self.indexes[:math.ceil(numIndexes/2)]
                for i in range(math.ceil(numIndexes/8)):
                    for j in range(8):
                        self.unique.append(getBit(uTemp[i], j))
        except:
            print('TLV Index Parse Fail')

    #decode Classifier output
    def parseClassifierOutput(self, data):
        classifierDataStruct = 'Ii'
        clOutSize = struct.calcsize(classifierDataStruct)
        self.classifierOutput = np.zeros((2,self.numDetectedTarget))
        for i in range(self.numDetectedTarget):
            self.classifierOutput[0,i], self.classifierOutput[1,i] = struct.unpack(classifierDataStruct, data[:clOutSize])
            data = data[clOutSize:]

#below is for labs that are compliant with SDK 3.x  This code can parse the point cloud TLV and point cloud side info TLV from the OOB demo.  
#It can parse the SDK3.x Compliant People Counting demo "tracker_dpc"
    #get SDK3.x Cartesian Point Cloud
    def parseSDK3xPoints(self, dataIn, numObj):
        pointStruct = '4f'
        pointLength = struct.calcsize(pointStruct)
        try:
            for i in range(numObj):
                self.pcBufPing[0,i], self.pcBufPing[1,i], self.pcBufPing[2,i], self.pcBufPing[3,i] = struct.unpack(pointStruct, dataIn[:pointLength])
                dataIn = dataIn[pointLength:]
            self.pcBufPing = self.pcBufPing[:,:numObj]
        except Exception as e:
            print(e)
            self.fail = 1

    #get Side Info SDK 3.x
    def parseSDK3xSideInfo(self, dataIn, numObj):
        sideInfoStruct = '2h'
        sideInfoLength = struct.calcsize(sideInfoStruct)
        try:
            for i in range(numObj):
                self.pcBufPing[4,i], unused = struct.unpack(sideInfoStruct, dataIn[:sideInfoLength])
                dataIn = dataIn[sideInfoLength:]
        except Exception as e:
            print(e)
            self.fail = 1

    #convert SDK compliant Polar Point Cloud to Cartesian
    def polar2CartSDK3(self):
        self.pcBufPing = np.empty((5,self.numDetectedObj))
        for n in range(0, self.numDetectedObj):
            self.pcBufPing[2,n] = self.pcPolar[0,n]*math.sin(self.pcPolar[2,n]) #z
            self.pcBufPing[0,n] = self.pcPolar[0,n]*math.cos(self.pcPolar[2,n])*math.sin(self.pcPolar[1,n]) #x
            self.pcBufPing[1,n] = self.pcPolar[0,n]*math.cos(self.pcPolar[2,n])*math.cos(self.pcPolar[1,n]) #y
        self.pcBufPing[3,:] = self.pcPolar[3,0:self.numDetectedObj] #doppler

    #decode SDK3.x Format Point Cloud in Polar Coordinates
    def parseSDK3xPolar(self, dataIn, tlvLength):
        pointStruct = '4f'
        pointLength = struct.calcsize(pointStruct)
        self.numDetectedObj = int(tlvLength/pointLength)
        try:
            for i in range(self.numDetectedObj):
                self.pcPolar[0,i], self.pcPolar[1,i], self.pcPolar[2,i], self.pcPolar[3,i] = struct.unpack(pointStruct, dataIn[:pointLength])
                dataIn = dataIn[pointLength:]
        except:
            self.fail = 1
            return
        self.polar2CartSDK3()

    #decode 3D People Counting Target List TLV

    #3D Struct format
    
    #uint32_t     tid;     /*! @brief   tracking ID */
    #float        posX;    /*! @brief   Detected target X coordinate, in m */
    #float        posY;    /*! @brief   Detected target Y coordinate, in m */
    #float        posZ;    /*! @brief   Detected target Z coordinate, in m */
    #float        velX;    /*! @brief   Detected target X velocity, in m/s */
    ##float       velY;    /*! @brief   Detected target Y velocity, in m/s */
    #float        velZ;    /*! @brief   Detected target Z velocity, in m/s */
    #float        accX;    /*! @brief   Detected target X acceleration, in m/s2 */
    #float        accY;    /*! @brief   Detected target Y acceleration, in m/s2 */
    #float        accZ;    /*! @brief   Detected target Z acceleration, in m/s2 */
    #float        ec[16];  /*! @brief   Target Error covarience matrix, [4x4 float], in row major order, range, azimuth, elev, doppler */
    #float        g;
    #float        confidenceLevel;    /*! @brief   Tracker confidence metric*/

    def parseDetectedTracksSDK3x(self, data, tlvLength):
        if (self.printVerbosity == 1):
            print(tlvLength)
        if (self.CaponEC):
            targetStruct = 'I27f'
        else:
            #targetStruct = 'I15f'
            targetStruct = 'I27f'
        targetSize = struct.calcsize(targetStruct)
        if (self.printVerbosity == 1):
            print('TargetSize=',targetSize)
        self.numDetectedTarget = int(tlvLength/targetSize)
        if (self.printVerbosity == 1):
            print('Num Detected Targets = ',self.numDetectedTarget)
        targets = np.empty((16,self.numDetectedTarget))
        rotTarget = [0,0,0]
        #theta = self.profile['elev_tilt']
        #print('theta = ',theta)
        #Rx = np.matrix([[ 1, 0           , 0           ],
        #           [ 0, math.cos(theta),-math.sin(theta)],
        #           [ 0, math.sin(theta), math.cos(theta)]])
        try:
            for i in range(self.numDetectedTarget):
                targetData = struct.unpack(targetStruct,data[:targetSize])
                if (self.printVerbosity == 1):
                    print(targetData)
                #tid, x, y
                if (self.CaponEC):
                    targets[0:13,i]=targetData[0:13]
                else:
                    #tid, pos x, pos y
                    targets[0:3,i]=targetData[0:3]
                    if (self.printVerbosity == 1):
                        print('Target Data TID,X,Y = ',targets[0:3,i])
                        print('i = ',i)
                    # pos z
                    targets[3,i] = targetData[3]
                    
                    #rotTargetDataX,rotTargetDataY,rotTargetDataZ = rotX (targetData[1],targetData[2],targetData[3],self.profile['elev_tilt'])
                    
                    #print('Target Data TID,X,Y = ',rotTargetDataX,', ',rotTargetDataY,', ',rotTargetDataZ)
                    #vel x, vel y
                    targets[4:6,i] = targetData[4:6]
                    #vel z
                    targets[6,i] = targetData[6]
                    # acc x, acc y
                    targets[7:9,i] = targetData[7:9]
                    # acc z
                    targets[9,i] = targetData[9]
                    #ec[16]
                    #targets[10:14,i]=targetData[10:14]
                    targets[10:13,i]=targetData[10:13]#Chris 2020-12-18
                    if (self.printVerbosity == 1):
                        print('ec = ',targets[10:13,i])
                    #g
                    #targets[14,i]=targetData[14]
                    targets[14,i]=targetData[26]
                    if (self.printVerbosity == 1):
                        print('g= ',targets[14,i])
                    #confidenceLevel
                    #targets[15,i]=targetData[15]
                    targets[15,i]=targetData[27]
                    if (self.printVerbosity == 1):
                        print('Confidence Level = ',targets[15,i])
                                        
                                        
                    #self.textStructCapon3D[[frame #],[header,pt cloud data,target info],index,data]
                    #[][header][magic, version, packetLength, platform, frameNum, subFrameNum, chirpMargin, frameMargin, uartSentTime, trackProcessTime, numTLVs, checksum]
                    #[][pt cloud][pt index][#elev, azim, doppler, range, snr]
                    #[][target][Target #][TID,x,y,z,vx,vy,vz,ax,ay,az]                    
                    if (self.saveTextFile and self.capon3D):
                        self.textStructCapon3D[self.frameNum%1000,2,i,0] = targets[0,i] #TID
                        self.textStructCapon3D[self.frameNum%1000,2,i,1] = targets[1,i] #x
                        self.textStructCapon3D[self.frameNum%1000,2,i,2] = targets[2,i] #y
                        self.textStructCapon3D[self.frameNum%1000,2,i,3] = targets[3,i] #z
                        self.textStructCapon3D[self.frameNum%1000,2,i,4] = targets[4,i] #vx
                        self.textStructCapon3D[self.frameNum%1000,2,i,5] = targets[5,i] #vy
                        self.textStructCapon3D[self.frameNum%1000,2,i,6] = targets[6,i] #vz
                        self.textStructCapon3D[self.frameNum%1000,2,i,7] = targets[7,i] #ax
                        self.textStructCapon3D[self.frameNum%1000,2,i,8] = targets[8,i] #ay
                        self.textStructCapon3D[self.frameNum%1000,2,i,9] = targets[9,i] #az
                        if (self.printVerbosity == 1):
                            print('target added to textStructCapon3D')
                data = data[targetSize:]
        except:
            print('Target TLV parse failed')
        self.targetBufPing = targets
        if (self.printVerbosity == 1):
            print(targets)
        


    #all TLV header decoding functions are below. Each lab with a Unique header or unique TLV set has its own header parsing function
    #decode Header and rest of TLVs for Legacy Labs and Indoor False detection mitigation
    def tlvHeader(self, data):
        #search for magic word
        self.targetBufPing = np.zeros((12,1))
        self.pcBufPing = np.zeros((5,self.maxPoints))
        self.indexes = []
        frameNum = -1
        self.numDetectedTarget = 0
        self.numDetectedObj = 0
        #search until we find magic word
        while (1):
            try:
                magic, version, platform, timestamp, packetLength, frameNum, subFrameNum, chirpMargin, frameMargin, uartSentTime, trackProcessTime, numTLVs, checksum =  struct.unpack('Q10I2H', data[:self.headerLength])
            except:
                #bad data, return
                self.fail = 1
                return data
            if (magic != self.magicWord):
                #wrong magic word, increment pointer by 1 and try again
                data = data[1:]
            else:
                #we have correct magic word, proceed to parse rest of data
                break
        
        #Sense and direct format
        #[][header][magic, version, platform, timestamp, packetLength, frameNum, subFrameNum, chirpMargin, frameMargin, uartSentTime, trackProcessTime, numTLVs, checksum]
        if (self.saveTextFile):
            self.textStruct2D[self.frameNum%1000,0,0,0] = magic
            self.textStruct2D[self.frameNum%1000,0,1,0] = version
            self.textStruct2D[self.frameNum%1000,0,2,0] = platform
            self.textStruct2D[self.frameNum%1000,0,3,0] = timestamp
            self.textStruct2D[self.frameNum%1000,0,4,0] = packetLength
            self.textStruct2D[self.frameNum%1000,0,5,0] = frameNum
            self.textStruct2D[self.frameNum%1000,0,6,0] = subFrameNum
            self.textStruct2D[self.frameNum%1000,0,7,0] = chirpMargin
            self.textStruct2D[self.frameNum%1000,0,8,0] = frameMargin
            self.textStruct2D[self.frameNum%1000,0,9,0] = uartSentTime
            self.textStruct2D[self.frameNum%1000,0,10,0] = trackProcessTime
            self.textStruct2D[self.frameNum%1000,0,11,0] = numTLVs
            self.textStruct2D[self.frameNum%1000,0,12,0] = checksum
            if (self.printVerbosity == 1):
                print('FrameNumber = ',self.textStruct2D[self.frameNum%1000,0,5,0])
                
        if (self.frameNum != frameNum):
            self.missedFrames += 1
            self.frameNum = frameNum
        self.frameNum += 1
        if (len(data) < packetLength):
            ndata = self.dataCom.read(packetLength-len(data))
            if (self.saveBinary):
                self.oldData += ndata
            data += ndata
        data = data[self.headerLength:]
        for i in range(numTLVs):
            try:
                tlvType, tlvLength = self.tlvHeaderDecode(data[:8])
            except:
                print('read fail: not enough data')
                self.missedFrames += 1
                self.fail=1
                break
            try:
                data = data[8:]
                if (tlvType == 6):
                    if(self.threeD):
                        self.parseDetectedObjects3D(data[:tlvLength], tlvLength-8)
                    elif(self.ifdm):
                        self.parseDetectedObjectsIFDM(data[:tlvLength], tlvLength-8)
                    else:
                        self.parseDetectedObjects(data[:tlvLength], tlvLength-8)
                elif (tlvType == 7):
                    if(self.threeD):
                        self.parseDetectedTracks3D(data[:tlvLength], tlvLength-8)
                    else:
                        self.parseDetectedTracks(data[:tlvLength], tlvLength-8)
                elif (tlvType == 8):
                    self.parseTargetAssociations(data[:tlvLength-8])
                elif (tlvType == 9):
                    self.parseClassifierOutput(data[:tlvLength-8])
                data = data[tlvLength-8:]
            except:
                print('Not enough data')
                print('Data length: ', len(data))
                print('Reported Packet Length: ', packetLength)
                self.fail=1
                return data
        return data

    #parsing for SDK 3.x Point Cloud
    def sdk3xTLVHeader(self, dataIn):
        #reset point buffers
        self.pcBufPing = np.zeros((5,self.maxPoints))
        headerStruct = 'Q8I'
        headerLength = struct.calcsize(headerStruct)
        tlvHeaderLength = 8
        #search until we find magic word
        while(1):
            try:
                magic, version, totalPacketLen, platform, self.frameNum, timeCPUCycles, self.numDetectedObj, numTLVs, subFrameNum = struct.unpack(headerStruct, dataIn[:headerLength])
            except:
                #bad data, return
                self.fail = 1
                return dataIn
            if (magic != self.magicWord):
                #wrong magic word, increment pointer by 1 and try again
                dataIn = dataIn[1:]
            else:
                #we have correct magic word, proceed to parse rest of data
                break
        dataIn = dataIn[headerLength:]
        remainingData = totalPacketLen - len(dataIn)
        count = 0
        #check to ensure we have all of the data
        while (remainingData > 0 and count < 3):
            newData = self.dataCom.read(remainingData)
            remainingData = totalPacketLen - len(dataIn) - len(newData)
            dataIn += newData
            count += 1
            if (self.saveBinary):
                self.oldData += newData
        #now check TLVs
        #print ('got tlvs sdk3x')
        for i in range(numTLVs):
            try:
                tlvType, tlvLength = self.tlvHeaderDecode(dataIn[:tlvHeaderLength])
            except Exception as e:
                print(e)
                print ('failed to read OOB SDK3.x TLV')
            dataIn = dataIn[tlvHeaderLength:]
            if (tlvType == 1):
                self.parseSDK3xPoints(dataIn[:tlvLength], self.numDetectedObj)
                dataIn = dataIn[tlvLength:]
            elif (tlvType == 7):
                self.parseSDK3xSideInfo(dataIn[:tlvLength], self.numDetectedObj)
                dataIn = dataIn[tlvLength:]
        return dataIn


    #parsing for SDK 3.x DPIF compliant People Counting
    def sdk3xPCHeader(self, dataIn):
        #reset point buffers
        self.pcBufPing = np.zeros((5,self.maxPoints))
        self.targetBufPing = np.zeros((13,20))
        self.indexes = []
        tlvHeaderLength = 8
        #search until we find magic word
        while (1):
            try:
                magic, version, platform, timestamp, packetLength, self.frameNum, subFrameNum, chirpMargin, frameMargin, uartSentTime, trackProcessTime, numTLVs, checksum =  struct.unpack('Q10I2H', dataIn[:self.headerLength])
            except:
                #bad data, return
                self.fail = 1
                return dataIn
            if (magic != self.magicWord):
                #wrong magic word, increment pointer by 1 and try again
                dataIn = dataIn[1:]
            else:
                #we have correct magic word, proceed to parse rest of data
                break
        dataIn = dataIn[self.headerLength:]
        remainingData = packetLength - len(dataIn)
        if (self.printVerbosity == 1):
            print('pl: ', packetLength)
            print('remainingData ', remainingData)
        #check to ensure we have all of the data
        #check to ensure we have all of the data
        count = 0
        while (remainingData > 0 and count < 3):
            if (self.printVerbosity == 1):
                print('RD Loop')
            newData = self.dataCom.read(remainingData)
            remainingData = packetLength - len(dataIn) - len(newData)
            dataIn += newData
            count += 1
            if (remainingData == 0):
                if (self.saveBinary):
                    self.oldData += newData
        #now check TLVs
        if (self.printVerbosity == 1):
            print('Frame: ', self.frameNum)
            print(len(dataIn))
            print(numTLVs)
        for i in range(numTLVs):
            try:
                #print("DataIn Type", type(dataIn))
                tlvType, tlvLength = self.tlvHeaderDecode(dataIn[:tlvHeaderLength])
                if (self.printVerbosity == 1):
                    print('TLV length = ',tlvLength)
            except Exception as e:
                if (self.printVerbosity == 1):
                    print(e)
                    print ('failed to read OOB SDK3.x TLV')
                    print('TLV num: ',i)
            dataIn = dataIn[tlvHeaderLength:]
            dataLength = tlvLength
            if (tlvType == 6):
                #DPIF Polar Coordinates
                #print('pointcloud lrpd')
                self.parseSDK3xPolar(dataIn[:dataLength], dataLength)
            elif (tlvType == 7):
                #target 3D
                self.parseDetectedTracksSDK3x(dataIn[:dataLength], dataLength)
            elif (tlvType == 8):
                #target index
                self.parseTargetAssociations(dataIn[:dataLength])
            elif (tlvType == 9):
                #side info
                self.parseSDK3xSideInfo(dataIn[:dataLength], self.numDetectedObj)
            dataIn = dataIn[dataLength:]
        return dataIn

    #parsing for 3D People Counting lab
    def Capon3DHeader_VS(self, dataIn):

        #reset point buffers
        self.pcBufPing = np.zeros((5,self.maxPoints))
        self.pcPolar = np.zeros((5,self.maxPoints))
        self.targetBufPing = np.zeros((13,20))
        self.numDetectedTarget = 0
        self.numDetectedObj = 0
        self.indexes = []
        tlvHeaderLength = 8
        headerLength = 48
        #stay in this loop until we find the magic word or run out of data to parse
        while (1):
            try:
                magic, version, packetLength, platform, self.frameNum, subFrameNum, chirpMargin, frameMargin, uartSentTime, trackProcessTime, numTLVs, checksum =  struct.unpack('Q9I2H', dataIn[:headerLength])
            except:
                #bad data, return
                self.fail = 1
                return dataIn
            if (magic != self.magicWord):
                #wrong magic word, increment pointer by 1 and try again
                dataIn = dataIn[1:]
            else:
                #got magic word, proceed to parse
                break
        dataIn = dataIn[headerLength:]
        #check to ensure we have all of the data
        while(1):
            remainingData = packetLength - len(dataIn) - headerLength
            # print('remainingData '+str(int(remainingData)))
            if (remainingData > 0):
                # print(remainingData)
                newData = self.dataCom.read(remainingData)
                dataIn += newData
                self.oldData += newData
            else:
                break
        #now check TLVs
        # print("\r\n hello")
        debug_print_flag = 0
        if(int(len(dataIn)+headerLength) != int(packetLength)):
            debug_print_flag = 1
            print('dataIn '+str(len(dataIn)))
        for i in range(numTLVs):
            try:
                #print("DataIn Type", type(dataIn))
                tlvType, tlvLength = self.tlvHeaderDecode(dataIn[:tlvHeaderLength])
            except Exception as e:                
                #print(e)
                #print ('failed to read OOB SDK3.x TLV')
                err = 1;
            if(debug_print_flag == 1):
                printbuf = 'packetLength:' + str(int(packetLength)) +', tlvType' + str(int(tlvType)) + ', tlvLength: '+ str(int(tlvLength)) + ', dataIn_Length: '+ str(len(dataIn))
                print(printbuf)
            dataIn = dataIn[tlvHeaderLength:]
            dataLength = tlvLength-tlvHeaderLength          
            if (tlvType == 6):
                #DPIF Polar Coordinates
                self.parseCapon3DPolar(dataIn[:dataLength], dataLength)
                dataIn = dataIn[dataLength:]
            elif (tlvType == 7):
                #target 3D
                self.parseDetectedTracks3D(dataIn[:dataLength], dataLength)
                dataIn = dataIn[dataLength:]
            elif (tlvType == 8):
                #target index
                self.parseTargetAssociations(dataIn[:dataLength])
                dataIn = dataIn[dataLength:]
            elif (tlvType == 9):
                print('type9')
                dataIn = dataIn[dataLength:]
            elif (tlvType == 10):
                #vital signs info
                printbuf = 'type10, length: ' + str(dataLength) 
                # print(printbuf)
                self.parseVitalSign(dataIn[:dataLength], dataLength)
                dataIn = dataIn[dataLength:]  
            elif (tlvType == 12):               
                objStruct = '1I'
                objSize = struct.calcsize(objStruct)                
                temp = np.zeros(1)
                temp = struct.unpack(objStruct,dataIn[:objSize])
                printbuf = 'type12, length: ' + str(dataLength) +', '+ str(temp)
                # print(printbuf)
                dataIn = dataIn[dataLength:]    
            else :
                dataIn = dataIn[dataLength:]
        return dataIn


    #parsing for 3D People Counting lab
    def Capon3DHeader(self, dataIn):
        
        #reset point buffers
        self.pcBufPing = np.zeros((5,self.maxPoints))
        self.pcPolar = np.zeros((5,self.maxPoints))
        self.targetBufPing = np.zeros((13,20))
        self.numDetectedTarget = 0
        self.numDetectedObj = 0
        self.indexes = []
        tlvHeaderLength = 8
        headerLength = 48
        #stay in this loop until we find the magic word or run out of data to parse
        while (1):
            try:
                magic, version, packetLength, platform, frameNum, subFrameNum, chirpMargin, frameMargin, uartSentTime, trackProcessTime, numTLVs, checksum =  struct.unpack('Q9I2H', dataIn[:headerLength])
            except Exception as e:
                #bad data, return
                #print("Cannot Read Frame Header")
                #print(e)
                self.fail = 1
                return dataIn
            if (magic != self.magicWord):
                #wrong magic word, increment pointer by 1 and try again
                dataIn = dataIn[1:]
            else:
                #got magic word, proceed to parse
                break
        

        dataIn = dataIn[headerLength:]
        remainingData = packetLength - len(dataIn) - headerLength
        #check to ensure we have all of the data
        #print('remaining data = ',remainingData)
        if (remainingData > 0):
            newData = self.dataCom.read(remainingData)
            remainingData = packetLength - len(dataIn) - headerLength - len(newData)
            dataIn += newData
            if (self.saveBinary):
                self.oldData += newData
        if (self.saveTextFile and self.capon3D):
            self.textStructCapon3D[self.frameNum%1000,0,0,0] = magic
            self.textStructCapon3D[self.frameNum%1000,0,1,0] = version
            self.textStructCapon3D[self.frameNum%1000,0,2,0] = packetLength
            self.textStructCapon3D[self.frameNum%1000,0,3,0] = platform
            self.textStructCapon3D[self.frameNum%1000,0,4,0] = frameNum
            self.textStructCapon3D[self.frameNum%1000,0,5,0] = subFrameNum
            self.textStructCapon3D[self.frameNum%1000,0,6,0] = chirpMargin
            self.textStructCapon3D[self.frameNum%1000,0,7,0] = frameMargin
            self.textStructCapon3D[self.frameNum%1000,0,8,0] = uartSentTime
            self.textStructCapon3D[self.frameNum%1000,0,9,0] = trackProcessTime
            self.textStructCapon3D[self.frameNum%1000,0,10,0] = numTLVs
            self.textStructCapon3D[self.frameNum%1000,0,11,0] = checksum
            if (self.printVerbosity == 1):
                print('FrameNumber = ',self.textStructCapon3D[self.frameNum%1000,0,4,0])
         
        #now check TLVs
        for i in range(numTLVs):
            #try:
            #print("DataIn Type", type(dataIn))
            try:
                tlvType, tlvLength = self.tlvHeaderDecode(dataIn[:tlvHeaderLength])
                dataIn = dataIn[tlvHeaderLength:]
                dataLength = tlvLength-tlvHeaderLength
            except:
                print('TLV Header Parsing Failure')
                self.fail = 1
                return dataIn
            if (tlvType == 6):
                #DPIF Polar Coordinates
                self.parseCapon3DPolar(dataIn[:dataLength], dataLength)
            elif (tlvType == 7):
                #target 3D
                self.parseDetectedTracksSDK3x(dataIn[:dataLength], dataLength)
            elif (tlvType == 8):
                #target index
                self.parseTargetAssociations(dataIn[:dataLength])
            elif (tlvType == 9):
                if (self.printVerbosity == 1):
                    print('type9')
                #side info
                #self.parseSDK3xSideInfo(dataIn[:dataLength], self.numDetectedObj)
            dataIn = dataIn[dataLength:]
            #except Exception as e:
            #    print(e)
            #    print ('failed to read OOB SDK3.x TLV')
        if (self.frameNum + 1 != frameNum):
            self.missedFrames += frameNum - (self.frameNum + 1)
        self.frameNum = frameNum
        return dataIn


    # This function is always called - first read the UART, then call a function to parse the specific demo output
    # This will return 1 frame of data. This must be called for each frame of data that is expected. It will return a dict containing:
    #   1. Point Cloud
    #   2. Target List
    #   3. Target Indexes
    #   4. number of detected points in point cloud
    #   5. number of detected targets
    #   6. frame number
    #   7. Fail - if one, data is bad
    #   8. classifier output
    # Point Cloud and Target structure are liable to change based on the lab. Output is always cartesian.
    def readAndParseUart(self):
        self.fail = 0
        if (self.replay):
            return self.replayHist()

        

        if(self.vital_signs == 1):
            numBytes = 10000
        else:
            numBytes = 4666

        data = self.dataCom.read(numBytes)
        if (self.byteData is None):
            self.byteData = data
        else:
            self.byteData += data
        if (self.saveBinary):
            self.oldData += data
        #try:
        if (self.SDK3xPointCloud == 1):
            self.byteData = self.sdk3xTLVHeader(self.byteData)
        elif (self.SDK3xPC == 1):
            self.byteData = self.sdk3xPCHeader(self.byteData)
        elif(self.vital_signs == 1):
            self.byteData = self.Capon3DHeader_VS(self.byteData)
        elif (self.capon3D == 1):
            self.byteData = self.Capon3DHeader(self.byteData)
        else:
            self.byteData = self.tlvHeader(self.byteData)
        #except Exception as e:
        #    print(e)
        #    self.fail = 1
        #return data after parsing and save to replay file
        if (self.fail):
            if(self.vital_signs == 1):
                return self.pcBufPing, self.targetBufPing, self.indexes, self.numDetectedObj, self.numDetectedTarget, self.frameNum, self.fail, self.classifierOutput, self.VSinfo, self.ADCRAW
            else:
                return self.pcBufPing, self.targetBufPing, self.indexes, self.numDetectedObj, self.numDetectedTarget, self.frameNum, self.fail, self.classifierOutput
        if (self.saveBinary):
            if (self.frameNum%1000 == 0):
                toSave = bytes(self.oldData)
                fileName = 'binData/pHistBytes_'+str(self.saveNum)+'.bin'
                self.saveNum += 1
                bfile = open(fileName, 'wb')
                bfile.write(toSave)
                self.oldData = []
                print ('Missed Frames ' + str(self.missedFrames)+'/1000')
                self.missedFrames = 0
                bfile.close
        if (self.saveTextFile):
            if (self.frameNum%1000 == 0):
                if (self.capon3D):
                    toSave = self.textStructCapon3D
                elif (self.ifdm):
                    toSave = self.textStruct2D
                print('Saved data file ', self.saveNumTxt)
                fileName = 'binData/pHistText_'+str(self.saveNumTxt)+'.csv'
                if (self.saveNumTxt < 75):
                    self.saveNumTxt += 1
                else:
                    self.saveNumTxt = 0
                tfile = open(fileName, 'w')
                tfile.write('This file contains parsed UART data in sensor centric coordinates\n')
                tfile.write('file format version 1.0\n')
                #tfile.write(str(toSave))
                
                
                if (self.capon3D):
                    #[frame #][header,pt cloud data,target info]
                    #[][header][magic, version, packetLength, platform, frameNum, subFrameNum, chirpMargin, frameMargin, uartSentTime, trackProcessTime, numTLVs, checksum]
                    #[][pt cloud][pt index][#elev, azim, doppler, range, snr]
                    #[][target][Target #][TID,x,y,z,vx,vy,vz,ax,ay,az]
                    
                    for i in range (1000):
                        tfile.write('magic, version, packetLength, platform, frameNum, subFrameNum, chirpMargin, frameMargin, uartSentTime, trackProcessTime, numTLVs, checksum\n')
                        for j in range (0,12):
                            tfile.write(str(self.textStructCapon3D[i,0,j,0]))
                            tfile.write(',')
                            #print(str(self.textStructCapon3D[i,0,j,0]))
                        tfile.write('\n')
                        tfile.write('elev, azim, doppler, range, snr\n')
                        for j in range (np.count_nonzero(self.textStructCapon3D[i,1,:,0])): #self.numDetectedObj):#len(self.textStructCapon3D[i,1,:,0]!=0)):
                            for k in range(5):
                                tfile.write(str(self.textStructCapon3D[i,1,j,k]))
                                tfile.write(',')
                            tfile.write('\n')

                        tfile.write('TID,x,y,z,vx,vy,vz,ax,ay,az\n')
                        for j in range (np.count_nonzero(self.textStructCapon3D[i,2,:,1])):
                            for k in range(10):
                                tfile.write(str(self.textStructCapon3D[i,2,j,k]))
                                tfile.write(',')
                            tfile.write('\n')
                    self.textStructCapon3D = np.zeros(1000*3*12*self.maxPoints).reshape((1000,3,12,self.maxPoints))#[frame #][header,pt cloud data,target info]
                    tfile.close
                    
                if (self.ifdm):
                    #Sense and direct format
                    #[frame #][header,pt cloud data,target info]
                    #[][header][magic, version, platform, timestamp, packetLength, frameNum, subFrameNum, chirpMargin, frameMargin, uartSentTime, trackProcessTime, numTLVs, checksum]
                    #[][pt cloud][pt index][#range, azim, doppler, snr]
                    #[][target][Target #][TID,x,y,vx,vy,ax,ay]
                    for i in range (1000):
                        tfile.write('magic, version, platform, timestamp, packetLength, frameNum, subFrameNum, chirpMargin, frameMargin, uartSentTime, trackProcessTime, numTLVs, checksum\n')
                        for j in range (13):
                             tfile.write(str(self.textStruct2D[i,0,j,0]))
                             tfile.write(',')
                        tfile.write('\n')
                        tfile.write('range, azim, doppler, snr\n')
                        for j in range (np.count_nonzero(self.textStruct2D[i,1,:,0])): 
                            for k in range(4):
                                tfile.write(str(self.textStruct2D[i,1,j,k]))
                                tfile.write(',')
                            tfile.write('\n')
                        tfile.write('TID,x,y,vx,vy,ax,ay\n')
                        for j in range (np.count_nonzero(self.textStruct2D[i,2,:,1])):
                            for k in range(7):
                                tfile.write(str(self.textStruct2D[i,2,j,k]))
                                tfile.write(',')
                            tfile.write('\n')
                    self.textStruct2D = np.zeros(1000*3*self.maxPoints*7).reshape((1000,3,self.maxPoints,7))#[frame #][header,pt cloud data,target info]
                    tfile.close
                    
                    
                    
        parseEnd = int(round(time.time()*1000))
        #print (self.pcBufPing)
        if(self.vital_signs == 1):
            return self.pcBufPing, self.targetBufPing, self.indexes, self.numDetectedObj, self.numDetectedTarget, self.frameNum, self.fail, self.classifierOutput, self.VSinfo, self.ADCRAW
        else:
            return self.pcBufPing, self.targetBufPing, self.indexes, self.numDetectedObj, self.numDetectedTarget, self.frameNum, self.fail, self.classifierOutput

    #find various utility functions here for connecting to COM Ports, send data, etc...
    #connect to com ports
    # Call this function to connect to the comport. This takes arguments self (intrinsic), uartCom, and dataCom. No return, but sets internal variables in the parser object.
    def connectComPorts(self, uartCom, dataCom):
        self.uartCom = serial.Serial(uartCom, 115200,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,timeout=0.3)
        self.dataCom = serial.Serial(dataCom, 921600,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,timeout=0.025)
        self.dataCom.reset_output_buffer()
        print('Connected')

    #send cfg over uart
    def sendCfg(self, cfg):
        for line in cfg:
            time.sleep(.03)
            self.uartCom.write(line.encode())
            ack = self.uartCom.readline()
            print(ack)
            ack = self.uartCom.readline()
            print(ack)
        time.sleep(3)
        self.uartCom.reset_input_buffer()
        self.uartCom.close()

    #send single command to device over UART Com.
    def sendLine(self, line):
        self.uartCom.write(line.encode())
        ack = self.uartCom.readline()
        print(ack)
        ack = self.uartCom.readline()
        print(ack)

    def replayHist(self):
        if (self.replayData):
            #print('reading data')
            #print('fail: ',self.fail)
            #print(len(self.replayData))
            #print(self.replayData[0:8])
            self.replayData = self.Capon3DHeader(self.replayData)
            #print('fail: ',self.fail)
            return self.pcBufPing, self.targetBufPing, self.indexes, self.numDetectedObj, self.numDetectedTarget, self.frameNum, self.fail, self.classifierOutput
            #frameData = self.replayData[0]
            #self.replayData = self.replayData[1:]
            #return frameData['PointCloud'], frameData['Targets'], frameData['Indexes'], frameData['Number Points'], frameData['NumberTracks'],frameData['frame'],0, frameData['ClassifierOutput'], frameData['Uniqueness']
        else:
            filename = 'overheadDebug/binData/pHistBytes_'+str(self.saveNum)+'.bin'
            #filename = 'Replay1Person10mShort/pHistRT'+str(self.saveNum)+'.pkl'
            self.saveNum+=1
            try:
                dfile = open(filename, 'rb', 0)
            except:
                print('cant open ', filename)
                return -1
            self.replayData = bytes(list(dfile.read()))
            if (self.replayData):
                print('entering replay')
                return self.replayHist()
            else:
                return -1
        
def getBit(byte, bitNum):
    mask = 1 << bitNum
    if (byte&mask):
        return 1
    else:
        return 0