/*
 *   @file  spiTest.cpp
 *
 *   @brief
 *      Unit Test code for the PC FTDI SPI driver. It will work as a SPI master to
 *  communicate with XWR1xxx device.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2016 Texas Instruments, Inc.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/

#include "stdio.h"
#include "stdlib.h"
#include "ftd2xx.h"
#include <unistd.h>
#include <cstring>


/**************************************************************************
 *************************** Local Definitions ****************************
 **************************************************************************/
/* Data Echo test defines */
#define COMMAND_LENGTH           12
#define MAGIC_NUMBER             0x3456
#define SPI_TEST_MSGLEN          128
#define SPI_TEST_SYNC_MSGLEN     0
#define SPI_TEST_LOOP            5000

#define DEBUG                    0


/*Port A Pin config,either input (0) & output(1)*/
#define FT_PORTA_BIT0_SPICLK1                 (1)
#define FT_PORTA_BIT1_MOSI1                   (1<<1)
#define FT_PORTA_BIT2_MISO1                   (0<<2)
#define FT_PORTA_BIT3_CS1                     (1<<3)
#define FT_PORTA_BIT4_TSWCONN_MONITOR_END1    (0<<4)
#define FT_PORTA_BIT5_TSWCONN_MONITOR_END2    (0<<5)
#define FT_PORTA_BIT6_FTDI_PMIC_NRST          (1<<6)
#define FT_PORTA_BIT7_120PIN_TMPSNS_ALERT     (0<<7)

/* Default Port A configuration */
#define FT_PORTA_DIR_CONFIG                   (FT_PORTA_BIT0_SPICLK1 | FT_PORTA_BIT1_MOSI1 | \
                                               FT_PORTA_BIT2_MISO1 | FT_PORTA_BIT3_CS1 | \
                                               FT_PORTA_BIT4_TSWCONN_MONITOR_END1 | \
                                               FT_PORTA_BIT5_TSWCONN_MONITOR_END2 | \
                                               FT_PORTA_BIT7_120PIN_TMPSNS_ALERT )

/* FTDI BIT MODE settings */
#define FT_BITMODE_RESET		        0x00
#define FT_BITMODE_ASYNC_BITBANG		0x01
#define FT_BITMODE_MPSSE				0x02
#define FT_BITMODE_SYNC_BITBANG			0x04
#define FT_BITMODE_MCU_HOST				0x08
#define FT_BITMODE_FAST_SERIAL			0x10
#define FT_BITMODE_CBUS_BITBANG			0x20
#define FT_BITMODE_SYNC_FIFO			0x40

/* FTDI MPSSE Command */

/* FTDI MPSSE CPU MODE COMMAND */
#define FT_COMMAND_ERROR_BAD_COMMAND    	    0xFA

/* FTDI DATA TRANSFER COMMAND */
#define FT_COMMAND_DATA_BYTESOUT_PVE_MSB    	0x10
#define FT_COMMAND_DATA_BYTESOUT_NVE_MSB    	0x11
#define FT_COMMAND_DATA_BITSOUT_PVE_MSB	     	0x12
#define FT_COMMAND_DATA_BITSOUT_NVE_MSB	     	0x13
#define FT_COMMAND_DATA_BYTESIN_PVE_MSB 	    0x20
#define FT_COMMAND_DATA_BYTESIN_NVE_MSB	0x24
#define FT_COMMAND_DATA_BITSIN_PVE_MSB      	0x22
#define FT_COMMAND_DATA_BITSIN_NVE_MSB      	0x26
#define FT_COMMAND_DATA_BYTESINOUT_PNVE_MSB 0x31
#define FT_COMMAND_DATA_BYTESINOUT_NPVE_MSB 0x34
#define FT_COMMAND_DATA_BITSINOUT_PNVE_MSB  	0x33
#define FT_COMMAND_DATA_BITSINOUT_NPVE_MSB  	0x36
#define FT_COMMAND_DATA_BYTESOUT_PVE_LSB    	0x18
#define FT_COMMAND_DATA_BYTESOUT_NVE_LSB    	0x19
#define FT_COMMAND_DATA_BITSOUT_PVE_LSB     	0x1A
#define FT_COMMAND_DATA_BITSOUT_NVE_LSB	 	    0x1B
#define FT_COMMAND_DATA_BYTESIN_PVE_LSB 	    0x28
#define FT_COMMAND_DATA_BYTESIN_NVE_LSB     	0x2C
#define FT_COMMAND_DATA_BITSIN_PVE_LSB      	0x2A
#define FT_COMMAND_DATA_BITSIN_NVE_LSB      	0x2E
#define FT_COMMAND_DATA_BYTESINOUT_PNVE_LSB 0x39
#define FT_COMMAND_DATA_BYTESINOUT_NPVE_LSB 0x3C
#define FT_COMMAND_DATA_BITSINOUT_PNVE_LSB  	0x3B
#define FT_COMMAND_DATA_BITSINOUT_NPVE_LSB  	0x3E

/* FTDI DATA DIRECTION COMMAND */
#define FT_COMMAND_SET_DATABITS_LOW         	0x80
#define FT_COMMAND_SET_DATABITS_HIGH        	0x82
#define FT_COMMAND_READ_DATABITS_LOW        	0x81
#define FT_COMMAND_READ_DATABITS_HIGH       	0x83

/* FTDI LOOPBACK MODE COMMAND */
#define FT_COMMAND_TDI_TO_TDO_LOOPBACK      	0x84
#define FT_COMMAND_TDO_TO_TDI_LOOPBACK  	    0x85

/* FTDI CLOCK DIVISOR COMMAND */
#define FT_COMMAND_CLOCK_DIVISOR            	0x86

/* Read one byte from target device */
#define FT_COMMAND_CPUMODE_READ_SHORT_ADDR  0x90
#define FT_COMMAND_CPUMODE_READ_EXT_ADDR	    0x91
#define FT_COMMAND_CPUMODE_WRITE_SHORT_ADDR 0x92
#define FT_COMMAND_CPUMODE_WRITE_EXT_ADDR   	0x93

/* FTDI MPSSE HOST EMULATION MODES COMMAND */
#define FT_COMMAND_FLUSH_BUFFER             	0x87
#define FT_COMMAND_WAIT_IOHIGH              	0x88
#define FT_COMMAND_WAIT_IOLOW               	0x89

/* FTDI MPSSE CLOCK COMMAND */
#define FT_COMMAND_CLOCK_DIVIDE5_DISABLE    	0x8A
#define FT_COMMAND_CLOCK_DIVIDE5_ENABLE     	0x8B
#define FT_COMMAND_CLOCK_3PHASE_ENABLE      	0x8C
#define FT_COMMAND_CLOCK_3PHASE_DISABLE     	0x8D
#define FT_COMMAND_CLOCK_PULSE_ON           	0x8F
#define FT_COMMAND_CLOCK_CONT_IOHIGH        	0x94
#define FT_COMMAND_CLOCK_CONT_IOLOW         	0x95
#define FT_COMMAND_TURNON_ADAPTIVE_CLOCK    	0x96
#define FT_COMMAND_TURNOFF_ADAPTIVE_CLOCK		0x97
#define FT_COMMAND_CLOCK_PULSE_GPIO1_HIGH   	0x9C
#define FT_COMMAND_CLOCK_PULSE_GPIO1_LOW    	0x9D


/* FTDI Response Timeout */
#define FT_FTDI_RESPONSE_TIMEOUT                 (50)

/* SBL Commands */
#define  SBL_COMMAND_SYNC_MSG                   0xAB01
#define  SBL_COMMAND_SYNC_MSG_WAIT              0xAB02
#define  SBL_COMMAND_SYNC_MSG_RCVD              0xAB03
#define  SBL_COMMAND_SM_MSG1                    0xAB04
#define  SBL_COMMAND_SM_MSG1_RESP               0xAB05
#define  SBL_COMMAND_UPDATE_FLASH               0xAB06
#define  SBL_COMMAND_AUTO_BOOT                  0xAB07 
#define  SBL_COMMAND_UPDATING_FLASH             0xAB08
#define  SBL_COMMAND_DATA                       0xAB0A
#define  SBL_COMMAND_ACK                        0xAB0B
#define  SBL_COMMAND_CHECKSUM_ERR               0xAB0C
#define  SBL_COMMAND_PRINTF                     0xAB0D
#define  SBL_COMMAND_TERMINATE                  0xAB10  

#define SWAP_BYTES(x)           (( (x&0xff) << 8) | ((x >>8) & 0xff))




/**************************************************************************
 *************************** Local Data Structure ******************************
 **************************************************************************/


/* SPI test message format */
typedef struct spiTestMsg
{
	unsigned short msgId;
	unsigned short seqNo;
	unsigned short dataLen;
	unsigned short checksum;
	unsigned char data[SPI_TEST_MSGLEN - 8];
}spiTestMsg;

/**************************************************************************
 *************************** Global Definitions ****************************
 **************************************************************************/


/* Global test sequence number */
unsigned short   gTestMsgSeqNo = 0;

/* FTDI device id for SPI */
unsigned short   gSPIDevId = 0;

FT_HANDLE ftHandle = 0;

/**************************************************************************
 *************************** Local Function Prototype***************************
 **************************************************************************/
FT_STATUS Write(LPVOID, DWORD, LPDWORD);
FT_STATUS SetCSLevel(UCHAR);

void SBL_printf(spiTestMsg*    pTestMsgIn)
{
   char  spiOut[120];  
   int spiOutLength;
   
   memset((void *)&spiOut,0,120);
   spiOutLength=SWAP_BYTES(pTestMsgIn->dataLen);
   for(int i=0;i<spiOutLength;i++)
	spiOut[i]=pTestMsgIn->data[i];
  
   printf("\nIW6843: %s",&spiOut);
   fflush(stdout);
   

}


/**
 *  @b Description
 *  @n
 *      This function set CS signal level
 *
 *  @retval
 *      FT Command Status.
 */
FT_STATUS SetCSLevel(UCHAR level)
{
	UCHAR buffer[12];
	DWORD sendBytes = 0;

	/* Set Data bits LowByte */
	buffer[0] = FT_COMMAND_SET_DATABITS_LOW;
	/* Set CS low */
	if (level == 0)
		buffer[1] = 0xC0;
	else
		buffer[1] = 0xC8;

	/* Set Pin Direction */
	buffer[2] = FT_PORTA_DIR_CONFIG;

	/* Write to FTDI MPSSE */
	FT_Write(ftHandle,buffer,3,&sendBytes);

	if (sendBytes != 3)
		return FT_IO_ERROR;

        return FT_OK;
}

#if 0    //Test code to send 2 bytes then toggle CS
/**
 *  @b Description
 *  @n
 *      This function reads data from FTDI Spi interface
 *
 *   @param[in] lpvBuffer                   Buffer to store the read data
 *   @param[in] dwBytesToRead         Number of bytes to read
 *   @param[in] lpdwBytesRead          Number of bytes actually read

 *  @retval
 *      FT Command Status.
 */
FT_STATUS Read(LPVOID lpvBuffer, DWORD dwBytesToRead, LPDWORD lpdwBytesRead)
{
	UCHAR   buffer[SPI_TEST_MSGLEN + COMMAND_LENGTH];
	DWORD   inBytes = 0;
	DWORD   outBytes = 0;
	DWORD   inDataAvail = 0;
	DWORD   i;
	FT_STATUS       status = FT_OK;
	DWORD   timeoutCnt = 0;
	UCHAR*  pInbuff = (UCHAR *)lpvBuffer;
	DWORD bytesToSend = 0;
	UCHAR dataLen = 2;


	*lpdwBytesRead = 0;
	for (i = 0; i < dwBytesToRead; i += dataLen, bytesToSend=0)
	{

		buffer[bytesToSend++] = FT_COMMAND_SET_DATABITS_LOW;

		/* Set CS low */
  	        buffer[bytesToSend++] = 0xC0;

		/* Set Pin Direction */
		buffer[bytesToSend++] = FT_PORTA_DIR_CONFIG;

		/* Data in only, bytes, clock in on rising edge */
		buffer[bytesToSend++] = FT_COMMAND_DATA_BYTESIN_PVE_MSB;

		/* Write Length LSB */
		buffer[bytesToSend++] = dataLen - 1;
		/* Write Length MSB */
		buffer[bytesToSend++] = 0;

		buffer[bytesToSend++] = FT_COMMAND_SET_DATABITS_LOW;

		/* Set CS High */
		buffer[bytesToSend++] = 0xC8;

		/* Set Pin Direction */
		buffer[bytesToSend++] = FT_PORTA_DIR_CONFIG;

		/* Send Immediate */
		buffer[bytesToSend++] = FT_COMMAND_FLUSH_BUFFER;

    status = FT_Write(ftHandle, buffer, bytesToSend, &outBytes);
		if (FT_OK != status)
		{
			printf("Error, couldn't send command to MSPEE \n");
			break;
		}

		do
		{
			/* Check if data is received */
			status = FT_GetQueueStatus(ftHandle,(DWORD *)&inDataAvail);
			Sleep(1);
			timeoutCnt++;
		} while ((FT_OK == status) && (inDataAvail < dataLen) &&
			(timeoutCnt < FT_FTDI_RESPONSE_TIMEOUT));

		if (FT_OK != status)
		{
			printf("Error: SPI Read Wait Data Available Timeout\n");
		}

		/* Read SPI data */
    status = FT_Read(ftHandle, (LPVOID)(pInbuff + i), dataLen, &inBytes);
		*lpdwBytesRead = *lpdwBytesRead + inBytes;
		//Sleep(1);

	}

	return status;
}
#else //Test code that expecting CS hold on XWR1xxx so that it can tranfer 128 bytes with 1 CS togging
/**
 *  @b Description
 *  @n
 *      This function reads data from FTDI Spi interface
 *
 *   @param[in] lpvBuffer             Buffer to store the read data
 *   @param[in] dwBytesToRead         Number of bytes to read
 *   @param[in] lpdwBytesRead         Number of bytes actually read

 *  @retval
 *      FT Command Status.
 */
FT_STATUS Read(LPVOID lpvBuffer, DWORD dwBytesToRead, LPDWORD lpdwBytesRead)
{
	UCHAR   buffer[SPI_TEST_MSGLEN + COMMAND_LENGTH];
	DWORD   outBytes = 0;
	DWORD   inDataAvail = 0;
	DWORD   i;
	FT_STATUS  status = FT_OK;
	DWORD   timeoutCnt = 0;
	DWORD bytesToSend = 0;


	*lpdwBytesRead = 0;

	buffer[bytesToSend++] = FT_COMMAND_SET_DATABITS_LOW;

	/* Set CS low */
        buffer[bytesToSend++] = 0xC0;

	/* Set Pin Direction */
	buffer[bytesToSend++] = FT_PORTA_DIR_CONFIG;

	/* Data in only, bytes, clock in on rising edge */
	buffer[bytesToSend++] = FT_COMMAND_DATA_BYTESIN_PVE_MSB;

	/* Write Length LSB */
	buffer[bytesToSend++] = dwBytesToRead -1 ; //dataLen - 1;
	/* Write Length MSB */
	buffer[bytesToSend++] = 0;

	buffer[bytesToSend++] = FT_COMMAND_SET_DATABITS_LOW;

	/* Set CS High */
	buffer[bytesToSend++] = 0xC8;

	/* Set Pin Direction */
	buffer[bytesToSend++] = FT_PORTA_DIR_CONFIG;

	/* Send Immediate */
	buffer[bytesToSend++] = FT_COMMAND_FLUSH_BUFFER;

        status = FT_Write(ftHandle,buffer,bytesToSend,&outBytes);
	if (FT_OK != status)
	{
		printf("Error, couldn't send command to MSPEE \n");
	}
        else
	{
		do
		{
			/* Check if data is received */
			status = FT_GetQueueStatus(ftHandle,(DWORD *)&inDataAvail);
			timeoutCnt++;
		} while ((FT_OK == status) && (inDataAvail < dwBytesToRead) &&
			(timeoutCnt < FT_FTDI_RESPONSE_TIMEOUT));

		if (FT_OK != status)
		{
			printf("Error: SPI Read Wait Data Available Timeout\n");
		}

		/* Read SPI data */
               status = FT_Read(ftHandle,lpvBuffer,dwBytesToRead,lpdwBytesRead);
               
   	}
Exit:
	return status;
}

#endif

#if 0 //Test code to send 2 bytes then toggle CS
/**
 *  @b Description
 *  @n
 *      This function writes data to FTDI Spi interface
 *
 *   @param[in] lpvBuffer                   Buffer to store the read data
 *   @param[in] dwBuffSize                Number of bytes to write
 *   @param[in] lpdwBytes                  Number of bytes actually write

 *  @retval
 *      FT Command Status.
 */
FT_STATUS Write(LPVOID lpvBuffer, DWORD dwBuffSize, LPDWORD lpdwBytes)
{
	UCHAR buffer[SPI_TEST_MSGLEN + COMMAND_LENGTH];
	UCHAR *pInBuff = (UCHAR *)lpvBuffer;
	UCHAR i=0, bytesToSend=0, j;
	UCHAR dataLen = 2;

	for (i = 0; i < dwBuffSize; i += dataLen, bytesToSend = 0)
	{
		buffer[bytesToSend++] = FT_COMMAND_SET_DATABITS_LOW;

		/* Set CS low */
  	        buffer[bytesToSend++] = 0xC0;

		/* Set Pin Direction */
		buffer[bytesToSend++] = FT_PORTA_DIR_CONFIG;

		/* write only in BYTES, data out on falling edge (-ve) */
		buffer[bytesToSend++] = FT_COMMAND_DATA_BYTESOUT_NVE_MSB;

		/* Write Length MSB */
		buffer[bytesToSend++] = dataLen - 1;

		/* Write Length LSB */
		buffer[bytesToSend++] = 0;

		/* Fill in Data in buffer */
		for (j = 0; j < dataLen; j++)
		{
			buffer[bytesToSend++] = pInBuff[j + i];
		}

		buffer[bytesToSend++] = FT_COMMAND_SET_DATABITS_LOW;

		/* Set CS high */
		buffer[bytesToSend++] = 0xC8;

		/* Set Pin Direction */
		buffer[bytesToSend++] = FT_PORTA_DIR_CONFIG;

		/* Send Immediate */
		buffer[bytesToSend++] = FT_COMMAND_FLUSH_BUFFER;

		/* Write to FTDI MPSSE */
    FT_Write(ftHandle, buffer, bytesToSend, lpdwBytes);
		/* Delay between CS toggling for every 2 bytes. Limitation of MibSPI on XWR1xxx */
		//Sleep(1);
	}

	return 0;
}
#else  //Test code that expecting CS hold on XWR1xxx so that it can tranfer 128 bytes with 1 CS togging
/**
 *  @b Description
 *  @n
 *      This function writes data to FTDI Spi interface
 *
 *   @param[in] lpvBuffer          Buffer to store the read data
 *   @param[in] dwBuffSize         Number of bytes to write
 *   @param[in] lpdwBytes          Number of bytes actually write

 *  @retval
 *      FT Command Status.
 */
FT_STATUS Write(LPVOID lpvBuffer, DWORD dwBuffSize, LPDWORD lpdwBytes)
{
	UCHAR buffer[SPI_TEST_MSGLEN + COMMAND_LENGTH];
	UCHAR bytesToSend=0;

	buffer[bytesToSend++] = FT_COMMAND_SET_DATABITS_LOW;

	/* Set CS low */
    buffer[bytesToSend++] = 0xC0;

	/* Set Pin Direction */
	buffer[bytesToSend++] = FT_PORTA_DIR_CONFIG;

	/* write only in BYTES, data out on falling edge (-ve) */
	buffer[bytesToSend++] = FT_COMMAND_DATA_BYTESOUT_NVE_MSB;

	/* Write Length LSB */
	buffer[bytesToSend++] = (dwBuffSize -1) & 0xFF;
	/* Write Length MSB */
	buffer[bytesToSend++] = (dwBuffSize -1) >>8;

	/* Fill in Data in buffer */
	memcpy((void *)(&buffer[bytesToSend]), lpvBuffer, dwBuffSize);

	bytesToSend += dwBuffSize;
	buffer[bytesToSend++] = FT_COMMAND_SET_DATABITS_LOW;

	/* Set CS high */
	buffer[bytesToSend++] = 0xC8;

	/* Set Pin Direction */
	buffer[bytesToSend++] = FT_PORTA_DIR_CONFIG;

	/* Send Immediate */
	buffer[bytesToSend++] = FT_COMMAND_FLUSH_BUFFER;

	/* Write to FTDI MPSSE */
  FT_Write(ftHandle,buffer,bytesToSend, lpdwBytes);

	return 0;
}

#endif

/**
 *  @b Description
 *  @n
 *      This function lists availabe  FTDI  devices
 *
 *  @retval
 *      FT Command Status.
 */
void Search(int VID, int PID)
{
	//search for Descriptions or Serial Numbers depending on the current mode
	FT_STATUS ftStatus;
	DWORD numDevs;
	DWORD index;
	char *BufDescription[64]; // pointer to array of 16 pointers
	char *BufSerialNumber[64];
  FT_DEVICE_LIST_INFO_NODE *devInfo;

  //must be closed to perform the ListDevices() function
	FT_Close(ftHandle);


  ftStatus = FT_SetVIDPID(VID,PID);
  if (ftStatus != FT_OK){
    printf("Failure to set VIDPID\n");
  }

	ftStatus = FT_CreateDeviceInfoList(&numDevs);
	if (ftStatus == FT_OK)
	{
		// FT_ListDevices OK, show number of devices
		printf("%d device(s) attached:\n", (int)numDevs);
		/* List all devices that are attached */

    //allocate storage for device list
		if(numDevs > 0){
        devInfo = (FT_DEVICE_LIST_INFO_NODE*)malloc(sizeof(FT_DEVICE_LIST_INFO_NODE)*numDevs);
        ftStatus = FT_GetDeviceInfoList(devInfo,&numDevs);

    		if (ftStatus==FT_OK)
    		{
    			/* List all devices that are attached */
    			for (index = 0; index < numDevs; index++)
    			{
    				printf("Dev%d: %s - %s\n", index, devInfo[index].SerialNumber, devInfo[index].Description);
    			}
    		}
    		else
    		{
    			printf("Error: List attached device failed: %d\n", ftStatus);
    		}

    		//free ram to avoid memory leaks
    		free(devInfo);
    	}
    	else
    	{
    		// FT_ListDevices failed
    		printf("FT_ListDevices failed\n");
    	}
    }
}

static void transfter_delay(unsigned int delay)
{
    unsigned int i;

    for(i = 0; i< delay *1000; i++)
	{
	}
}


/**
 *  @b Description
 *  @n
 *      This function opens SPI device and initializes SPI device
 *
 *  @retval
 *      FT Command Status.
 */
void LoadSPIDevice(int VID, int PID)
{
	unsigned char  txbuf[SPI_TEST_MSGLEN], rxbuf[SPI_TEST_MSGLEN];
	DWORD          ret_bytes;
	unsigned int   devNo;
	unsigned char  txConfBuffer[16];
	unsigned char  rxConfBuffer[16];
	DWORD          inBytes;
	DWORD          outBytes;
	spiTestMsg*    pTestMsgOut;
	spiTestMsg*    pTestMsgIn;
        unsigned int   sblSM=1;
        char           input;  

	Search(VID,PID);

        FT_STATUS status = FT_SetVIDPID(VID,PID);

	//open the device
	status = FT_Open(gSPIDevId,&ftHandle);
	if (status != FT_OK)
	{
		printf("Error: XWR1xxx board is not responding\n");
	}
	else
	{
		printf("SPI device(%d) is opened, handle = %x\n", gSPIDevId, ftHandle);
		status = FT_ResetDevice(ftHandle);

		/* Enable SPI Device */
		status = FT_SetBitMode(ftHandle, 0,FT_BITMODE_RESET);

		if (status != FT_OK)
		{
			printf("MPSSE Init Error\n");
		}

		status = FT_SetTimeouts(ftHandle,3000, 3000);//extend timeout while board DLP-USB2 finishes reset

    if (status != FT_OK)
		{
			printf("Timeout Set Error\n");
		}
		status = FT_Purge(ftHandle, FT_PURGE_RX || FT_PURGE_TX);

    if (status != FT_OK)
		{
			printf("Purge Error\n");
		}
		/* Set Usb Parameters*/
		status = FT_SetUSBParameters(ftHandle,4096, 4096);

		/* Set Latency Timer */
		status = FT_SetLatencyTimer(ftHandle,16);
		if (status != FT_OK)
		{
			printf("SPI USB parameters config Error\n");
		}
		/* Enable SPI Device */
		status = FT_SetBitMode(ftHandle,0, FT_BITMODE_MPSSE);
		if (status != FT_OK)
		{
			printf("MPSSE Init Error\n");
		}

		/* Use 60MHz master clock (disable divide by 5) */
		txConfBuffer[0] = FT_COMMAND_CLOCK_DIVIDE5_DISABLE;
		/* Turn off adaptive clocking (may be needed for ARM) */
		txConfBuffer[1] = FT_COMMAND_TURNOFF_ADAPTIVE_CLOCK;
		/* Disable three-phase clocking */
		txConfBuffer[2] = FT_COMMAND_CLOCK_3PHASE_DISABLE;
		/* Set divider command */
		txConfBuffer[3] = FT_COMMAND_CLOCK_DIVISOR;
		/* Divider low value . 60Mhz/(((high * 256+low)+1)*2)) ,
		   0xE -> 2MHz SPI clock
		   0x5 -> 5MHz
		   0x4 -> 6MHz
		   0x2 -> 10MHz
		   0x1 -> 15MHz
		   0x0 -> 30MHz
		*/
		txConfBuffer[4] = 0x4;
		/* Divider high value */
		txConfBuffer[5] = 0x00;
		/* Configure initial state of pins command: */
		txConfBuffer[6] = FT_COMMAND_SET_DATABITS_LOW;
		/* initial value: */
		txConfBuffer[7] = 0xC8;
		/* Direction: */
		txConfBuffer[8] = FT_PORTA_DIR_CONFIG;

                status = FT_Write(ftHandle,txConfBuffer,9,&outBytes);
		if (status != FT_OK)
		{
			printf("MSPEE Config Error\n");
		}
		/* Clear Rx queue: */
		do
		{
			status = FT_GetQueueStatus(ftHandle,&inBytes);
			if (FT_OK != status)
			{
				printf("Communication time out!\n");
				break;
			}
			status = FT_Read(ftHandle, (LPVOID)&rxConfBuffer[0], inBytes, &outBytes);
		} while (inBytes > 0);

		//test for SYNC Message of XWR1xxx board
		pTestMsgOut = (spiTestMsg*)&txbuf[0];
		pTestMsgIn = (spiTestMsg*)&rxbuf[0];

		/* Send Sync wait message to the board */		
		pTestMsgOut->msgId = SBL_COMMAND_SYNC_MSG_WAIT;
		pTestMsgOut->seqNo = 0;
		pTestMsgOut->dataLen = 0;
		Write((LPVOID)pTestMsgOut, SPI_TEST_MSGLEN, &ret_bytes);

	
		while (1)
		{
			sleep(3);
			Read(pTestMsgIn, SPI_TEST_MSGLEN, &ret_bytes);
			sleep(3);
                        if(pTestMsgIn->msgId == SWAP_BYTES(SBL_COMMAND_PRINTF))
                        {
                           SBL_printf(pTestMsgIn);
                           continue;
                        } 

			if (pTestMsgIn->msgId != SWAP_BYTES(SBL_COMMAND_SYNC_MSG))
			{
#if DEBUG
 		                printf("Incorrect sync message %x\n", pTestMsgIn->msgId);
#endif
			        pTestMsgOut->msgId = SBL_COMMAND_SYNC_MSG_WAIT;
                 		pTestMsgOut->dataLen = 0;
                                transfter_delay(100);
				Write((LPVOID)pTestMsgOut, SPI_TEST_MSGLEN, &ret_bytes);
                                transfter_delay(100);
				continue;
			}
			/* Got the correct Sync message back */
#if DEBUG
                        printf("Sync message Recvd 0x%X\n", SWAP_BYTES(pTestMsgIn->msgId));
#endif
 			pTestMsgOut->msgId = SBL_COMMAND_SYNC_MSG_RCVD;
                 	pTestMsgOut->dataLen = 0;
			Write((LPVOID)pTestMsgOut, SPI_TEST_MSGLEN, &ret_bytes);
                        break;
		}
   
	}

	FT_SetTimeouts(ftHandle,300, 300);
}

void SBLLoadTest()
{

        unsigned char  txbuf[SPI_TEST_MSGLEN], rxbuf[SPI_TEST_MSGLEN];
	DWORD          ret_bytes;
	unsigned int   devNo;
	unsigned char  txConfBuffer[16];
	unsigned char  rxConfBuffer[16];
	DWORD          inBytes;
	DWORD          outBytes;
	spiTestMsg*    pTestMsgOut;
	spiTestMsg*    pTestMsgIn;
        unsigned int   sblSM=1;
        char           input;  
        unsigned char  inputFile[256];
        FILE           *pFile; 
        int            count=3;   
        
        

        /* Get in/Out message pointer */
	pTestMsgOut = (spiTestMsg*)&txbuf[0];
	pTestMsgIn = (spiTestMsg*)&rxbuf[0];

        /* SBL State Machine 1: Wait for SM command from host */
        while (sblSM == 1)
               {
		  Read(pTestMsgIn, SPI_TEST_MSGLEN, &ret_bytes);
                  if(pTestMsgIn->msgId == SWAP_BYTES(SBL_COMMAND_PRINTF))
                  {
                    SBL_printf(pTestMsgIn);
                    continue;
                  } 
		  if (pTestMsgIn->msgId != SWAP_BYTES(SBL_COMMAND_SM_MSG1))
		  {
			pTestMsgOut->msgId = SBL_COMMAND_SYNC_MSG_RCVD;
			pTestMsgOut->dataLen = 0;
		        Write((LPVOID)pTestMsgOut, SPI_TEST_MSGLEN, &ret_bytes);
                        continue;
		  }
		  /* User Input for flash Image */
                  printf ("Do you want to stop auto boot and Update Meta Image, Enter y or n: ");
                  scanf ("%c",(char*)&input);
                  if (input == 'y')
                  {
         		/* Update Flash Image */
                        pTestMsgOut->msgId = SBL_COMMAND_UPDATE_FLASH;
                 	pTestMsgOut->dataLen = 0;
			Write((LPVOID)pTestMsgOut, SPI_TEST_MSGLEN, &ret_bytes);
                        sblSM+=1;
                  }
                  else if (input == 'n')     
                  {
         		/* Auto boot Image */
                        pTestMsgOut->msgId = SBL_COMMAND_AUTO_BOOT;
                 	pTestMsgOut->dataLen = 0;
			Write((LPVOID)pTestMsgOut, SPI_TEST_MSGLEN, &ret_bytes);
                        sblSM=255;
                  }
                  else
                  {
                      printf("\n\nWrong Input\n");    
                      continue;
		  }	

			

		break;

              }
      
      /* SBL State Machine 2: Open the binary file and load */ 
       while (sblSM == 2)
            {
                  pTestMsgOut->msgId = SBL_COMMAND_UPDATE_FLASH;
                  pTestMsgOut->dataLen = 0;
		  Write((LPVOID)pTestMsgOut, SPI_TEST_MSGLEN, &ret_bytes);	        

                  Read(pTestMsgIn, SPI_TEST_MSGLEN, &ret_bytes);
                
		  if(pTestMsgIn->msgId == SWAP_BYTES(SBL_COMMAND_PRINTF))
                  {
                    SBL_printf(pTestMsgIn);
                    continue;
                  } 
		  if (pTestMsgIn->msgId == SWAP_BYTES(SBL_COMMAND_UPDATING_FLASH))
                  { 
                    printf ("Enter the image to load: ");
                    scanf ("%s",(char *)&inputFile);
                    pFile=fopen((char *)&inputFile, "rb");
                    if(pFile == NULL)
                    {
                       printf("Could not open file %s \n",inputFile); 
                       sblSM=255;
                       continue;    
                    }    

                    sblSM++;
                    break;
                  }  
            }       

   /* SBL State Machine 3: Transfer the binary image to the board */   
      while (sblSM == 3)
      {
             bool eof=false;
             unsigned char ch;
             int i; 

           do{
                unsigned int retry=0;   
                unsigned short checksum=0;           
                pTestMsgOut->msgId = SBL_COMMAND_DATA;
                pTestMsgOut->dataLen = SPI_TEST_MSGLEN-8;
                pTestMsgOut->seqNo =  gTestMsgSeqNo;
               
	 	 /* Transfer 120 bytes at a time and swap the bytes before sending 
     		Compute the checksum for validation      
		 */              
                for(i=0; i < (SPI_TEST_MSGLEN-8)/2; i++)
                {
                   
                   if(!feof(pFile))
                   {
                     fread(&ch,1,1,pFile);   
                     checksum+=ch;
                     pTestMsgOut->data[2*i+1] = ch;  
                   }  
                   else 
                   {  
#if DEBUG
                     printf("EOF reached: sending last packet\n");
#endif
                     eof = true;
                     pTestMsgOut->dataLen=2*i; 
                     break;    
                   }

                   if(!feof(pFile))
                   {
                     fread(&ch,1,1,pFile);   
                     checksum+=ch;
                     pTestMsgOut->data[2*i] = ch;  
                   }  
                   else 
                   {  
#if DEBUG
                     printf("EOF reached: sending last packet\n");
#endif
                     eof = true;
                     pTestMsgOut->dataLen=2*i+1; 
                     break;    
                   }    
                       
                }
                pTestMsgOut->checksum=checksum;
                Write((LPVOID)pTestMsgOut, SPI_TEST_MSGLEN, &ret_bytes);     
                printf("\rTransfering packet no %d",gTestMsgSeqNo);               
                fflush(stdout); 

 /* Wait for Ack from board before sending next packet */
                while(1)
                {

                  Read(pTestMsgIn, SPI_TEST_MSGLEN, &ret_bytes);
		  if(pTestMsgIn->msgId == SWAP_BYTES(SBL_COMMAND_PRINTF))
                  {
                    SBL_printf(pTestMsgIn);
                    continue;
                  } 

	          if (pTestMsgIn->msgId == SWAP_BYTES(SBL_COMMAND_ACK) &&  pTestMsgIn->seqNo == SWAP_BYTES(gTestMsgSeqNo))
                  {
#if DEBUG
                    printf("Packet received successfully\n");
#endif
                    gTestMsgSeqNo++;
                    break; 
                  }
                  else if (pTestMsgIn->msgId == SWAP_BYTES(SBL_COMMAND_CHECKSUM_ERR) &&  pTestMsgIn->seqNo == SWAP_BYTES(gTestMsgSeqNo))
                  {
#if DEBUG                    
                    printf("Packet retransmission requested\n");
#endif
                    Write((LPVOID)pTestMsgOut, SPI_TEST_MSGLEN, &ret_bytes);
                    retry++;
                    continue; 
                  }
                  else
                  {
                    Write((LPVOID)pTestMsgOut, SPI_TEST_MSGLEN, &ret_bytes);    
                    retry++;  
                  }
               }     

         }while(!eof);   

  /* Send Terminate Command */
         pTestMsgOut->msgId = SBL_COMMAND_TERMINATE;
         pTestMsgOut->dataLen = 0;
         pTestMsgOut->seqNo =  0;
         Write((LPVOID)pTestMsgOut, SPI_TEST_MSGLEN, &ret_bytes);

 /* Wait to receive Terminate Command */
         while(1)
         {
            pTestMsgOut->msgId = SBL_COMMAND_TERMINATE;
            pTestMsgOut->dataLen = 0;
            pTestMsgOut->seqNo =  0;
            Write((LPVOID)pTestMsgOut, SPI_TEST_MSGLEN, &ret_bytes);
            Read(pTestMsgIn, SPI_TEST_MSGLEN, &ret_bytes);
            if(pTestMsgIn->msgId == SWAP_BYTES(SBL_COMMAND_PRINTF))
            {
              SBL_printf(pTestMsgIn);
              continue;
            } 

            if(pTestMsgIn->msgId == SWAP_BYTES(SBL_COMMAND_TERMINATE))
            {
              sblSM=255;
              break;
            }  

         }
         

     }        
    /* SBL State Machine 255: Terminate the connection */
    while (sblSM == 255)
    {
	sleep(1);
	Write((LPVOID)pTestMsgOut, SPI_TEST_MSGLEN, &ret_bytes);
	sleep(1);
	Read(pTestMsgIn, SPI_TEST_MSGLEN, &ret_bytes);
	if (pTestMsgIn->msgId == SWAP_BYTES(SBL_COMMAND_TERMINATE) || count-- < 0)
	{ 
		printf("\n\nExiting Application\n");
		break;
	}  
    if(pTestMsgIn->msgId == SWAP_BYTES(SBL_COMMAND_PRINTF))
    {
         SBL_printf(pTestMsgIn);
    } 

  }         

}

/* Test Program for SPI test */
int main(int argc, char* argv[])
{
	if(argc != 3){
		printf("Invalid function call.\n Ex: ./spiTest (VID) (PID)\n");
	}
	else{
		printf("SPI Test Program started.\n");

		int VID = strtol(argv[1],NULL,16);
		int PID = strtol(argv[2],NULL,16);

		LoadSPIDevice(VID,PID);

		/* Sync is done, ready for data pattern test */
		SBLLoadTest();

		return 0;
	}
}
