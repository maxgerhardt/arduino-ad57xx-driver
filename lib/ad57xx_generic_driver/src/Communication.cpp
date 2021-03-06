/***************************************************************************//**
 *   @file   Communication.c
 *   @brief  Implementation of Communication Driver.
 *   @author DBogdan (dragos.bogdan@analog.com)
********************************************************************************
 * Copyright 2012(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
********************************************************************************
 *   SVN Revision: 781
*******************************************************************************/

/******************************************************************************/
/* Include Files                                                              */
/******************************************************************************/
#include "Communication.h"
//#include <Wire.h> //I2C
#include <SPI.h> //SPI
#include <PinMgtLayer.h>

/***************************************************************************//**
 * @brief Initializes the I2C communication peripheral.
 *
 * @param clockFreq - I2C clock frequency (Hz).
 *                    Example: 100000 - I2C clock frequency is 100 kHz.
 * @return status - Result of the initialization procedure.
 *                  Example: 1 - if initialization was successful;
 *                           0 - if initialization was unsuccessful.
*******************************************************************************/
unsigned char I2C_Init(unsigned long clockFreq)
{
    // Add your code here.
	//Don't. This function is never called.
	return 0;
}

/***************************************************************************//**
 * @brief Writes data to a slave device.
 *
 * @param slaveAddress - Adress of the slave device.
 * @param dataBuffer - Pointer to a buffer storing the transmission data.
 * @param bytesNumber - Number of bytes to write.
 * @param stopBit - Stop condition control.
 *                  Example: 0 - A stop condition will not be sent;
 *                           1 - A stop condition will be sent.
 *
 * @return status - Number of written bytes.
*******************************************************************************/
unsigned char I2C_Write(unsigned char slaveAddress,
                        unsigned char* dataBuffer,
                        unsigned char bytesNumber,
                        unsigned char stopBit)
{
    // Add your code here.
	//Don't. This function is never called.
	return 0;
}

/***************************************************************************//**
 * @brief Reads data from a slave device.
 *
 * @param slaveAddress - Adress of the slave device.
 * @param dataBuffer - Pointer to a buffer that will store the received data.
 * @param bytesNumber - Number of bytes to read.
 * @param stopBit - Stop condition control.
 *                  Example: 0 - A stop condition will not be sent;
 *                           1 - A stop condition will be sent.
 *
 * @return status - Number of read bytes.
*******************************************************************************/
unsigned char I2C_Read(unsigned char slaveAddress,
                       unsigned char* dataBuffer,
                       unsigned char bytesNumber,
                       unsigned char stopBit)
{
    // Add your code here.
	//Don't. This function is never called.
	return 0;
}

/***************************************************************************//**
 * @brief Initializes the SPI communication peripheral.
 *
 * @param lsbFirst - Transfer format (0 or 1).
 *                   Example: 0x0 - MSB first.
 *                            0x1 - LSB first.
 * @param clockFreq - SPI clock frequency (Hz).
 *                    Example: 1000 - SPI clock frequency is 1 kHz.
 * @param clockPol - SPI clock polarity (0 or 1).
 *                   Example: 0x0 - Idle state for clock is a low level; active
 *                                  state is a high level;
 *	                      0x1 - Idle state for clock is a high level; active
 *                                  state is a low level.
 * @param clockEdg - SPI clock edge (0 or 1).
 *                   Example: 0x0 - Serial output data changes on transition
 *                                  from idle clock state to active clock state;
 *                            0x1 - Serial output data changes on transition
 *                                  from active clock state to idle clock state.
 *
 * @return status - Result of the initialization procedure.
 *                  Example: 1 - if initialization was successful;
 *                           0 - if initialization was unsuccessful.
*******************************************************************************/
SPISettings spiSettings;
unsigned char SPI_Init(unsigned char lsbFirst,
                       unsigned long clockFreq,
                       unsigned char clockPol,
                       unsigned char clockEdg)
{
    // Add your code here.
	int spiMode = 0;
	//decode modes according to https://www.arduino.cc/en/Reference/SPI
	if(clockPol == 0) {
		spiMode = clockEdg ? SPI_MODE1 : SPI_MODE0;
	} else {
		spiMode = clockEdg ? SPI_MODE3 : SPI_MODE2;
	}
	int bitOrder = lsbFirst ? LSBFIRST : MSBFIRST;
	//modify SPI settings object used in transactions
	spiSettings = SPISettings(clockFreq, bitOrder, spiMode);

	//set slave-select pins to output
	PinMgt_PinOutput(PIN_ID_AD57XX_SLAVE_SELECT);

	//this is used on the fly
	return 1;
}

/***************************************************************************//**
 * @brief Reads data from SPI.
 *
 * @param slaveDeviceId - The ID of the selected slave device.
 * @param data - Data represents the write buffer as an input parameter and the
 *               read buffer as an output parameter.
 * @param bytesNumber - Number of bytes to read.
 *
 * @return Number of read bytes.
*******************************************************************************/
unsigned char SPI_Read(unsigned char slaveDeviceId,
                       unsigned char* data,
                       unsigned char bytesNumber)
{
    // Add your code here.
	SPI.beginTransaction(spiSettings);
	//select SPI device by pulling SS low
	PinMgt_PinWrite(PIN_ID_AD57XX_SLAVE_SELECT, PINMGR_OUTPUT_LOW);
	//transfer data, read data is saved in "data" again
	SPI.transfer(data, bytesNumber);
	SPI.endTransaction();
	//deselect device again
	PinMgt_PinWrite(PIN_ID_AD57XX_SLAVE_SELECT, PINMGR_OUTPUT_HIGH);
	return bytesNumber;
}

/***************************************************************************//**
 * @brief Writes data to SPI.
 *
 * @param slaveDeviceId - The ID of the selected slave device.
 * @param data - Data represents the write buffer.
 * @param bytesNumber - Number of bytes to write.
 *
 * @return Number of written bytes.
*******************************************************************************/
unsigned char SPI_Write(unsigned char slaveDeviceId,
                        unsigned char* data,
                        unsigned char bytesNumber)
{
    // Add your code here.
	//write is the same thing as read except for that
	//the contents of the write buffer isn't changed.
	//we could also call SPI_Read() but that changes buffer contents,
	//which actually wouldn't matter because the calling code
	//always restores the buffer content.
	SPI.beginTransaction(spiSettings);
	PinMgt_PinWrite(PIN_ID_AD57XX_SLAVE_SELECT, PINMGR_OUTPUT_LOW);
	for(unsigned char i = 0; i < bytesNumber; i++) {
		SPI.transfer(data[i]);
		//ignore returned value
	}
	SPI.endTransaction();
	PinMgt_PinWrite(PIN_ID_AD57XX_SLAVE_SELECT, PINMGR_OUTPUT_HIGH);
	return bytesNumber;
}
