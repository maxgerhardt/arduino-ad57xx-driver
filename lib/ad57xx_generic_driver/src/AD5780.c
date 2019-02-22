/***************************************************************************//**
 *   @file   AD5780.c
 *   @brief  Implementation of AD5780 Driver.
 *   @author Dan Nechita
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
#include "AD5780.h"			    // AD5780 definitions.
#include "Communication.h"		// Communication definitions.

/**************************************************************************//**
 * @brief Initializes the communication with the device.
 *
 * @return 1.
*******************************************************************************/
unsigned char AD5780_Init(void)
{  
    AD5780_CLR_OUT;    
    AD5780_LDAC_OUT;
    AD5780_RESET_OUT;
    AD5780_CLR_HIGH;
    AD5780_LDAC_HIGH;
    AD5780_RESET_HIGH;    
    SPI_Init(0, 1000000, 1, 0);
    
    return(1);    
}

/***************************************************************************//**
 * @brief The part is placed in normal mode or its output is clamped to the 
 *        ground.
 *
 * @param state - Enables/disables the output.
 *                Example: 1 - Enables output.
 *                         0 - Disables output.
 *
 * @return none.
*******************************************************************************/
void AD5780_EnableOutput(unsigned char state)
{
    unsigned long oldControl = 0;
    unsigned long newControl = 0;
    
    oldControl = AD5780_GetRegisterValue(AD5780_REG_CTRL, 3);
    oldControl = oldControl & ~(AD5780_CTRL_OPGND); // Clears OPGND bit.
    newControl = oldControl | AD5780_CTRL_OPGND * (!state);
    AD5780_SetRegisterValue(AD5780_REG_CTRL, newControl, 3);
}

/***************************************************************************//**
 * @brief Writes data into a register.
 *
 * @param registerAddress - Address of the register.
 * @param registerValue - Value of the register.
 * @param bytesNumber - Number of bytes to be written.
 *
 * @return none.
*******************************************************************************/
void AD5780_SetRegisterValue(unsigned char registerAddress,
                             unsigned long registerValue,
                             unsigned char bytesNumber)
{
    unsigned char writeCommand[4] = {0, 0, 0, 0};
    unsigned long spiWord         = 0;
    unsigned char* dataPointer    = (unsigned char*)&spiWord;
    unsigned char bytesNr         = bytesNumber;
    
    spiWord = AD5780_RW_(0) | 
              AD5780_ADDR_REG(registerAddress) |
              registerValue;
    writeCommand[0] = 0x01;
    while(bytesNr != 0)
    {
        writeCommand[bytesNr] = *dataPointer;
        dataPointer ++;
        bytesNr --;
    }    
    SPI_Write(AD5780_SLAVE_ID, writeCommand, bytesNumber);
}

/***************************************************************************//**
 * @brief Reads the value of a register.
 *
 * @param registerAddress - Address of the register.
 * @param bytesNumber - Number of bytes that will be read.
 *
 * @return dataRead - Value of the register.
*******************************************************************************/
unsigned long AD5780_GetRegisterValue(unsigned char registerAddress,
                                       unsigned char bytesNumber)
{
    unsigned char registerWord[4] = {0, 0, 0, 0}; 
    unsigned long dataRead        = 0x0;
    unsigned char i               = 0;
   
    registerWord[0] = 0x01;
    registerWord[1] = (AD5780_RW_(1) | AD5780_ADDR_REG(registerAddress)) >> 16;
    SPI_Write(AD5780_SLAVE_ID, registerWord, 3);
    registerWord[0] = 0x01;
    registerWord[1] = 0x00;
    registerWord[2] = 0x00;
    registerWord[3] = 0x00;
    SPI_Read(AD5780_SLAVE_ID, registerWord, bytesNumber);
    for(i = 0;i < bytesNumber;i ++) 
    {
        dataRead = (dataRead << 8) + registerWord[i];
    }
    
    return(dataRead);
}

/***************************************************************************//**
 * @brief Writes to the DAC register.
 *
 * @param value - The value to be written to DAC.
 *
 * @return none.
*******************************************************************************/
void AD5780_SetDacValue(unsigned long value)
{
    AD5780_LDAC_LOW;
    AD5780_SetRegisterValue(AD5780_REG_DAC, 
                            AD5780_DAC_DATA(value), 
                            3);
    AD5780_LDAC_HIGH;
}

/***************************************************************************//**
 * @brief Asserts RESET, CLR and LDAC in a software manner.
 *
 * @param instructionBit - A Software Control Register bit.
 *                         Example: AD5780_SOFT_CTRL_LDAC  - Load DAC
 *                                  AD5780_SOFT_CTRL_CLR   - Clear
 *                                  AD5780_SOFT_CTRL_RESET - Reset
 *
 * @return none.
*******************************************************************************/
void AD5780_SoftInstruction(unsigned char instructionBit)
{
    int i = 0;
    
    AD5780_SetRegisterValue(AD5780_CMD_WR_SOFT_CTRL, instructionBit, 3);
    // Waits for the instruction to take effect.
    for(i = 0; i < 100; i++)
    {
    	//may not be defined by your toolchain
    	//if not, try placing
    	//__asm__ __volatile__ ("nop\n\t");
    	//here.
        //__no_operation();
    	__asm__ __volatile__ ("nop\n\t");
    }
}

/***************************************************************************//**
 * @brief Writes to Control Register.
 *
 * @param setupWord - Is a 24-bit value that sets or clears the Control Register
 *                    bits.
 *                    Example: AD5780_CTRL_BIN2SC | AD5780_CTRL_SDODIS - sets
 *                             the DAC register to use offset binary coding and 
 *                             disables SDO pin(tristated).     
 *
 * @return none.
*******************************************************************************/
void AD5780_Setup(unsigned long setupWord)
{
    AD5780_SetRegisterValue(AD5780_REG_CTRL, setupWord, 3);
}
