#include <Arduino.h>
#include <AD57xx.h>

void setup() {
	Serial.begin(115200);
	Serial.println("FIRMWARE STARTUP");

	/* same setup as in the RL78G13 driver example's Main.c */
	if(AD57XX_Init(AD5780))
	{
		Serial.println("AD57XX OK");
	}
	else
	{
		Serial.println("AD57XX Err");
	}

	/* Resets the device to its power-on state. */
	AD57XX_SoftInstruction(AD57XX_SOFT_CTRL_RESET);

	/* Enables the DAC output. */
	AD57XX_EnableOutput(1);

	/* The DAC register is set to use offset binary coding. */
	AD57XX_Setup(AD57XX_CTRL_BIN2SC);

	/* Sets the value to which the DAC output is set when CLEAR is enabled. */
	AD57XX_SetClearCode(0x20000);

	/* Performs a soft CLEAR operation. */
	AD57XX_SoftInstruction(AD57XX_SOFT_CTRL_CLR);

    /* Reads and displays the internal registers. */
     /* Read DAC. */
    long result = AD57XX_GetRegisterValue(AD57XX_REG_DAC);
    result = (result & 0xFFFFC) >> 2;
    Serial.print("DAC REG: ");
    Serial.println(result, HEX);

    /* Read Control. */
    result = AD57XX_GetRegisterValue(AD57XX_REG_CTRL);
    result = (result & 0x3E);  // Only Bits 5 through 1 are holding information.
    Serial.print("CTRL REG: ");
    Serial.println(result, HEX);

    /* Read ClearCode. */
    result = AD57XX_GetRegisterValue(AD57XX_REG_CLR_CODE);
    result = (result & 0xFFFFC) >> 2;
    Serial.print("CLEAR CODE: ");
    Serial.println(result, HEX);
}

long dacVal  = 0;
long step    = 0;
long maxCode = 0x3FFFF;    // 18 bits
long minCode = 0;

void loop() {
	/* generate a triangle signal. */
	if(dacVal >= maxCode - step)
	{
		step = -250;
	}
	else if(dacVal <= minCode - step)
	{
		step = 250;
	}
	dacVal += step;
	AD57XX_SetDacValue(dacVal);
	//50 microseconds is a very short amount of time
	//if pinMode() and digitalWrite() is used in the pin management layer,
	//these will take so long that 50uS is small in comparison..
	delayMicroseconds(50);
}
