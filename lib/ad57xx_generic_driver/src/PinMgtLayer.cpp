#include <PinMgtLayer.h>
#include <Arduino.h>

/* since PIN_ID_GPIO1 .. 3 is defined as 0..2 we can
   use them in a lookup table.
   Write your pins here.
  */
static const int pinMapping[] = {
		7, //GPIO1 (LDAC pin) is D7
		8, //GPIO2 (CLR pin) is D8
		9, //GPIO3 (RESET pin) is D9
		6  //slave select (SYNC pin) is D6
};

/* implements setting pins to output and writing values to pins.
 * Done through the slow pinMode and digitalWrite() APIs.
 * The original code uses direct register manipulation.
 * Implement this in the best way possible according to your MCU,
 * since this impacts overall readout speed.
 * */

void PinMgt_PinOutput(int pin_id) {
	//sanity check
	if(pin_id < PIN_ID_GPIO1 || pin_id > PIN_ID_AD57XX_SLAVE_SELECT)
		return;
	pinMode(pinMapping[pin_id], OUTPUT);
}

void PinMgt_PinWrite(int pin_id, int value) {
	//sanity check
	if(pin_id < PIN_ID_GPIO1 || pin_id > PIN_ID_AD57XX_SLAVE_SELECT)
		return;
	//value macros are equivalent to Arduino's LOW/HIGH macros
	digitalWrite(pinMapping[pin_id], value);
}
