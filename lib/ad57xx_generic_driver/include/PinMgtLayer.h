#ifndef LIB_AD57XX_GENERIC_DRIVER_INCLUDE_PINMGTLAYER_H_
#define LIB_AD57XX_GENERIC_DRIVER_INCLUDE_PINMGTLAYER_H_

#ifdef __cplusplus
extern "C" {
#endif

/* IDs for pins to differentiate GPIO1..2 */
#define PIN_ID_GPIO1 0
#define PIN_ID_GPIO2 1
#define PIN_ID_GPIO3 2
//use this macro to refer to the SS pin called "SYNC"
#define PIN_ID_AD57XX_SLAVE_SELECT 3

/* HIGH/LOW macros without using Arduino macros */
#define PINMGR_OUTPUT_LOW	0
#define PINMGR_OUTPUT_HIGH	1

// put slave select pin here
#define PIN_ID_AD57XX_SLAVE_SELECT 3

/**
 * sets the given pin id into output mode
 */
void PinMgt_PinOutput(int pin_id);

/*
 * Writes high or low to the pin
 */
void PinMgt_PinWrite(int pin_id, int value);

#ifdef __cplusplus
}
#endif

#endif /* LIB_AD57XX_GENERIC_DRIVER_INCLUDE_PINMGTLAYER_H_ */
