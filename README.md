# Arduino AD57xx Driver

An example on how to use [Analog Device's AD57XX
driver](https://wiki.analog.com/resources/tools-software/uc-drivers/renesas/ad5780#driver_description) in
Arduino code. 

The example implements the generation of a triangle wave. 

# Setup

For this driver to work you have to give it the correct pins in the PinMgtLayer.cpp file of the library. There,
change the pins for LDAC, CLR, RESET and SYNC (SPI slave-select) to any free GPIO pins you have. Also, connect
SDO (=MISO) SDIN (=MOSI) and SCLK to your board's SPI bus.

Check the connections on J3 on the eval board for pin mappings
([datasheet](https://www.analog.com/media/en/technical-documentation/user-guides/UG-256.pdf)). 

You must also connect common DGND (digital GND) between the eval and the Arduino board.

# Speed

For GPIO manipulation, `pinMode()` and `digitalWrite()` is used. These routines are slow. Consider exchanging
the PinMgtLayer.cpp implementation for a version using register-manipulation according to your board's
microcontroller.

For the Arduino Uno, use https://github.com/NicksonYap/digitalWriteFast


# Good luck

None of this code is tested for functionality since I don't have the board. Please tell me if you encounter any
errors. It does compile though and should make sense :).
