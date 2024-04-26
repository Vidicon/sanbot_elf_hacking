# sanbot elf hacking - Matthijs Branch
Reverse engineering and hacking the Sanbot Elf


## STM32 programmer connector - CON 1

* Programming cable runs internally down to battery cover plate for easy access.
* PCB connector located at the North-West corner of the STM32.
* Most left pin of connector defined as #1.

|   Pin 1  |   Pin 2  |   Pin 3  |   Pin 4 |   Pin 5 |
|----------|----------|----------|----------|--------|
|   White  |   Black  |   Black  |   Black  |   Black |
|   <unknown>      |   3V3    |   GND  |  SWCLK  | SWDIO  |

* Confirmed: STM Cube Programmer works with these pins connected.
* To flash the STM32, write protection must be removed from within the **"STM32CubeProgrammer"** software. 
* Set "Read Out Protection" (RDP) to level "AA" did the trick (I think).

## STM32 code - Matthijs
RGBLeds:
* LeftArm : Red / Green / Blue / White
* RightArm : Red / Green / Blue / White
* Base  : Red / Green / Blue / White
* Blinking : Slow (2 Hz), Fast (3 Hz), Very Fast (5 Hz)
* Selftest implemented.

## Battery pack
* BMS on battery is a very advanced chip.
* Note: 1 very new (2023) battery does not work on multiple robots.


