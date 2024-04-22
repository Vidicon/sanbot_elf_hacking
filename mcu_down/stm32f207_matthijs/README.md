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

Confirmed: STM Cube Programmer detects STM.

Write protected? 
Read protected yes!

## STM32 code - Matthijs


## Battery pack
BMS on battery is chip 


