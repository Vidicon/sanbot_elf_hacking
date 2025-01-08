# MCU_Down hardware
![MCU_Head_connectors](../docs/MCU_Head_connectors.png)

[!CAUTION]
connector is 180deg rotated compart to body_pcb! 

---
## Connectors:
### Power in (J1)
|   Logic_12V  |   Logic_12V  |   Motor_12V  |   Motor_12V  |
|----------|----------|----------|----------|
|   GND  |   GND  |   GND  |   GND  |

---
### USB Enable
The UBS port/hub needs a low enable signal to work.
|   Item |   Pin  |   LOW  |
|----------|----------|----------|
|   USB port enable  |   PG1  |   LOW  |

---
### RGB LED (J5 & J6)

|  Item        |      Red  |   Green  |   Blue  |
|----------|----------|----------|----------|
| Head Left  |   PE8  |   PE10  |   PE12  |
| Head Right |   PD13  |   PD14  |   PD15  |

Notes Matthijs:
* IO pins are correct.
* Pull pins **DOWN** to turn LED **ON**
* All pins **HIGH** = **WHITE**

---

### HEAD Pan & Tilt motors (J13 & J14)

|  Item     | Encoder Ch1  |   Encoder Ch2  |   PWM | DIR | ENABLE|
|---------- |-------------|----------|----------|---------|----------|
| Head PAN  | TIM3 CH1-PC6  | TIM3 CH2-PC7    |  TIM8 CH4 - PC9   | PD12 | PE15|
| Head TILT | TIM1 CH1-PE9    |TIM1 CH2-PE11  |TIM8 CH 3 - PC8|  PD11   | PE14| 


|  Item     | Limit Pos/Up  |  Limit Neg/Down  |
|---------- |-------------|----------|
| Head PAN  |  PF2 |  PF3  | 
| Head TILT | PF4 | PF5 |

---
### Read Flash

SPI1, 8 bits, MSB, 18 MBits/s
CPOL = Low
CPHA = 1 Edge
CRC = disable
NSS signal = software

---
### Write Eyes

**Left eye**
SPI2, Transmit only, 8 bits, MSB, 9.0 MBits/s
CPOL = Low
CPHA = 1 Edge
CRC = disable
NSS signal = software

SPI2_SCK = PB13
SPI2_MOSI = PB15

Chip select (CS) = PB14 = OLED_L_CS
DC pin (DC) = PD6 = OLED_DC
RESET pin (Reset) = PD5 = OLED_RESET

***Right eye***
SPI3

Chip select (CS) = PA15 = OLED_R_CS

  OLED_HandleTypeDef left_eye;
  left_eye.hspi = &hspi2;
  left_eye.cs = toGPIO(OLED_L_CS_GPIO_Port, OLED_L_CS_Pin);
  left_eye.dc = toGPIO(OLED_DC_GPIO_Port, OLED_DC_Pin);
  left_eye.reset = toGPIO(OLED_RESET_GPIO_Port, OLED_RESET_Pin);

  OLED_HandleTypeDef right_eye;
  right_eye.hspi = &hspi3;
  right_eye.cs = toGPIO(OLED_R_CS_GPIO_Port, OLED_R_CS_Pin);
  right_eye.dc = toGPIO(OLED_DC_GPIO_Port, OLED_DC_Pin);
  right_eye.reset = toGPIO(OLED_RESET_GPIO_Port, OLED_RESET_Pin);