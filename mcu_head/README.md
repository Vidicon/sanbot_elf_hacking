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
