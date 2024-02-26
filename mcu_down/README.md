# MCU_Down hardware
![MCU_Down_connectors](../docs/MCU_Down_connectors.png)


## Connectors:

### Power in (J1)

|   GND  |   GND  |   GND  |   GND  |
|----------|----------|----------|----------|
|   Logic_12V  |   Logic_12V  |   Motor_12V  |   Motor_12V  |

---

### Motors (J8 J9 J10 J14 J15)

| Pin 1 | Pin 2 | Pin 3 | Pin 4 | Pin 5 | Pin 6 | Pin 7 |
|----------|----------|----------|----------|----------|----------|----------|
|   encoder_A  |   Enable  |   PWM  |   encoder_B  |   Dir  |   GND  |   12v  |

 - PWM is at 10Khz 3.3v
 - Enable is active low 3.3v
 - DIR pull down to reverse motor direction 3.3v

---

### RGB LED (J51 J52 J53)

| Pin 1 | Pin 2 | Pin 3 | Pin 4 |
|----------|----------|----------|----------|
|   GND  |   Red  |   Green  |   Blue  |

---

### Touch Sensors (J33 J34 J36 J37 J39 J42)

| Pin 1 | Pin 2 |
|----------|----------|
|   GND  |   Sense  |

---

### Pir Sensors (J35 J44)

| Pin 1 | Pin 2 | Pin 3 |
|----------|----------|----------|
|   signal  |   GND  |   5v  |

---

### Distence sensor 4x (J26)

| Pin 1 | Pin 2 | Pin 3 | Pin 4 | Pin 5 | Pin 6 | Pin 7 | Pin 8 | Pin 9 | Pin 10 | Pin 11 | Pin 12 | Pin 13 | Pin 14 |
|----------|----------|----------|----------|----------|----------|----------|----------|----------|----------|----------|----------|----------|----------|
|   3.3v  |   CLK  |   GND  |   Data  |   CS1  |   CS2  |   CS3  |   CS4  |   NC  |   NC  |   NC  |   NC  |   NC  |   NC  |

---

### Distence sensor 1x (J21 J24 J28)

| Pin 1 | Pin 2 | Pin 3 | Pin 4 | Pin 5 | Pin 6 |
|----------|----------|----------|----------|----------|----------|
|   3.3v  |   CLK  |    3.3v   |   CS  |   GND  |   Data  |

 - CLK of J21 J24 J28 are connected in parrelel with a 22R resistor per connector
 - Data of J21 J24 J28 are connected in parrelel with a 22R resistor per connector
 - CLK and Data have 2K2 pullups to 3.3v

---

### IR Docking sensor 1x (J47 J49)

| Pin 1 | Pin 2 | Pin 3 | Pin 4 | Pin 5 |
|----------|----------|----------|----------|----------|
|   ??1  |   ??2  |    ??3   |   GND  |   3.3  |

