# Wiring

<p align="center">
  <img src="https://raw.githubusercontent.com/Robotics-BUT/fenrir-project/refs/heads/main/media/scheme-wiring.png" alt="Wiring Overview" width="500">
</p>
<p align="center"><em>Figure 1. Wiring overview of the Fenrir robot.</em></p>

------------------------------------------------------------------------------

**Note**: Sensor and motor numbering follows a left-to-right convention:
- **Ultrasounds 1** - left, Ultrasound 2 - center, Ultrasound 3 - right
- **Line sensors:** 1 - left, Line sensor 2 - right
- **Motors:** 1 - left, Motor 2 - right
- **Encoders:** 1 - left, Encoder 2 - right

---------------------------------------------------------------------------------

<div align="center">
  
| **Module A**       | **Pin**  | **Module B**        | **Pin**        | **Type**              | **Length** | **Note** |
|--------------------|----------|---------------------|----------------|-----------------------|------------|----------|
| Battery 1          | -        | PD Module           | -              | USB-C - USB-C         | 15cm       | -        |
| Battery 2          | -        | Raspberry Pi        | -              | USB-C - USB-C         | 15cm       | -        |
| Raspberry Pi       | -        | Camera              | -              | CSI Cable             | 10cm       | -        |
| Raspberry Pi       | 3v3      | GPS                 | VCC            | wire (blue)           | 10cm       | 3V3      |
| Raspberry Pi       | GND      | GPS                 | GND            | wire (black)          | 10cm       | GND      |
| Raspberry Pi       | Rx       | GPS                 | TxD            | wire (purple)         | 10cm       | UART     |
| Raspberry Pi       | 5V       | I2C Board           | 5V             | wire (red)            | 8cm        | 5V       |
| Raspberry Pi       | GND      | I2C Board           | GND            | wire (black)          | 8cm        | GND      |
| Raspberry Pi       | SCL      | I2C Board           | SCL            | wire (yellow)         | 8cm        | SCL      |
| Raspberry Pi       | SDA      | I2C Board           | SDA            | wire (green)          | 8cm        | SDA      |
| Raspberry Pi       | -        | LIDAR               | -              | USB-A - microUSB      | 10cm       | -        |
| I2C Board          | 5V       | LCD                 | VCC            | wire (red)            | 8cm        | 5V       |
| I2C Board          | GND      | LCD                 | GND            | wire (black)          | 8cm        | GND      |
| I2C Board          | SCL      | LCD                 | SCL            | wire (yellow)         | 8cm        | SCL      |
| I2C Board          | SDA      | LCD                 | SDA            | wire (green)          | 8cm        | SDA      |
| I2C Board          | 5V       | Arduino             | VCC            | wire (red)            | 15cm       | 5V       |
| I2C Board          | GND      | Arduino             | GND            | wire (black)          | 15cm       | GND      |
| I2C Board          | SCL      | Arduino             | SCL            | wire (yellow)         | 15cm       | SCL      |
| I2C Board          | SDA      | Arduino             | SDA            | wire (green)          | 15cm       | SDA      |
| Arduino            | US1      | Ultrasound 1        | Vcc            | wire (red)            | 10cm       | 5V       |
| Arduino            | US1      | Ultrasound 1        | GND            | wire (black)          | 10cm       | GND      |
| Arduino            | US1      | Ultrasound 1        | Echo/Trig      | wire (blue)           | 10cm       | signal   |
| Arduino            | US2      | Ultrasound 2        | Vcc            | wire (red)            | 10cm       | 5V       |
| Arduino            | US2      | Ultrasound 2        | GND            | wire (black)          | 10cm       | GND      |
| Arduino            | US2      | Ultrasound 2        | Echo/Trig      | wire (blue)           | 10cm       | signal   |
| Arduino            | US3      | Ultrasound 3        | Vcc            | wire (red)            | 15cm       | 5V       |
| Arduino            | US3      | Ultrasound 3        | GND            | wire (black)          | 15cm       | GND      |
| Arduino            | US3      | Ultrasound 3        | Echo/Trig      | wire (blue)           | 15cm       | signal   |
| Arduino            | Line1    | Line Sens. 1        | VCC            | wire (red)            | 15cm       | 5V       |
| Arduino            | Line1    | Line Sens. 1        | GND            | wire (black)          | 15cm       | GND      |
| Arduino            | Line1    | Line Sens. 1        | A0             | wire (blue)           | 15cm       | signal   |
| Arduino            | Line2    | Line Sens. 2        | VCC            | wire (red)            | 15cm       | 5V       |
| Arduino            | Line2    | Line Sens. 2        | GND            | wire (black)          | 15cm       | GND      |
| Arduino            | Line2    | Line Sens. 2        | A0             | wire (blue)           | 15cm       | signal   |
| Arduino            | 5V       | Encoder 1           | -              | wire (yellow)         | 20cm       | 5V       |
| Arduino            | GND      | Encoder 1           | -              | wire (brown)          | 20cm       | GND      |
| Arduino            | EN1A     | Encoder 1           | -              | wire (red)            | 20cm       | A        |
| Arduino            | EN1B     | Encoder 1           | -              | wire (orange)         | 20cm       | B        |
| Arduino            | 5V       | Encoder 2           | -              | wire (yellow)         | 20cm       | 5V       |
| Arduino            | GND      | Encoder 2           | -              | wire (brown)          | 20cm       | GND      |
| Arduino            | EN2A     | Encoder 2           | -              | wire (red)            | 20cm       | A        |
| Arduino            | EN2B     | Encoder 2           | -              | wire (orange)         | 20cm       | B        |
| Arduino            | GND      | PD Module           | OUT-           | wire (black)          | -          | -        |
| Arduino            | M1A      | H-Bridge            | IN3            | wire (yellow)         | 20cm       | PWM A+   |
| Arduino            | M1B      | H-Bridge            | IN4            | wire (green)          | 20cm       | PWM A-   |
| Arduino            | M2A      | H-Bridge            | IN1            | wire (yellow)         | 20cm       | PWM B+   |
| Arduino            | M2B      | H-Bridge            | IN2            | wire (green)          | 20cm       | PWM B-   |
| PD Module          | +        | H-Bridge            | +12V           | wire (orange)         | 15cm       | 12V      |
| PD Module          | -        | H-Bridge            | GND            | wire (black)          | 15cm       | GND      |
| H-Bridge           | OUT1+    | Motor 1             | -              | wire (green)          | 20cm       | +        |
| H-Bridge           | OUT1-    | Motor 1             | -              | wire (blue)           | 20cm       | -        |
| H-Bridge           | OUT2+    | Motor 2             | -              | wire (green)          | 20cm       | +        |
| H-Bridge           | OUT2-    | Motor 2             | -              | wire (blue)           | 20cm       | -        |

</div>

