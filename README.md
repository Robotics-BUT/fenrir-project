# Fenrir Project - Universal Robotic Platform

<img src="./media/fenrir_logo.png" width="200">

The Fenrir Project is an open source of a multi purpose education robot dedicated for wide range of applications and usage in high-schools and universities.

The robot is based on Raspberry Pi 4 and Arduino Nano. The Raspberry Pi provides computational power, ROS2 support, WiFi connection and USB support. The Arduino Nano extends the PRi IOs and handles peripherals.

The development is currently still under development

## Robot Overview

### Robot visualization 

![Robot](./media/render.png)

### Overview schematic

![Overview schematic](./media/scheme-overview.png)

### Power supply schematic

![Power supply schematic](./media/scheme-power-supply.png)

### Wiring Overview


![Wiring Overview](./media/scheme-wiring.png)



**Note**: The sensors numbering goes from left to right
 - Ultrasound 1 - left, Ultrasound 2 - center, Ultrasound 3 - right
 - Line sensor 1 - left, Line sensor 2 - right
 - Motor 1 - left, Motor 2 - right
 - Encoder 1 - left, Encoder 2 - right

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


## Equipment

| Role                   | Component           | URL                                                                                                                                                                                                                                                                                             |
|------------------------|---------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Main Computer          | RPi                 | [link](https://cz.mouser.com/ProductDetail/Raspberry-Pi/SC01959?qs=T%252BzbugeAwjhNCW2QVhlotw%3D%3D)                                                                                                                                                                                            |
| Helper Microcontroller | Arduino Nano        | [link](https://www.laskakit.cz/arduino-nano-r3--atmega328p-klon--pripajene-piny/)                                                                                                                                                                                                               |
| 2x Battery             | Powerbank 10000 mAh | [link](https://www.alza.cz/alzapower-parade-power-delivery?dq=7177723)                                                                                                                                                                                                                          |
| Camera                 | RPi Camera v2       | [link](https://rpishop.cz/mipi-kamerove-moduly/329-raspberry-pi-kamera-modul-v2.html?utm_source=google&utm_medium=cpc&utm_campaign=CZ-PMax-Raspberry%20Pi&utm_id=19691368073&gad_source=1&gclid=Cj0KCQiAr7C6BhDRARIsAOUKifiiX4p72F4J8UeeA5uyoK2IDEEryKLRjE0e5gLhURTCTTf2cGW-ZpQaAr76EALw_wcB)   |
| GPS                    | NEO-7M module       | [link](https://dratek.cz/arduino/1733-gps-satelitni-urceni-polohy-neo-7m-modul.html?_gl=1*16f2vtr*_up*MQ..*_gs*MQ..&gclid=Cj0KCQiAr7C6BhDRARIsAOUKifhd7u9T5IjiCyc4w0n-WqehlzG5F2pNwJ4JP5M_eQDHW-daU_NkSKYaAn-_EALw_wcB)                                                                         |
| 2D LiDAR               | RPLiDAR A1M8        | [link](https://rpishop.cz/lidary/1631-rplidar-a1m8-360stupnovy-laserovy-scanner-kit-dosah-12m.html)                                                                                                                                                                                             |
| LCD                    | HD44780 20x4 I2C    | [link](https://dratek.cz/arduino/1421-eses-i2c-20x4-display-pro-jednodeskove-pocitace.html?_gl=1*16f2vtr*_up*MQ..*_gs*MQ..&gclid=Cj0KCQiAr7C6BhDRARIsAOUKifhd7u9T5IjiCyc4w0n-WqehlzG5F2pNwJ4JP5M_eQDHW-daU_NkSKYaAn-_EALw_wcB)                                                                  |
| 3x Ultrasound          | SRF-05              | [link](https://dratek.cz/arduino/1735-meric-vzdalenosti-ultrazvukovy-5pin-hy-srf05-pro-arduino.html?gad_source=1&gclid=Cj0KCQiAr7C6BhDRARIsAOUKifhd7u9T5IjiCyc4w0n-WqehlzG5F2pNwJ4JP5M_eQDHW-daU_NkSKYaAn-_EALw_wcB)                                                                            |
| 2x Line Sensor         | TCRT5000 module     | [link](https://www.laskakit.cz/arduino-infracerveny-senzor-sledovani-cary-s-lm393/)                                                                                                                                                                                                             |
| H Bridge               | L298N module        | [link](https://www.laskakit.cz/h-mustek-pro-krokovy-motor-l298n--dualni-motorovy-modul/)                                                                                                                                                                                                        |
| Motor with Encoder     | DG01D‐E             | [link](https://cz.mouser.com/ProductDetail/SparkFun/ROB-16413?qs=vmHwEFxEFR%2F20T1Ah9zr5w%3D%3D)                                                                                                                                                                                                |
| IMU                    | MPU6050             | [link](https://pajenicko.cz/gyroskop-akcelerometr-gy-521-s-mpu6050-i2c)                                                                                                                                                                                                                         |
| Barometer              | BME280              | [link](https://pajenicko.cz/senzor-na-mereni-teploty-vlhkosti-tlaku-bme280)                                                                                                                                                                                                                     |
| Real Time Clock        | DS3231              | [link](https://www.laskakit.cz/arduino-rtc-hodiny-realneho-casu-ds3231-at24c32/?utm_source=google&utm_medium=cpc&utm_campaign=1_PMax_%5BCZ%5D_Top_tROAS_540&utm_id=18587021915&gad_source=1&gclid=Cj0KCQiAr7C6BhDRARIsAOUKifg6UUgWdBpultpbW-joaJkbae34LBj8l6PJlVsHSvLgqnOoyetU0BsaAva-EALw_wcB) |
| ADC                    | ADS1115             | [link](https://pajenicko.cz/ad-prevodnik-ads1115-s-programovatelnym-zesilenim-16bit-4-kanaly-i2c-rozhrani)                                                                                                                                                                                      |
| UWB (not yet)          | DWM1000             | [link](https://www.laskakit.cz/decawave-dwm1000-lokalizacni-modul/)                                                                                                                                                                                                                             |
| DC/DC Step Up          | XL6009              | [link](https://www.laskakit.cz/step-up-boost-menic-s-xl6009--cervena/)                                                                                                                                                                                                                          |
| Power Delivery         | USB-C PD module     | [link](https://www.laskakit.cz/usb-c-pd-qc-prepinac-napajeciho-napeti/?utm_source=google&utm_medium=cpc&utm_campaign=1_PMax_%5BCZ%5D_Rest_tROAS_540&utm_id=20484705754&gad_source=1&gclid=Cj0KCQiAr7C6BhDRARIsAOUKifjczsmkcfvSqKLF3lqhT2G3EaCcH9kkrkxwS225di7C_nkNrmd77JoaAueFEALw_wcB)         |
| Switch                 |                     | [link](https://pajenicko.cz/kulaty-kolebkovy-spinac-se-zelenou-led-12v-20a)                                                                                                                                                                                                                     |

## Bill Of Material



## Authors

Adam Ligocki 

