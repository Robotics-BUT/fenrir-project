# Arduino Firmware

The firmware is available here ![Firmware](/software/arduino_nano/main/main.ino).

This Arduino Nano program works as a motor controller that talks with a main computer (for example a Raspberry Pi) using I2C communication. The Nano is an I2C slave – it waits for messages from the master and answers when needed. When the master sends data, the Nano updates target speeds or control settings. When the master asks for data, the Nano sends back measurements like encoder counts or wheel speeds. This communication happens using interrupt callbacks, so it does not stop the main program loop. In this way, the Nano can control motors and communicate at the same time.

Arduino handles the ultrasound sensors, line sensors, possible extension for current sensor and controls the motors.

The robot wheels have encoders that measure how fast they rotate. Each encoder gives electrical pulses when the wheel moves. The Nano uses interrupts to count these pulses very quickly without missing them. The number of pulses in a short time tells how fast the wheel is turning. In the main loop, the program reads these counts and calculates the wheel speed. These measured speeds are compared with the target speeds received from the master to see if the motors are running correctly or need adjustment.

To keep the wheels turning at the right speed, the Nano uses a PID controller. In every cycle of the main loop, the program calculates the difference between target and actual speed and adjusts the motor power with PWM signals. The direction pins control forward or backward rotation, and the PWM value sets how strong the motor runs. The main loop repeats this many times per second, keeping the movement smooth and stable. Meanwhile, I2C and encoder interrupts work in the background, so the system always knows the latest speed and can react quickly to commands from the master.

## Memory Space

<div align = center>

| Byte | Value                      | Length | R/W |
|------|----------------------------|--------|-----|
| 0    | ultrasound 0 distance [cm] | 1B     | R   |
| 1    | ultrasound 1 distance [cm] | 1B     | R   |
| 2    | ultrasound 2 distance [cm] | 1B     | R   |
| 3    | encoder A [-]              | 4B     | R   |
| 7    | encoder B [-]              | 4B     | R   |
| 11   | current probe [-]          | 2B     | R   |
| 13   | line sensor 0 [-]          | 2B     | R   |
| 15   | line sensor 1 [-]          | 2B     | R   |
| 17   | motor 0 speed [-]          | 2B     | W   |
| 18   | motor 1 speed [cm]         | 1B     | W   |
| 19   | reserverd                  | 1B     | -   |

</div>
