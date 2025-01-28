import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Empty
from std_msgs.msg import String
from std_msgs.msg import UInt8MultiArray
from std_msgs.msg import UInt16MultiArray
from std_msgs.msg import UInt32MultiArray
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray

from sensor_msgs.msg import FluidPressure
from sensor_msgs.msg import Temperature
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField

from geometry_msgs.msg import Quaternion

import busio
import board
from adafruit_bme280 import basic, advanced as adafruit_bme280
from adafruit_mpu6050 import MPU6050
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import adafruit_ds3231
from RPLCD.i2c import CharLCD

from .arduino import *

ENABLE_ARDUINO = True
ENABLE_BME280 = False
ENABLE_IMU = True
ENABLE_MAG = False
ENABLE_ADC = False
ENABLE_RTC = False
ENABLE_LCD = False
VERBOSE = False



class I2cHandlerNode(Node):
    def __init__(self):
        super().__init__('i2c_handler')

        self.frame = "/bpc_prp_robot"
        self.i2c = busio.I2C(board.SCL, board.SDA, frequency=100000)

        self.motor_watchdog = 0
        self.motor_watchdog_timer = self.create_timer(0.1, self.on_motor_watchdog)

        if ENABLE_ARDUINO:
            self.arduino = Arduino(self.i2c)
            self.ultrasounds_publisher = self.create_publisher(UInt8MultiArray, '/bpc_prp_robot/ultrasounds', 0)
            self.line_sensors_publisher = self.create_publisher(UInt16MultiArray, '/bpc_prp_robot/line_sensors', 0)
            self.current_probes_publisher = self.create_publisher(UInt16MultiArray, '/bpc_prp_robot/current_probes', 0)
            self.encoders_publishers = self.create_publisher(UInt32MultiArray, '/bpc_prp_robot/encoders', 0)
            self.motor_speeds_subscriber = self.create_subscription(UInt8MultiArray, '/bpc_prp_robot/set_motor_speeds',
                                                                    self.on_set_motor_speeds, 0)
            self.arduino_slow_timer = self.create_timer(0.2, self.on_arduino_slow)
            self.arduino_fast_timer = self.create_timer(0.01, self.on_arduino_fast)

        if ENABLE_BME280:
            self.bme280 = adafruit_bme280.Adafruit_BME280_I2C(self.i2c, 118)
            self.bme280.sea_level_pressure = 1013.25

            self.bme280.mode = adafruit_bme280.MODE_NORMAL
            self.bme280.standby_period = adafruit_bme280.STANDBY_TC_0_5
            self.bme280.iir_filter = adafruit_bme280.IIR_FILTER_X16
            self.bme280.overscan_pressure = adafruit_bme280.OVERSCAN_X1
            self.bme280.overscan_humidity = adafruit_bme280.OVERSCAN_X1
            self.bme280.overscan_temperature = adafruit_bme280.OVERSCAN_X1

            self.press_publisher = self.create_publisher(FluidPressure, '/bpc_prp_robot/air_press', 0)
            self.temp_publisher = self.create_publisher(Temperature, '/bpc_prp_robot/temp', 0)

            self.bme_280_timer = self.create_timer(0.1, self.on_bme280)

        if ENABLE_IMU:
            self.mpu = MPU6050(self.i2c)
            self.imu_publisher = self.create_publisher(Imu, '/bpc_prp_robot/imu', 0)
            self.imu_timer = self.create_timer(0.01, self.on_imu)

        if ENABLE_MAG:
            self.mag_publisher = self.create_publisher(MagneticField, '/bpc_prp_robot/mag', 0)
            self.mag_timer = self.create_timer(0.1, self.on_mag)

        if ENABLE_IMU or ENABLE_MAG:
            pass

        if ENABLE_ADC:
            self.ads = ADS.ADS1115(self.i2c)
            self.ads_channel0 = AnalogIn(self.ads, ADS.P0)
            self.ads_channel1 = AnalogIn(self.ads, ADS.P1)
            self.ads_channel2 = AnalogIn(self.ads, ADS.P2)
            self.ads_channel3 = AnalogIn(self.ads, ADS.P3)
            self.adc_publisher = self.create_publisher(Float32MultiArray, '/bpc_prp_robot/adc', 0)
            self.ads_timer = self.create_timer(0.1, self.on_ads)

        if ENABLE_RTC:
            self.rtc = adafruit_ds3231.DS3231(self.i2c)
            self.time_publisher = self.create_publisher(Int32MultiArray, '/bpc_prp_robot/time', 0)
            self.time_subscriber = self.create_subscription(Int32MultiArray, '/bpc_prp_robot/set_time',
                                                            self.on_set_time, 0)
            self.ds3231 = self.create_timer(1, self.on_ds3231)

        if ENABLE_LCD:
            self.lcd = CharLCD(i2c_expander='PCF8574', address=0x27, port=1, cols=20, rows=4, dotsize=8)
            self.lcd_text_subscriber = self.create_subscription(String, '/bpc_prp_robot/set_lcd_text',
                                                                self.on_set_lcd_text, 0)
            self.lcd_cursor_subscriber = self.create_subscription(UInt8MultiArray, '/bpc_prp_robot/set_lcd_cursor',
                                                                  self.on_set_lcd_cursor, 0)
            self.lcd_clear = self.create_subscription(Empty, '/bpc_prp_robot/set_lcd_clear', self.on_set_lcd_clear, 0)

        print("Init done")

    def on_arduino_slow(self):
        us_msg = UInt8MultiArray()
        us_msg.data.append(self.arduino.ultrasound_0())
        us_msg.data.append(self.arduino.ultrasound_1())
        us_msg.data.append(self.arduino.ultrasound_2())
        self.ultrasounds_publisher.publish(us_msg)

        current_msg = UInt16MultiArray()
        current_msg.data.append(self.arduino.current())
        self.current_probes_publisher.publish(current_msg)

    def on_arduino_fast(self):
        line_sensors_msg = UInt16MultiArray()
        ls_0 = self.arduino.line_sensor_0()
        ls_1 = self.arduino.line_sensor_1()
        line_sensors_msg.data.append(ls_0)
        line_sensors_msg.data.append(ls_1)
        self.line_sensors_publisher.publish(line_sensors_msg)

        en_a = self.arduino.encoder_a()
        en_b = self.arduino.encoder_b()
        encoder_msg = UInt32MultiArray()
        encoder_msg.data.append(en_a)
        encoder_msg.data.append(en_b)
        self.encoders_publishers.publish(encoder_msg)

    def on_imu(self):
        try:
            acc = self.mpu.acceleration
            gyro = self.mpu.gyro
            imu_msg = Imu()
            imu_msg.header.frame_id = self.frame
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.orientation = Quaternion()
            imu_msg.linear_acceleration.x = acc[0]
            imu_msg.linear_acceleration.y = acc[1]
            imu_msg.linear_acceleration.z = acc[2]
            imu_msg.angular_velocity.x = gyro[0]
            imu_msg.angular_velocity.y = gyro[1]
            imu_msg.angular_velocity.z = gyro[2]
            self.imu_publisher.publish(imu_msg)
            if VERBOSE:
                print('Imu:', imu_msg)
        except ValueError as e:
            print(f"Unable to communicate with the MPU6050. {e}")
        except (IOError, OSError) as e:
            print(f"Hardware communication error: {e}")
        except Exception as e:
            print(f"Unexpected error: {e}")


    def on_mag(self):
        try:
            mag_msg = MagneticField()
            mag_msg.header.frame_id = self.frame
            mag_msg.header.stamp = self.get_clock().now().to_msg()
            mag_msg.magnetic_field.x = 0.0
            mag_msg.magnetic_field.y = 0.0
            mag_msg.magnetic_field.z = 0.0
            self.mag_publisher.publish(mag_msg)
            if VERBOSE:
                print('Mag:', mag_msg)
	        
        except ValueError as e:
            print(f"Unable to communicate with the Magnetometer. {e}")
        except (IOError, OSError) as e:
            print(f"Hardware communication error: {e}")
        except Exception as e:
            print(f"Unexpected error: {e}")

    def on_bme280(self):
        try:
            temperature = self.bme280.temperature
            relative_humidity = self.bme280.relative_humidity
            pressure = self.bme280.pressure
            altitude = self.bme280.altitude

            press_msg = FluidPressure()
            press_msg.header.frame_id = self.frame
            press_msg.header.stamp = self.get_clock().now().to_msg()
            press_msg.fluid_pressure = pressure
            self.press_publisher.publish(press_msg)

            temp_msg = Temperature()
            temp_msg.header.frame_id = self.frame
            temp_msg.header.stamp = self.get_clock().now().to_msg()
            temp_msg.temperature = temperature
            self.temp_publisher.publish(temp_msg)

            if VERBOSE:
                print('Bme280:', temperature, relative_humidity, pressure, altitude)
                
        except ValueError as e:
            print(f"Unable to communicate with the BME280. {e}")
        except (IOError, OSError) as e:
            print(f"Hardware communication error: {e}")
        except Exception as e:
            print(f"Unexpected error: {e}")

    def on_ads(self):
        try:
            v = [self.ads_channel0.voltage, self.ads_channel0.voltage, self.ads_channel0.voltage, self.ads_channel0.voltage]
            volt_msg = Float32MultiArray()
            volt_msg.data.extend(v)
            self.adc_publisher.publish(volt_msg)

            if VERBOSE:
                print('Adc:', v)

        except ValueError as e:
            print(f"Unable to communicate with the ADC. {e}")
        except (IOError, OSError) as e:
            print(f"Hardware communication error: {e}")
        except Exception as e:
            print(f"Unexpected error: {e}")

    def on_ds3231(self):
        try:
            (Y, M, D, h, m, s, ms, us, ns) = self.rtc.datetime
            time_msg = Int32MultiArray()
            time_msg.data.extend([Y, M, D, h, m, s, ms, us, ns])
            self.time_publisher.publish(time_msg)

            if VERBOSE:
                print("time:", (Y, M, D, h, m, s, ms, us, ns))

        except ValueError as e:
            print(f"Unable to communicate with the DS3231. {e}")
        except (IOError, OSError) as e:
            print(f"Hardware communication error: {e}")
        except Exception as e:
            print(f"Unexpected error: {e}")

    def on_set_time(self, msg: Int32MultiArray):
        try:
            if len(msg.data) == 9:
                self.rtc.datetime = time.struct_time(
                    (msg.data[0], msg.data[1], msg.data[2],
                     msg.data[3], msg.data[4], msg.data[5],
                     msg.data[6], msg.data[7], msg.data[8]))
                     
        except ValueError as e:
            print(f"Unable to communicate with the DS3231. {e}")
        except (IOError, OSError) as e:
            print(f"Hardware communication error: {e}")
        except Exception as e:
            print(f"Unexpected error: {e}")

    def on_set_lcd_text(self, msg: String):
        try:
            self.lcd.write_string(msg.data)
            if VERBOSE:
                print('lcd:', msg)
                     
        except ValueError as e:
            print(f"Unable to communicate with the LCD text. {e}")
        except (IOError, OSError) as e:
            print(f"Hardware communication error: {e}")
        except Exception as e:
            print(f"Unexpected error: {e}")

    def on_set_lcd_cursor(self, msg: UInt8MultiArray):
        try:
            if len(msg.data) == 2:
                self.lcd.cursor_pos = (msg.data[0], msg.data[1])
            if VERBOSE:
                print('lcd cursor:', msg)
                     
        except ValueError as e:
            print(f"Unable to communicate with the LCD cursor. {e}")
        except (IOError, OSError) as e:
            print(f"Hardware communication error: {e}")
        except Exception as e:
            print(f"Unexpected error: {e}")

    def on_set_lcd_clear(self, msg: UInt8MultiArray):
        try:
            self.lcd.clear()
            if VERBOSE:
                print('lcd clear')
                     
        except ValueError as e:
            print(f"Unable to communicate with the LCD clear. {e}")
        except (IOError, OSError) as e:
            print(f"Hardware communication error: {e}")
        except Exception as e:
            print(f"Unexpected error: {e}")

    def on_motor_watchdog(self):
        try:
            if self.motor_watchdog < 10: # period 0.1s -> 1s timeout
                self.motor_watchdog += 1
            else:
                self.arduino.set_motor_0(127)
                self.arduino.set_motor_1(127)
                
        except Exception as e:
            print(f"Unexpected error: {e}")


    def on_set_motor_speeds(self, msg: UInt8MultiArray):
        try:
            self.motor_watchdog = 0
            if len(msg.data) == 2:
                self.arduino.set_motor_0(msg.data[0])
                self.arduino.set_motor_1(msg.data[1])
            if VERBOSE:
                print(f"motor speed set: {msg.data[0]}, {msg.data[1]}") 
                
        except Exception as e:
            print(f"Unexpected error: {e}")

def main(args=None):
    rclpy.init(args=args)

    i2c_handler_node = I2cHandlerNode()

    try:
        rclpy.spin(i2c_handler_node)
    except KeyboardInterrupt:
        pass
    finally:
        i2c_handler_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
