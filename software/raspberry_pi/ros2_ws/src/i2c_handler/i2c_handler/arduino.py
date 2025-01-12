import time

REG_ADDR_ULTRASOUND_0 = 0x00
REG_ADDR_ULTRASOUND_1 = 0x01
REG_ADDR_ULTRASOUND_2 = 0x02

REG_ADDR_ENCODER_A_BYTE_3 = 0x03
REG_ADDR_ENCODER_A_BYTE_2 = 0x04
REG_ADDR_ENCODER_A_BYTE_1 = 0x05
REG_ADDR_ENCODER_A_BYTE_0 = 0x06

REG_ADDR_ENCODER_B_BYTE_3 = 0x07
REG_ADDR_ENCODER_B_BYTE_2 = 0x08
REG_ADDR_ENCODER_B_BYTE_1 = 0x09
REG_ADDR_ENCODER_B_BYTE_0 = 0x0A

REG_ADDR_CURRENT_PROBE_BYTE_1 = 0x0B
REG_ADDR_CURRENT_PROBE_BYTE_0 = 0x0C

REG_ADDR_LINE_SENSOR_0_BYTE_1 = 0x0D
REG_ADDR_LINE_SENSOR_0_BYTE_0 = 0x0E

REG_ADDR_LINE_SENSOR_1_BYTE_1 = 0x0F
REG_ADDR_LINE_SENSOR_1_BYTE_0 = 0x10

REG_ADDR_MOTOR_0_SPEED = 0x11
REG_ADDR_MOTOR_1_SPEED = 0x12

REG_ADDR_RESERVED = 0x13

DEVICE_I2C_ADDRESS = 0x50


class Arduino:

    def __init__(self, i2c):
        self.i2c = i2c

    def ultrasound_0(self):
        return self._read_byte(REG_ADDR_ULTRASOUND_0)

    def ultrasound_1(self):
        return self._read_byte(REG_ADDR_ULTRASOUND_1)

    def ultrasound_2(self):
        return self._read_byte(REG_ADDR_ULTRASOUND_2)

    def encoder_a(self):
        return ((self._read_byte(REG_ADDR_ENCODER_A_BYTE_3) << 24) +
                (self._read_byte(REG_ADDR_ENCODER_A_BYTE_2) << 16) +
                (self._read_byte(REG_ADDR_ENCODER_A_BYTE_1) << 8) +
                self._read_byte(REG_ADDR_ENCODER_A_BYTE_0))

    def encoder_b(self):
        return ((self._read_byte(REG_ADDR_ENCODER_B_BYTE_3) << 24) +
                (self._read_byte(REG_ADDR_ENCODER_B_BYTE_2) << 16) +
                (self._read_byte(REG_ADDR_ENCODER_B_BYTE_1) << 8) +
                self._read_byte(REG_ADDR_ENCODER_B_BYTE_0))

    def current(self):
        return ((self._read_byte(REG_ADDR_CURRENT_PROBE_BYTE_1) << 8) +
                self._read_byte(REG_ADDR_CURRENT_PROBE_BYTE_0))

    def line_sensor_0(self):
        return ((self._read_byte(REG_ADDR_LINE_SENSOR_0_BYTE_1) << 8) +
                self._read_byte(REG_ADDR_LINE_SENSOR_0_BYTE_0))

    def line_sensor_1(self):
        return ((self._read_byte(REG_ADDR_LINE_SENSOR_1_BYTE_1) << 8) +
                self._read_byte(REG_ADDR_LINE_SENSOR_1_BYTE_0))

    def set_motor_0(self, value):
        self._write_byte(REG_ADDR_MOTOR_0_SPEED, value)

    def set_motor_1(self, value):
        self._write_byte(REG_ADDR_MOTOR_1_SPEED, value)

    def sleep_us(self, microseconds):
        end_time = time.perf_counter() + (microseconds / 1_000_000)
        while time.perf_counter() < end_time:
            pass

    def _read_byte(self, register_addr):
        self.i2c.writeto(DEVICE_I2C_ADDRESS, bytes([register_addr]))
        result = bytearray(1)
        self.i2c.readfrom_into(DEVICE_I2C_ADDRESS, result)
        time.sleep(0.0001)
        self.sleep_us(20)
        return result[0]

    def _write_byte(self, register_addr, byte_value):
        data_to_write = bytes([register_addr, byte_value])
        self.i2c.writeto(DEVICE_I2C_ADDRESS, data_to_write)
        time.sleep(0.0001)
        self.sleep_us(20)
