from dataclasses import dataclass
from spidev import SpiDev
from typing import List

import RPi.GPIO as gpio
import math

@dataclass
class Bmp388CalibrationData:
    TemperatureFactor1: float
    TemperatureFactor2: float
    TemperatureFactor3: float
    PressureFactor1: float
    PressureFactor2: float
    PressureFactor3: float
    PressureFactor4: float
    PressureFactor5: float
    PressureFactor6: float
    PressureFactor7: float
    PressureFactor8: float
    PressureFactor9: float
    PressureFactor10: float
    PressureFactor11: float
    MeasuredTemp: float = 0

READ_COMMAND = 0x80 # 0b10000000
WRITE_COMMAND = 0x00 # 0b00000000
CHIP_SELECT_PIN = 29

ERROR_MAP = {
    0x00000001: 'Fatal error.',
    0x00000010: 'Command execution failed. Cleared on read.',
    0x00000100: 'Sensor configuration error detected. Cleared on read.',
}

STATUS_MAP = {
    0x00010000: 'Command decoder is ready to accept a new command.',
    0x00100000: 'Data ready for pressure.',
    0x01000000: 'Data ready for temperature sensor.',
}

class Bmp388:
    spi: SpiDev
    chip_select_pin: int
    calibration_data: Bmp388CalibrationData

    def __init__(self, bus: int, device: int, chip_select_pin: int):
        self.chip_select_pin = chip_select_pin
        gpio.setmode(gpio.BOARD)
        gpio.setwarnings(False)
        gpio.setup(chip_select_pin, gpio.OUT)

        self.spi = SpiDev()
        self.spi.open(bus, device)
        self.spi.mode = 3
        self.spi.max_speed_hz = 100000

        # Enable temp/pressure readings
        self.set_reg(0x1B, 0b00110011)

        self.calibration_data = self.get_calibration_data()

    def __del__(self):
        self.spi.close()

    def reg_data_to_bytearray(self, reg_data: List[int]) -> bytearray:
        data = bytearray(len(reg_data))
        for index, byte in enumerate(reg_data):
            data[index] = byte
        return data

    def get_reg(self, reg, length) -> List[int]:
        gpio.output(self.chip_select_pin, gpio.LOW)

        self.spi.xfer2([reg|READ_COMMAND])
        bytes = self.spi.readbytes(length + 1)

        assert(bytes[0] == 0b11111111) # dummy byte

        gpio.output(self.chip_select_pin, gpio.HIGH)

        return bytes[1:]

    def set_reg(self, reg, data) -> List[int]:
        gpio.output(self.chip_select_pin, gpio.LOW)

        self.spi.xfer2([reg|WRITE_COMMAND])
        self.spi.xfer2([data])

        gpio.output(self.chip_select_pin, gpio.HIGH)

    def get_chip_id(self) -> int:
        return self.get_reg(0x00, 1)[0]

    def get_err(self) -> List[str]:
        reg_data = self.get_reg(0x02, 1)[0]

        errors = []
        for mask, desc in ERROR_MAP.items():
            if reg_data & mask:
                errors.append(desc)

        return errors

    def get_status(self) -> List[str]:
        reg_data = self.get_reg(0x03, 1)[0]

        status = []
        for mask, desc in STATUS_MAP.items():
            if reg_data & mask:
                status.append(desc)

        return status

    def uint8_int(self, num: int) -> int:
        if(num > 127):
            num = num - 256
        return num

    def merge_bytes(self, bytes: bytearray | List[int], unsigned: bool = True) -> int:
        output = 0
        for index, byte in enumerate(bytes):
            data = (byte if unsigned else self.uint8_int(byte)) << (8*index)
            output |= data

        return output

    def get_sensor_time(self) -> int:
        reg_data = self.get_reg(0x0C, 3)
        data = self.reg_data_to_bytearray(reg_data)

        clock = self.merge_bytes(data)

        return round(clock/29000)

    def get_temp(self) -> float:
        reg_data = self.get_reg(0x07, 3)
        data = self.reg_data_to_bytearray(reg_data)

        raw_temp = self.merge_bytes(data)

        partial_data_1 = raw_temp - self.calibration_data.TemperatureFactor1
        partial_data_2 = partial_data_1 * self.calibration_data.TemperatureFactor2

        temp = partial_data_2 + (partial_data_1 * partial_data_1) * self.calibration_data.TemperatureFactor3
        self.calibration_data.MeasuredTemp = temp

        return round(temp, 2)

    def get_pressure(self) -> float:
        reg_data = self.get_reg(0x04, 3)
        data = self.reg_data_to_bytearray(reg_data)

        raw_pressure = self.merge_bytes(data)

        partial_data_1 = self.calibration_data.PressureFactor6 * self.calibration_data.MeasuredTemp
        partial_data_2 = self.calibration_data.PressureFactor7 * (self.calibration_data.MeasuredTemp * self.calibration_data.MeasuredTemp)
        partial_data_3 = self.calibration_data.PressureFactor8 * (self.calibration_data.MeasuredTemp * self.calibration_data.MeasuredTemp * self.calibration_data.MeasuredTemp)
        partial_out_1 = self.calibration_data.PressureFactor5 + partial_data_1 + partial_data_2 + partial_data_3

        partial_data_1 = self.calibration_data.PressureFactor2 * self.calibration_data.MeasuredTemp
        partial_data_2 = self.calibration_data.PressureFactor3 * (self.calibration_data.MeasuredTemp * self.calibration_data.MeasuredTemp)
        partial_data_3 = self.calibration_data.PressureFactor4 * (self.calibration_data.MeasuredTemp * self.calibration_data.MeasuredTemp * self.calibration_data.MeasuredTemp)
        partial_out2 = raw_pressure * (self.calibration_data.PressureFactor1 + partial_data_1 + partial_data_2 + partial_data_3)

        partial_data_1 = raw_pressure * raw_pressure
        partial_data_2 = self.calibration_data.PressureFactor9 + self.calibration_data.PressureFactor10 * self.calibration_data.MeasuredTemp
        partial_data_3 = partial_data_1 * partial_data_2
        partial_data_4 = partial_data_3 + (raw_pressure * raw_pressure * raw_pressure) * self.calibration_data.PressureFactor11
        comp_press = partial_out_1 + partial_out2 + partial_data_4

        return comp_press


    def get_calibration_data(self) -> Bmp388CalibrationData:
        reg_data = self.get_reg(0x31, 21)

        calibration_data = Bmp388CalibrationData(
            self.merge_bytes(reg_data[:2]) / math.pow(2, -8),
            self.merge_bytes(reg_data[2:4]) / math.pow(2, 30),
            self.merge_bytes(reg_data[4:5], unsigned=False) / math.pow(2, 48),
            self.merge_bytes(reg_data[5:7], unsigned=False) / math.pow(2, 20),
            self.merge_bytes(reg_data[7:9], unsigned=False) / math.pow(2, 29),
            self.merge_bytes(reg_data[9:10], unsigned=False) / math.pow(2, 32),
            self.merge_bytes(reg_data[10:11], unsigned=False) / math.pow(2, 37),
            self.merge_bytes(reg_data[11:13]) / math.pow(2, -3),
            self.merge_bytes(reg_data[13:15]) / math.pow(2, 6),
            self.merge_bytes(reg_data[15:16], unsigned=False) / math.pow(2, 8),
            self.merge_bytes(reg_data[16:17], unsigned=False) / math.pow(2, 15),
            self.merge_bytes(reg_data[17:19], unsigned=False) / math.pow(2, 48),
            self.merge_bytes(reg_data[19:20], unsigned=False) / math.pow(2, 48),
            self.merge_bytes(reg_data[20:21], unsigned=False) / math.pow(2, 65),
        )

        return calibration_data

sensor = Bmp388(0, 0, 29)

print(sensor.get_temp())
print(sensor.get_pressure())
print(sensor.get_status())
print(sensor.get_err())
