import asyncio
import os
from dataclasses import dataclass
from typing import List

import RPi.GPIO as gpio
from spidev import SpiDev


@dataclass
class Bmp388CalibrationData:
    t1: float
    t2: float
    t3: float
    p1: float
    p2: float
    p3: float
    p4: float
    p5: float
    p6: float
    p7: float
    p8: float
    p9: float
    p10: float
    p11: float
    temp: float = 0

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

    def get_sensor_time(self) -> int:
        reg_data = self.get_reg(0x0C, 3)

        clock = int.from_bytes(reg_data, "little", signed=False)

        return round(clock/29000)

    def get_temp(self) -> float:
        reg_data = self.get_reg(0x07, 3)
        raw_temp = int.from_bytes(reg_data, "little", signed=False)

        partial_data_1 = raw_temp - self.calibration_data.t1
        partial_data_2 = partial_data_1 * self.calibration_data.t2

        temp = partial_data_2 + (partial_data_1 * partial_data_1) * self.calibration_data.t3
        self.calibration_data.temp = temp

        return round(temp, 2)

    def get_pressure(self) -> float:
        reg_data = self.get_reg(0x04, 3)
        raw_pressure = int.from_bytes(reg_data, "little", signed=False)

        partial_data_1 = self.calibration_data.p6 * self.calibration_data.temp
        partial_data_2 = self.calibration_data.p7 * (self.calibration_data.temp * self.calibration_data.temp)
        partial_data_3 = self.calibration_data.p8 * (self.calibration_data.temp * self.calibration_data.temp * self.calibration_data.temp)
        partial_out_1 = self.calibration_data.p5 + partial_data_1 + partial_data_2 + partial_data_3

        partial_data_1 = self.calibration_data.p2 * self.calibration_data.temp
        partial_data_2 = self.calibration_data.p3 * (self.calibration_data.temp * self.calibration_data.temp)
        partial_data_3 = self.calibration_data.p4 * (self.calibration_data.temp * self.calibration_data.temp * self.calibration_data.temp)
        partial_out2 = raw_pressure * (self.calibration_data.p1 + partial_data_1 + partial_data_2 + partial_data_3)

        partial_data_1 = raw_pressure * raw_pressure
        partial_data_2 = self.calibration_data.p9 + self.calibration_data.p10 * self.calibration_data.temp
        partial_data_3 = partial_data_1 * partial_data_2
        partial_data_4 = partial_data_3 + (raw_pressure * raw_pressure * raw_pressure) * self.calibration_data.p11
        comp_press = partial_out_1 + partial_out2 + partial_data_4

        return round(comp_press / 100, 2)


    def get_calibration_data(self) -> Bmp388CalibrationData:
        reg_data = self.get_reg(0x31, 21)

        nvm_par_t1 = int.from_bytes(reg_data[:2], "little", signed=False)
        nvm_par_t2 = int.from_bytes(reg_data[2:4], "little", signed=False)
        nvm_par_t3 = int.from_bytes(reg_data[4:5], "little", signed=True)
        nvm_par_p1 = int.from_bytes(reg_data[5:7], "little", signed=True)
        nvm_par_p2 = int.from_bytes(reg_data[7:9], "little", signed=True)
        nvm_par_p3 = int.from_bytes(reg_data[9:10], "little", signed=True)
        nvm_par_p4 = int.from_bytes(reg_data[10:11], "little", signed=True)
        nvm_par_p5 = int.from_bytes(reg_data[11:13], "little", signed=False)
        nvm_par_p6 = int.from_bytes(reg_data[13:15], "little", signed=False)
        nvm_par_p7 = int.from_bytes(reg_data[15:16], "little", signed=True)
        nvm_par_p8 = int.from_bytes(reg_data[16:17], "little", signed=True)
        nvm_par_p9 = int.from_bytes(reg_data[17:19], "little", signed=True)
        nvm_par_p10 = int.from_bytes(reg_data[19:20], "little", signed=True)
        nvm_par_p11 = int.from_bytes(reg_data[20:21], "little", signed=True)

        calibration_data = Bmp388CalibrationData(
            nvm_par_t1 / 2 ** -8,
            nvm_par_t2 / 2 ** 30,
            nvm_par_t3 / 2 ** 48,
            (nvm_par_p1 - 2 ** 14) / 2 ** 20,
            (nvm_par_p2 - 2 ** 14) / 2 ** 29,
            nvm_par_p3 / 2 ** 32,
            nvm_par_p4 / 2 ** 37,
            nvm_par_p5 / 2 ** -3,
            nvm_par_p6 / 2 ** 6,
            nvm_par_p7 / 2 ** 8,
            nvm_par_p8 / 2 ** 15,
            nvm_par_p9 / 2 ** 48,
            nvm_par_p10 /2 ** 48,
            nvm_par_p11 /2 ** 65,
        )

        return calibration_data

async def main():
    sensor = Bmp388(0, 0, 29)

    while True:
        await asyncio.sleep(1)
        os.system('clear')
        print(f"Chip ID: {sensor.get_chip_id()}")
        print(f"Sensor time: {sensor.get_sensor_time()}")
        print(f"Temperature: {sensor.get_temp()}")
        print(f"Pressure: {sensor.get_pressure()}")

if __name__ == "__main__":
    asyncio.run(main())
