# -*- coding: utf-8 -*-
"""
Read functions for measurement values of the HDC3020 Sensor via I2c interface.

Copyright 2024 MW technologies

Disclaimer:
This application example is non-binding and does not claim to be complete with
regard to configuration and equipment as well as all eventualities. The
application example is intended to provide assistance with the HDC3020 sensor
module design-in and is provided "as is".You yourself are responsible for the
proper operation of the products described. This application example does not
release you from the obligation to handle the product safely during
application, installation, operation and maintenance. By using this application
example, you acknowledge that we cannot be held liable for any damage beyond
the liability regulations described.

We reserve the right to make changes to this application example at any time
without notice. In case of discrepancies between the suggestions in this
application example and other MW technologies publications, such as catalogues, the content
of the other documentation takes precedence. We assume no liability for
the information contained in this document.
"""


# pylint: disable=E0401
from smbus2 import SMBus, i2c_msg
import math
# pylint: enable=E0401

CRC8_ONEWIRE_POLY = 0x31
CRC8_ONEWIRE_START = 0xFF
HDC3020_COMMAND_READ_SINGLE_SHOT_MODE_0 = 0x2400
HDC3020_COMMAND_READ_SINGLE_SHOT_MODE_1 = 0x240B
HDC3020_COMMAND_READ_SINGLE_SHOT_MODE_2 = 0x2416
HDC3020_COMMAND_READ_SINGLE_SHOT_MODE_3 = 0x24FF
HDC3020_COMMAND_READ_PERIODIC_MEASUREMENT = 0xE000
HDC3020_COMMAND_READ_MINIMUM_MEASUREMENT_T = 0xE002
HDC3020_COMMAND_READ_MAXIMUM_MEASUREMENT_T = 0xE003
HDC3020_COMMAND_READ_MINIMUM_MEASUREMENT_RH = 0xE004
HDC3020_COMMAND_READ_MAXIMUM_MEASUREMENT_RH = 0xE005
HDC3020_COMMAND_CLEAR_STATUS_REGISTER = 0x3041
HDC3020_COMMAND_READ_STATUS_REGISTER = 0xF32D
HDC3020_COMMAND_READ_REGISTER_2 = 0xF352
HDC3020_COMMAND_START_PERIODIC_MEASUREMENT = 0x201E
HDC3020_COMMAND_END_PERIODIC_MEASUREMENT = 0x3093
HDC3020_COMMAND_SOFT_RESET = 0x30A2
HDC3020_COMMAND_HEATER_CONFIGURE = 0x306E
HDC3020_COMMAND_HEATER_ON = 0x306D
HDC3020_COMMAND_HEATER_OFF = 0x3066
HDC3020_COMMAND_READ_MANUFACTOR_ID = 0x3781
HDC3020_COMMAND_READ_MANUFACTOR_ID_0 = 0x3683
HDC3020_COMMAND_READ_MANUFACTOR_ID_1 = 0x3684
HDC3020_COMMAND_READ_MANUFACTOR_ID_2 = 0x3685
HDC3020_COMMAND_CHANGE_SET_LOW_ALERT = 0x6100
HDC3020_COMMAND_CHANGE_CLEAR_LOW_ALERT = 0x610B
HDC3020_COMMAND_CHANGE_SET_HIGH_ALERT = 0x611D
HDC3020_COMMAND_CHANGE_CLEAR_HIGH_ALERT = 0x6116
HDC3020_COMMAND_READ_SET_LOW_ALERT = 0xE102
HDC3020_COMMAND_READ_CLEAR_LOW_ALERT = 0xE109
HDC3020_COMMAND_READ_SET_HIGH_ALERT = 0xE11F
HDC3020_COMMAND_READ_CLEAR_HIGH_ALERT = 0xE114
HDC3020_COMMAND_INTO_NON_VOLATILE_MEMORY = 0x6155
HDC3020_COMMAND_OFFSET_VALUE = 0xA004
HDC3020_COMMAND_CHANGE_DEFAULT_POWER_ON = 0x61BB
HDC3020_COMMAND_START_PERIODIC_MEASUREMENT_0_5_MODE_0 = 0x2032
HDC3020_COMMAND_START_PERIODIC_MEASUREMENT_0_5_MODE_1 = 0x2024
HDC3020_COMMAND_START_PERIODIC_MEASUREMENT_0_5_MODE_2 = 0x202F
HDC3020_COMMAND_START_PERIODIC_MEASUREMENT_0_5_MODE_3 = 0x20FF
HDC3020_COMMAND_START_PERIODIC_MEASUREMENT_1_MODE_0 = 0x2130
HDC3020_COMMAND_START_PERIODIC_MEASUREMENT_1_MODE_1 = 0x2126
HDC3020_COMMAND_START_PERIODIC_MEASUREMENT_1_MODE_2 = 0x212D
HDC3020_COMMAND_START_PERIODIC_MEASUREMENT_1_MODE_3 = 0x21FF
HDC3020_COMMAND_START_PERIODIC_MEASUREMENT_2_MODE_0 = 0x2236
HDC3020_COMMAND_START_PERIODIC_MEASUREMENT_2_MODE_1 = 0x2220
HDC3020_COMMAND_START_PERIODIC_MEASUREMENT_2_MODE_2 = 0x222B
HDC3020_COMMAND_START_PERIODIC_MEASUREMENT_2_MODE_3 = 0x22FF  
HDC3020_COMMAND_START_PERIODIC_MEASUREMENT_4_MODE_0 = 0x2334
HDC3020_COMMAND_START_PERIODIC_MEASUREMENT_4_MODE_1 = 0x2322
HDC3020_COMMAND_START_PERIODIC_MEASUREMENT_4_MODE_2 = 0x2329
HDC3020_COMMAND_START_PERIODIC_MEASUREMENT_4_MODE_3 = 0x23FF
HDC3020_COMMAND_START_PERIODIC_MEASUREMENT_10_MODE_0 = 0x2737
HDC3020_COMMAND_START_PERIODIC_MEASUREMENT_10_MODE_1 = 0x2721
HDC3020_COMMAND_START_PERIODIC_MEASUREMENT_10_MODE_2 = 0x272A
HDC3020_COMMAND_START_PERIODIC_MEASUREMENT_10_MODE_3 = 0x27FF

def get_status_string(status_code):
    """Return string from status_code."""
    status_string = {
        0: "Success",
        1: "Not acknowledge error",
        2: "Checksum error",
    }

    if status_code < len(status_string):
        return status_string[status_code]
    return "Unknown error"


def calc_crc8(buf, start, end):
    ''' calculate crc8 checksum  '''
    crc_val = CRC8_ONEWIRE_START
    for j in range(start, end):
        cur_val = buf[j]
        for _ in range(8):
            if ((crc_val ^ cur_val) & 0x80) != 0:
                crc_val = (crc_val << 1) ^ CRC8_ONEWIRE_POLY
            else:
                crc_val = crc_val << 1
            cur_val = cur_val << 1
    crc_val &= 0xFF
    return crc_val


class HDC3020():
    """Implements communication with HDC3020 over i2c with a specific address."""

    def __init__(self, i2c_address):
        self.i2c_address = i2c_address

    def get_single_shot_temp_hum(self, Mode): #Mode : 0 = low noise, 1 , 2 , 3 = lowest power
        """Let the sensor take a measurement and return the temperature and humidity values."""
        if Mode == 0:
            i2c_response = self.wire_write_read(
                [(HDC3020_COMMAND_READ_SINGLE_SHOT_MODE_0 >> 8),
                 (HDC3020_COMMAND_READ_SINGLE_SHOT_MODE_0 & 0xFF)], 6)
        elif Mode == 1:
            i2c_response = self.wire_write_read(
                [(HDC3020_COMMAND_READ_SINGLE_SHOT_MODE_1 >> 8),
                 (HDC3020_COMMAND_READ_SINGLE_SHOT_MODE_1 & 0xFF)], 6)
        elif Mode == 2:
            i2c_response = self.wire_write_read(
                [(HDC3020_COMMAND_READ_SINGLE_SHOT_MODE_2 >> 8),
                 (HDC3020_COMMAND_READ_SINGLE_SHOT_MODE_2 & 0xFF)], 6)
        else:
            i2c_response = self.wire_write_read(
                [(HDC3020_COMMAND_READ_SINGLE_SHOT_MODE_3 >> 8),
                 (HDC3020_COMMAND_READ_SINGLE_SHOT_MODE_3 & 0xFF)], 6)
        if (i2c_response[2] == calc_crc8(i2c_response, 0, 2)) & (i2c_response[5] 
                                                                 == calc_crc8(i2c_response, 3, 5)):
            temperature = -45 + (175 * ((i2c_response[0]) * 256 + i2c_response[1])/(65536 - 1))
            humidity = 100 * ((float)(i2c_response[3]) * 256 + i2c_response[4]) / (65536 - 1)
            return temperature, humidity
        else:
            raise Warning(get_status_string(2))

    def get_periodic_measurement_temp_hum(self):
        """Get the last measurement from the periodic measurement for temperature and humidity"""
        i2c_response = self.wire_write_read(
            [(HDC3020_COMMAND_READ_PERIODIC_MEASUREMENT >> 8),
             (HDC3020_COMMAND_READ_PERIODIC_MEASUREMENT & 0xFF)], 6)
        if (i2c_response[2] == calc_crc8(i2c_response, 0, 2)) & (i2c_response[5] ==
                                                                 calc_crc8(i2c_response, 3, 5)):
            temperature = -45 + (175 * ((i2c_response[0]) * 256 + i2c_response[1])/(65536 - 1))
            humidity = 100 * ((float)(i2c_response[3]) * 256 + i2c_response[4]) / (65536 - 1)
            return temperature, humidity
        else:
            raise Warning(get_status_string(2))

    def get_dewpoint(self, temperature, humidity):
        """Get the calculated dewpoint"""
        if(temperature > 0):
            dewpoint = 243.12 * ((17.62 * temperature) / (243.12 + temperature) + math.log(humidity / 100))/((17.62 * 243.12) / (243.12 + temperature) - math.log(humidity / 100))
        else:
            dewpoint = 272.62 * ((((22.46*temperature) / (272.62 + temperature)) + math.log(humidity / 100)) / (((22.46 * 272.62) / (272.62 + temperature)) - math.log(humidity / 100)))
        return dewpoint

    def get_periodic_measurement_min_temp(self):
        """get the minimum temperature from the periodic measurement"""
        i2c_response = self.wire_write_read(
            [(HDC3020_COMMAND_READ_MINIMUM_MEASUREMENT_T >> 8),
             (HDC3020_COMMAND_READ_MINIMUM_MEASUREMENT_T & 0xFF)], 3)
        if (i2c_response[2] == calc_crc8(i2c_response, 0, 2)):
            temperature = -45 + (175 * ((i2c_response[0]) * 256 + i2c_response[1])/(65536 - 1))
            return temperature
        else:
            raise Warning(get_status_string(2))

    def get_periodic_measurement_max_temp(self):
        """get the maximum temperature from the periodic measurement"""
        i2c_response = self.wire_write_read(
            [(HDC3020_COMMAND_READ_MAXIMUM_MEASUREMENT_T >> 8),
             (HDC3020_COMMAND_READ_MAXIMUM_MEASUREMENT_T & 0xFF)], 3)
        if (i2c_response[2] == calc_crc8(i2c_response, 0, 2)):
            temperature = -45 + (175 * ((i2c_response[0]) * 256 + i2c_response[1])/(65536 - 1))
            return temperature
        else:
            raise Warning(get_status_string(2))

    def get_periodic_measurement_min_hum(self):
        """get the minimum humidity from the periodic measurement"""
        i2c_response = self.wire_write_read(
            [(HDC3020_COMMAND_READ_MINIMUM_MEASUREMENT_RH >> 8),
             (HDC3020_COMMAND_READ_MINIMUM_MEASUREMENT_RH & 0xFF)], 3)
        if (i2c_response[2] == calc_crc8(i2c_response, 0, 2)):
            humidity = 100 * ((float)(i2c_response[0]) * 256 + i2c_response[1]) / (65536 - 1)
            return humidity
        else:
            raise Warning(get_status_string(2))

    def get_periodic_measurement_max_hum(self):
        """get the maximum humidity from the periodic measurement"""
        i2c_response = self.wire_write_read(
            [(HDC3020_COMMAND_READ_MAXIMUM_MEASUREMENT_RH >> 8),
             (HDC3020_COMMAND_READ_MAXIMUM_MEASUREMENT_RH & 0xFF)], 3)
        if (i2c_response[2] == calc_crc8(i2c_response, 0, 2)):
            humidity = 100 * ((float)(i2c_response[0]) * 256 + i2c_response[1]) / (65536 - 1)
            return humidity
        else:
            raise Warning(get_status_string(2))

    def change_default_device_power_on(self, send_bytes1, send_bytes2):
        """change the default power on status, more info in the datasheet"""
        send_byte1 = send_byte1 & 255
        send_byte2 = send_byte2 & 255
        self.wire_write(
            [(HDC3020_COMMAND_CHANGE_DEFAULT_POWER_ON >> 8),
             (HDC3020_COMMAND_CHANGE_DEFAULT_POWER_ON & 0xFF),
             send_bytes1, send_bytes2,
             calc_crc8([send_bytes1, send_bytes2], 0, 2)])

    def change_set_low_alert(self, send_bytes1, send_bytes2):
        """change the low alert limits, more info in the datasheet"""
        send_byte1 = send_byte1 & 255
        send_byte2 = send_byte2 & 255
        self.wire_write(
            [(HDC3020_COMMAND_CHANGE_SET_LOW_ALERT >> 8),
             (HDC3020_COMMAND_CHANGE_SET_LOW_ALERT & 0xFF),
             send_bytes1, send_bytes2,
             calc_crc8([send_bytes1, send_bytes2], 0, 2)])

    def change_clear_low_alert(self, send_bytes1, send_bytes2):
        """change the low clear limits, more info in the datasheet"""
        send_byte1 = send_byte1 & 255
        send_byte2 = send_byte2 & 255
        self.wire_write(
            [(HDC3020_COMMAND_CHANGE_CLEAR_LOW_ALERT >> 8),
             (HDC3020_COMMAND_CHANGE_CLEAR_LOW_ALERT & 0xFF),
             send_bytes1, send_bytes2,
             calc_crc8([send_bytes1, send_bytes2], 0, 2)])

    def change_set_high_alert(self, send_bytes1, send_bytes2):
        """change the high alert limits, more info in the datasheet"""
        send_byte1 = send_byte1 & 255
        send_byte2 = send_byte2 & 255
        self.wire_write(
            [(HDC3020_COMMAND_CHANGE_SET_HIGH_ALERT >> 8),
             (HDC3020_COMMAND_CHANGE_SET_HIGH_ALERT & 0xFF),
             send_bytes1, send_bytes2,
             calc_crc8([send_bytes1, send_bytes2], 0, 2)])

    def change_clear_high_alert(self, send_bytes1, send_bytes2):
        """change the high clear limits, more info in the datasheet"""
        send_byte1 = send_byte1 & 255
        send_byte2 = send_byte2 & 255
        self.wire_write(
            [(HDC3020_COMMAND_CHANGE_CLEAR_HIGH_ALERT >> 8),
             (HDC3020_COMMAND_CHANGE_CLEAR_HIGH_ALERT & 0xFF),
             send_bytes1, send_bytes2,
             calc_crc8([send_bytes1, send_bytes2], 0, 2)])

    def read_set_low_alert(self):
        """read set low alert limit, more info in the datasheet"""
        i2c_response = self.wire_write_read(
            [(HDC3020_COMMAND_READ_SET_LOW_ALERT >> 8),
             (HDC3020_COMMAND_READ_SET_LOW_ALERT & 0xFF)], 3)
        if (i2c_response[2] == calc_crc8(i2c_response, 0, 2)):
            return i2c_response[0], i2c_response[1]
        else:
            raise Warning(get_status_string(2))

    def read_clear_low_alert(self):
        """read clear low alert limit, more info in the datasheet"""
        i2c_response = self.wire_write_read(
            [(HDC3020_COMMAND_READ_CLEAR_LOW_ALERT >> 8),
             (HDC3020_COMMAND_READ_CLEAR_LOW_ALERT & 0xFF)], 3)
        if (i2c_response[2] == calc_crc8(i2c_response, 0, 2)):
            return i2c_response[0], i2c_response[1]
        else:
            raise Warning(get_status_string(2))

    def read_set_high_alert(self):
        """read set high alert limit, more info in the datasheet"""
        i2c_response = self.wire_write_read(
            [(HDC3020_COMMAND_READ_SET_HIGH_ALERT >> 8),
             (HDC3020_COMMAND_READ_SET_HIGH_ALERT & 0xFF)], 3)
        if (i2c_response[2] == calc_crc8(i2c_response, 0, 2)):
            return i2c_response[0], i2c_response[1]
        else:
            raise Warning(get_status_string(2))

    def read_clear_high_alert(self):
        """read clear high alert limit, more info in the datasheet"""
        i2c_response = self.wire_write_read(
            [(HDC3020_COMMAND_READ_CLEAR_HIGH_ALERT >> 8),
             (HDC3020_COMMAND_READ_CLEAR_HIGH_ALERT & 0xFF)], 3)
        if (i2c_response[2] == calc_crc8(i2c_response, 0, 2)):
            return i2c_response[0], i2c_response[1]
        else:
            raise Warning(get_status_string(2))

    def deactivate_environmental_tracking(self):
        """deacticate the environmental trackingread"""
        self.wire_write(
            [(HDC3020_COMMAND_CHANGE_SET_LOW_ALERT >> 8), 
             (HDC3020_COMMAND_CHANGE_SET_LOW_ALERT & 0xFF), 
             0xFF, 0xFF, 0xAC])
        self.wire_write(
            [(HDC3020_COMMAND_CHANGE_SET_HIGH_ALERT >> 8),
             (HDC3020_COMMAND_CHANGE_SET_HIGH_ALERT & 0xFF),
             0x00, 0x00, 0x81])

    def transfer_thresholds_into_non_volatile_memory(self):
        """transfer_thresholds_into_non_volatile_memory"""
        self.wire_write(
            [(HDC3020_COMMAND_INTO_NON_VOLATILE_MEMORY >> 8), 
             (HDC3020_COMMAND_INTO_NON_VOLATILE_MEMORY & 0xFF)])

    def change_offset_value(self, send_bytes1, send_bytes2):
        """change offset value, more info in the datasheet"""
        send_byte1 = send_byte1 & 255
        send_byte2 = send_byte2 & 255
        self.wire_write(
            [(HDC3020_COMMAND_OFFSET_VALUE >> 8),
             (HDC3020_COMMAND_OFFSET_VALUE & 0xFF),
             send_bytes1, send_bytes2,
             calc_crc8([send_bytes1, send_bytes2], 0, 2)])

    def read_offset_value(self):
        """read offset value, more info in the datasheet"""
        i2c_response = self.wire_write_read(
            [(HDC3020_COMMAND_OFFSET_VALUE >> 8),
             (HDC3020_COMMAND_OFFSET_VALUE & 0xFF)], 3)
        if (i2c_response[2] == calc_crc8(i2c_response, 0, 2)):
            return i2c_response[0], i2c_response[1]
        else:
            raise Warning(get_status_string(2))

    def change_heater_current(self, send_bytes1, send_bytes2):
        """change heater current, more info in the datasheet"""
        send_byte1 = send_byte1 & 255
        send_byte2 = send_byte2 & 255
        self.wire_write(
            [(HDC3020_COMMAND_HEATER_CONFIGURE >> 8),
             (HDC3020_COMMAND_HEATER_CONFIGURE & 0xFF),
             send_bytes1, send_bytes2,
             calc_crc8([send_bytes1, send_bytes2], 0, 2)])

    def read_heater_current(self):
        """read heater current, more info in the datasheet"""
        i2c_response = self.wire_write_read(
            [(HDC3020_COMMAND_HEATER_CONFIGURE >> 8),
             (HDC3020_COMMAND_HEATER_CONFIGURE & 0xFF)], 3)
        if (i2c_response[2] == calc_crc8(i2c_response, 0, 2)):
            return i2c_response[0], i2c_response[1]
        else:
            raise Warning(get_status_string(2))

    def start_periodic_measurement(self, measurement_per_seconds, mode): #measurementPerSeconds: 0 = 0.5 mps, 1 = 1mps, 2 = 2mps, 3 = 4mps, 4 = 10mps;  Mode : 0 = low noise, 1 , 2 , 3 = lowest power
        """starts the periodic measurement"""
        if measurement_per_seconds == 0:
            if mode == 0:
                self.wire_write(
                    [(HDC3020_COMMAND_START_PERIODIC_MEASUREMENT_0_5_MODE_0 >> 8),
                     (HDC3020_COMMAND_START_PERIODIC_MEASUREMENT_0_5_MODE_0 & 0xFF)])
            elif mode == 1:
                self.wire_write(
                    [(HDC3020_COMMAND_START_PERIODIC_MEASUREMENT_0_5_MODE_1 >> 8),
                     (HDC3020_COMMAND_START_PERIODIC_MEASUREMENT_0_5_MODE_1 & 0xFF)])
            elif mode == 2:
                self.wire_write(
                    [(HDC3020_COMMAND_START_PERIODIC_MEASUREMENT_0_5_MODE_2 >> 8),
                     (HDC3020_COMMAND_START_PERIODIC_MEASUREMENT_0_5_MODE_2 & 0xFF)])
            else:
                self.wire_write(
                    [(HDC3020_COMMAND_START_PERIODIC_MEASUREMENT_0_5_MODE_3 >> 8),
                     (HDC3020_COMMAND_START_PERIODIC_MEASUREMENT_0_5_MODE_3 & 0xFF)])
        if measurement_per_seconds == 1:
            if mode == 0:
                self.wire_write(
                    [(HDC3020_COMMAND_START_PERIODIC_MEASUREMENT_1_MODE_0 >> 8),
                     (HDC3020_COMMAND_START_PERIODIC_MEASUREMENT_1_MODE_0 & 0xFF)])
            elif mode == 1:
                self.wire_write(
                    [(HDC3020_COMMAND_START_PERIODIC_MEASUREMENT_1_MODE_1 >> 8),
                     (HDC3020_COMMAND_START_PERIODIC_MEASUREMENT_1_MODE_1 & 0xFF)])
            elif mode == 2:
                self.wire_write(
                    [(HDC3020_COMMAND_START_PERIODIC_MEASUREMENT_1_MODE_2 >> 8),
                     (HDC3020_COMMAND_START_PERIODIC_MEASUREMENT_1_MODE_2 & 0xFF)])
            else:
                self.wire_write(
                    [(HDC3020_COMMAND_START_PERIODIC_MEASUREMENT_1_MODE_3 >> 8),
                     (HDC3020_COMMAND_START_PERIODIC_MEASUREMENT_1_MODE_3 & 0xFF)])
        if measurement_per_seconds == 2:
            if mode == 0:
                self.wire_write(
                    [(HDC3020_COMMAND_START_PERIODIC_MEASUREMENT_2_MODE_0 >> 8),
                     (HDC3020_COMMAND_START_PERIODIC_MEASUREMENT_2_MODE_0 & 0xFF)])
            elif mode == 1:
                self.wire_write(
                    [(HDC3020_COMMAND_START_PERIODIC_MEASUREMENT_2_MODE_1 >> 8),
                     (HDC3020_COMMAND_START_PERIODIC_MEASUREMENT_2_MODE_1 & 0xFF)])
            elif mode == 2:
                self.wire_write(
                    [(HDC3020_COMMAND_START_PERIODIC_MEASUREMENT_2_MODE_2 >> 8),
                     (HDC3020_COMMAND_START_PERIODIC_MEASUREMENT_2_MODE_2 & 0xFF)])
            else:
                self.wire_write(
                    [(HDC3020_COMMAND_START_PERIODIC_MEASUREMENT_2_MODE_3 >> 8),
                     (HDC3020_COMMAND_START_PERIODIC_MEASUREMENT_2_MODE_3 & 0xFF)])
        if measurement_per_seconds == 3:
            if mode == 0:
                self.wire_write(
                    [(HDC3020_COMMAND_START_PERIODIC_MEASUREMENT_4_MODE_0 >> 8),
                     (HDC3020_COMMAND_START_PERIODIC_MEASUREMENT_4_MODE_0 & 0xFF)])
            elif mode == 1:
                self.wire_write(
                    [(HDC3020_COMMAND_START_PERIODIC_MEASUREMENT_4_MODE_1 >> 8),
                     (HDC3020_COMMAND_START_PERIODIC_MEASUREMENT_4_MODE_1 & 0xFF)])
            elif mode == 2:
                self.wire_write(
                    [(HDC3020_COMMAND_START_PERIODIC_MEASUREMENT_4_MODE_2 >> 8),
                     (HDC3020_COMMAND_START_PERIODIC_MEASUREMENT_4_MODE_2 & 0xFF)])
            else:
                self.wire_write(
                    [(HDC3020_COMMAND_START_PERIODIC_MEASUREMENT_4_MODE_3 >> 8),
                     (HDC3020_COMMAND_START_PERIODIC_MEASUREMENT_4_MODE_3 & 0xFF)])
        if measurement_per_seconds == 4:
            if mode == 0:
                self.wire_write(
                    [(HDC3020_COMMAND_START_PERIODIC_MEASUREMENT_10_MODE_0 >> 8),
                     (HDC3020_COMMAND_START_PERIODIC_MEASUREMENT_10_MODE_0 & 0xFF)])
            elif mode == 1:
                self.wire_write(
                    [(HDC3020_COMMAND_START_PERIODIC_MEASUREMENT_10_MODE_1 >> 8),
                     (HDC3020_COMMAND_START_PERIODIC_MEASUREMENT_10_MODE_1 & 0xFF)])
            elif mode == 2:
                self.wire_write(
                    [(HDC3020_COMMAND_START_PERIODIC_MEASUREMENT_10_MODE_2 >> 8),
                     (HDC3020_COMMAND_START_PERIODIC_MEASUREMENT_10_MODE_2 & 0xFF)])
            else:
                self.wire_write(
                    [(HDC3020_COMMAND_START_PERIODIC_MEASUREMENT_10_MODE_3 >> 8),
                     (HDC3020_COMMAND_START_PERIODIC_MEASUREMENT_10_MODE_3 & 0xFF)])

    def end_periodic_measurement(self):
        """ends the periodic measurement"""
        self.wire_write([(HDC3020_COMMAND_END_PERIODIC_MEASUREMENT >> 8),
                          (HDC3020_COMMAND_END_PERIODIC_MEASUREMENT & 0xFF)])

    def heater_on(self):
        """turns the heater on """
        self.wire_write([(HDC3020_COMMAND_HEATER_ON >> 8),
                          (HDC3020_COMMAND_HEATER_ON & 0xFF)])

    def heater_off(self):
        """turns the heater off"""
        self.wire_write([(HDC3020_COMMAND_HEATER_OFF >> 8),
                          (HDC3020_COMMAND_HEATER_OFF & 0xFF)])

    def read_identification(self):
        """reads the identification number"""
        i2c_response1 = self.wire_write_read(
            [(HDC3020_COMMAND_READ_MANUFACTOR_ID_0 >> 8),
             (HDC3020_COMMAND_READ_MANUFACTOR_ID_0 & 0xFF)], 3)
        i2c_response2 = self.wire_write_read(
            [(HDC3020_COMMAND_READ_MANUFACTOR_ID_1 >> 8),
             (HDC3020_COMMAND_READ_MANUFACTOR_ID_1 & 0xFF)], 3)
        i2c_response3 = self.wire_write_read(
            [(HDC3020_COMMAND_READ_MANUFACTOR_ID_1 >> 8),
             (HDC3020_COMMAND_READ_MANUFACTOR_ID_1 & 0xFF)], 3)
        if i2c_response1[2] == calc_crc8(i2c_response1, 0, 2) and i2c_response2[2] == calc_crc8(i2c_response2, 0, 2) and i2c_response3[2] == calc_crc8(i2c_response3, 0, 2):
            return [i2c_response1[0], i2c_response1[1], i2c_response2[0], i2c_response2[1], i2c_response3[0], i2c_response3[1]]
        else:
            raise Warning(get_status_string(2))

    def reset(self):
        """resets the sensor"""
        self.wire_write([(HDC3020_COMMAND_SOFT_RESET >> 8),
                         (HDC3020_COMMAND_SOFT_RESET & 0xFF)])

    def constant_heater_on_off(self):
        """get the informatio if the heater is on or off"""
        i2c_response = self.wire_write_read(
            [(HDC3020_COMMAND_READ_STATUS_REGISTER >> 8),
             (HDC3020_COMMAND_READ_STATUS_REGISTER & 0xFF)], 3)
        if i2c_response[2] == calc_crc8(i2c_response, 0, 2):
            i2c_response[0] = (i2c_response[0] << 2) & 255
            return i2c_response[0] >> 7
        else:
            raise Warning(get_status_string(2))

    def clear_statusregister(self):
        """clear the status register"""
        self.wire_write([(HDC3020_COMMAND_CLEAR_STATUS_REGISTER >> 8),
                         (HDC3020_COMMAND_CLEAR_STATUS_REGISTER & 0xFF)])

    def wire_write_read(self,  buf, receiving_bytes):
        """write a command to the sensor to get different answers like temperature values,..."""
        write_command = i2c_msg.write(self.i2c_address, buf)
        read_command = i2c_msg.read(self.i2c_address, receiving_bytes)
        with SMBus(1) as hdc3020_communication:
            hdc3020_communication.i2c_rdwr(write_command, read_command)
        return list(read_command)

    def wire_write(self, buf):
        """write to the sensor"""
        write_command = i2c_msg.write(self.i2c_address, buf)
        with SMBus(1) as hdc3020_communication:
            hdc3020_communication.i2c_rdwr(write_command)
