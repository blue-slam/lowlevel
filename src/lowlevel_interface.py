import struct
import smbus

# I2C Address Map
I2C_ADDR = 20
BUTTON_A_ADDR = 0
BUTTON_B_ADDR = 1
BUTTON_C_ADDR = 2
CELL_COUNT_ADDR = 3
BATTERY_MILLIVOLTS_ADDR = 4
LOW_VOLTAGE_CUTOFF_ADDR = 6
LEFT_MOTOR_COUNT_ADDR = 8
RIGHT_MOTOR_COUNT_ADDR = 12
LEFT_MOTOR_SPEED_ADDR = 16
RIGHT_MOTOR_SPEED_ADDR = 18
LEFT_MOTOR_ADDR = 20
RIGHT_MOTOR_ADDR = 22
CLEAR_MOTOR_COUNTS_ADDR = 24
YELLOW_LED_ADDR = 25
GREEN_LED_ADDR = 26
RED_LED_ADDR = 27
PLAY_NOTES_ADDR = 28
NOTES_ADDR = 29


class Interface:
    def __init__(self):
        self._bus = smbus.SMBus(1)
        self._i2c_addr = I2C_ADDR
        self.name = 'blue_slam'

    def read_unpack(self, address, size, fmt):
        # Ideally we could do this:
        #    byte_list = self.bus.read_i2c_block_data(self.i2c_addr, address, size)
        # But the AVR's TWI module can't handle a quick write->read transition,
        # since the STOP interrupt will occasionally happen after the START
        # condition, and the TWI module is disabled until the interrupt can
        # be processed.

        self._bus.write_byte(self._i2c_addr, address)
        byte_list = []
        for n in range(0, size):
            byte_list.append(self._bus.read_byte(self._i2c_addr))
        return struct.unpack(fmt, bytes(bytearray(byte_list)))

    def write_pack(self, address, fmt, *data):
        data_array = map(ord, list(struct.pack(fmt, *data)))
        self._bus.write_i2c_block_data(self._i2c_addr, address, data_array)

    def set_leds(self, yellow, green, red):
        self.write_pack(YELLOW_LED_ADDR, 'BBB', yellow, green, red)

    def set_yellow_led(self, on):
        self.write_pack(YELLOW_LED_ADDR, 'B', on)

    def set_green_led(self, on):
        self.write_pack(GREEN_LED_ADDR, 'B', on)

    def set_red_led(self, on):
        self.write_pack(RED_LED_ADDR, 'B', on)

    def play_notes(self, notes):
        self.write_pack(PLAY_NOTES_ADDR, 'B17s', 1, notes.encode("ascii"))

    def set_motor_speeds(self, left, right):
        self.write_pack(LEFT_MOTOR_ADDR, '<hh', left, right)

    def clear_motor_counts(self):
        self.write_pack(CLEAR_MOTOR_COUNTS_ADDR, 'B', True)

    def get_motor_counts(self):
        return self.read_unpack(LEFT_MOTOR_COUNT_ADDR, 8, '<ll')

    def get_motor_rates(self):
        return self.read_unpack(LEFT_MOTOR_SPEED_ADDR, 4, '<hh')

    def get_motor_state(self):
        return self.read_unpack(LEFT_MOTOR_COUNT_ADDR, 12, '<llhh')

    def is_button_pushed(self):
        value = self.read_unpack(BUTTON_A_ADDR, 3, 'BBB')
        if value[0] + value[1] + value[2] > 0:
            self.write_pack(BUTTON_A_ADDR, 'BBB', 0, 0, 0)
        return value

    def is_button_a_pushed(self):
        value = self.read_unpack(BUTTON_A_ADDR, 1, 'B')[0]
        if value > 0:
            self.write_pack(BUTTON_A_ADDR, 'B', 0)
            return True
        return False

    def is_button_b_pushed(self):
        value = self.read_unpack(BUTTON_B_ADDR, 1, 'B')[0]
        if value > 0:
            self.write_pack(BUTTON_B_ADDR, 'B', 0)
            return True
        return False

    def is_button_c_pushed(self):
        value = self.read_unpack(BUTTON_C_ADDR, 1, 'B')[0]
        if value > 0:
            self.write_pack(BUTTON_C_ADDR, 'B', 0)
            return True
        return False

    def get_cell_count(self):
        return self.read_unpack(CELL_COUNT_ADDR, 1, 'B')[0]

    def get_low_voltage_cutoff(self):
        return self.read_unpack(LOW_VOLTAGE_CUTOFF_ADDR, 2, '<H')[0]

    def set_low_voltage_cutoff(self, millivolts):
        self.write_pack(6, 'H', millivolts)

    def get_battery_millivolts(self):
        return self.read_unpack(BATTERY_MILLIVOLTS_ADDR, 2, '<H')[0]

    def get_battery_state(self):
        return self.read_unpack(CELL_COUNT_ADDR, 5, '<BHH')
