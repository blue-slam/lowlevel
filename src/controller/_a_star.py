from threading import Lock
import rospy
from _a_star_interface import AStarInterface
from std_msgs.msg import Byte, String, Empty
from sensor_msgs.msg import BatteryState
from lowlevel.msg import Buttons
from lowlevel.msg import MotorPosition
from lowlevel.msg import MotorCmd


class AStar:
    def __init__(self, params):

        self._lock = Lock()
        self._interface = AStarInterface()

        self._battery_design_capacity = params['battery']['design_capacity']
        self._battery_serial_number = params['battery']['serial_number']
        self._motors_max_cmd = params['motors']['max_cmd']

        self._left_motor_cmd = 0
        self._right_motor_cmd = 0

        self._position_reset = False

        self._led_changed = False
        self._yellow_led = False
        self._green_led = False
        self._red_led = False

        self._buzzer_notes = None

        self._battery_state = BatteryState()
        self._motor_position = MotorPosition()

        root_topic = params['controller']['root_topic']

        self._battery_state_pub = rospy.Publisher(root_topic + '/battery', BatteryState, queue_size=10, latch=True)
        self._buttons_pub = rospy.Publisher(root_topic + '/buttons', Buttons, queue_size=10)
        self._motor_position_pub = rospy.Publisher(root_topic + '/motors/position', MotorPosition, queue_size=10)

        self._motor_cmd_sub = rospy.Subscriber(root_topic + '/motors/cmd', MotorCmd, self._motor_cmd_handler)
        self._position_reset_sub = rospy.Subscriber(root_topic + '/motors/reset', Empty, self._position_reset_handler)
        self._yellow_led_sub = rospy.Subscriber(root_topic + '/leds/yellow', Byte, self._yellow_led_handler)
        self._green_led_sub = rospy.Subscriber(root_topic + '/leds/green', Byte, self._green_led_handler)
        self._red_led_sub = rospy.Subscriber(root_topic + '/leds/red', Byte, self._red_led_handler)
        self._buzzer_sub = rospy.Subscriber(root_topic + '/buzzer', String, self._buzzer_handler)

        self._interface.set_motor_speeds(self._left_motor_cmd, self._right_motor_cmd)
        self._interface.clear_motor_counts()
        self._interface.set_leds(self._yellow_led, self._green_led, self._red_led)

        self._button_rate = rospy.Rate(params['controller']['button_sample_rate'])
        self._battery_rate = rospy.Rate(params['controller']['battery_sample_rate'])

    def _yellow_led_handler(self, msg):
        with self._lock:
            self._yellow_led = msg.data != 0
            self._led_changed = True

    def _green_led_handler(self, msg):
        with self._lock:
            self._green_led = msg.data != 0
            self._led_changed = True

    def _red_led_handler(self, msg):
        with self._lock:
            self._red_led = msg.data != 0
            self._led_changed = True

    def _buzzer_handler(self, msg):
        with self._lock:
            self._buzzer_notes = msg.data

    def _position_reset_handler(self):
        with self._lock:
            self._position_reset = True

    def _motor_cmd_handler(self, msg):
        with self._lock:
            self._left_motor_cmd = min(self._motors_max_cmd, max(-self._motors_max_cmd, msg.left_motor))
            self._right_motor_cmd = min(self._motors_max_cmd, max(-self._motors_max_cmd, msg.right_motor))

    def _reset_position(self):
        if self._position_reset:
            while True:
                try:
                    self._interface.clear_motor_counts()
                    self._position_reset = False
                except IOError:
                    print('position_reset: IOERROR')

    def _set_motor_speeds(self):
        try:
            self._interface.set_motor_speeds(self._left_motor_cmd, self._right_motor_cmd)
        except IOError:
            print('motor: IOError')

    def _publish_position(self):
        try:
            left_count, right_count = self._interface.get_motor_rates()
            self._motor_position.left_count = left_count
            self._motor_position.right_count = right_count
        except IOError:
            print('position: IOError')
        finally:
            self._motor_position_pub.publish(self._motor_position)

    def _publish_button_state(self):
        if self._button_rate.remaining().to_sec() <= 0.0:
            self._button_rate.last_time = rospy.rostime.get_rostime()
            try:
                button_a, button_b, button_c = self._interface.is_button_pushed()
                if button_a or button_b or button_c:
                    buttons_msg = Buttons()
                    buttons_msg.header.stamp = rospy.Time.now()
                    buttons_msg.button_a = button_a
                    buttons_msg.button_b = button_b
                    buttons_msg.button_c = button_c
                    self._buttons_pub.publish(buttons_msg)
            except IOError:
                print('button: IOError')

    def _publish_battery_state(self):
        if self._battery_rate.remaining().to_sec() <= 0.0:
            try:
                cell_count, battery_millivolts, low_voltage_cutoff = self._interface.get_battery_state()
                low_voltage_cutoff /= 1000.0

                self._battery_state = BatteryState()
                self._battery_state.header.stamp = rospy.Time.now()
                self._battery_state.voltage = battery_millivolts / 1000.0
                self._battery_state.current = float('nan')
                self._battery_state.charge = float('nan')
                self._battery_state.capacity = float('nan')
                self._battery_state.design_capacity = self._battery_design_capacity
                self._battery_state.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
                if cell_count > 0:
                    self._battery_state.present = True
                    self._battery_state.percentage = (self._battery_state.voltage - low_voltage_cutoff) / \
                                                     ((cell_count * 4.2) - low_voltage_cutoff)
                    self._battery_state.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
                    self._battery_state.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
                    self._battery_state.cell_voltage = [self._battery_state.voltage / cell_count for _ in
                                                        range(0, cell_count)]
                    self._battery_state.location = '0'
                    self._battery_state.serial_number = self._battery_serial_number
                else:
                    self._battery_state.present = False
                    self._battery_state.percentage = 0.0
                    self._battery_state.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
                    self._battery_state.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
                    self._battery_state.location = ''
                    self._battery_state.serial_number = ''
            except IOError:
                print('battery: IOError')
            finally:
                self._battery_rate.last_time = rospy.rostime.get_rostime()
                self._battery_state_pub.publish(self._battery_state)

    def _set_leds(self):
        if self._led_changed:
            try:
                self._interface.set_leds(self._yellow_led, self._green_led, self._red_led)
                self._led_changed = False
            except IOError:
                print('led: IOError')

    def _play_buzzer(self):
        if self._buzzer_notes is not None:
            try:
                self._interface.play_notes(self._buzzer_notes)
                self._buzzer_notes = None
            except IOError:
                print('buzzer: IOError')

    def supervise(self):
        with self._lock:
            self._reset_position()
            self._set_motor_speeds()
            self._publish_position()
            self._publish_button_state()
            self._publish_battery_state()
            self._set_leds()
            self._play_buzzer()

    def shutdown(self):
        with self._lock:
            while True:
                try:
                    self._interface.set_motor_speeds(0, 0)
                    self._interface.clear_motor_counts()
                    self._interface.set_leds(False, False, False)
                    break
                except IOError:
                    print("shutdown: IOError")
