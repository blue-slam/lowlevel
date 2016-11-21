from threading import Lock

import numpy as np
import rospy

from _a_star_interface import AStarInterface
from std_msgs.msg import Byte, String, Empty
from sensor_msgs.msg import BatteryState
from lowlevel.msg import Buttons
from control_msgs.msg import JointCommand
from sensor_msgs.msg import JointState

class BlueSlam:

    def __init__(self, robot_name):

        self._lock = Lock()
        self._interface = AStarInterface()

        self._battery_design_capacity = rospy.get_param(robot_name + '/battery/design_capacity')
        self._battery_serial_number = rospy.get_param(robot_name + '/battery/serial_number')

        self._gear_ratio = rospy.get_param(robot_name + '/motors/gear_ratio')
        self._encoder_ratio = rospy.get_param(robot_name + '/motors/encoder_ratio')
        self._max_cmd = rospy.get_param(robot_name + '/motors/max_cmd')
        self._max_rpm = rospy.get_param(robot_name + '/motors/max_rpm')
        self._max_volts = rospy.get_param(robot_name + '/motors/max_volts')
        self._min_volts = rospy.get_param(robot_name + '/motors/min_volts')
        self._r = rospy.get_param(robot_name + '/geometry/r')
        self._d = rospy.get_param(robot_name + '/geometry/d')
        self._kp = rospy.get_param(robot_name + '/motors/kp')
        self._ki = rospy.get_param(robot_name + '/motors/ki')
        self._kd = rospy.get_param(robot_name + '/motors/kd')

        self._ku = 0
        self._kv = (30 * (self._max_volts - self._min_volts)) / (self._max_rpm * np.pi)
        self._ke = np.full([2, 1], (2 * np.pi) / (self._encoder_ratio * self._gear_ratio))

        self._last_wheel_stamp = rospy.get_rostime()
        self._joint_state = JointState()
        self._joint_state.name = ['left_motor_to_left_motor_output_shaft_joint', 'right_motor_to_right_motor_output_shaft_joint', 'castor_backet_to_castor_joint', 'castor_to_castor_wheel_joint']
        self._joint_state.position = [0.0, 0.0, 0.0, 0.0]
        self._joint_state.velocity = [0.0, 0.0, 0.0, 0.0]
        self._measured_wheel_vel = np.zeros([2, 1])
        self._measured_wheel_pos = np.zeros([2, 1])
        self._ready = 0

        self._last_joint_cmd_stamp = rospy.get_rostime()
        self._wheel_vel_des = np.zeros([2, 1])
        self._u = np.zeros([2, 1])
        self._error = np.zeros([2, 1])
        self._integral = np.zeros([2, 1])

        self._position_reset = False

        self._led_changed = False
        self._yellow = False
        self._green = False
        self._red = False

        self._buzzer_notes = None

        self._battery_state = BatteryState()

        self._battery_state_pub = rospy.Publisher('/states/battery', BatteryState, queue_size=10, latch=True)
        self._joint_state_pub = rospy.Publisher('/states/motors', JointState, queue_size=10)

        self._buttons_pub = rospy.Publisher('/events/buttons', Buttons, queue_size=10)

        self._joint_cmd_sub = rospy.Subscriber('/commands/motors', JointCommand, self._joint_cmd_handler)
        self._reset_sub = rospy.Subscriber('/commands/reset', Empty, self._reset_handler)
        self._yellow_sub = rospy.Subscriber('/commands/yellow', Byte, self._yellow_handler)
        self._green_sub = rospy.Subscriber('/commands/green', Byte, self._green_handler)
        self._red_sub = rospy.Subscriber('/commands/red', Byte, self._red_handler)
        self._buzzer_sub = rospy.Subscriber('/commands/buzzer', String, self._buzzer_handler)

        self._button_rate = rospy.Rate(rospy.get_param(robot_name + '/controller/button_sample_rate'))
        self._battery_rate = rospy.Rate(rospy.get_param(robot_name + '/controller/battery_sample_rate'))
        self._cmd_timeout = rospy.get_param(robot_name + '/controller/cmd_timeout')

        self._interface.set_motor_speeds(self._u[0, 0], self._u[1, 0])
        self._interface.clear_motor_counts()
        self._interface.set_leds(self._yellow, self._green, self._red)

    def _yellow_handler(self, msg):
        with self._lock:
            self._yellow = msg.data != 0
            self._led_changed = True

    def _green_handler(self, msg):
        with self._lock:
            self._green = msg.data != 0
            self._led_changed = True

    def _red_handler(self, msg):
        with self._lock:
            self._red = msg.data != 0
            self._led_changed = True

    def _set_leds(self):
        if self._led_changed:
            try:
                self._interface.set_leds(self._yellow, self._green, self._red)
                self._led_changed = False
            except IOError:
                print('led: IOError')

    def _buzzer_handler(self, msg):
        with self._lock:
            self._buzzer_notes = msg.data

    def _play_buzzer(self):
        if self._buzzer_notes is not None:
            try:
                self._interface.play_notes(self._buzzer_notes)
                self._buzzer_notes = None
            except IOError:
                print('buzzer: IOError')

    def _reset_handler(self, msg):
        with self._lock:
            self._position_reset = True

    def _reset(self):
        if self._position_reset:
            while True:
                try:
                    self._interface.clear_motor_counts()
                    self._position_reset = False
                    break;
                except IOError:
                    print('position_reset: IOERROR')

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
                    self._ku = self._max_cmd / self._battery_state.voltage
                else:
                    self._battery_state.present = False
                    self._battery_state.percentage = 0.0
                    self._battery_state.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
                    self._battery_state.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
                    self._battery_state.location = ''
                    self._battery_state.serial_number = ''
                    self._ku = 0

            except IOError:
                print('battery: IOError')
            finally:
                self._battery_rate.last_time = rospy.rostime.get_rostime()
                self._battery_state_pub.publish(self._battery_state)

    def _joint_cmd_handler(self, msg):
        with self._lock:
            self._last_joint_cmd_stamp = rospy.get_rostime()
            self._wheel_vel_des[0, 0] = msg.velocity[0]
            self._wheel_vel_des[1, 0] = msg.velocity[1]

    def _update_joint_state(self):
        try:
            stamp = rospy.get_rostime()
            dt = (stamp - self._last_wheel_stamp).to_sec()

            left_count, right_count = self._interface.get_motor_counts()
            new_position = np.array([[left_count], [right_count]]) * self._ke
            # print('count: [{:f},{:f}] rad: [{:f},{:f}] dis: [{:f},{:f}]'.format(left_count, right_count, new_position[0, 0], new_position[1, 0], new_position[0, 0] * self._r, new_position[1, 0] * self._r))

            if dt > 0:
                self._measured_wheel_vel = (new_position - self._measured_wheel_pos) / dt
                self._measured_wheel_pos = new_position

                self._joint_state.header.stamp = stamp
                self._joint_state.position[0] = self._measured_wheel_pos[0]
                self._joint_state.position[1] = self._measured_wheel_pos[1]

                self._joint_state.velocity[0] = self._measured_wheel_vel[0]
                self._joint_state.velocity[1] = self._measured_wheel_vel[1]

                self._joint_state_pub.publish(self._joint_state)

                self._last_wheel_stamp = stamp
                self._ready += 1

                if self._ready < 10:
                    return

                error = self._wheel_vel_des - self._measured_wheel_vel
                integral = self._integral + error * dt
                derivative = (error - self._error) / dt

                v = self._kp * error + self._ki * self._integral + self._kd * derivative
                u = self._u + np.round(self._ku * ((v * self._kv) + (np.sign(v) * self._min_volts)))

                if np.abs(error[0, 0]) == 0 or np.abs(u[0, 0]) > self._max_cmd:
                    integral[0, 0] = 0.0
                    if v[0, 0] == 0:
                        u[0, 0] = 0
                if np.abs(error[1, 0]) == 0 or np.abs(u[1, 0]) > self._max_cmd:
                    integral[1, 0] = 0.0
                    if v[1, 0] == 0:
                        u[1, 0] = 0

                if (stamp - self._last_joint_cmd_stamp).to_sec() > self._cmd_timeout:
                    u = np.zeros([2, 1])
                    error = np.zeros([2, 1])
                    integral = np.zeros([2, 1])

                self._u = u
                self._error = error
                self._integral = integral

                while True:
                    try:
                        self._interface.set_motor_speeds(u[0, 0], u[1, 0])
                        # print('wheel_vel: [{:f},{:f}] wheel_des: [{:f},{:f}] error: [{:f},{:f}] u: [{:f},{:f}]'.format(
                        #     self._measured_wheel_vel[0, 0], self._measured_wheel_vel[1, 0],
                        #     self._wheel_vel_des[0, 0], self._wheel_vel_des[1, 0],
                        #     self._error[0, 0], self._error[1, 0],
                        #     self._u[0, 0], self._u[1, 0]))
                        break
                    except IOError:
                        print("motor: IOError")

            else:
                print('bad dt: {:f}'.format(dt))

        except IOError:
            print('position: IOError')

    def update(self, current_time, period):
        with self._lock:
            self._reset()
            self._update_joint_state()
            self._publish_button_state()
            self._publish_battery_state()
            self._set_leds()
            self._play_buzzer()
